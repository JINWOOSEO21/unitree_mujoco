#pragma once

/**
 * 키보드를 Unitree 무선 조종기처럼 쓰는 조이스틱 구현.
 *
 * 왜 이 자리인가
 * --------------
 * unitree_rl_lab 의 go2_ctrl 은 FSM 전환 조건을 `lowstate->joystick` 에서 읽는다
 * (deploy/include/FSM/FSMState.h). 그 joystick 은 rt/wireless_controller 토픽이
 * 아니라 **rt/lowstate 안의 wireless_remote 40바이트**에서 복원된다
 * (sdk2 의 go2_sub.h: memcpy(&key, &msg_.wireless_remote()[0], 40)).
 * 따라서 "키보드 → rt/wireless_controller 발행" 방식으로는 FSM 이 반응하지 않는다.
 *
 * 대신 시뮬레이터가 원래 게임패드를 꽂는 자리(XBoxJoystick / SwitchJoystick)에
 * 키보드 구현을 하나 더 끼운다. 그러면 시뮬레이터가 이 값을 lowstate.wireless_remote
 * 와 rt/wireless_controller 양쪽에 실어 주므로 **go2_ctrl 은 한 줄도 고칠 필요가 없다.**
 *
 * 키 배치 (config.yaml 에서 joystick_type: "keyboard")
 * ---------------------------------------------------
 *   1 : LT + A     Passive -> FixStand   (일어서기)
 *   2 : start      FixStand -> RL 정책
 *   0 : LT + B     -> Passive            (힘 빼기 / 비상정지)
 *   w/s : 전진/후진 (ly)      a/d : 좌/우 (lx)      q/e : 좌/우 회전 (rx)
 *   space : 스틱 전부 0
 *
 * 버튼은 누른 순간 kLatchMs 동안 눌린 것으로 유지했다가 저절로 떼진다. FSM 의
 * `.on_pressed` 가 상승 엣지를 봐야 하는데, 터미널 키 입력에는 "뗌" 이벤트가
 * 없기 때문이다.
 *
 * 잡 컨트롤 (`&` 로 띄웠을 때 멈추던 문제)
 * ----------------------------------------
 * 백그라운드 프로세스 그룹이 제어 터미널에 tcsetattr 을 하면 SIGTTOU, read 를 하면
 * SIGTTIN 을 받고 **기본 동작이 프로세스 정지**다. 정지되면 GLFW 이벤트 루프가
 * 멈춰 창이 "응답 없음" 이 된다. SIGCONT 로 깨워도 중단된 syscall 이 재시작되며
 * 다시 정지하므로 빠져나오지 못한다.
 *
 * 그래서 터미널은 **포그라운드 프로세스 그룹일 때만** 건드린다. 판정은 매 반복
 * `tcgetpgrp(0) == getpgrp()` 로 한다 — 덕분에 `./unitree_mujoco &` 로 띄워도
 * 멈추지 않고, 나중에 `fg` 하면 키보드가 저절로 살아난다. 두 신호는 무시로
 * 돌려 놓아(SIG_IGN) 경계에서 걸치더라도 정지 대신 오류 반환이 되게 한다.
 */

#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <thread>

#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

class KeyboardJoystick : public unitree::common::UnitreeJoystick
{
public:
    KeyboardJoystick() : unitree::common::UnitreeJoystick()
    {
        // 백그라운드에서 터미널을 건드려도 **정지되지 않게** 한다.
        // SIGTTOU 를 무시하면 tcsetattr 이 그냥 성공하고, SIGTTIN 을 무시하면
        // read 가 정지 대신 EIO 로 실패한다 (POSIX). 둘 다 아래 로직이 처리한다.
        std::signal(SIGTTOU, SIG_IGN);
        std::signal(SIGTTIN, SIG_IGN);

        if (!isatty(STDIN_FILENO)) {
            std::cerr << "[KeyboardJoystick] stdin 이 터미널이 아니다. 키 입력을 받을 수 없다."
                      << std::endl;
        }

        print_help();
        running_ = true;
        // raw 모드 전환은 read_loop 이 포그라운드 여부를 보며 직접 관리한다.
        thread_ = std::thread([this] { this->read_loop(); });
    }

    // 주의: UnitreeJoystick 에는 가상 소멸자가 없어 override 를 붙일 수 없다.
    // shared_ptr 은 생성 시점의 실제 타입으로 삭제자를 기억하므로 이 소멸자는 정상 호출된다.
    ~KeyboardJoystick()
    {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        disable_raw();  // SIGTTOU 를 무시해 뒀으므로 백그라운드에서도 안전하다
    }

    void update() override
    {
        const auto now = Clock::now();
        auto held = [&](const std::atomic<int64_t>& until) {
            return now.time_since_epoch().count() < until.load();
        };

        auto after = [&](const std::atomic<int64_t>& until, int64_t delay_ms) {
            // 눌린 구간 [t0, t0+kLatchMs] 중 앞쪽 delay_ms 를 뺀 구간
            const int64_t u = until.load();
            const int64_t now_ns = now.time_since_epoch().count();
            const int64_t start_ns =
                u - std::chrono::nanoseconds(std::chrono::milliseconds(kLatchMs - delay_ms)).count();
            return now_ns < u && now_ns >= start_ns;
        };

        const bool k1 = held(until_1_);
        const bool k2 = held(until_2_);
        const bool k0 = held(until_0_);

        // 1 -> LT+A, 0 -> LT+B.
        // 주의: UnitreeJoystick 의 LT/RT 는 Button 이 아니라 **Axis** 라서 smooth=0.03
        // 저역통과를 거친다. 0 -> 1 로 올려도 threshold 0.5 를 넘기까지 약 23 update
        // (1kHz 브리지에서 ~23ms) 가 걸린다. LT 와 A 를 동시에 눌러 버리면 A 의
        // on_pressed 상승 엣지가 LT.pressed 가 아직 false 인 순간에 발생해서
        // go2_ctrl 의 `LT + A.on_pressed` 조건이 영원히 성립하지 않는다.
        // 그래서 LT 를 먼저 켜고 A/B 를 kButtonDelayMs 만큼 늦게 누른다.
        LT(k1 || k0);
        A(after(until_1_, kButtonDelayMs));
        B(after(until_0_, kButtonDelayMs));
        start(k2);

        back(false);
        LB(false);
        RB(false);
        X(false);
        Y(false);
        up(false);
        down(false);
        left(false);
        right(false);
        RT(false);

        lx(lx_.load());
        ly(ly_.load());
        rx(rx_.load());
        ry(0.0);
    }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr int64_t kLatchMs = 1000;       // 키를 누른 것으로 유지하는 시간
    static constexpr int64_t kButtonDelayMs = 300;  // LT 가 Axis smoothing 을 통과할 여유

    // kButtonDelayMs 를 80 → 300 으로 늘린 이유 (실측):
    // LT 는 Button 이 아니라 **Axis** 이고, 저역통과(smooth=0.03)를 **두 번** 거친다.
    //   ① 시뮬레이터의 joystick.update() 가 smoothing 한 값을 wireless_remote 에 싣고
    //   ② go2_ctrl 이 그 바이트를 다시 자기 UnitreeJoystick 으로 extract+update 한다.
    // 한 단만 보면 threshold 0.5 까지 ~23 update(1kHz 에서 23ms)지만, 두 단이 겹치면
    // 그보다 훨씬 길고 **부하에 따라 늘어난다**. 파쿠르 지형(241x561 hfield)을 얹어
    // 브리지가 1kHz 에서 밀리자 80ms 로는 A 의 상승 엣지가 LT.pressed 보다 먼저 와서
    // `LT + A.on_pressed` 가 성립하지 않았다 — 첫 번째 누름이 그냥 씹혔다.
    // (실측: 평지에서는 3/3 성공, 파쿠르 지형에서는 첫 누름이 반복 실패.)
    // 300ms 면 두 단 합보다 충분히 크고, latch 1s 안에서 A 가 700ms 켜져 있다.

    void latch(std::atomic<int64_t>& until)
    {
        until.store((Clock::now() + std::chrono::milliseconds(kLatchMs)).time_since_epoch().count());
    }

    static void print_help()
    {
        std::cout << "\n[KeyboardJoystick] 키보드를 조종기로 사용한다 (이 터미널에 포커스를 둘 것)\n"
                  << "  1 : LT+A  일어서기(FixStand)\n"
                  << "  2 : start RL 정책 시작\n"
                  << "  0 : LT+B  Passive(힘 빼기)\n"
                  << "  w/s 전진·후진   a/d 좌·우   q/e 회전   space 정지\n"
                  << "  h : 이 도움말 다시 보기\n"
                  << "  (누른 키는 [key] 줄로 표시된다. 아무 표시도 없으면 이 터미널에\n"
                  << "   포커스가 없거나 백그라운드로 실행된 것이다.)\n"
                  << std::endl;
    }

    /// stdin 이 터미널이고, 우리가 그 터미널의 포그라운드 프로세스 그룹인가.
    static bool stdin_is_foreground()
    {
        if (!isatty(STDIN_FILENO)) return false;
        const pid_t fg = tcgetpgrp(STDIN_FILENO);
        return fg != -1 && fg == getpgrp();
    }

    void enable_raw()
    {
        if (raw_active_) return;
        if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) return;
        struct termios raw = old_termios_;
        // 캐노니컬 모드/에코 끄기 = 엔터 없이 한 글자씩 즉시 읽는다.
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;   // 없으면 즉시 반환
        raw.c_cc[VTIME] = 1;  // 0.1s 대기 — 이 타임아웃이 폴링 주기가 된다
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;
        raw_active_ = true;
    }

    void disable_raw()
    {
        if (!raw_active_) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
        raw_active_ = false;
    }

    void read_loop()
    {
        bool warned_background = false;
        while (running_) {
            if (!stdin_is_foreground()) {
                // 백그라운드(또는 stdin 이 터미널이 아님) — 터미널을 놓고 쉰다.
                disable_raw();
                if (!warned_background && isatty(STDIN_FILENO)) {
                    std::cerr << "[KeyboardJoystick] 백그라운드로 실행돼 키 입력을 받지 않는다. "
                                 "`fg` 로 포그라운드에 올리면 자동으로 살아난다."
                              << std::endl;
                    warned_background = true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            if (!raw_active_) {
                enable_raw();
                if (warned_background) {
                    std::cerr << "[KeyboardJoystick] 포그라운드 복귀 — 키 입력을 다시 받는다."
                              << std::endl;
                    warned_background = false;
                }
                if (!raw_active_) {  // 터미널을 못 잡았다 — 다음 바퀴에 재시도
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
            }

            char c = 0;
            const ssize_t n = ::read(STDIN_FILENO, &c, 1);
            // n == 0 은 VTIME 타임아웃(정상). n < 0 은 EIO 등 — 어느 쪽이든
            // raw 모드의 0.1s 타임아웃이 폴링 주기라 바쁜 대기가 되지 않는다.
            if (n <= 0) {
                if (n < 0) std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            // raw 모드는 ECHO 를 끄기 때문에 누른 키가 화면에 찍히지 않는다.
            // 아무 표시가 없으면 "키가 먹었는지" 를 알 길이 없어서(그리고 실제로
            // 그것 때문에 로봇이 안 움직이는 것을 키보드 탓으로 오해하기 쉬워서)
            // 인식한 키를 한 줄로 알려 준다.
            switch (c) {
                case '1': latch(until_1_); echo_key("1", "LT+A  일어서기(FixStand)"); break;
                case '2': latch(until_2_); echo_key("2", "start RL 정책 시작"); break;
                case '0': latch(until_0_); echo_key("0", "LT+B  Passive(힘 빼기)"); break;
                case 'w': ly_.store(clamp(ly_.load() + 0.25)); echo_stick("w"); break;
                case 's': ly_.store(clamp(ly_.load() - 0.25)); echo_stick("s"); break;
                case 'a': lx_.store(clamp(lx_.load() - 0.25)); echo_stick("a"); break;
                case 'd': lx_.store(clamp(lx_.load() + 0.25)); echo_stick("d"); break;
                case 'q': rx_.store(clamp(rx_.load() - 0.25)); echo_stick("q"); break;
                case 'e': rx_.store(clamp(rx_.load() + 0.25)); echo_stick("e"); break;
                case ' ':
                    lx_.store(0.0);
                    ly_.store(0.0);
                    rx_.store(0.0);
                    echo_stick("space");
                    break;
                case 'h': print_help(); break;
                default:
                    // 모르는 키도 알려 준다 — 포커스가 여기 있다는 것 자체가 정보다.
                    if (c >= 0x20 && c < 0x7f) {
                        std::printf("[key] '%c' — 할당되지 않은 키다 (h: 도움말)\n", c);
                        std::fflush(stdout);
                    }
                    break;
            }
        }
    }

    static void echo_key(const char* key, const char* what)
    {
        std::printf("[key] %s → %s\n", key, what);
        std::fflush(stdout);
    }

    void echo_stick(const char* key)
    {
        std::printf("[key] %s → 스틱 lx=%+.2f ly=%+.2f rx=%+.2f\n",
                    key, lx_.load(), ly_.load(), rx_.load());
        std::fflush(stdout);
    }

    static double clamp(double v) { return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v); }

    std::atomic<bool> running_{false};
    std::thread thread_;
    struct termios old_termios_{};
    bool raw_active_ = false;

    std::atomic<int64_t> until_1_{0}, until_2_{0}, until_0_{0};
    std::atomic<double> lx_{0.0}, ly_{0.0}, rx_{0.0};
};
