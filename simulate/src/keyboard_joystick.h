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
 */

#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

class KeyboardJoystick : public unitree::common::UnitreeJoystick
{
public:
    KeyboardJoystick() : unitree::common::UnitreeJoystick()
    {
        if (!isatty(STDIN_FILENO)) {
            std::cerr << "[KeyboardJoystick] stdin 이 터미널이 아니다. 키 입력을 받을 수 없다."
                      << std::endl;
        } else {
            tcgetattr(STDIN_FILENO, &old_termios_);
            struct termios raw = old_termios_;
            // 캐노니컬 모드/에코 끄기 = 엔터 없이 한 글자씩 즉시 읽는다.
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;   // 없으면 즉시 반환
            raw.c_cc[VTIME] = 1;  // 0.1s 대기
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            restore_termios_ = true;
        }

        print_help();
        running_ = true;
        thread_ = std::thread([this] { this->read_loop(); });
    }

    // 주의: UnitreeJoystick 에는 가상 소멸자가 없어 override 를 붙일 수 없다.
    // shared_ptr 은 생성 시점의 실제 타입으로 삭제자를 기억하므로 이 소멸자는 정상 호출된다.
    ~KeyboardJoystick()
    {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (restore_termios_) tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
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
    static constexpr int64_t kLatchMs = 500;       // 키를 누른 것으로 유지하는 시간
    static constexpr int64_t kButtonDelayMs = 80;  // LT 가 Axis smoothing 을 통과할 여유

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
                  << std::endl;
    }

    void read_loop()
    {
        while (running_) {
            char c = 0;
            const ssize_t n = ::read(STDIN_FILENO, &c, 1);
            if (n <= 0) continue;
            switch (c) {
                case '1': latch(until_1_); break;
                case '2': latch(until_2_); break;
                case '0': latch(until_0_); break;
                case 'w': ly_.store(clamp(ly_.load() + 0.25)); break;
                case 's': ly_.store(clamp(ly_.load() - 0.25)); break;
                case 'a': lx_.store(clamp(lx_.load() - 0.25)); break;
                case 'd': lx_.store(clamp(lx_.load() + 0.25)); break;
                case 'q': rx_.store(clamp(rx_.load() - 0.25)); break;
                case 'e': rx_.store(clamp(rx_.load() + 0.25)); break;
                case ' ':
                    lx_.store(0.0);
                    ly_.store(0.0);
                    rx_.store(0.0);
                    break;
                default: break;
            }
        }
    }

    static double clamp(double v) { return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v); }

    std::atomic<bool> running_{false};
    std::thread thread_;
    struct termios old_termios_{};
    bool restore_termios_ = false;

    std::atomic<int64_t> until_1_{0}, until_2_{0}, until_0_{0};
    std::atomic<double> lx_{0.0}, ly_{0.0}, rx_{0.0};
};
