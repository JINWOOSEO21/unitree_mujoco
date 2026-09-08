"""KeyboardJoystick 회귀 테스트 — 사람이 앉아서 키를 누르는 것을 pty 로 재현한다.

왜 필요한가
-----------
이 기능은 **사람이 터미널에 키를 눌러야** 동작해서 눈으로만 확인하기 쉽고,
그래서 조용히 깨진 적이 있다:

  `./unitree_mujoco &` 로 띄우면 창이 "응답 없음" 이 됐다. 백그라운드 프로세스
  그룹이 제어 터미널에 tcsetattr 을 하면 SIGTTOU, read 를 하면 SIGTTIN 을 받고
  **기본 동작이 프로세스 정지**이기 때문이다. 정지하면 GLFW 이벤트 루프가 멈춘다.

pty 를 만들어 두 상황을 자동으로 재현한다.

  [1] 백그라운드(`&`) — 시뮬레이터가 정지하지 않는가
  [2] 포그라운드      — 키를 누르면 lowstate.wireless_remote 에 뜨는가

[2] 는 wireless_echo 로 확인한다. 그 도구는 go2_ctrl 의 FSM 이 읽는 경로와
**똑같은 경로**(lowstate.wireless_remote → memcpy → UnitreeJoystick::extract)를
보므로, 여기 뜨면 go2_ctrl 도 반드시 같은 값을 본다.

사용법 (X 디스플레이가 필요하다):
    DISPLAY=:1 python3 example/test_keyboard_pty.py
"""
from __future__ import annotations

import argparse
import os
import pty
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SIM_DIR = REPO / "simulate" / "build"
ECHO = REPO / "example" / "cpp" / "build" / "wireless_echo"


def spawn_sim_on_pty(foreground: bool, sim_dir: Path, display: str):
    """시뮬레이터를 pty 에 붙여 띄운다. 반환 (master_fd, sim_pid, leader_pid).

    구조: 세션 리더 A(셸 역할)가 pty 를 제어 터미널로 잡고 포그라운드가 된 뒤
    시뮬레이터 B 를 fork 한다.
      foreground=True  → B 는 A 의 pgrp 그대로 = 포그라운드
      foreground=False → B 가 setpgid(0,0) 으로 별도 pgrp = 백그라운드 (셸의 `&`)

    A 자신이 setpgid 를 부를 수는 없다 — 세션 리더는 pgrp 를 바꿀 수 없어 EPERM 이다.
    """
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    rd, wr = os.pipe()

    leader = os.fork()
    if leader == 0:
        os.close(rd)
        os.setsid()
        fd = os.open(slave_name, os.O_RDWR)  # setsid 뒤에 열면 제어 터미널이 된다
        os.dup2(fd, 0)
        os.dup2(fd, 1)
        os.dup2(fd, 2)
        if fd > 2:
            os.close(fd)
        os.close(master)
        os.tcsetpgrp(0, os.getpgrp())

        sim = os.fork()
        if sim == 0:
            os.close(wr)
            if not foreground:
                os.setpgid(0, 0)
            os.chdir(sim_dir)
            # SIM_ARGS 환경변수로 추가 인자 전달 (예: SIM_ARGS="-s scene_parkour_stairs.xml")
            extra = os.environ.get("SIM_ARGS", "").split()
            os.execve("./unitree_mujoco", ["./unitree_mujoco", *extra],
                      dict(os.environ, DISPLAY=display))
            os._exit(127)
        os.write(wr, str(sim).encode())
        os.close(wr)
        os.waitpid(sim, 0)
        os._exit(0)

    os.close(slave)
    os.close(wr)
    sim_pid = int(os.read(rd, 32).decode())
    os.close(rd)
    os.set_blocking(master, False)
    return master, sim_pid, leader


def drain(fd, buf):
    try:
        while True:
            d = os.read(fd, 65536)
            if not d:
                break
            buf.append(d)
    except (BlockingIOError, OSError):
        pass


def proc_state(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().split(") ", 1)[1].split()[0]
    except FileNotFoundError:
        return "gone"


def kill_tree(pid):
    for fn in (lambda: os.killpg(os.getpgid(pid), signal.SIGKILL),
               lambda: os.kill(pid, signal.SIGKILL)):
        try:
            fn()
            break
        except (ProcessLookupError, PermissionError):
            continue
    try:
        os.waitpid(pid, 0)
    except ChildProcessError:
        pass


def test_background(sim_dir: Path, display: str) -> bool:
    print("=" * 70)
    print("[1] `&` 상황: 백그라운드 프로세스 그룹에서 정지하지 않는가")
    master, sim, leader = spawn_sim_on_pty(False, sim_dir, display)
    buf = []
    for _ in range(60):
        time.sleep(0.1)
        drain(master, buf)
    st = proc_state(sim)
    out = b"".join(buf).decode(errors="replace")
    print(f"    프로세스 상태 = {st}   (T = 정지 = 창이 응답 없음)")
    for line in out.splitlines():
        if "KeyboardJoystick" in line:
            print(f"    | {line.strip()}")
    kill_tree(sim)
    kill_tree(leader)
    os.close(master)
    ok = st in ("S", "R", "D")
    print(f"    → {'PASS' if ok else 'FAIL'}\n")
    return ok


def test_keys(sim_dir: Path, echo: Path, display: str) -> bool:
    print("=" * 70)
    print("[2] 포그라운드: 키가 lowstate.wireless_remote 에 뜨는가")
    master, sim, leader = spawn_sim_on_pty(True, sim_dir, display)
    time.sleep(3.0)
    buf = []
    drain(master, buf)
    if proc_state(sim) == "gone":
        print("    시뮬레이터가 기동하지 못했다:")
        for ln in b"".join(buf).decode(errors="replace").splitlines()[-15:]:
            print(f"    | {ln}")
        kill_tree(leader)
        os.close(master)
        return False

    proc = subprocess.Popen([str(echo), "lo"], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True,
                            stdin=subprocess.DEVNULL)
    time.sleep(2.0)
    for key, label in ((b"1", "1 → LT+A (일어서기)"), (b"2", "2 → start (정책 시작)"),
                       (b"0", "0 → LT+B (Passive)"), (b"w", "w → 전진")):
        print(f"    키 입력: {label}")
        os.write(master, key)
        time.sleep(1.2)
        drain(master, buf)

    time.sleep(0.5)
    proc.send_signal(signal.SIGINT)
    try:
        out, _ = proc.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
    kill_tree(sim)
    kill_tree(leader)
    os.close(master)

    print("    --- wireless_echo (go2_ctrl 이 보는 것과 동일) ---")
    seen = [ln for ln in out.splitlines() if ln.startswith("buttons:")]
    for ln in seen[:12]:
        print(f"    | {ln}")
    if len(seen) > 12:
        print(f"    | ... ({len(seen)-12}줄 더)")

    checks = {
        "LT (1/0 키)": any("LT" in ln for ln in seen),
        "A (1 키)": any("LT A" in ln for ln in seen),
        "B (0 키)": any("LT B" in ln for ln in seen),
        "start (2 키)": any("start" in ln for ln in seen),
        "ly (w 키)": any("ly=+0.2" in ln or "ly=+0.1" in ln for ln in seen),
    }
    print()
    for k, v in checks.items():
        print(f"    {k:14s} {'감지됨' if v else '안 뜸'}")
    ok = all(checks.values())
    print(f"    → {'PASS' if ok else 'FAIL'}\n")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim-dir", default=str(SIM_DIR))
    ap.add_argument("--echo", default=str(ECHO))
    ap.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    a = ap.parse_args()

    echo = Path(a.echo)
    if not echo.exists():
        print(f"wireless_echo 가 없다: {echo}\n  example/cpp 를 먼저 빌드할 것")
        return 1

    r1 = test_background(Path(a.sim_dir), a.display)
    r2 = test_keys(Path(a.sim_dir), echo, a.display)
    print("=" * 70)
    ok = r1 and r2
    print(f"[RESULT] {'OK' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
