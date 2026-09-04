"""토크 포화 모델(IsaacLab ParkourDCMotor) 게이트.

무엇을 재는가
-------------
정지 상태(dq ~ 0)에서는 모델이

    tau_max = clip(sat*(1 - dq/dq_lim), 0, eff_lim)  ->  eff_lim

으로 수렴한다. 그래서 도달 불가능한 관절 목표를 큰 kp 로 명령하면, 실제 적용
토크는 **관절별 effort_limit 에서 정확히 잘려야** 한다:

    hip  (SDK 0, 3, 6, 9)  -> 35 N.m
    thigh/calf (나머지)     -> 40 N.m

이 한 번의 시험이 세 가지를 동시에 본다.
  1. 모델이 실제로 켜져 있는가 (안 켜져 있으면 토크가 한계를 넘는다)
  2. 관절별 값이 올바른 순서로 들어갔는가 (hip 과 thigh 의 한계가 다르다)
  3. MJCF 의 ctrlrange 가 우리 모델보다 먼저 자르지 않는가
     (원래 ±23.7 이라 hip 35 / thigh 40 을 낼 수 없었다)

읽는 값은 lowstate.tau_est = MuJoCo jointactuatorfrc, 즉 실제 적용된 토크다.

    DISPLAY=:1 python3 example/test_motor_saturation.py
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "example"))
from test_keyboard_pty import drain, kill_tree, proc_state, spawn_sim_on_pty  # noqa: E402

SIDECAR = Path.home() / "workspace/codes/unitree_rl_lab/deploy/parkour"

HIP_SDK = [0, 3, 6, 9]           # FR, FL, RR, RL 의 hip
EXPECT = np.array([35.0 if i in HIP_SDK else 40.0 for i in range(12)])


PAIRS_NPZ = "/home/seo-jinwoo/.claude/jobs/c76485cb/tmp/motor_pairs.npz"


def run_probe(seconds: float) -> dict:
    """lowcmd 로 도달 불가능한 목표를 명령하고 tau_est 를 모은다."""
    code = f'''
import sys, time
import numpy as np
sys.path.insert(0, "{SIDECAR}")
from em_sidecar.dds_compat import init_dds
init_dds(0, "lo")
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

taus, dqs = [], []
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(lambda m: (taus.append([m.motor_state[i].tau_est for i in range(12)]),
                    dqs.append([m.motor_state[i].dq for i in range(12)])), 10)
pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()
crc = CRC()
cmd = unitree_go_msg_dds__LowCmd_()
cmd.head[0], cmd.head[1] = 0xFE, 0xEF
cmd.level_flag = 0xFF
for i in range(20):
    cmd.motor_cmd[i].mode = 0x01
    cmd.motor_cmd[i].q = 0.0
    cmd.motor_cmd[i].kp = 0.0
    cmd.motor_cmd[i].dq = 0.0
    cmd.motor_cmd[i].kd = 0.0
    cmd.motor_cmd[i].tau = 0.0
t0 = time.time()
while time.time() - t0 < {seconds}:
    for i in range(12):
        # 도달 불가능한 목표 + 큰 kp -> 항상 포화 영역
        cmd.motor_cmd[i].q = 5.0
        cmd.motor_cmd[i].kp = 500.0
        cmd.motor_cmd[i].kd = 0.0
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)
    time.sleep(0.002)

t = np.array(taus); d = np.array(dqs)
if len(t) == 0:
    print("NO_DATA")
else:
    print("TAU_MAX=" + ",".join(f"{{v:.4f}}" for v in np.abs(t).max(axis=0)))
    print("DQ_MAX=" + ",".join(f"{{v:.3f}}" for v in np.abs(d).max(axis=0)))
    print("N=" + str(len(t)))
    np.savez("{PAIRS_NPZ}", tau=t, dq=d)
'''
    import subprocess
    r = subprocess.run(["conda", "run", "-n", "env_isaaclab", "python", "-u", "-c", code],
                       capture_output=True, text=True, timeout=seconds + 120)
    out = {}
    for ln in r.stdout.splitlines():
        if "=" in ln and ln.split("=")[0].isupper():
            k, v = ln.split("=", 1)
            out[k] = v
    if not out:
        print(r.stdout[-800:])
        print(r.stderr[-800:])
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--display", default=os.environ.get("DISPLAY", ":1"))
    ap.add_argument("--seconds", type=float, default=4.0)
    a = ap.parse_args()

    print("시뮬레이터 기동")
    master, sim, leader = spawn_sim_on_pty(True, REPO / "simulate" / "build", a.display)
    time.sleep(4.0)
    buf: list = []
    drain(master, buf)
    if proc_state(sim) == "gone":
        print("기동 실패:", b"".join(buf).decode(errors="replace")[-600:])
        kill_tree(leader)
        return 1
    log = b"".join(buf).decode(errors="replace")
    active = "[motor] 토크 포화 모델 활성" in log
    print(f"  포화 모델 활성 로그: {'있음' if active else '없음'}")

    print(f"{a.seconds:.0f}초 동안 도달 불가능 목표를 명령하고 tau_est 관측")
    out = run_probe(a.seconds)
    kill_tree(sim)
    kill_tree(leader)
    os.close(master)

    if "TAU_MAX" not in out:
        print("[RESULT] FAILED — 데이터를 못 받았다")
        return 1

    tau = np.array([float(x) for x in out["TAU_MAX"].split(",")])
    dq = np.array([float(x) for x in out["DQ_MAX"].split(",")])

    names = []
    for leg in ("FR", "FL", "RR", "RL"):
        names += [f"{leg}_hip", f"{leg}_thigh", f"{leg}_calf"]

    print(f"\n{'관절':10s} {'|tau|max':>9s} {'기대한계':>9s} {'|dq|max':>9s}")
    fails = []
    for i in range(12):
        over = tau[i] > EXPECT[i] + 0.5
        print(f"  {names[i]:10s} {tau[i]:9.3f} {EXPECT[i]:9.1f} {dq[i]:9.2f}"
              f"{'   ← 한계 초과' if over else ''}")
        if over:
            fails.append(f"{names[i]}: {tau[i]:.2f} > {EXPECT[i]:.1f}")

    # --- 속도 의존 곡선까지 본다 -------------------------------------------
    # 위 검사는 평평한 상한(dq~0)만 본다. 모델의 핵심은 속도가 오를수록 한계가
    # 줄어드는 것이므로, 표본마다 그 시점의 dq 로 계산한 포락선 안에 있는지 잰다.
    SAT = np.array([35.0 if i in HIP_SDK else 45.0 for i in range(12)])
    VLIM = np.array([52.4 if i in HIP_SDK else 30.1 for i in range(12)])
    try:
        pr = np.load(PAIRS_NPZ)
        T, D = pr["tau"], pr["dq"]
        # tau 와 dq 의 **관측 시점이 정확히 같지 않다.** 브리지는 sensordata[t] 로
        # ctrl 을 쓰고, lowstate 는 물리가 한두 스텝 더 간 뒤의 sensordata 를 싣는다.
        # 관절이 초당 수십 rad 로 흔들릴 때 그 한두 스텝이 크다. 그래서 "최근 몇
        # 샘플 중 어떤 dq 로든 설명되는가" 로 판정한다. 이래도 식이 틀렸거나 값/순서가
        # 어긋나거나 모델이 꺼져 있으면 위반이 남는다.
        # (실측: 같은 시점 418건 → dq[t-2] 34건 → ±3 창 0건. 시점 어긋남이 맞다.)
        W = 3
        ok = np.zeros_like(T, dtype=bool)
        for sh in range(-W, W + 1):
            Ds = np.roll(D, -sh, axis=0)
            r = Ds / VLIM
            tmax = np.clip(SAT * (1.0 - r), 0.0, EXPECT)
            tmin = np.clip(SAT * (-1.0 - r), -EXPECT, 0.0)
            ok |= (T <= tmax + 0.5) & (T >= tmin - 0.5)
        n_bad = int((~ok).sum())
        n_fast = int((np.abs(D) > 5.0).sum())
        # 속도 항이 실제로 물렸는지: 고속 표본에서 한계가 평평한 값보다 낮아야 한다
        r0 = np.abs(D) / VLIM
        bound = np.clip(SAT * (1.0 - r0), 0.0, EXPECT)
        n_reduced = int(((np.abs(D) > 5.0) & (bound < EXPECT - 1.0)).sum())
        print(f"\n  속도 의존 포락선 검사 (관측 시점 ±{W} 샘플 허용)")
        print(f"    표본 {T.size}개, |dq|>5 rad/s 인 것 {n_fast}개, "
              f"그중 한계가 실제로 낮아진 것 {n_reduced}개")
        print(f"    설명되지 않는 위반 {n_bad}개")
        if n_bad:
            fails.append(f"포락선 위반 {n_bad}개 — 속도 의존 항 확인 필요")
        if n_reduced < 100:
            fails.append("한계가 낮아진 고속 표본이 적어 속도 의존 항을 못 봤다")
    except FileNotFoundError:
        fails.append("표본 파일이 없다")

    # 한계에 실제로 닿았는가 (모델이 켜져 있고 ctrlrange 가 막지 않는다는 증거)
    reached = tau >= EXPECT - 1.0
    n_reached = int(reached.sum())
    print(f"\n  한계에 도달한 관절 {n_reached}/12")
    if n_reached < 12:
        fails.append(f"한계에 못 미친 관절 {12 - n_reached}개 — ctrlrange 가 먼저 자르는지 확인")
    if not active:
        fails.append("포화 모델 활성 로그가 없다 (config.yaml 의 motor_saturation 확인)")

    print()
    for f in fails:
        print(f"  FAIL: {f}")
    print(f"[RESULT] {'OK' if not fails else 'FAILED'}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
