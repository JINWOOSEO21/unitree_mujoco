"""C++ 로 이식한 L1 스캔 운동학이 파이썬 원본과 같은 값을 내는지 대조한다.

핵심: 원본을 **재구현하지 않고 그대로 불러온다.**
`l1_scan_ray_caster.py` 는 isaaclab 을 import 하지만, 우리가 쓸
`valid_frame_times` / `directions_from_angles` 는 순수 torch 함수다. 그래서
isaaclab 관련 모듈만 가짜로 끼워 넣고 파일을 그대로 적재한다. 이렇게 해야
"원본과 같다"는 말이 실제로 검증된다 (베껴 적으면 같은 실수를 두 번 하게 된다).

사용법:
    python verify_l1_against_python.py \
        --dump-bin ./build/l1_dump_dirs \
        --parkour-repo ~/workspace/codes/Isaaclab_Parkour
"""

import argparse
import importlib.util
import subprocess
import sys
import types

import numpy as np


def load_original_module(parkour_repo: str):
    """isaaclab 을 스텁으로 대체하고 원본 모듈을 적재한다."""
    src = f"{parkour_repo}/parkour_isaaclab/sensors/l1_scan_ray_caster.py"

    def stub(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class _Any:
        """상속·데코레이터 어디에 쓰여도 통과하는 더미."""
        def __init__(self, *a, **k):
            pass
        def __call__(self, *a, **k):
            return self
        def __class_getitem__(cls, item):
            return cls

    stub("isaaclab")
    stub("isaaclab.sensors", MultiMeshRayCaster=_Any, MultiMeshRayCasterCfg=_Any)
    stub("isaaclab.sensors.ray_caster")
    stub("isaaclab.sensors.ray_caster.patterns", LidarPatternCfg=_Any)
    stub("isaaclab.utils", configclass=lambda c: c)
    stub("isaaclab.utils.math", quat_apply=lambda *a, **k: None)

    spec = importlib.util.spec_from_file_location("l1_orig", src)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def python_directions(mod, t_end, phase_pitch_deg, phase_yaw_deg,
                      num_rays=2160, pitch_hz=180.0, samples_per_second=43200.0,
                      yaw_period_s=1.0 / 11.0, theta_deg=45.0, ksi_deg=45.0):
    import math
    import torch

    cls = mod.L1ScanRayCaster
    t, alpha = cls.valid_frame_times(
        torch.tensor([[t_end]], dtype=torch.float64),
        torch.tensor([[phase_pitch_deg]], dtype=torch.float64),
        num_rays, pitch_hz, samples_per_second,
    )
    beta = torch.deg2rad(360.0 * t / yaw_period_s + phase_yaw_deg)
    dirs = cls.directions_from_angles(
        alpha.unsqueeze(0).expand_as(t), beta,
        math.radians(theta_deg), math.radians(ksi_deg),
    )
    return dirs[0].numpy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump-bin", required=True)
    ap.add_argument("--parkour-repo", required=True)
    ap.add_argument("--tol", type=float, default=1e-9)
    args = ap.parse_args()

    mod = load_original_module(args.parkour_repo)
    print(f"원본 모듈 적재 OK: {mod.L1ScanRayCaster}")

    fails = 0
    for t_end, pp, py in [(1.0, 123.4, 56.7), (0.35, 0.0, 0.0), (7.77, 359.9, 180.0)]:
        out = subprocess.run([args.dump_bin, str(t_end), str(pp), str(py)],
                             capture_output=True, text=True, check=True)
        cpp = np.array([[float(v) for v in line.split()]
                        for line in out.stdout.strip().splitlines()])
        pyd = python_directions(mod, t_end, pp, py)

        assert cpp.shape == pyd.shape, f"shape 불일치 {cpp.shape} vs {pyd.shape}"
        d = np.abs(cpp - pyd)
        ok = d.max() <= args.tol
        fails += (not ok)
        print(f"\n[t_end={t_end}, phase=({pp}, {py})]  rays {cpp.shape[0]}")
        print(f"  max|diff| = {d.max():.3e}   mean = {d.mean():.3e}   (허용 {args.tol:.0e})")
        # 방향이 단위벡터인지도 같이 본다
        print(f"  |dir| 범위 = [{np.linalg.norm(cpp, axis=1).min():.9f}, "
              f"{np.linalg.norm(cpp, axis=1).max():.9f}]")
        print(f"  => {'PASS' if ok else 'FAIL'}")

    print("\n" + ("전체 PASS — C++ 이식이 파이썬 원본과 일치" if fails == 0
                  else f"{fails}개 실패"))
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
