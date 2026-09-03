/**
 * L1 스캔 운동학이 만드는 레이 방향을 stdout 으로 덤프한다 (검증 전용).
 *
 * Isaaclab_Parkour 의 파이썬 원본과 같은 값이 나오는지 대조하기 위한 것.
 * DDS 도 MuJoCo 도 쓰지 않는다 — l1_scan_kinematics.h 만 있으면 된다.
 *
 * 사용법:  ./l1_dump_dirs <t_end> <phase_pitch_deg> <phase_yaw_deg>
 * 출력  :  한 줄에 "x y z" (rays_per_frame 줄), 소수점 12자리
 */
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "l1_scan_kinematics.h"

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::fprintf(stderr, "usage: %s <t_end> <phase_pitch_deg> <phase_yaw_deg>\n", argv[0]);
        return 1;
    }
    const double t_end = std::atof(argv[1]);
    const double pp = std::atof(argv[2]);
    const double py = std::atof(argv[3]);

    l1::ScanParams p;
    std::vector<double> dirs(p.rays_per_frame * 3);
    l1::compute_frame_directions(p, t_end, pp, py, dirs.data());

    for (int i = 0; i < p.rays_per_frame; i++) {
        std::printf("%.12f %.12f %.12f\n", dirs[3 * i], dirs[3 * i + 1], dirs[3 * i + 2]);
    }
    return 0;
}
