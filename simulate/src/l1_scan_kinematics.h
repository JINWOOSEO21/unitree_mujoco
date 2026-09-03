#pragma once

/**
 * Unitree L1 이중 모터 스캔 운동학 — **순수 수학**. DDS 도 MuJoCo 도 의존하지 않는다.
 *
 * 분리해 둔 이유: 이 부분은 Isaaclab_Parkour 의 파이썬 원본
 * (parkour_isaaclab/sensors/l1_scan_ray_caster.py 의 valid_frame_times /
 * directions_from_angles)을 이식한 것이라, **원본과 같은 값을 내는지 대조**할 수
 * 있어야 한다. DDS·MuJoCo 가 섞여 있으면 단독 테스트가 불가능하다.
 *
 * 모델 요약
 *  - 반사경(pitch) 180Hz, 코어(yaw) 11Hz, 샘플 클럭 43,200/s
 *  - α(t) = ((360·180·t + φ) mod 360) − 180 이고 α ∈ (−90°, 90°) 인 반주기만 유효
 *  - 회전당 240 샘플 중 120 유효 → 21,600 pts/s, 0.1s 프레임 = 18회전 × 120 = 2,160점
 *  - 포인트 기하 (unilidar_sdk MavLink 문서):
 *      A = −cosθ·sinψ + sinθ·sinα·cosψ,   B = cosα·cosψ
 *      x = cosβ·A − sinβ·B,  y = sinβ·A + cosβ·B,  z = sinθ·sinψ + cosθ·sinα·cosψ
 */

#include <cmath>
#include <vector>

namespace l1
{

struct ScanParams
{
    int    rays_per_frame     = 2160;
    double samples_per_second = 43200.0;
    double pitch_hz           = 180.0;
    double yaw_period_s       = 1.0 / 11.0;
    double theta_deg          = 45.0;
    double ksi_deg            = 45.0;
};

/**
 * t_end 이전의 마지막 완결 프레임에 대한 레이 방향(센서 프레임, 단위벡터)을 채운다.
 * out 은 rays_per_frame*3 크기여야 한다. 순서는 파이썬 원본과 같다
 * (오래된 회전부터, 회전 안에서는 α 오름차순).
 */
inline void compute_frame_directions(const ScanParams& p, double t_end,
                                     double phase_pitch_deg, double phase_yaw_deg,
                                     double* out)
{
    const double samples_per_rev = p.samples_per_second / p.pitch_hz;          // 240
    const int    valid_per_rev   = static_cast<int>(std::lround(samples_per_rev / 2.0));  // 120
    const int    n_revs          = p.rays_per_frame / valid_per_rev;           // 18
    const double rev_T           = 1.0 / p.pitch_hz;

    // α 가 −90° 를 지나는 최초 시각: (360·pitch_hz·t + φ) mod 360 == 90
    double t_cross0 = std::fmod(90.0 - phase_pitch_deg, 360.0);
    if (t_cross0 < 0) t_cross0 += 360.0;
    t_cross0 /= (360.0 * p.pitch_hz);

    // t_end 이전에 창이 완결(시작 + 반주기 ≤ t_end)된 마지막 회전 번호
    const double n_last = std::floor((t_end - t_cross0 - 0.5 * rev_T) * p.pitch_hz);

    const double th = p.theta_deg * M_PI / 180.0;
    const double ks = p.ksi_deg * M_PI / 180.0;
    const double sin_t = std::sin(th), cos_t = std::cos(th);
    const double sin_k = std::sin(ks), cos_k = std::cos(ks);

    int k = 0;
    for (int r = n_revs - 1; r >= 0; r--) {
        const double rev = n_last - r;
        for (int i = 0; i < valid_per_rev; i++) {
            const double alpha_deg = -90.0 + (i + 0.5) * (360.0 / samples_per_rev);
            const double a = alpha_deg * M_PI / 180.0;
            const double t = t_cross0 + rev * rev_T + (i + 0.5) / p.samples_per_second;
            const double beta = (360.0 * t / p.yaw_period_s + phase_yaw_deg) * M_PI / 180.0;

            const double sin_a = std::sin(a), cos_a = std::cos(a);
            const double sin_b = std::sin(beta), cos_b = std::cos(beta);

            const double A = -cos_t * sin_k + sin_t * sin_a * cos_k;
            const double B = cos_a * cos_k;
            double v0 = cos_b * A - sin_b * B;
            double v1 = sin_b * A + cos_b * B;
            double v2 = sin_t * sin_k + cos_t * sin_a * cos_k;

            const double n = std::sqrt(v0 * v0 + v1 * v1 + v2 * v2);
            const double inv = n > 1e-9 ? 1.0 / n : 0.0;
            out[3 * k + 0] = v0 * inv;
            out[3 * k + 1] = v1 * inv;
            out[3 * k + 2] = v2 * inv;
            k++;
        }
    }
}

}  // namespace l1
