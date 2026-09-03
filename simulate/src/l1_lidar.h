#pragma once

/**
 * Unitree L1 4D LiDAR 시뮬레이션 — MuJoCo 레이캐스팅으로 점군을 만들어
 * rt/utlidar/cloud (sensor_msgs::PointCloud2) 로 발행한다.
 *
 * 왜 필요한가
 * ----------
 * 파쿠르 정책의 관측 scan[53:185] 은 elevation map 을 샘플한 값이고, 그 elevation
 * map 은 L1 점군에서 만들어진다. 시뮬레이터에 LiDAR 가 없으면 정책 입력의 132칸을
 * 채울 수 없다. 실기와 **같은 토픽·같은 메시지 타입**으로 내보내야 소비자(EM
 * 파이프라인)가 sim/실기를 구분하지 않는다.
 *
 * 스캔 모델 (Isaaclab_Parkour/parkour_isaaclab/sensors/l1_scan_ray_caster.py 이식)
 * -----------------------------------------------------------------------------
 * L1 은 이중 모터다. 반사경(pitch, 고속 180Hz)과 코어(yaw, 저속 11Hz)가 동시에 돌며
 * 한 점씩 찍는다. unilidar_sdk 의 MavLink 파싱 문서가 공개한 포인트 기하:
 *
 *     A = (-cosθ·sinψ + sinθ·sinα·cosψ)
 *     B =  cosα·cosψ
 *     x = cosβ·A - sinβ·B
 *     y = sinβ·A + cosβ·B
 *     z =  sinθ·sinψ + cosθ·sinα·cosψ
 *
 * α = pitch 각, β = yaw 각, θ/ψ = 기기 내부 광학 상수(고정 캘리브레이션 틸트).
 *
 * 샘플 클럭 43,200/s 에서 α(t) = ((360·180·t + φ) mod 360) - 180 이고,
 * **α ∈ (-90°, 90°) 인 반주기만 유효**하다(나머지 반주기는 하우징이 가림).
 * 회전당 240 샘플 중 120 점이 유효 → 21,600 pts/s (공식 스펙).
 * 한 프레임 0.1s = 18 회전 × 120 = 2,160 점.
 *
 * 방향에 랜덤성은 없다(결정론). 초기 모터 위상만 랜덤이다.
 *
 * 좌표계
 * ------
 * 점은 **센서 프레임**으로 낸다. 실기의 rt/utlidar/cloud 도 그렇고, EM 파이프라인이
 * 센서 프레임 점군 + 별도 pose 를 받도록 만들어져 있다.
 *
 * self-hit 은 걸러내지 않는다
 * --------------------------
 * 로봇 자신에 맞은 점도 그대로 내보낸다. 실기 LiDAR 가 그렇게 동작하고, 학습 때도
 * 로봇 collision mesh 를 캐스트 대상에 포함한 뒤 **소비자 쪽**에서 캡슐 필터를
 * 걸었기 때문이다. 여기서 미리 지우면 학습 분포와 달라진다.
 */

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include "l1_scan_kinematics.h"  // 스캔 운동학(순수 수학, 단독 테스트 대상)

namespace l1
{

struct Config
{
    // 스캔 파라미터 — 실기 aux 패킷(sys/com_rotation_period, theta/ksi_angle)이
    // 정밀값을 주므로 실기 확보 시 교체할 것.
    int    rays_per_frame     = 2160;      // 0.1s 분량
    double samples_per_second = 43200.0;   // 원시 샘플 클럭
    double pitch_hz           = 180.0;     // 반사경(고속)
    double yaw_period_s       = 1.0 / 11.0;// 코어(저속) 주기
    double theta_deg          = 45.0;      // 고정 캘리브레이션 틸트
    double ksi_deg            = 45.0;
    double max_range          = 10.0;      // [m]
    double min_range          = 0.05;      // 하우징 내부 반사 제거

    // 마운트 (deploy.yaml / GO2_LIDAR_CFG 와 같은 값)
    // base 링크 기준 위치와 회전. rot 은 X축 180° = 돔 축(+Z)을 아래로.
    double mount_pos[3]  = {0.28, 0.0, 0.10};
    double mount_quat[4] = {0.0, 1.0, 0.0, 0.0};  // (w, x, y, z)

    std::string topic    = "rt/utlidar/cloud";
    std::string frame_id = "utlidar_lidar";
    double      publish_hz = 10.0;         // 프레임(0.1s) 주기
};

class Lidar
{
public:
    Lidar(const mjModel* m, const Config& cfg = Config())
    : m_(m), cfg_(cfg)
    {
        // 몸통 body 이름은 MJCF 출처마다 다르다:
        //   unitree 공식 go2.xml -> "base_link",  mujoco_menagerie -> "base",
        //   구형 unitree 모델    -> "trunk"
        const char* trunk_names[3] = {"base_link", "base", "trunk"};
        for (const char* n : trunk_names) {
            base_body_ = mj_name2id(m_, mjOBJ_BODY, n);
            if (base_body_ >= 0) break;
        }

        // 스캔 운동학 파라미터를 순수 수학부로 넘긴다.
        scan_.rays_per_frame     = cfg_.rays_per_frame;
        scan_.samples_per_second = cfg_.samples_per_second;
        scan_.pitch_hz           = cfg_.pitch_hz;
        scan_.yaw_period_s       = cfg_.yaw_period_s;
        scan_.theta_deg          = cfg_.theta_deg;
        scan_.ksi_deg            = cfg_.ksi_deg;

        // 초기 모터 위상은 랜덤(고정). 이후는 결정론적.
        std::mt19937 rng(12345);
        std::uniform_real_distribution<double> u(0.0, 360.0);
        phase_pitch_deg_ = u(rng);
        phase_yaw_deg_   = u(rng);

        dirs_.resize(cfg_.rays_per_frame * 3);
        points_.resize(cfg_.rays_per_frame * 3);
        publisher_.reset(new unitree::robot::ChannelPublisher<
                         sensor_msgs::msg::dds_::PointCloud2_>(cfg_.topic));
        publisher_->InitChannel();
        init_msg_layout();
    }

    bool ok() const { return base_body_ >= 0; }

    /** 시뮬레이션 시각 t_end 기준으로 직전 0.1s 프레임을 캐스트해 발행한다. */
    void update(const mjData* d)
    {
        if (!ok()) return;
        const double now = d->time;
        if (now - last_publish_ < 1.0 / cfg_.publish_hz) return;
        last_publish_ = now;

        // --- 센서 pose (world) ---
        // base 의 world pose 에 마운트 offset 을 얹는다.
        double base_pos[3], base_mat[9];
        std::memcpy(base_pos, d->xpos + 3 * base_body_, sizeof(base_pos));
        std::memcpy(base_mat, d->xmat + 9 * base_body_, sizeof(base_mat));

        // 브리지 스레드는 첫 물리 스텝보다 먼저 돌 수 있고, 그때 d->xmat 은 0 이다.
        // 그대로 쓰면 회전행렬이 영행렬이라 방향 벡터가 길이 0 이 되고
        // mj_ray 가 "vector length is too small" 로 죽는다. 자세가 설 때까지 건너뛴다.
        if (mju_norm3(base_mat) < 0.5) {
            last_publish_ = -1e9;  // 다음 호출에서 곧바로 다시 시도
            return;
        }

        double mount_mat[9];
        mju_quat2Mat(mount_mat, cfg_.mount_quat);

        double sensor_pos[3];
        mju_mulMatVec3(sensor_pos, base_mat, cfg_.mount_pos);
        mju_addTo3(sensor_pos, base_pos);

        double sensor_mat[9];  // world <- sensor
        mju_mulMatMat(sensor_mat, base_mat, mount_mat, 3, 3, 3);

        // --- 레이 방향 생성 (센서 프레임) ---
        compute_directions(now);

        // --- 캐스트 ---
        int n_valid = 0;
        int geomid = -1;
        for (int k = 0; k < cfg_.rays_per_frame; k++) {
            double dir_s[3] = {dirs_[3 * k], dirs_[3 * k + 1], dirs_[3 * k + 2]};
            double dir_w[3];
            mju_mulMatVec3(dir_w, sensor_mat, dir_s);
            if (mju_norm3(dir_w) < 1e-6) continue;  // 방어 (mj_ray 는 영벡터에 죽는다)

            // flg_static=1: 지형(static geom)도 맞힌다.
            // bodyexclude=-1: 로봇 자신도 포함 — 실기와 같게 self-hit 을 남긴다.
            const mjtNum dist = mj_ray(m_, d, sensor_pos, dir_w, nullptr, 1, -1, &geomid);
            if (dist < cfg_.min_range || dist > cfg_.max_range) continue;

            // 센서 프레임 좌표 = 방향 × 거리 (원점이 곧 센서)
            points_[3 * n_valid + 0] = static_cast<float>(dir_s[0] * dist);
            points_[3 * n_valid + 1] = static_cast<float>(dir_s[1] * dist);
            points_[3 * n_valid + 2] = static_cast<float>(dir_s[2] * dist);
            n_valid++;
        }

        publish(now, n_valid);
    }

    int last_point_count() const { return last_n_; }

private:
    void compute_directions(double t_end)
    {
        // 실제 계산은 l1_scan_kinematics.h 에 있다 — DDS/MuJoCo 없이 단독 테스트하기 위해
        // 분리했고, 파이썬 원본과의 대조도 그쪽에서 한다.
        l1::compute_frame_directions(scan_, t_end, phase_pitch_deg_, phase_yaw_deg_, dirs_.data());
    }

    void init_msg_layout()
    {
        // x, y, z 각각 float32. PointCloud2 표준 레이아웃.
        const char* names[3] = {"x", "y", "z"};
        std::vector<sensor_msgs::msg::dds_::PointField_> fields(3);
        for (int i = 0; i < 3; i++) {
            fields[i].name(names[i]);
            fields[i].offset(4 * i);
            fields[i].datatype(7);  // FLOAT32
            fields[i].count(1);
        }
        msg_.fields(fields);
        msg_.height(1);            // 정렬되지 않은 점군
        msg_.point_step(12);
        msg_.is_bigendian(false);
        msg_.is_dense(true);
        msg_.header().frame_id(cfg_.frame_id);
    }

    void publish(double t, int n)
    {
        last_n_ = n;
        msg_.width(n);
        msg_.row_step(12 * n);
        auto& data = msg_.data();
        data.resize(12 * n);
        if (n > 0) std::memcpy(data.data(), points_.data(), 12 * n);

        msg_.header().stamp().sec(static_cast<int32_t>(t));
        msg_.header().stamp().nanosec(static_cast<uint32_t>((t - std::floor(t)) * 1e9));
        publisher_->Write(msg_);
    }

    const mjModel* m_;
    Config cfg_;
    int base_body_ = -1;

    ScanParams scan_;
    double phase_pitch_deg_ = 0, phase_yaw_deg_ = 0;

    std::vector<double> dirs_;
    std::vector<float> points_;
    double last_publish_ = -1e9;
    int last_n_ = 0;

    sensor_msgs::msg::dds_::PointCloud2_ msg_;
    unitree::robot::ChannelPublisherPtr<sensor_msgs::msg::dds_::PointCloud2_> publisher_;
};

}  // namespace l1
