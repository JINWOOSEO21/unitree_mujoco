/**
 * rt/utlidar/cloud (PointCloud2) 를 구독해 점군 통계를 찍는다.
 *
 * L1 시뮬레이션 게이트 검증용:
 *  - 프레임당 점 개수가 2160(= 21,600 pts/s x 0.1s)에 가까운가
 *  - 평지에서 지면 z 가 센서 높이의 음수값 근처인가
 *  - 방위각(yaw) 분포가 360° 를 덮는가
 *
 * 사용법: ./lidar_echo [interface]   (기본 lo, domain 0)
 */
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#define TOPIC_CLOUD "rt/utlidar/cloud"

using namespace unitree::robot;
using Cloud = sensor_msgs::msg::dds_::PointCloud2_;

int main(int argc, const char** argv)
{
    ChannelFactory::Instance()->Init(0, argc > 1 ? argv[1] : "lo");

    ChannelSubscriberPtr<Cloud> sub;
    sub.reset(new ChannelSubscriber<Cloud>(TOPIC_CLOUD));
    int frame = 0;
    sub->InitChannel(
        [&frame](const void* message) {
            const Cloud* c = static_cast<const Cloud*>(message);
            const int n = static_cast<int>(c->width());
            std::vector<float> p(3 * n);
            if (n > 0) std::memcpy(p.data(), c->data().data(), 12 * n);

            // 통계
            float zmin = 1e9f, zmax = -1e9f, rmin = 1e9f, rmax = -1e9f;
            int az_bins[12] = {0};
            for (int i = 0; i < n; i++) {
                const float x = p[3 * i], y = p[3 * i + 1], z = p[3 * i + 2];
                zmin = std::min(zmin, z);
                zmax = std::max(zmax, z);
                const float r = std::sqrt(x * x + y * y + z * z);
                rmin = std::min(rmin, r);
                rmax = std::max(rmax, r);
                float az = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);
                if (az < 0) az += 360.0f;
                az_bins[std::min(11, static_cast<int>(az / 30.0f))]++;
            }
            int covered = 0;
            for (int b = 0; b < 12; b++) if (az_bins[b] > 0) covered++;

            std::cout << std::fixed << std::setprecision(3);
            std::cout << "[frame " << ++frame << "] 점 " << n << "개"
                      << "  z[" << zmin << ", " << zmax << "]"
                      << "  r[" << rmin << ", " << rmax << "]"
                      << "  방위 30도구간 " << covered << "/12 커버" << std::endl;
        },
        1);

    while (true) sleep(1);
    return 0;
}
