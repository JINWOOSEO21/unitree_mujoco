/**
 * rt/lowstate 를 구독해 관절 상태 / IMU / 발 접촉력을 찍는다.
 *
 * 왜 필요한가: 시뮬레이터(unitree_mujoco)와 제어기(go2_ctrl) 사이는 DDS 로만 이어져
 * 있어서, 화면을 못 볼 때 "브리지가 살아 있는가" 를 확인할 방법이 없다. 이 도구가
 * 로봇 쪽 눈 역할을 한다. sim2real 로 가서도 그대로 쓸 수 있다(실기의 lowstate 를 본다).
 *
 * 사용법:
 *   ./lowstate_echo            # domain 0, lo  (시뮬레이터 config 와 맞춤)
 *   ./lowstate_echo eth0       # domain 0, 지정 인터페이스 (실기)
 */
#include <unistd.h>

#include <cmath>
#include <iomanip>
#include <iostream>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#define TOPIC_LOWSTATE "rt/lowstate"

using namespace unitree::robot;
using LowState = unitree_go::msg::dds_::LowState_;

int main(int argc, const char** argv)
{
    // 시뮬레이터 config.yaml 의 domain_id / interface 와 반드시 같아야 붙는다.
    ChannelFactory::Instance()->Init(0, argc > 1 ? argv[1] : "lo");

    ChannelSubscriberPtr<LowState> sub;
    sub.reset(new ChannelSubscriber<LowState>(TOPIC_LOWSTATE));
    int count = 0;
    sub->InitChannel(
        [&count](const void* message) {
            const LowState* s = static_cast<const LowState*>(message);
            if (count++ % 200 != 0) return;  // lowstate 는 빠르게 온다. 솎아서 찍는다.

            const auto& q = s->imu_state().quaternion();
            const auto& g = s->imu_state().gyroscope();
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "[" << count << "] quat(w,x,y,z)=" << q[0] << " " << q[1] << " " << q[2]
                      << " " << q[3] << "  gyro=" << g[0] << " " << g[1] << " " << g[2] << "\n";

            std::cout << "      q[0..11] =";
            for (int i = 0; i < 12; ++i) std::cout << " " << s->motor_state()[i].q();
            std::cout << "\n      foot_force =";
            for (int i = 0; i < 4; ++i) std::cout << " " << s->foot_force()[i];
            std::cout << std::endl;
        },
        1);

    while (true) sleep(1);
    return 0;
}
