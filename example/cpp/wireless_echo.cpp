/**
 * rt/lowstate 의 wireless_remote(40바이트)를 조이스틱 상태로 풀어 찍는다.
 *
 * go2_ctrl 의 FSM 이 실제로 읽는 경로와 **똑같은 경로**를 본다:
 *   lowstate.wireless_remote -> memcpy -> UnitreeJoystick::extract
 * 그래서 이 도구에 버튼이 뜨면 go2_ctrl 도 반드시 같은 값을 본다.
 * KeyboardJoystick 이 제대로 붙었는지 확인하는 용도.
 *
 * 사용법: ./wireless_echo [interface]   (기본 lo, domain 0)
 */
#include <unistd.h>

#include <cstring>
#include <iostream>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

#define TOPIC_LOWSTATE "rt/lowstate"

using namespace unitree::robot;
using LowState = unitree_go::msg::dds_::LowState_;

int main(int argc, const char** argv)
{
    ChannelFactory::Instance()->Init(0, argc > 1 ? argv[1] : "lo");

    unitree::common::UnitreeJoystick joy;
    std::string last;

    ChannelSubscriberPtr<LowState> sub;
    sub.reset(new ChannelSubscriber<LowState>(TOPIC_LOWSTATE));
    sub->InitChannel(
        [&joy, &last](const void* message) {
            const LowState* s = static_cast<const LowState*>(message);

            unitree::common::REMOTE_DATA_RX key;
            std::memcpy(&key, &s->wireless_remote()[0], 40);
            joy.extract(key);
            joy.update();

            std::string cur;
            if (joy.LT.pressed) cur += "LT ";
            if (joy.RT.pressed) cur += "RT ";
            if (joy.A.pressed) cur += "A ";
            if (joy.B.pressed) cur += "B ";
            if (joy.X.pressed) cur += "X ";
            if (joy.Y.pressed) cur += "Y ";
            if (joy.start.pressed) cur += "start ";
            if (joy.back.pressed) cur += "back ";
            char buf[96];
            // Axis 의 값은 operator()() 로 읽는다 (data_ 는 private).
            snprintf(buf, sizeof(buf), "| lx=%+.2f ly=%+.2f rx=%+.2f", joy.lx(), joy.ly(), joy.rx());
            cur += buf;

            if (cur != last) {  // 바뀔 때만 찍는다
                std::cout << "buttons: " << cur << std::endl;
                last = cur;
            }
        },
        1);

    while (true) sleep(1);
    return 0;
}
