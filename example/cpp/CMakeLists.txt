cmake_minimum_required(VERSION 3.16)
project(stand_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(stand_go2 stand_go2.cpp)
target_link_libraries(stand_go2  unitree_sdk2)

# rt/lowstate 를 눈으로 보기 위한 디버그 도구 (sim/실기 공용)
add_executable(lowstate_echo lowstate_echo.cpp)
target_link_libraries(lowstate_echo unitree_sdk2)

# wireless_remote 를 조이스틱 상태로 풀어 보는 도구 (go2_ctrl 의 FSM 과 같은 경로)
add_executable(wireless_echo wireless_echo.cpp)
target_link_libraries(wireless_echo unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)