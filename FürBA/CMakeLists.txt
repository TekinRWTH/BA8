cmake_minimum_required(VERSION 3.5)
project(BA)
find_package(Franka REQUIRED)
find_package(PahoMqttCpp REQUIRED)

set(CMAKE_CXX_STANDARD 11)
add_executable(QoS0voll ExperimentQoS0voll.cpp)
target_link_libraries(QoS0voll Franka::Franka paho-mqttpp3 paho-mqtt3as)


