cmake_minimum_required(VERSION 3.5)
project(BA)
find_package(Franka REQUIRED)

set(CMAKE_CXX_STANDARD 11)
add_executable(klaro Publik.cpp)
target_link_libraries(klaro Franka::Franka)


cmake_minimum_required(VERSION 3.5)
project(BA)
find_package(Franka REQUIRED)
find_package(PahoMqttCpp REQUIRED)

set(CMAKE_CXX_STANDARD 11)
add_executable(BA Experiment.cpp)
target_link_libraries(BA Franka::Franka paho-mqttpp3 paho-mqtt3as)
