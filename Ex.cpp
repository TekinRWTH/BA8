#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

int main(int argc, char** argv) {
franka::Robot robot(argv[1]);
RobotState robot;
    
    //size_t count = 0;
    //robot.read([&count](const franka::RobotState& robot_state) {
      // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
      // should not be done in a control loop.
      std::cout << robot.O_T_EE.data() << std::endl;
      //return count++ < 100;
    //});
    std::cout << "Done." << std::endl;
  
  return 0;
}