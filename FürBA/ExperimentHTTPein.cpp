#include <iostream>
#include <stdlib.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "HTTPRequest.hpp"

struct franka::RobotState robot_state;

using namespace std;

array<double, 16ul> O_T_EE = robot_state.O_T_EE;

double O_T_EE0 = O_T_EE.data()[0];

std::string O_T_EE_0 = std::to_string(O_T_EE0);

	







int main(int argc, char** argv) {
  franka::Robot robot(argv[1]);
  
  try {
		

    //for(;;){


	
    
	
    franka::Robot robot(argv[1]);



    array<double, 16ul> O_T_EE = robot_state.O_T_EE;
   
  


    size_t count = 0;
    robot.read([&count, &O_T_EE](const franka::RobotState& robot_state) {
      

      using namespace std;

    array<double, 16ul> O_T_EE = robot_state.O_T_EE;






	double O_T_EE0 = O_T_EE.data()[0];





	std::string O_T_EE_0 = std::to_string(O_T_EE0);


    try
    {

         http::Request request{"http://localhost:2000"};
         const string OTEE0 = O_T_EE_0;
         const auto response = request.send("POST", OTEE0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });


        cout << std::string{response.body.begin(), response.body.end()} << '\n'; // print the result
    }
         catch (const std::exception& e)
    {   
           cerr << "Request failed, error: " << e.what() << '\n';
    }
	      


	  
	  	  
	  
	  
	  return count++ < 3000;
    });

    std::cout << "Done." << std::endl;
    } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
    }
    //}
	

  return 0;

}
