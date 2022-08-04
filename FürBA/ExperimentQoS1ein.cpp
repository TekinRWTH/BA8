#include <iostream>
#include <stdlib.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <cstdlib>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include "paho.mqtt.cpp/src/mqtt/async_client.h"

struct franka::RobotState robot_state;

using namespace std;

const string DFLT_SERVER_ADDRESS	{ "tcp://localhost:1883" };
const string CLIENT_ID				{ "paho_cpp_async_publish" };


const char* LWT_PAYLOAD = "Last will and testament.";
const string TOPIC { "hello" };

const string TOPIC1_0 { "O_T_EE_0" };
const string TOPIC1_1 { "O_T_EE_1" };
const string TOPIC1_2 { "O_T_EE_2" };
const string TOPIC1_3 { "O_T_EE_3" };
const string TOPIC1_4 { "O_T_EE_4" };
const string TOPIC1_5 { "O_T_EE_5" };
const string TOPIC1_6 { "O_T_EE_6" };
const string TOPIC1_7 { "O_T_EE_7" };
const string TOPIC1_8 { "O_T_EE_8" };
const string TOPIC1_9 { "O_T_EE_9" };
const string TOPIC1_10 { "O_T_EE_10" };
const string TOPIC1_11 { "O_T_EE_11" };
const string TOPIC1_12 { "O_T_EE_12" };
const string TOPIC1_13 { "O_T_EE_13" };
const string TOPIC1_14 { "O_T_EE_14" };
const string TOPIC1_15 { "O_T_EE_15" };

const string TOPIC2_0 { "O_T_EE_d_0" };
const string TOPIC2_1 { "O_T_EE_d_1" };
const string TOPIC2_2 { "O_T_EE_d_2" };
const string TOPIC2_3 { "O_T_EE_d_3" };
const string TOPIC2_4 { "O_T_EE_d_4" };
const string TOPIC2_5 { "O_T_EE_d_5" };
const string TOPIC2_6 { "O_T_EE_d_6" };
const string TOPIC2_7 { "O_T_EE_d_7" };
const string TOPIC2_8 { "O_T_EE_d_8" };
const string TOPIC2_9 { "O_T_EE_d_9" };
const string TOPIC2_10 { "O_T_EE_d_10" };
const string TOPIC2_11 { "O_T_EE_d_11" };
const string TOPIC2_12 { "O_T_EE_d_12" };
const string TOPIC2_13 { "O_T_EE_d_13" };
const string TOPIC2_14 { "O_T_EE_d_14" };
const string TOPIC2_15 { "O_T_EE_d_15" };

const string TOPIC3_0 { "F_T_EE_0" };
const string TOPIC3_1 { "F_T_EE_1" };
const string TOPIC3_2 { "F_T_EE_2" };
const string TOPIC3_3 { "F_T_EE_3" };
const string TOPIC3_4 { "F_T_EE_4" };
const string TOPIC3_5 { "F_T_EE_5" };
const string TOPIC3_6 { "F_T_EE_6" };
const string TOPIC3_7 { "F_T_EE_7" };
const string TOPIC3_8 { "F_T_EE_8" };
const string TOPIC3_9 { "F_T_EE_9" };
const string TOPIC3_10 { "F_T_EE_10" };
const string TOPIC3_11 { "F_T_EE_11" };
const string TOPIC3_12 { "F_T_EE_12" };
const string TOPIC3_13 { "F_T_EE_13" };
const string TOPIC3_14 { "F_T_EE_14" };
const string TOPIC3_15 { "F_T_EE_15" };


const string TOPIC4 { "m_ee" };                        

const string TOPIC5_0 { "EE_T_K_0" };
const string TOPIC5_1 { "EE_T_K_1" };
const string TOPIC5_2 { "EE_T_K_2" };
const string TOPIC5_3 { "EE_T_K_3" };
const string TOPIC5_4 { "EE_T_K_4" };
const string TOPIC5_5 { "EE_T_K_5" };
const string TOPIC5_6 { "EE_T_K_6" };
const string TOPIC5_7 { "EE_T_K_7" };
const string TOPIC5_8 { "EE_T_K_8" };
const string TOPIC5_9 { "EE_T_K_9" };
const string TOPIC5_10 { "EE_T_K_10" };
const string TOPIC5_11 { "EE_T_K_11" };
const string TOPIC5_12 { "EE_T_K_12" };
const string TOPIC5_13 { "EE_T_K_13" };
const string TOPIC5_14 { "EE_T_K_14" };
const string TOPIC5_15 { "EE_T_K_15" };

const string TOPIC6 { "m_load" };

const string TOPIC7 { "m_total" };

const string TOPIC8_0 { "elbow_0" };
const string TOPIC8_1 { "elbow_1" };

const string TOPIC9_0 { "F_x_Cee_0" };
const string TOPIC9_1 { "F_x_Cee_1" };
const string TOPIC9_2 { "F_x_Cee_2" };

const string TOPIC10_0 { "I_ee_0" };
const string TOPIC10_1 { "I_ee_1" };
const string TOPIC10_2 { "I_ee_2" };
const string TOPIC10_3 { "I_ee_3" };
const string TOPIC10_4 { "I_ee_4" };
const string TOPIC10_5 { "I_ee_5" };
const string TOPIC10_6 { "I_ee_6" };
const string TOPIC10_7 { "I_ee_7" };
const string TOPIC10_8 { "I_ee_8" };

const string TOPIC11_0 { "F_x_Ctotal_0" };
const string TOPIC11_1 { "F_x_Ctotal_1" };
const string TOPIC11_2 { "F_x_Ctotal_2" };

const string TOPIC12_0 { "F_x_Cload_0" };
const string TOPIC12_1 { "F_x_Cload_1" };
const string TOPIC12_2 { "F_x_Cload_2" };

const string TOPIC13_0 { "I_load_0" };
const string TOPIC13_1 { "I_load_1" };
const string TOPIC13_2 { "I_load_2" };
const string TOPIC13_3 { "I_load_3" };
const string TOPIC13_4 { "I_load_4" };
const string TOPIC13_5 { "I_load_5" };
const string TOPIC13_6 { "I_load_6" };
const string TOPIC13_7 { "I_load_7" };
const string TOPIC13_8 { "I_load_8" };

const string TOPIC14_0 { "I_total_0" };
const string TOPIC14_1 { "I_total_1" };
const string TOPIC14_2 { "I_total_2" };
const string TOPIC14_3 { "I_total_3" };
const string TOPIC14_4 { "I_total_4" };
const string TOPIC14_5 { "I_total_5" };
const string TOPIC14_6 { "I_total_6" };
const string TOPIC14_7 { "I_total_7" };
const string TOPIC14_8 { "I_total_8" };

const string TOPIC15_0 { "elbow_d_0" };
const string TOPIC15_1 { "elbow_d_1" };

const string TOPIC16_0 { "tau_J_0" };
const string TOPIC16_1 { "tau_J_1" };
const string TOPIC16_2 { "tau_J_2" };
const string TOPIC16_3 { "tau_J_3" };
const string TOPIC16_4 { "tau_J_4" };
const string TOPIC16_5 { "tau_J_5" };
const string TOPIC16_6 { "tau_J_6" };

const string TOPIC17_0 { "tau_J_d_0" };
const string TOPIC17_1 { "tau_J_d_1" };
const string TOPIC17_2 { "tau_J_d_2" };
const string TOPIC17_3 { "tau_J_d_3" };
const string TOPIC17_4 { "tau_J_d_4" };
const string TOPIC17_5 { "tau_J_d_5" };
const string TOPIC17_6 { "tau_J_d_6" };

const string TOPIC18_0 { "dtau_J_0" };
const string TOPIC18_1 { "dtau_J_1" };
const string TOPIC18_2 { "dtau_J_2" };
const string TOPIC18_3 { "dtau_J_3" };
const string TOPIC18_4 { "dtau_J_4" };
const string TOPIC18_5 { "dtau_J_5" };
const string TOPIC18_6 { "dtau_J_6" };

const string TOPIC19_0 { "q_0" };
const string TOPIC19_1 { "q_1" };
const string TOPIC19_2 { "q_2" };
const string TOPIC19_3 { "q_3" };
const string TOPIC19_4 { "q_4" };
const string TOPIC19_5 { "q_5" };
const string TOPIC19_6 { "q_6" };

const string TOPIC20_0 { "dq_0" };
const string TOPIC20_1 { "dq_1" };
const string TOPIC20_2 { "dq_2" };
const string TOPIC20_3 { "dq_3" };
const string TOPIC20_4 { "dq_4" };
const string TOPIC20_5 { "dq_5" };
const string TOPIC20_6 { "dq_6" };

const string TOPIC21_0 { "q_d_0" };
const string TOPIC21_1 { "q_d_1" };
const string TOPIC21_2 { "q_d_2" };
const string TOPIC21_3 { "q_d_3" };
const string TOPIC21_4 { "q_d_4" };
const string TOPIC21_5 { "q_d_5" };
const string TOPIC21_6 { "q_d_6" };

const string TOPIC22_0 { "dq_d_0" };
const string TOPIC22_1 { "dq_d_1" };
const string TOPIC22_2 { "dq_d_2" };
const string TOPIC22_3 { "dq_d_3" };
const string TOPIC22_4 { "dq_d_4" };
const string TOPIC22_5 { "dq_d_5" };
const string TOPIC22_6 { "dq_d_6" };

const string TOPIC23_0 { "joint_contact_0" };
const string TOPIC23_1 { "joint_contact_1" };
const string TOPIC23_2 { "joint_contact_2" };
const string TOPIC23_3 { "joint_contact_3" };
const string TOPIC23_4 { "joint_contact_4" };
const string TOPIC23_5 { "joint_contact_5" };
const string TOPIC23_6 { "joint_contact_6" };

const string TOPIC24_0 { "cartesian_contact_0" };
const string TOPIC24_1 { "cartesian_contact_1" };
const string TOPIC24_2 { "cartesian_contact_2" };
const string TOPIC24_3 { "cartesian_contact_3" };
const string TOPIC24_4 { "cartesian_contact_4" };
const string TOPIC24_5 { "cartesian_contact_5" };

const string TOPIC25_0 { "joint_collision_0" };
const string TOPIC25_1 { "joint_collision_1" };
const string TOPIC25_2 { "joint_collision_2" };
const string TOPIC25_3 { "joint_collision_3" };
const string TOPIC25_4 { "joint_collision_4" };
const string TOPIC25_5 { "joint_collision_5" };
const string TOPIC25_6 { "joint_collision_6" };

const string TOPIC26_0 { "cartesian_collision_0" };
const string TOPIC26_1 { "cartesian_collision_1" };
const string TOPIC26_2 { "cartesian_collision_2" };
const string TOPIC26_3 { "cartesian_collision_3" };
const string TOPIC26_4 { "cartesian_collision_4" };
const string TOPIC26_5 { "cartesian_collision_5" };

const string TOPIC27_0 { "tau_ext_hat_filtered_0" };
const string TOPIC27_1 { "tau_ext_hat_filtered_1" };
const string TOPIC27_2 { "tau_ext_hat_filtered_2" };
const string TOPIC27_3 { "tau_ext_hat_filtered_3" };
const string TOPIC27_4 { "tau_ext_hat_filtered_4" };
const string TOPIC27_5 { "tau_ext_hat_filtered_5" };
const string TOPIC27_6 { "tau_ext_hat_filtered_6" };

const string TOPIC28_0 { "O_F_ext_hat_K_0" };
const string TOPIC28_1 { "O_F_ext_hat_K_1" };
const string TOPIC28_2 { "O_F_ext_hat_K_2" };
const string TOPIC28_3 { "O_F_ext_hat_K_3" };
const string TOPIC28_4 { "O_F_ext_hat_K_4" };
const string TOPIC28_5 { "O_F_ext_hat_K_5" };

const string TOPIC29_0 { "K_F_ext_hat_K_0" };
const string TOPIC29_1 { "K_F_ext_hat_K_1" };
const string TOPIC29_2 { "K_F_ext_hat_K_2" };
const string TOPIC29_3 { "K_F_ext_hat_K_3" };
const string TOPIC29_4 { "K_F_ext_hat_K_4" };
const string TOPIC29_5 { "K_F_ext_hat_K_5" };

const string TOPIC30_0 { "theta_0" };
const string TOPIC30_1 { "theta_1" };
const string TOPIC30_2 { "theta_2" };
const string TOPIC30_3 { "theta_3" };
const string TOPIC30_4 { "theta_4" };
const string TOPIC30_5 { "theta_5" };
const string TOPIC30_6 { "theta_6" };

const string TOPIC31_0 { "dtheta_0" };
const string TOPIC31_1 { "dtheta_1" };
const string TOPIC31_2 { "dtheta_2" };
const string TOPIC31_3 { "dtheta_3" };
const string TOPIC31_4 { "dtheta_4" };
const string TOPIC31_5 { "dtheta_5" };
const string TOPIC31_6 { "dtheta_6" };

const string TOPIC32_0 { "F_T_NE_0"};
const string TOPIC32_1 { "F_T_NE_1"};
const string TOPIC32_2 { "F_T_NE_2"};
const string TOPIC32_3 { "F_T_NE_3"};
const string TOPIC32_4 { "F_T_NE_4"};
const string TOPIC32_5 { "F_T_NE_5"};
const string TOPIC32_6 { "F_T_NE_6"};
const string TOPIC32_7 { "F_T_NE_7"};
const string TOPIC32_8 { "F_T_NE_8"};
const string TOPIC32_9 { "F_T_NE_9"};
const string TOPIC32_10 { "F_T_NE_10"};
const string TOPIC32_11 { "F_T_NE_11"};
const string TOPIC32_12 { "F_T_NE_12"};
const string TOPIC32_13 { "F_T_NE_13"};
const string TOPIC32_14 { "F_T_NE_14"};
const string TOPIC32_15 { "F_T_NE_15"};

const string TOPIC33_0 { "NE_T_EE_0" };
const string TOPIC33_1 { "NE_T_EE_1" };
const string TOPIC33_2 { "NE_T_EE_2" };
const string TOPIC33_3 { "NE_T_EE_3" };
const string TOPIC33_4 { "NE_T_EE_4" };
const string TOPIC33_5 { "NE_T_EE_5" };
const string TOPIC33_6 { "NE_T_EE_6" };
const string TOPIC33_7 { "NE_T_EE_7" };
const string TOPIC33_8 { "NE_T_EE_8" };
const string TOPIC33_9 { "NE_T_EE_9" };
const string TOPIC33_10 { "NE_T_EE_10" };
const string TOPIC33_11 { "NE_T_EE_11" };
const string TOPIC33_12 { "NE_T_EE_12" };
const string TOPIC33_13 { "NE_T_EE_13" };
const string TOPIC33_14 { "NE_T_EE_14" };
const string TOPIC33_15 { "NE_T_EE_15" };

const string TOPIC34_0 { "elbow_d_0" };
const string TOPIC34_1 { "elbow_d_1" };

const string TOPIC35_0 { "elbow_c_0" };
const string TOPIC35_1 { "elbow_c_1" };

const string TOPIC36_0 { "delbow_c_0" };
const string TOPIC36_1 { "delbow_c_1" };

const string TOPIC37_0 { "ddelbow_c_0" };
const string TOPIC37_1 { "ddelbow_c_1" };

const string TOPIC38_0 { "ddq_d_0" };
const string TOPIC38_1 { "ddq_d_1" };
const string TOPIC38_2 { "ddq_d_2" };
const string TOPIC38_3 { "ddq_d_3" };
const string TOPIC38_4 { "ddq_d_4" };
const string TOPIC38_5 { "ddq_d_5" };
const string TOPIC38_6 { "ddq_d_6" };

const string TOPIC39_0 { "O_dP_EE_d_0" };
const string TOPIC39_1 { "O_dP_EE_d_1" };
const string TOPIC39_2 { "O_dP_EE_d_2" };
const string TOPIC39_3 { "O_dP_EE_d_3" };
const string TOPIC39_4 { "O_dP_EE_d_4" };
const string TOPIC39_5 { "O_dP_EE_d_5" };

const string TOPIC40_0 { "O_T_EE_c_0" };
const string TOPIC40_1 { "O_T_EE_c_1" };
const string TOPIC40_2 { "O_T_EE_c_2" };
const string TOPIC40_3 { "O_T_EE_c_3" };
const string TOPIC40_4 { "O_T_EE_c_4" };
const string TOPIC40_5 { "O_T_EE_c_5" };
const string TOPIC40_6 { "O_T_EE_c_6" };
const string TOPIC40_7 { "O_T_EE_c_7" };
const string TOPIC40_8 { "O_T_EE_c_8" };
const string TOPIC40_9 { "O_T_EE_c_9" };
const string TOPIC40_10 { "O_T_EE_c_10" };
const string TOPIC40_11 { "O_T_EE_c_11" };
const string TOPIC40_12 { "O_T_EE_c_12" };
const string TOPIC40_13 { "O_T_EE_c_13" };
const string TOPIC40_14 { "O_T_EE_c_14" };
const string TOPIC40_15 { "O_T_EE_c_15" };

const string TOPIC41_0 { "O_dP_EE_c_0" };
const string TOPIC41_1 { "O_dP_EE_c_1" };
const string TOPIC41_2 { "O_dP_EE_c_2" };
const string TOPIC41_3 { "O_dP_EE_c_3" };
const string TOPIC41_4 { "O_dP_EE_c_4" };
const string TOPIC41_5 { "O_dP_EE_c_5" };

const string TOPIC42_0 { "O_ddP_EE_c_0" };
const string TOPIC42_1 { "O_ddP_EE_c_1" };
const string TOPIC42_2 { "O_ddP_EE_c_2" };
const string TOPIC42_3 { "O_ddP_EE_c_3" };
const string TOPIC42_4 { "O_ddP_EE_c_4" };
const string TOPIC42_5 { "O_ddP_EE_c_5" };


array<double, 16ul> O_T_EE = robot_state.O_T_EE;




double O_T_EE0 = O_T_EE.data()[0];

std::string O_T_EE_0 = std::to_string(O_T_EE0);




const int  QOS = 1;



const auto TIMEOUT = std::chrono::seconds(10);


int main(int argc, char** argv) {
  string	address  = DFLT_SERVER_ADDRESS,
			clientID = CLIENT_ID;

	mqtt::async_client client(address, clientID);
	

	auto connOpts = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
		.finalize();
  
  franka::Robot robot(argv[1]);
  
  try {
		
		mqtt::token_ptr conntok = client.connect(connOpts);
		
		conntok->wait();
		
    //for(;;){


	
    try {
	#include "paho.mqtt.cpp/src/mqtt/async_client.h"
    
	string	address  = DFLT_SERVER_ADDRESS,
	clientID = CLIENT_ID;

	mqtt::async_client client(address, clientID);

    auto connOpts = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
		.finalize();

    mqtt::token_ptr conntok = client.connect(connOpts);
		
		conntok->wait();
  
    franka::Robot robot(argv[1]);



    array<double, 16ul> O_T_EE = robot_state.O_T_EE;




    size_t count = 0;
    robot.read([&count, &O_T_EE, &client](const franka::RobotState& robot_state) {
      
    


      using namespace std;

      const string DFLT_SERVER_ADDRESS	{ "tcp://localhost:1883" };
      const string CLIENT_ID			{ "paho_cpp_async_publish" };

	  const char* LWT_PAYLOAD = "Last will and testament.";
      const string TOPIC { "hello" };
	  
      const string TOPIC1_0 { "O_T_EE_0" };
      const string TOPIC1_1 { "O_T_EE_1" };
      const string TOPIC1_2 { "O_T_EE_2" };
	  const string TOPIC1_3 { "O_T_EE_3" };
	   const string TOPIC1_4 { "O_T_EE_4" };
		const string TOPIC1_5 { "O_T_EE_5" };
		const string TOPIC1_6 { "O_T_EE_6" };
		const string TOPIC1_7 { "O_T_EE_7" };
		const string TOPIC1_8 { "O_T_EE_8" };
		const string TOPIC1_9 { "O_T_EE_9" };
		const string TOPIC1_10 { "O_T_EE_10" };
		const string TOPIC1_11 { "O_T_EE_11" };
		const string TOPIC1_12 { "O_T_EE_12" };
		const string TOPIC1_13 { "O_T_EE_13" };
		const string TOPIC1_14 { "O_T_EE_14" };
		const string TOPIC1_15 { "O_T_EE_15" };

	const string TOPIC2_0 { "O_T_EE_d_0" };
	const string TOPIC2_1 { "O_T_EE_d_1" };
	const string TOPIC2_2 { "O_T_EE_d_2" };
	const string TOPIC2_3 { "O_T_EE_d_3" };
	const string TOPIC2_4 { "O_T_EE_d_4" };
	const string TOPIC2_5 { "O_T_EE_d_5" };
	const string TOPIC2_6 { "O_T_EE_d_6" };
	const string TOPIC2_7 { "O_T_EE_d_7" };
	const string TOPIC2_8 { "O_T_EE_d_8" };
	const string TOPIC2_9 { "O_T_EE_d_9" };
	const string TOPIC2_10 { "O_T_EE_d_10" };
	const string TOPIC2_11 { "O_T_EE_d_11" };
	const string TOPIC2_12 { "O_T_EE_d_12" };
	const string TOPIC2_13 { "O_T_EE_d_13" };
	const string TOPIC2_14 { "O_T_EE_d_14" };
	const string TOPIC2_15 { "O_T_EE_d_15" };

	const string TOPIC3_0 { "F_T_EE_0" };
	const string TOPIC3_1 { "F_T_EE_0" };
	const string TOPIC3_2 { "F_T_EE_0" };
	const string TOPIC3_3 { "F_T_EE_0" };
	const string TOPIC3_4 { "F_T_EE_0" };
	const string TOPIC3_5 { "F_T_EE_0" };
	const string TOPIC3_6 { "F_T_EE_0" };
	const string TOPIC3_7 { "F_T_EE_0" };
	const string TOPIC3_8 { "F_T_EE_0" };
	const string TOPIC3_9 { "F_T_EE_0" };
	const string TOPIC3_10 { "F_T_EE_0" };
	const string TOPIC3_11 { "F_T_EE_0" };
	const string TOPIC3_12 { "F_T_EE_0" };
	const string TOPIC3_13 { "F_T_EE_0" };
	const string TOPIC3_14 { "F_T_EE_0" };
	const string TOPIC3_15 { "F_T_EE_0" };


	const string TOPIC4 { "m_ee" };                        

	const string TOPIC5_0 { "EE_T_K_0" };
	const string TOPIC5_1 { "EE_T_K_1" };
	const string TOPIC5_2 { "EE_T_K_2" };
	const string TOPIC5_3 { "EE_T_K_3" };
	const string TOPIC5_4 { "EE_T_K_4" };
	const string TOPIC5_5 { "EE_T_K_5" };
	const string TOPIC5_6 { "EE_T_K_6" };
	const string TOPIC5_7 { "EE_T_K_7" };
	const string TOPIC5_8 { "EE_T_K_8" };
	const string TOPIC5_9 { "EE_T_K_9" };
	const string TOPIC5_10 { "EE_T_K_10" };
	const string TOPIC5_11 { "EE_T_K_11" };
	const string TOPIC5_12 { "EE_T_K_12" };
	const string TOPIC5_13 { "EE_T_K_13" };
	const string TOPIC5_14 { "EE_T_K_14" };
	const string TOPIC5_15 { "EE_T_K_15" };

	const string TOPIC6 { "m_load" };

	const string TOPIC7 { "m_total" };

	const string TOPIC8_0 { "elbow_0" };
	const string TOPIC8_1 { "elbow_1" };

	const string TOPIC9_0 { "F_x_Cee_0" };
	const string TOPIC9_1 { "F_x_Cee_1" };
	const string TOPIC9_2 { "F_x_Cee_2" };

	const string TOPIC10_0 { "I_ee_0" };
	const string TOPIC10_1 { "I_ee_1" };
	const string TOPIC10_2 { "I_ee_2" };
	const string TOPIC10_3 { "I_ee_3" };
	const string TOPIC10_4 { "I_ee_4" };
	const string TOPIC10_5 { "I_ee_5" };
	const string TOPIC10_6 { "I_ee_6" };
	const string TOPIC10_7 { "I_ee_7" };
	const string TOPIC10_8 { "I_ee_8" };

	const string TOPIC11_0 { "F_x_Ctotal_0" };
	const string TOPIC11_1 { "F_x_Ctotal_1" };
	const string TOPIC11_2 { "F_x_Ctotal_2" };

	const string TOPIC12_0 { "F_x_Cload_0" };
	const string TOPIC12_1 { "F_x_Cload_1" };
	const string TOPIC12_2 { "F_x_Cload_2" };

	const string TOPIC13_0 { "I_load_0" };
	const string TOPIC13_1 { "I_load_1" };
	const string TOPIC13_2 { "I_load_2" };
	const string TOPIC13_3 { "I_load_3" };
	const string TOPIC13_4 { "I_load_4" };
	const string TOPIC13_5 { "I_load_5" };
	const string TOPIC13_6 { "I_load_6" };
	const string TOPIC13_7 { "I_load_7" };
	const string TOPIC13_8 { "I_load_8" };

	const string TOPIC14_0 { "I_total_0" };
	const string TOPIC14_1 { "I_total_1" };
	const string TOPIC14_2 { "I_total_2" };
	const string TOPIC14_3 { "I_total_3" };
	const string TOPIC14_4 { "I_total_4" };
	const string TOPIC14_5 { "I_total_5" };
	const string TOPIC14_6 { "I_total_6" };
	const string TOPIC14_7 { "I_total_7" };
	const string TOPIC14_8 { "I_total_8" };

	const string TOPIC15_0 { "elbow_d_0" };
	const string TOPIC15_1 { "elbow_d_1" };

	const string TOPIC16_0 { "tau_J_0" };
	const string TOPIC16_1 { "tau_J_1" };
	const string TOPIC16_2 { "tau_J_2" };
	const string TOPIC16_3 { "tau_J_3" };
	const string TOPIC16_4 { "tau_J_4" };
	const string TOPIC16_5 { "tau_J_5" };
	const string TOPIC16_6 { "tau_J_6" };

	const string TOPIC17_0 { "tau_J_d_0" };
	const string TOPIC17_1 { "tau_J_d_1" };
	const string TOPIC17_2 { "tau_J_d_2" };
	const string TOPIC17_3 { "tau_J_d_3" };
	const string TOPIC17_4 { "tau_J_d_4" };
	const string TOPIC17_5 { "tau_J_d_5" };
	const string TOPIC17_6 { "tau_J_d_6" };

	const string TOPIC18_0 { "dtau_J_0" };
	const string TOPIC18_1 { "dtau_J_1" };
	const string TOPIC18_2 { "dtau_J_2" };
	const string TOPIC18_3 { "dtau_J_3" };
	const string TOPIC18_4 { "dtau_J_4" };
	const string TOPIC18_5 { "dtau_J_5" };
	const string TOPIC18_6 { "dtau_J_6" };

	const string TOPIC19_0 { "q_0" };
	const string TOPIC19_1 { "q_1" };
	const string TOPIC19_2 { "q_2" };
	const string TOPIC19_3 { "q_3" };
	const string TOPIC19_4 { "q_4" };
	const string TOPIC19_5 { "q_5" };
	const string TOPIC19_6 { "q_6" };

	const string TOPIC20_0 { "dq_0" };
	const string TOPIC20_1 { "dq_1" };
	const string TOPIC20_2 { "dq_2" };
	const string TOPIC20_3 { "dq_3" };
	const string TOPIC20_4 { "dq_4" };
	const string TOPIC20_5 { "dq_5" };
	const string TOPIC20_6 { "dq_6" };

	const string TOPIC21_0 { "q_d_0" };
	const string TOPIC21_1 { "q_d_1" };
	const string TOPIC21_2 { "q_d_2" };
	const string TOPIC21_3 { "q_d_3" };
	const string TOPIC21_4 { "q_d_4" };
	const string TOPIC21_5 { "q_d_5" };
	const string TOPIC21_6 { "q_d_6" };

	const string TOPIC22_0 { "dq_d_0" };
	const string TOPIC22_1 { "dq_d_1" };
	const string TOPIC22_2 { "dq_d_2" };
	const string TOPIC22_3 { "dq_d_3" };
	const string TOPIC22_4 { "dq_d_4" };
	const string TOPIC22_5 { "dq_d_5" };
	const string TOPIC22_6 { "dq_d_6" };

	const string TOPIC23_0 { "joint_contact_0" };
	const string TOPIC23_1 { "joint_contact_1" };
	const string TOPIC23_2 { "joint_contact_2" };
	const string TOPIC23_3 { "joint_contact_3" };
	const string TOPIC23_4 { "joint_contact_4" };
	const string TOPIC23_5 { "joint_contact_5" };
	const string TOPIC23_6 { "joint_contact_6" };

	const string TOPIC24_0 { "cartesian_contact_0" };
	const string TOPIC24_1 { "cartesian_contact_1" };
	const string TOPIC24_2 { "cartesian_contact_2" };
	const string TOPIC24_3 { "cartesian_contact_3" };
	const string TOPIC24_4 { "cartesian_contact_4" };
	const string TOPIC24_5 { "cartesian_contact_5" };

	const string TOPIC25_0 { "joint_collision_0" };
	const string TOPIC25_1 { "joint_collision_1" };
	const string TOPIC25_2 { "joint_collision_2" };
	const string TOPIC25_3 { "joint_collision_3" };
	const string TOPIC25_4 { "joint_collision_4" };
	const string TOPIC25_5 { "joint_collision_5" };
	const string TOPIC25_6 { "joint_collision_6" };

	const string TOPIC26_0 { "cartesian_collision_0" };
	const string TOPIC26_1 { "cartesian_collision_1" };
	const string TOPIC26_2 { "cartesian_collision_2" };
	const string TOPIC26_3 { "cartesian_collision_3" };
	const string TOPIC26_4 { "cartesian_collision_4" };
	const string TOPIC26_5 { "cartesian_collision_5" };

	const string TOPIC27_0 { "tau_ext_hat_filtered_0" };
	const string TOPIC27_1 { "tau_ext_hat_filtered_1" };
	const string TOPIC27_2 { "tau_ext_hat_filtered_2" };
	const string TOPIC27_3 { "tau_ext_hat_filtered_3" };
	const string TOPIC27_4 { "tau_ext_hat_filtered_4" };
	const string TOPIC27_5 { "tau_ext_hat_filtered_5" };
	const string TOPIC27_6 { "tau_ext_hat_filtered_6" };

	const string TOPIC28_0 { "O_F_ext_hat_K_0" };
	const string TOPIC28_1 { "O_F_ext_hat_K_1" };
	const string TOPIC28_2 { "O_F_ext_hat_K_2" };
	const string TOPIC28_3 { "O_F_ext_hat_K_3" };
	const string TOPIC28_4 { "O_F_ext_hat_K_4" };
	const string TOPIC28_5 { "O_F_ext_hat_K_5" };

	const string TOPIC29_0 { "K_F_ext_hat_K_0" };
	const string TOPIC29_1 { "K_F_ext_hat_K_1" };
	const string TOPIC29_2 { "K_F_ext_hat_K_2" };
	const string TOPIC29_3 { "K_F_ext_hat_K_3" };
	const string TOPIC29_4 { "K_F_ext_hat_K_4" };
	const string TOPIC29_5 { "K_F_ext_hat_K_5" };	

	const string TOPIC30_0 { "theta_0" };
	const string TOPIC30_1 { "theta_1" };
	const string TOPIC30_2 { "theta_2" };
	const string TOPIC30_3 { "theta_3" };
	const string TOPIC30_4 { "theta_4" };
	const string TOPIC30_5 { "theta_5" };
	const string TOPIC30_6 { "theta_6" };

	const string TOPIC31_0 { "dtheta_0" };
	const string TOPIC31_1 { "dtheta_1" };
	const string TOPIC31_2 { "dtheta_2" };
	const string TOPIC31_3 { "dtheta_3" };
	const string TOPIC31_4 { "dtheta_4" };
	const string TOPIC31_5 { "dtheta_5" };
	const string TOPIC31_6 { "dtheta_6" };

	const string TOPIC32_0 { "F_T_NE_0"};
	const string TOPIC32_1 { "F_T_NE_1"};
	const string TOPIC32_2 { "F_T_NE_2"};
	const string TOPIC32_3 { "F_T_NE_3"};
	const string TOPIC32_4 { "F_T_NE_4"};
	const string TOPIC32_5 { "F_T_NE_5"};
	const string TOPIC32_6 { "F_T_NE_6"};
	const string TOPIC32_7 { "F_T_NE_7"};
	const string TOPIC32_8 { "F_T_NE_8"};
	const string TOPIC32_9 { "F_T_NE_9"};
	const string TOPIC32_10 { "F_T_NE_10"};
	const string TOPIC32_11 { "F_T_NE_11"};
	const string TOPIC32_12 { "F_T_NE_12"};
	const string TOPIC32_13 { "F_T_NE_13"};
	const string TOPIC32_14 { "F_T_NE_14"};
	const string TOPIC32_15 { "F_T_NE_15"};

	const string TOPIC33_0 { "NE_T_EE_0" };
	const string TOPIC33_1 { "NE_T_EE_1" };
	const string TOPIC33_2 { "NE_T_EE_2" };
	const string TOPIC33_3 { "NE_T_EE_3" };
	const string TOPIC33_4 { "NE_T_EE_4" };
	const string TOPIC33_5 { "NE_T_EE_5" };
	const string TOPIC33_6 { "NE_T_EE_6" };
	const string TOPIC33_7 { "NE_T_EE_7" };
	const string TOPIC33_8 { "NE_T_EE_8" };
	const string TOPIC33_9 { "NE_T_EE_9" };
	const string TOPIC33_10 { "NE_T_EE_10" };
	const string TOPIC33_11 { "NE_T_EE_11" };
	const string TOPIC33_12 { "NE_T_EE_12" };
	const string TOPIC33_13 { "NE_T_EE_13" };
	const string TOPIC33_14 { "NE_T_EE_14" };
	const string TOPIC33_15 { "NE_T_EE_15" };

	const string TOPIC34_0 { "elbow_d_0" };
	const string TOPIC34_1 { "elbow_d_1" };

	const string TOPIC35_0 { "elbow_c_0" };
	const string TOPIC35_1 { "elbow_c_1" };

	const string TOPIC36_0 { "delbow_c_0" };
	const string TOPIC36_1 { "delbow_c_1" };

	const string TOPIC37_0 { "ddelbow_c_0" };
	const string TOPIC37_1 { "ddelbow_c_1" };

	const string TOPIC38_0 { "ddq_d_0" };
	const string TOPIC38_1 { "ddq_d_1" };
	const string TOPIC38_2 { "ddq_d_2" };
	const string TOPIC38_3 { "ddq_d_3" };
	const string TOPIC38_4 { "ddq_d_4" };
	const string TOPIC38_5 { "ddq_d_5" };
	const string TOPIC38_6 { "ddq_d_6" };

	const string TOPIC39_0 { "O_dP_EE_d_0" };
	const string TOPIC39_1 { "O_dP_EE_d_1" };
	const string TOPIC39_2 { "O_dP_EE_d_2" };
	const string TOPIC39_3 { "O_dP_EE_d_3" };
	const string TOPIC39_4 { "O_dP_EE_d_4" };
	const string TOPIC39_5 { "O_dP_EE_d_5" };

	const string TOPIC40_0 { "O_T_EE_c_0" };
	const string TOPIC40_1 { "O_T_EE_c_1" };
	const string TOPIC40_2 { "O_T_EE_c_2" };
	const string TOPIC40_3 { "O_T_EE_c_3" };
	const string TOPIC40_4 { "O_T_EE_c_4" };
	const string TOPIC40_5 { "O_T_EE_c_5" };
	const string TOPIC40_6 { "O_T_EE_c_6" };
	const string TOPIC40_7 { "O_T_EE_c_7" };
	const string TOPIC40_8 { "O_T_EE_c_8" };
	const string TOPIC40_9 { "O_T_EE_c_9" };
	const string TOPIC40_10 { "O_T_EE_c_10" };
	const string TOPIC40_11 { "O_T_EE_c_11" };
	const string TOPIC40_12 { "O_T_EE_c_12" };
	const string TOPIC40_13 { "O_T_EE_c_13" };
	const string TOPIC40_14 { "O_T_EE_c_14" };
	const string TOPIC40_15 { "O_T_EE_c_15" };

	const string TOPIC41_0 { "O_dP_EE_c_0" };
	const string TOPIC41_1 { "O_dP_EE_c_1" };
	const string TOPIC41_2 { "O_dP_EE_c_2" };
	const string TOPIC41_3 { "O_dP_EE_c_3" };
	const string TOPIC41_4 { "O_dP_EE_c_4" };
	const string TOPIC41_5 { "O_dP_EE_c_5" };

	const string TOPIC42_0 { "O_ddP_EE_c_0" };
	const string TOPIC42_1 { "O_ddP_EE_c_1" };
	const string TOPIC42_2 { "O_ddP_EE_c_2" };
	const string TOPIC42_3 { "O_ddP_EE_c_3" };
	const string TOPIC42_4 { "O_ddP_EE_c_4" };
	const string TOPIC42_5 { "O_ddP_EE_c_5" };







    array<double, 16ul> O_T_EE = robot_state.O_T_EE;
 
double O_T_EE0 = O_T_EE.data()[0];





    std::string O_T_EE_0 = std::to_string(O_T_EE0);



	const int  QOS = 1;

	const auto TIMEOUT = std::chrono::seconds(10);










    mqtt::message_ptr OTEE0 = mqtt::make_message(TOPIC1_0, O_T_EE_0);
		OTEE0->set_qos(QOS);
		client.publish(OTEE0)->wait_for(TIMEOUT);
	
	  
	  
	  
	  
	  
	  return count++ < 3000;
    });

    std::cout << "Done." << std::endl;
    } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
    }
    //}
	}
	catch (const mqtt::exception& exc) {
		cerr << exc.what() << endl;
		return 1;
	}

  return 0;

}
