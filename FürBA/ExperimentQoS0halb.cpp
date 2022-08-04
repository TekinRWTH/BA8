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
array<double, 16ul> O_T_EE_d = robot_state.O_T_EE_d;
array< double, 16 > F_T_EE = robot_state.F_T_EE; 
array< double, 16 > F_T_NE = robot_state.F_T_NE;
array< double, 16 > NE_T_EE = robot_state.NE_T_EE;
array< double, 16 > EE_T_K = robot_state.EE_T_K;
double m_ee = robot_state.m_ee;
array< double, 9 > I_ee = robot_state.I_ee;
array< double, 3 > F_x_Cee = robot_state.F_x_Cee;
double m_load = robot_state.m_load;
array< double, 9 > I_load = robot_state.I_load;
array< double, 3 > F_x_Cload = robot_state.F_x_Cload;
double m_total = robot_state.m_total;
array< double, 9 > I_total = robot_state.I_total;
array< double, 3 > F_x_Ctotal = robot_state.F_x_Ctotal;
array< double, 2 > elbow = robot_state.elbow;
array< double, 2 > elbow_d = robot_state.elbow_d;
array< double, 2 > elbow_c = robot_state.elbow_c;
array< double, 2 > delbow_c = robot_state.delbow_c;
array< double, 2 > ddelbow_c = robot_state.ddelbow_c;
array< double, 7 > tau_J_d = robot_state.tau_J_d;



double O_T_EE0 = O_T_EE.data()[0];
double O_T_EE1 = O_T_EE.data()[1];
double O_T_EE2 = O_T_EE.data()[2];
double O_T_EE3 = O_T_EE.data()[3];
double O_T_EE4 = O_T_EE.data()[4];
double O_T_EE5 = O_T_EE.data()[5];
double O_T_EE6 = O_T_EE.data()[6];
double O_T_EE7 = O_T_EE.data()[7];
double O_T_EE8 = O_T_EE.data()[8];
double O_T_EE9 = O_T_EE.data()[9];
double O_T_EE10 = O_T_EE.data()[10];
double O_T_EE11 = O_T_EE.data()[11];
double O_T_EE12 = O_T_EE.data()[12];
double O_T_EE13 = O_T_EE.data()[13];
double O_T_EE14 = O_T_EE.data()[14];
double O_T_EE15 = O_T_EE.data()[15];

double O_T_EE_d0 = O_T_EE_d.data()[0];
double O_T_EE_d1 = O_T_EE_d.data()[1];
double O_T_EE_d2 = O_T_EE_d.data()[2];
double O_T_EE_d3 = O_T_EE_d.data()[3];
double O_T_EE_d4 = O_T_EE_d.data()[4];
double O_T_EE_d5 = O_T_EE_d.data()[5];
double O_T_EE_d6 = O_T_EE_d.data()[6];
double O_T_EE_d7 = O_T_EE_d.data()[7];
double O_T_EE_d8 = O_T_EE_d.data()[8];
double O_T_EE_d9 = O_T_EE_d.data()[9];
double O_T_EE_d10 = O_T_EE_d.data()[10];
double O_T_EE_d11 = O_T_EE_d.data()[11];
double O_T_EE_d12 = O_T_EE_d.data()[12];
double O_T_EE_d13 = O_T_EE_d.data()[13];
double O_T_EE_d14 = O_T_EE_d.data()[14];
double O_T_EE_d15 = O_T_EE_d.data()[15];

double F_T_EE0 = F_T_EE.data()[0];
double F_T_EE1 = F_T_EE.data()[1];
double F_T_EE2 = F_T_EE.data()[2];
double F_T_EE3 = F_T_EE.data()[3];
double F_T_EE4 = F_T_EE.data()[4];
double F_T_EE5 = F_T_EE.data()[5];
double F_T_EE6 = F_T_EE.data()[6];
double F_T_EE7 = F_T_EE.data()[7];
double F_T_EE8 = F_T_EE.data()[8];
double F_T_EE9 = F_T_EE.data()[9];
double F_T_EE10 = F_T_EE.data()[10];
double F_T_EE11 = F_T_EE.data()[11];
double F_T_EE12 = F_T_EE.data()[12];
double F_T_EE13 = F_T_EE.data()[13];
double F_T_EE14 = F_T_EE.data()[14];
double F_T_EE15 = F_T_EE.data()[15];

double F_T_NE0 = F_T_NE.data()[0];
double F_T_NE1 = F_T_NE.data()[1];
double F_T_NE2 = F_T_NE.data()[2];
double F_T_NE3 = F_T_NE.data()[3];
double F_T_NE4 = F_T_NE.data()[4];
double F_T_NE5 = F_T_NE.data()[5];
double F_T_NE6 = F_T_NE.data()[6];
double F_T_NE7 = F_T_NE.data()[7];
double F_T_NE8 = F_T_NE.data()[8];
double F_T_NE9 = F_T_NE.data()[9];
double F_T_NE10 = F_T_NE.data()[10];
double F_T_NE11 = F_T_NE.data()[11];
double F_T_NE12 = F_T_NE.data()[12];
double F_T_NE13 = F_T_NE.data()[13];
double F_T_NE14 = F_T_NE.data()[14];
double F_T_NE15 = F_T_NE.data()[15];

double NE_T_EE0 = NE_T_EE.data()[0];
double NE_T_EE1 = NE_T_EE.data()[1];
double NE_T_EE2 = NE_T_EE.data()[2];
double NE_T_EE3 = NE_T_EE.data()[3];
double NE_T_EE4 = NE_T_EE.data()[4];
double NE_T_EE5 = NE_T_EE.data()[5];
double NE_T_EE6 = NE_T_EE.data()[6];
double NE_T_EE7 = NE_T_EE.data()[7];
double NE_T_EE8 = NE_T_EE.data()[8];
double NE_T_EE9 = NE_T_EE.data()[9];
double NE_T_EE10 = NE_T_EE.data()[10];
double NE_T_EE11 = NE_T_EE.data()[11];
double NE_T_EE12 = NE_T_EE.data()[12];
double NE_T_EE13 = NE_T_EE.data()[13];
double NE_T_EE14 = NE_T_EE.data()[14];
double NE_T_EE15 = NE_T_EE.data()[15];

double EE_T_K0 = EE_T_K.data()[0];
double EE_T_K1 = EE_T_K.data()[1];
double EE_T_K2 = EE_T_K.data()[2];
double EE_T_K3 = EE_T_K.data()[3];
double EE_T_K4 = EE_T_K.data()[4];
double EE_T_K5 = EE_T_K.data()[5];
double EE_T_K6 = EE_T_K.data()[6];
double EE_T_K7 = EE_T_K.data()[7];
double EE_T_K8 = EE_T_K.data()[8];
double EE_T_K9 = EE_T_K.data()[9];
double EE_T_K10 = EE_T_K.data()[10];
double EE_T_K11 = EE_T_K.data()[11];
double EE_T_K12 = EE_T_K.data()[12];
double EE_T_K13 = EE_T_K.data()[13];
double EE_T_K14 = EE_T_K.data()[14];
double EE_T_K15 = EE_T_K.data()[15];

double I_ee0 = I_ee.data()[0];
double I_ee1 = I_ee.data()[1];
double I_ee2 = I_ee.data()[2];
double I_ee3 = I_ee.data()[3];
double I_ee4 = I_ee.data()[4];
double I_ee5 = I_ee.data()[5];
double I_ee6 = I_ee.data()[6];
double I_ee7 = I_ee.data()[7];
double I_ee8 = I_ee.data()[8];

double F_x_Cee0 = F_x_Cee.data()[0];
double F_x_Cee1 = F_x_Cee.data()[1];
double F_x_Cee2 = F_x_Cee.data()[2];

double I_load0 = I_load.data()[0];
double I_load1 = I_load.data()[1];
double I_load2 = I_load.data()[2];
double I_load3 = I_load.data()[3];
double I_load4 = I_load.data()[4];
double I_load5 = I_load.data()[5];
double I_load6 = I_load.data()[6];
double I_load7 = I_load.data()[7];
double I_load8 = I_load.data()[8];

double F_x_Cload0 = F_x_Cload.data()[0];
double F_x_Cload1 = F_x_Cload.data()[1];
double F_x_Cload2 = F_x_Cload.data()[2];

double I_total0 = I_total.data()[0];
double I_total1 = I_total.data()[1];
double I_total2 = I_total.data()[2];
double I_total3 = I_total.data()[3];
double I_total4 = I_total.data()[4];
double I_total5 = I_total.data()[5];
double I_total6 = I_total.data()[6];
double I_total7 = I_total.data()[7];
double I_total8 = I_total.data()[8];

double F_x_Ctotal0 = F_x_Ctotal.data()[0];
double F_x_Ctotal1 = F_x_Ctotal.data()[1];
double F_x_Ctotal2 = F_x_Ctotal.data()[2];

double elbow0 = elbow.data()[0];
double elbow1 = elbow.data()[1];

double elbow_d0 = elbow_d.data()[0];
double elbow_d1 = elbow_d.data()[1];

double elbow_c0 = elbow_c.data()[0];
double elbow_c1 = elbow_c.data()[1];

double delbow_c0 = delbow_c.data()[0];
double delbow_c1 = delbow_c.data()[1];

double ddelbow_c0 = ddelbow_c.data()[0];
double ddelbow_c1 = ddelbow_c.data()[1];

double tau_J_d0 = tau_J_d.data()[0];
double tau_J_d1 = tau_J_d.data()[1];
double tau_J_d2 = tau_J_d.data()[2];
double tau_J_d3 = tau_J_d.data()[3];
double tau_J_d4 = tau_J_d.data()[4];
double tau_J_d5 = tau_J_d.data()[5];
double tau_J_d6 = tau_J_d.data()[6];

    std::string O_T_EE_0 = std::to_string(O_T_EE0);
	std::string O_T_EE_1 = std::to_string(O_T_EE1);
	std::string O_T_EE_2 = std::to_string(O_T_EE2);
	std::string O_T_EE_3 = std::to_string(O_T_EE3);
	std::string O_T_EE_4 = std::to_string(O_T_EE4);
	std::string O_T_EE_5 = std::to_string(O_T_EE5);
	std::string O_T_EE_6 = std::to_string(O_T_EE6);
	std::string O_T_EE_7 = std::to_string(O_T_EE7);
	std::string O_T_EE_8 = std::to_string(O_T_EE8);
	std::string O_T_EE_9 = std::to_string(O_T_EE9);
	std::string O_T_EE_10 = std::to_string(O_T_EE10);
	std::string O_T_EE_11 = std::to_string(O_T_EE11);
	std::string O_T_EE_12 = std::to_string(O_T_EE12);
	std::string O_T_EE_13 = std::to_string(O_T_EE13);
	std::string O_T_EE_14 = std::to_string(O_T_EE14);
	std::string O_T_EE_15 = std::to_string(O_T_EE15);

	std::string O_T_EE_d_0 = std::to_string(O_T_EE_d0);
	std::string O_T_EE_d_1 = std::to_string(O_T_EE_d1);
	std::string O_T_EE_d_2 = std::to_string(O_T_EE_d2);
	std::string O_T_EE_d_3 = std::to_string(O_T_EE_d3);
	std::string O_T_EE_d_4 = std::to_string(O_T_EE_d4);
	std::string O_T_EE_d_5 = std::to_string(O_T_EE_d5);
	std::string O_T_EE_d_6 = std::to_string(O_T_EE_d6);
	std::string O_T_EE_d_7 = std::to_string(O_T_EE_d7);
	std::string O_T_EE_d_8 = std::to_string(O_T_EE_d8);
	std::string O_T_EE_d_9 = std::to_string(O_T_EE_d9);
	std::string O_T_EE_d_10 = std::to_string(O_T_EE_d10);
	std::string O_T_EE_d_11 = std::to_string(O_T_EE_d11);
	std::string O_T_EE_d_12 = std::to_string(O_T_EE_d12);
	std::string O_T_EE_d_13 = std::to_string(O_T_EE_d13);
	std::string O_T_EE_d_14 = std::to_string(O_T_EE_d14);
	std::string O_T_EE_d_15 = std::to_string(O_T_EE_d15);

	std::string F_T_EE_0 = std::to_string(F_T_EE0);
	std::string F_T_EE_1 = std::to_string(F_T_EE1);
	std::string F_T_EE_2 = std::to_string(F_T_EE2);
	std::string F_T_EE_3 = std::to_string(F_T_EE3);
	std::string F_T_EE_4 = std::to_string(F_T_EE4);
	std::string F_T_EE_5 = std::to_string(F_T_EE5);
	std::string F_T_EE_6 = std::to_string(F_T_EE6);
	std::string F_T_EE_7 = std::to_string(F_T_EE7);
	std::string F_T_EE_8 = std::to_string(F_T_EE8);
	std::string F_T_EE_9 = std::to_string(F_T_EE9);
	std::string F_T_EE_10 = std::to_string(F_T_EE10);
	std::string F_T_EE_11 = std::to_string(F_T_EE11);
	std::string F_T_EE_12 = std::to_string(F_T_EE12);
	std::string F_T_EE_13 = std::to_string(F_T_EE13);
	std::string F_T_EE_14 = std::to_string(F_T_EE14);
	std::string F_T_EE_15 = std::to_string(F_T_EE15);

	std::string F_T_NE_0 = std::to_string(F_T_NE0);
	std::string F_T_NE_1 = std::to_string(F_T_NE1);
	std::string F_T_NE_2 = std::to_string(F_T_NE2);
	std::string F_T_NE_3 = std::to_string(F_T_NE3);
	std::string F_T_NE_4 = std::to_string(F_T_NE4);
	std::string F_T_NE_5 = std::to_string(F_T_NE5);
	std::string F_T_NE_6 = std::to_string(F_T_NE6);
	std::string F_T_NE_7 = std::to_string(F_T_NE7);
	std::string F_T_NE_8 = std::to_string(F_T_NE8);
	std::string F_T_NE_9 = std::to_string(F_T_NE9);
	std::string F_T_NE_10 = std::to_string(F_T_NE10);
	std::string F_T_NE_11 = std::to_string(F_T_NE11);
	std::string F_T_NE_12 = std::to_string(F_T_NE12);
	std::string F_T_NE_13 = std::to_string(F_T_NE13);
	std::string F_T_NE_14 = std::to_string(F_T_NE14);
	std::string F_T_NE_15 = std::to_string(F_T_NE15);

	std::string NE_T_EE_0 = std::to_string(NE_T_EE0);
	std::string NE_T_EE_1 = std::to_string(NE_T_EE1);
	std::string NE_T_EE_2 = std::to_string(NE_T_EE2);
	std::string NE_T_EE_3 = std::to_string(NE_T_EE3);
	std::string NE_T_EE_4 = std::to_string(NE_T_EE4);
	std::string NE_T_EE_5 = std::to_string(NE_T_EE5);
	std::string NE_T_EE_6 = std::to_string(NE_T_EE6);
	std::string NE_T_EE_7 = std::to_string(NE_T_EE7);
	std::string NE_T_EE_8 = std::to_string(NE_T_EE8);
	std::string NE_T_EE_9 = std::to_string(NE_T_EE9);
	std::string NE_T_EE_10 = std::to_string(NE_T_EE10);
	std::string NE_T_EE_11 = std::to_string(NE_T_EE11);
	std::string NE_T_EE_12 = std::to_string(NE_T_EE12);
	std::string NE_T_EE_13 = std::to_string(NE_T_EE13);
	std::string NE_T_EE_14 = std::to_string(NE_T_EE14);
	std::string NE_T_EE_15 = std::to_string(NE_T_EE15);

	std::string EE_T_K_0 = std::to_string(EE_T_K0);
	std::string EE_T_K_1 = std::to_string(EE_T_K1);
	std::string EE_T_K_2 = std::to_string(EE_T_K2);
	std::string EE_T_K_3 = std::to_string(EE_T_K3);
	std::string EE_T_K_4 = std::to_string(EE_T_K4);
	std::string EE_T_K_5 = std::to_string(EE_T_K5);
	std::string EE_T_K_6 = std::to_string(EE_T_K6);
	std::string EE_T_K_7 = std::to_string(EE_T_K7);
	std::string EE_T_K_8 = std::to_string(EE_T_K8);
	std::string EE_T_K_9 = std::to_string(EE_T_K9);
	std::string EE_T_K_10 = std::to_string(EE_T_K10);
	std::string EE_T_K_11 = std::to_string(EE_T_K11);
	std::string EE_T_K_12 = std::to_string(EE_T_K12);
	std::string EE_T_K_13 = std::to_string(EE_T_K13);
	std::string EE_T_K_14 = std::to_string(EE_T_K14);
	std::string EE_T_K_15 = std::to_string(EE_T_K15);

	std::string m_ee_0 = std::to_string(m_ee);

	std::string I_ee_0 = std::to_string(I_ee0);
	std::string I_ee_1 = std::to_string(I_ee1);
	std::string I_ee_2 = std::to_string(I_ee2);
	std::string I_ee_3 = std::to_string(I_ee3);
	std::string I_ee_4 = std::to_string(I_ee4);
	std::string I_ee_5 = std::to_string(I_ee5);
	std::string I_ee_6 = std::to_string(I_ee6);
	std::string I_ee_7 = std::to_string(I_ee7);
	std::string I_ee_8 = std::to_string(I_ee8);

	std::string F_x_Cee_0 = std::to_string(F_x_Cee0);
	std::string F_x_Cee_1 = std::to_string(F_x_Cee1);
	std::string F_x_Cee_2 = std::to_string(F_x_Cee2);

	std::string m_load_0 = std::to_string(m_load);

	std::string I_load_0 = std::to_string(I_load0);
	std::string I_load_1 = std::to_string(I_load1);
	std::string I_load_2 = std::to_string(I_load2);
	std::string I_load_3 = std::to_string(I_load3);
	std::string I_load_4 = std::to_string(I_load4);
	std::string I_load_5 = std::to_string(I_load5);
	std::string I_load_6 = std::to_string(I_load6);
	std::string I_load_7 = std::to_string(I_load7);
	std::string I_load_8 = std::to_string(I_load8);
	
	std::string F_x_Cload_0 = std::to_string(F_x_Cload0);
	std::string F_x_Cload_1 = std::to_string(F_x_Cload1);
	std::string F_x_Cload_2 = std::to_string(F_x_Cload2);

	std::string m_total_0 = std::to_string(m_total);

	std::string I_total_0 = std::to_string(I_total0);
	std::string I_total_1 = std::to_string(I_total1);
	std::string I_total_2 = std::to_string(I_total2);
	std::string I_total_3 = std::to_string(I_total3);
	std::string I_total_4 = std::to_string(I_total4);
	std::string I_total_5 = std::to_string(I_total5);
	std::string I_total_6 = std::to_string(I_total6);
	std::string I_total_7 = std::to_string(I_total7);
	std::string I_total_8 = std::to_string(I_total8);

	std::string F_x_Ctotal_0 = std::to_string(F_x_Ctotal0);
	std::string F_x_Ctotal_1 = std::to_string(F_x_Ctotal1);
	std::string F_x_Ctotal_2 = std::to_string(F_x_Ctotal2);

	std::string elbow_0 = std::to_string(elbow0);
	std::string elbow_1 = std::to_string(elbow1);

	std::string elbow_d_0 = std::to_string(elbow_d0);
	std::string elbow_d_1 = std::to_string(elbow_d1);

	std::string elbow_c_0 = std::to_string(elbow_c0);
	std::string elbow_c_1 = std::to_string(elbow_c1);

	std::string delbow_c_0 = std::to_string(delbow_c0);
	std::string delbow_c_1 = std::to_string(delbow_c1);

	std::string ddelbow_c_0 = std::to_string(ddelbow_c0);
	std::string ddelbow_c_1 = std::to_string(ddelbow_c1);

	std::string tau_J_d_0 = std::to_string(tau_J_d0);
	std::string tau_J_d_1 = std::to_string(tau_J_d1);
	std::string tau_J_d_2 = std::to_string(tau_J_d2);
	std::string tau_J_d_3 = std::to_string(tau_J_d3);
	std::string tau_J_d_4 = std::to_string(tau_J_d4);
	std::string tau_J_d_5 = std::to_string(tau_J_d5);
	std::string tau_J_d_6 = std::to_string(tau_J_d6);




const int  QOS = 0;



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
    array<double, 16ul> O_T_EE_d = robot_state.O_T_EE_d;
    array< double, 16 > F_T_EE = robot_state.F_T_EE; 
    array< double, 16 > F_T_NE = robot_state.F_T_NE;
    array< double, 16 > NE_T_EE = robot_state.NE_T_EE;
    array< double, 16 > EE_T_K = robot_state.EE_T_K;
    double m_ee = robot_state.m_ee;
    array< double, 9 > I_ee = robot_state.I_ee;
    array< double, 3 > F_x_Cee = robot_state.F_x_Cee;
    double m_load = robot_state.m_load;
    array< double, 9 > I_load = robot_state.I_load;
    array< double, 3 > F_x_Cload = robot_state.F_x_Cload;
    double m_total = robot_state.m_total;
    array< double, 9 > I_total = robot_state.I_total;
    array< double, 3 > F_x_Ctotal = robot_state.F_x_Ctotal;
    array< double, 2 > elbow = robot_state.elbow;
    array< double, 2 > elbow_d = robot_state.elbow_d;
    array< double, 2 > elbow_c = robot_state.elbow_c;
    array< double, 2 > delbow_c = robot_state.delbow_c;
    array< double, 2 > ddelbow_c = robot_state.ddelbow_c;
    array< double, 7 > tau_J_d = robot_state.tau_J_d;



    size_t count = 0;
    robot.read([&count, &O_T_EE, &O_T_EE_d, &F_T_EE, &F_T_NE, &NE_T_EE, &EE_T_K, &m_ee, &I_ee, &F_x_Cee, &m_load, &I_load, &F_x_Cload, &m_total, &I_total, &F_x_Ctotal, &elbow, &elbow_d, &elbow_c, &delbow_c, &ddelbow_c, &tau_J_d, &client](const franka::RobotState& robot_state) {
      
    


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
    array<double, 16ul> O_T_EE_d = robot_state.O_T_EE_d;
    array< double, 16 > F_T_EE = robot_state.F_T_EE; 
    array< double, 16 > F_T_NE = robot_state.F_T_NE;
    array< double, 16 > NE_T_EE = robot_state.NE_T_EE;
    array< double, 16 > EE_T_K = robot_state.EE_T_K;
    double m_ee = robot_state.m_ee;
    array< double, 9 > I_ee = robot_state.I_ee;
    array< double, 3 > F_x_Cee = robot_state.F_x_Cee;
    double m_load = robot_state.m_load;
    array< double, 9 > I_load = robot_state.I_load;
    array< double, 3 > F_x_Cload = robot_state.F_x_Cload;
    double m_total = robot_state.m_total;
    array< double, 9 > I_total = robot_state.I_total;
    array< double, 3 > F_x_Ctotal = robot_state.F_x_Ctotal;
    array< double, 2 > elbow = robot_state.elbow;
    array< double, 2 > elbow_d = robot_state.elbow_d;
    array< double, 2 > elbow_c = robot_state.elbow_c;
    array< double, 2 > delbow_c = robot_state.delbow_c;
    array< double, 2 > ddelbow_c = robot_state.ddelbow_c;
    array< double, 7 > tau_J_d = robot_state.tau_J_d;
 







double O_T_EE0 = O_T_EE.data()[0];
double O_T_EE1 = O_T_EE.data()[1];
double O_T_EE2 = O_T_EE.data()[2];
double O_T_EE3 = O_T_EE.data()[3];
double O_T_EE4 = O_T_EE.data()[4];
double O_T_EE5 = O_T_EE.data()[5];
double O_T_EE6 = O_T_EE.data()[6];
double O_T_EE7 = O_T_EE.data()[7];
double O_T_EE8 = O_T_EE.data()[8];
double O_T_EE9 = O_T_EE.data()[9];
double O_T_EE10 = O_T_EE.data()[10];
double O_T_EE11 = O_T_EE.data()[11];
double O_T_EE12 = O_T_EE.data()[12];
double O_T_EE13 = O_T_EE.data()[13];
double O_T_EE14 = O_T_EE.data()[14];
double O_T_EE15 = O_T_EE.data()[15];

double O_T_EE_d0 = O_T_EE_d.data()[0];
double O_T_EE_d1 = O_T_EE_d.data()[1];
double O_T_EE_d2 = O_T_EE_d.data()[2];
double O_T_EE_d3 = O_T_EE_d.data()[3];
double O_T_EE_d4 = O_T_EE_d.data()[4];
double O_T_EE_d5 = O_T_EE_d.data()[5];
double O_T_EE_d6 = O_T_EE_d.data()[6];
double O_T_EE_d7 = O_T_EE_d.data()[7];
double O_T_EE_d8 = O_T_EE_d.data()[8];
double O_T_EE_d9 = O_T_EE_d.data()[9];
double O_T_EE_d10 = O_T_EE_d.data()[10];
double O_T_EE_d11 = O_T_EE_d.data()[11];
double O_T_EE_d12 = O_T_EE_d.data()[12];
double O_T_EE_d13 = O_T_EE_d.data()[13];
double O_T_EE_d14 = O_T_EE_d.data()[14];
double O_T_EE_d15 = O_T_EE_d.data()[15];

double F_T_EE0 = F_T_EE.data()[0];
double F_T_EE1 = F_T_EE.data()[1];
double F_T_EE2 = F_T_EE.data()[2];
double F_T_EE3 = F_T_EE.data()[3];
double F_T_EE4 = F_T_EE.data()[4];
double F_T_EE5 = F_T_EE.data()[5];
double F_T_EE6 = F_T_EE.data()[6];
double F_T_EE7 = F_T_EE.data()[7];
double F_T_EE8 = F_T_EE.data()[8];
double F_T_EE9 = F_T_EE.data()[9];
double F_T_EE10 = F_T_EE.data()[10];
double F_T_EE11 = F_T_EE.data()[11];
double F_T_EE12 = F_T_EE.data()[12];
double F_T_EE13 = F_T_EE.data()[13];
double F_T_EE14 = F_T_EE.data()[14];
double F_T_EE15 = F_T_EE.data()[15];

double F_T_NE0 = F_T_NE.data()[0];
double F_T_NE1 = F_T_NE.data()[1];
double F_T_NE2 = F_T_NE.data()[2];
double F_T_NE3 = F_T_NE.data()[3];
double F_T_NE4 = F_T_NE.data()[4];
double F_T_NE5 = F_T_NE.data()[5];
double F_T_NE6 = F_T_NE.data()[6];
double F_T_NE7 = F_T_NE.data()[7];
double F_T_NE8 = F_T_NE.data()[8];
double F_T_NE9 = F_T_NE.data()[9];
double F_T_NE10 = F_T_NE.data()[10];
double F_T_NE11 = F_T_NE.data()[11];
double F_T_NE12 = F_T_NE.data()[12];
double F_T_NE13 = F_T_NE.data()[13];
double F_T_NE14 = F_T_NE.data()[14];
double F_T_NE15 = F_T_NE.data()[15];

double NE_T_EE0 = NE_T_EE.data()[0];
double NE_T_EE1 = NE_T_EE.data()[1];
double NE_T_EE2 = NE_T_EE.data()[2];
double NE_T_EE3 = NE_T_EE.data()[3];
double NE_T_EE4 = NE_T_EE.data()[4];
double NE_T_EE5 = NE_T_EE.data()[5];
double NE_T_EE6 = NE_T_EE.data()[6];
double NE_T_EE7 = NE_T_EE.data()[7];
double NE_T_EE8 = NE_T_EE.data()[8];
double NE_T_EE9 = NE_T_EE.data()[9];
double NE_T_EE10 = NE_T_EE.data()[10];
double NE_T_EE11 = NE_T_EE.data()[11];
double NE_T_EE12 = NE_T_EE.data()[12];
double NE_T_EE13 = NE_T_EE.data()[13];
double NE_T_EE14 = NE_T_EE.data()[14];
double NE_T_EE15 = NE_T_EE.data()[15];

double EE_T_K0 = EE_T_K.data()[0];
double EE_T_K1 = EE_T_K.data()[1];
double EE_T_K2 = EE_T_K.data()[2];
double EE_T_K3 = EE_T_K.data()[3];
double EE_T_K4 = EE_T_K.data()[4];
double EE_T_K5 = EE_T_K.data()[5];
double EE_T_K6 = EE_T_K.data()[6];
double EE_T_K7 = EE_T_K.data()[7];
double EE_T_K8 = EE_T_K.data()[8];
double EE_T_K9 = EE_T_K.data()[9];
double EE_T_K10 = EE_T_K.data()[10];
double EE_T_K11 = EE_T_K.data()[11];
double EE_T_K12 = EE_T_K.data()[12];
double EE_T_K13 = EE_T_K.data()[13];
double EE_T_K14 = EE_T_K.data()[14];
double EE_T_K15 = EE_T_K.data()[15];

double I_ee0 = I_ee.data()[0];
double I_ee1 = I_ee.data()[1];
double I_ee2 = I_ee.data()[2];
double I_ee3 = I_ee.data()[3];
double I_ee4 = I_ee.data()[4];
double I_ee5 = I_ee.data()[5];
double I_ee6 = I_ee.data()[6];
double I_ee7 = I_ee.data()[7];
double I_ee8 = I_ee.data()[8];

double F_x_Cee0 = F_x_Cee.data()[0];
double F_x_Cee1 = F_x_Cee.data()[1];
double F_x_Cee2 = F_x_Cee.data()[2];

double I_load0 = I_load.data()[0];
double I_load1 = I_load.data()[1];
double I_load2 = I_load.data()[2];
double I_load3 = I_load.data()[3];
double I_load4 = I_load.data()[4];
double I_load5 = I_load.data()[5];
double I_load6 = I_load.data()[6];
double I_load7 = I_load.data()[7];
double I_load8 = I_load.data()[8];

double F_x_Cload0 = F_x_Cload.data()[0];
double F_x_Cload1 = F_x_Cload.data()[1];
double F_x_Cload2 = F_x_Cload.data()[2];

double I_total0 = I_total.data()[0];
double I_total1 = I_total.data()[1];
double I_total2 = I_total.data()[2];
double I_total3 = I_total.data()[3];
double I_total4 = I_total.data()[4];
double I_total5 = I_total.data()[5];
double I_total6 = I_total.data()[6];
double I_total7 = I_total.data()[7];
double I_total8 = I_total.data()[8];

double F_x_Ctotal0 = F_x_Ctotal.data()[0];
double F_x_Ctotal1 = F_x_Ctotal.data()[1];
double F_x_Ctotal2 = F_x_Ctotal.data()[2];

double elbow0 = elbow.data()[0];
double elbow1 = elbow.data()[1];

double elbow_d0 = elbow_d.data()[0];
double elbow_d1 = elbow_d.data()[1];

double elbow_c0 = elbow_c.data()[0];
double elbow_c1 = elbow_c.data()[1];

double delbow_c0 = delbow_c.data()[0];
double delbow_c1 = delbow_c.data()[1];

double ddelbow_c0 = ddelbow_c.data()[0];
double ddelbow_c1 = ddelbow_c.data()[1];

double tau_J_d0 = tau_J_d.data()[0];
double tau_J_d1 = tau_J_d.data()[1];
double tau_J_d2 = tau_J_d.data()[2];
double tau_J_d3 = tau_J_d.data()[3];
double tau_J_d4 = tau_J_d.data()[4];
double tau_J_d5 = tau_J_d.data()[5];
double tau_J_d6 = tau_J_d.data()[6];



    std::string O_T_EE_0 = std::to_string(O_T_EE0);
	std::string O_T_EE_1 = std::to_string(O_T_EE1);
	std::string O_T_EE_2 = std::to_string(O_T_EE2);
	std::string O_T_EE_3 = std::to_string(O_T_EE3);
	std::string O_T_EE_4 = std::to_string(O_T_EE4);
	std::string O_T_EE_5 = std::to_string(O_T_EE5);
	std::string O_T_EE_6 = std::to_string(O_T_EE6);
	std::string O_T_EE_7 = std::to_string(O_T_EE7);
	std::string O_T_EE_8 = std::to_string(O_T_EE8);
	std::string O_T_EE_9 = std::to_string(O_T_EE9);
	std::string O_T_EE_10 = std::to_string(O_T_EE10);
	std::string O_T_EE_11 = std::to_string(O_T_EE11);
	std::string O_T_EE_12 = std::to_string(O_T_EE12);
	std::string O_T_EE_13 = std::to_string(O_T_EE13);
	std::string O_T_EE_14 = std::to_string(O_T_EE14);
	std::string O_T_EE_15 = std::to_string(O_T_EE15);

	std::string O_T_EE_d_0 = std::to_string(O_T_EE_d0);
	std::string O_T_EE_d_1 = std::to_string(O_T_EE_d1);
	std::string O_T_EE_d_2 = std::to_string(O_T_EE_d2);
	std::string O_T_EE_d_3 = std::to_string(O_T_EE_d3);
	std::string O_T_EE_d_4 = std::to_string(O_T_EE_d4);
	std::string O_T_EE_d_5 = std::to_string(O_T_EE_d5);
	std::string O_T_EE_d_6 = std::to_string(O_T_EE_d6);
	std::string O_T_EE_d_7 = std::to_string(O_T_EE_d7);
	std::string O_T_EE_d_8 = std::to_string(O_T_EE_d8);
	std::string O_T_EE_d_9 = std::to_string(O_T_EE_d9);
	std::string O_T_EE_d_10 = std::to_string(O_T_EE_d10);
	std::string O_T_EE_d_11 = std::to_string(O_T_EE_d11);
	std::string O_T_EE_d_12 = std::to_string(O_T_EE_d12);
	std::string O_T_EE_d_13 = std::to_string(O_T_EE_d13);
	std::string O_T_EE_d_14 = std::to_string(O_T_EE_d14);
	std::string O_T_EE_d_15 = std::to_string(O_T_EE_d15);

	std::string F_T_EE_0 = std::to_string(F_T_EE0);
	std::string F_T_EE_1 = std::to_string(F_T_EE1);
	std::string F_T_EE_2 = std::to_string(F_T_EE2);
	std::string F_T_EE_3 = std::to_string(F_T_EE3);
	std::string F_T_EE_4 = std::to_string(F_T_EE4);
	std::string F_T_EE_5 = std::to_string(F_T_EE5);
	std::string F_T_EE_6 = std::to_string(F_T_EE6);
	std::string F_T_EE_7 = std::to_string(F_T_EE7);
	std::string F_T_EE_8 = std::to_string(F_T_EE8);
	std::string F_T_EE_9 = std::to_string(F_T_EE9);
	std::string F_T_EE_10 = std::to_string(F_T_EE10);
	std::string F_T_EE_11 = std::to_string(F_T_EE11);
	std::string F_T_EE_12 = std::to_string(F_T_EE12);
	std::string F_T_EE_13 = std::to_string(F_T_EE13);
	std::string F_T_EE_14 = std::to_string(F_T_EE14);
	std::string F_T_EE_15 = std::to_string(F_T_EE15);

	std::string F_T_NE_0 = std::to_string(F_T_NE0);
	std::string F_T_NE_1 = std::to_string(F_T_NE1);
	std::string F_T_NE_2 = std::to_string(F_T_NE2);
	std::string F_T_NE_3 = std::to_string(F_T_NE3);
	std::string F_T_NE_4 = std::to_string(F_T_NE4);
	std::string F_T_NE_5 = std::to_string(F_T_NE5);
	std::string F_T_NE_6 = std::to_string(F_T_NE6);
	std::string F_T_NE_7 = std::to_string(F_T_NE7);
	std::string F_T_NE_8 = std::to_string(F_T_NE8);
	std::string F_T_NE_9 = std::to_string(F_T_NE9);
	std::string F_T_NE_10 = std::to_string(F_T_NE10);
	std::string F_T_NE_11 = std::to_string(F_T_NE11);
	std::string F_T_NE_12 = std::to_string(F_T_NE12);
	std::string F_T_NE_13 = std::to_string(F_T_NE13);
	std::string F_T_NE_14 = std::to_string(F_T_NE14);
	std::string F_T_NE_15 = std::to_string(F_T_NE15);

	std::string NE_T_EE_0 = std::to_string(NE_T_EE0);
	std::string NE_T_EE_1 = std::to_string(NE_T_EE1);
	std::string NE_T_EE_2 = std::to_string(NE_T_EE2);
	std::string NE_T_EE_3 = std::to_string(NE_T_EE3);
	std::string NE_T_EE_4 = std::to_string(NE_T_EE4);
	std::string NE_T_EE_5 = std::to_string(NE_T_EE5);
	std::string NE_T_EE_6 = std::to_string(NE_T_EE6);
	std::string NE_T_EE_7 = std::to_string(NE_T_EE7);
	std::string NE_T_EE_8 = std::to_string(NE_T_EE8);
	std::string NE_T_EE_9 = std::to_string(NE_T_EE9);
	std::string NE_T_EE_10 = std::to_string(NE_T_EE10);
	std::string NE_T_EE_11 = std::to_string(NE_T_EE11);
	std::string NE_T_EE_12 = std::to_string(NE_T_EE12);
	std::string NE_T_EE_13 = std::to_string(NE_T_EE13);
	std::string NE_T_EE_14 = std::to_string(NE_T_EE14);
	std::string NE_T_EE_15 = std::to_string(NE_T_EE15);

	std::string EE_T_K_0 = std::to_string(EE_T_K0);
	std::string EE_T_K_1 = std::to_string(EE_T_K1);
	std::string EE_T_K_2 = std::to_string(EE_T_K2);
	std::string EE_T_K_3 = std::to_string(EE_T_K3);
	std::string EE_T_K_4 = std::to_string(EE_T_K4);
	std::string EE_T_K_5 = std::to_string(EE_T_K5);
	std::string EE_T_K_6 = std::to_string(EE_T_K6);
	std::string EE_T_K_7 = std::to_string(EE_T_K7);
	std::string EE_T_K_8 = std::to_string(EE_T_K8);
	std::string EE_T_K_9 = std::to_string(EE_T_K9);
	std::string EE_T_K_10 = std::to_string(EE_T_K10);
	std::string EE_T_K_11 = std::to_string(EE_T_K11);
	std::string EE_T_K_12 = std::to_string(EE_T_K12);
	std::string EE_T_K_13 = std::to_string(EE_T_K13);
	std::string EE_T_K_14 = std::to_string(EE_T_K14);
	std::string EE_T_K_15 = std::to_string(EE_T_K15);

	std::string m_ee_0 = std::to_string(m_ee);

	std::string I_ee_0 = std::to_string(I_ee0);
	std::string I_ee_1 = std::to_string(I_ee1);
	std::string I_ee_2 = std::to_string(I_ee2);
	std::string I_ee_3 = std::to_string(I_ee3);
	std::string I_ee_4 = std::to_string(I_ee4);
	std::string I_ee_5 = std::to_string(I_ee5);
	std::string I_ee_6 = std::to_string(I_ee6);
	std::string I_ee_7 = std::to_string(I_ee7);
	std::string I_ee_8 = std::to_string(I_ee8);

	std::string F_x_Cee_0 = std::to_string(F_x_Cee0);
	std::string F_x_Cee_1 = std::to_string(F_x_Cee1);
	std::string F_x_Cee_2 = std::to_string(F_x_Cee2);

	std::string m_load_0 = std::to_string(m_load);

	std::string I_load_0 = std::to_string(I_load0);
	std::string I_load_1 = std::to_string(I_load1);
	std::string I_load_2 = std::to_string(I_load2);
	std::string I_load_3 = std::to_string(I_load3);
	std::string I_load_4 = std::to_string(I_load4);
	std::string I_load_5 = std::to_string(I_load5);
	std::string I_load_6 = std::to_string(I_load6);
	std::string I_load_7 = std::to_string(I_load7);
	std::string I_load_8 = std::to_string(I_load8);
	
	std::string F_x_Cload_0 = std::to_string(F_x_Cload0);
	std::string F_x_Cload_1 = std::to_string(F_x_Cload1);
	std::string F_x_Cload_2 = std::to_string(F_x_Cload2);

	std::string m_total_0 = std::to_string(m_total);

	std::string I_total_0 = std::to_string(I_total0);
	std::string I_total_1 = std::to_string(I_total1);
	std::string I_total_2 = std::to_string(I_total2);
	std::string I_total_3 = std::to_string(I_total3);
	std::string I_total_4 = std::to_string(I_total4);
	std::string I_total_5 = std::to_string(I_total5);
	std::string I_total_6 = std::to_string(I_total6);
	std::string I_total_7 = std::to_string(I_total7);
	std::string I_total_8 = std::to_string(I_total8);

	std::string F_x_Ctotal_0 = std::to_string(F_x_Ctotal0);
	std::string F_x_Ctotal_1 = std::to_string(F_x_Ctotal1);
	std::string F_x_Ctotal_2 = std::to_string(F_x_Ctotal2);

	std::string elbow_0 = std::to_string(elbow0);
	std::string elbow_1 = std::to_string(elbow1);

	std::string elbow_d_0 = std::to_string(elbow_d0);
	std::string elbow_d_1 = std::to_string(elbow_d1);

	std::string elbow_c_0 = std::to_string(elbow_c0);
	std::string elbow_c_1 = std::to_string(elbow_c1);

	std::string delbow_c_0 = std::to_string(delbow_c0);
	std::string delbow_c_1 = std::to_string(delbow_c1);

	std::string ddelbow_c_0 = std::to_string(ddelbow_c0);
	std::string ddelbow_c_1 = std::to_string(ddelbow_c1);

	std::string tau_J_d_0 = std::to_string(tau_J_d0);
	std::string tau_J_d_1 = std::to_string(tau_J_d1);
	std::string tau_J_d_2 = std::to_string(tau_J_d2);
	std::string tau_J_d_3 = std::to_string(tau_J_d3);
	std::string tau_J_d_4 = std::to_string(tau_J_d4);
	std::string tau_J_d_5 = std::to_string(tau_J_d5);
	std::string tau_J_d_6 = std::to_string(tau_J_d6);


	const int  QOS = 0;

	const auto TIMEOUT = std::chrono::seconds(10);










    mqtt::message_ptr OTEE0 = mqtt::make_message(TOPIC1_0, O_T_EE_0);
		OTEE0->set_qos(QOS);
		client.publish(OTEE0)->wait_for(TIMEOUT);

    mqtt::message_ptr OTEE1 = mqtt::make_message(TOPIC1_1, O_T_EE_1);
		OTEE1->set_qos(QOS);
		client.publish(OTEE1)->wait_for(TIMEOUT);
    
    mqtt::message_ptr OTEE2 = mqtt::make_message(TOPIC1_2, O_T_EE_2);
		OTEE2->set_qos(QOS);
		client.publish(OTEE2)->wait_for(TIMEOUT);

    mqtt::message_ptr OTEE3 = mqtt::make_message(TOPIC1_3, O_T_EE_3);
		OTEE3->set_qos(QOS);
		client.publish(OTEE3)->wait_for(TIMEOUT);
    
	mqtt::message_ptr OTEE4 = mqtt::make_message(TOPIC1_4, O_T_EE_4);
		OTEE4->set_qos(QOS);
		client.publish(OTEE4)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEE5 = mqtt::make_message(TOPIC1_5, O_T_EE_5);
		OTEE5->set_qos(QOS);
		client.publish(OTEE5)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEE6 = mqtt::make_message(TOPIC1_6, O_T_EE_6);
		OTEE6->set_qos(QOS);
		client.publish(OTEE6)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEE7 = mqtt::make_message(TOPIC1_7, O_T_EE_7);
		OTEE7->set_qos(QOS);
		client.publish(OTEE7)->wait_for(TIMEOUT);			

    mqtt::message_ptr OTEE8 = mqtt::make_message(TOPIC1_8, O_T_EE_8);
		OTEE8->set_qos(QOS);
		client.publish(OTEE8)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEE9 = mqtt::make_message(TOPIC1_9, O_T_EE_9);
		OTEE9->set_qos(QOS);
		client.publish(OTEE9)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEE10 = mqtt::make_message(TOPIC1_10, O_T_EE_10);
		OTEE10->set_qos(QOS);
		client.publish(OTEE10)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEE11 = mqtt::make_message(TOPIC1_11, O_T_EE_11);
		OTEE11->set_qos(QOS);
		client.publish(OTEE11)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEE12 = mqtt::make_message(TOPIC1_12, O_T_EE_12);
		OTEE12->set_qos(QOS);
		client.publish(OTEE12)->wait_for(TIMEOUT);		

	mqtt::message_ptr OTEE13 = mqtt::make_message(TOPIC1_13, O_T_EE_13);
		OTEE13->set_qos(QOS);
		client.publish(OTEE13)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEE14 = mqtt::make_message(TOPIC1_14, O_T_EE_14);
		OTEE14->set_qos(QOS);
		client.publish(OTEE14)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEE15 = mqtt::make_message(TOPIC1_15, O_T_EE_15);
		OTEE15->set_qos(QOS);
		client.publish(OTEE15)->wait_for(TIMEOUT);				



	mqtt::message_ptr OTEEd0 = mqtt::make_message(TOPIC2_0, O_T_EE_d_0);
		OTEEd0->set_qos(QOS);
		client.publish(OTEEd0)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd1 = mqtt::make_message(TOPIC2_1, O_T_EE_d_1);
		OTEEd1->set_qos(QOS);
		client.publish(OTEEd1)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd2 = mqtt::make_message(TOPIC2_2, O_T_EE_d_2);
		OTEEd2->set_qos(QOS);
		client.publish(OTEEd2)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd3 = mqtt::make_message(TOPIC2_3, O_T_EE_d_3);
		OTEEd3->set_qos(QOS);
		client.publish(OTEEd3)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd4 = mqtt::make_message(TOPIC2_4, O_T_EE_d_4);
		OTEEd4->set_qos(QOS);
		client.publish(OTEEd4)->wait_for(TIMEOUT);
	
	mqtt::message_ptr OTEEd5 = mqtt::make_message(TOPIC2_5, O_T_EE_d_5);
		OTEEd5->set_qos(QOS);
		client.publish(OTEEd5)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd6 = mqtt::make_message(TOPIC2_6, O_T_EE_d_6);
		OTEEd6->set_qos(QOS);
		client.publish(OTEEd6)->wait_for(TIMEOUT);	

	mqtt::message_ptr OTEEd7 = mqtt::make_message(TOPIC2_7, O_T_EE_d_7);
		OTEEd7->set_qos(QOS);
		client.publish(OTEEd7)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd8 = mqtt::make_message(TOPIC2_8, O_T_EE_d_8);
		OTEEd8->set_qos(QOS);
		client.publish(OTEEd8)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd9 = mqtt::make_message(TOPIC2_9, O_T_EE_d_9);
		OTEEd9->set_qos(QOS);
		client.publish(OTEEd9)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd10 = mqtt::make_message(TOPIC2_10, O_T_EE_d_10);
		OTEEd10->set_qos(QOS);
		client.publish(OTEEd10)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd11 = mqtt::make_message(TOPIC2_11, O_T_EE_d_11);
		OTEEd11->set_qos(QOS);
		client.publish(OTEEd11)->wait_for(TIMEOUT);
		
	mqtt::message_ptr OTEEd12 = mqtt::make_message(TOPIC2_12, O_T_EE_d_12);
		OTEEd12->set_qos(QOS);
		client.publish(OTEEd12)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd13 = mqtt::make_message(TOPIC2_13, O_T_EE_d_13);
		OTEEd13->set_qos(QOS);
		client.publish(OTEEd13)->wait_for(TIMEOUT);

	mqtt::message_ptr OTEEd14 = mqtt::make_message(TOPIC2_14, O_T_EE_d_14);
		OTEEd14->set_qos(QOS);
		client.publish(OTEEd14)->wait_for(TIMEOUT);	
    
	mqtt::message_ptr OTEEd15 = mqtt::make_message(TOPIC2_15, O_T_EE_d_15);
		OTEEd15->set_qos(QOS);
		client.publish(OTEEd15)->wait_for(TIMEOUT);	

    
	
	
	mqtt::message_ptr FTEE0 = mqtt::make_message(TOPIC3_0, F_T_EE_0);
		FTEE0->set_qos(QOS);
		client.publish(FTEE0)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTEE1 = mqtt::make_message(TOPIC3_1, F_T_EE_1);
		FTEE1->set_qos(QOS);
		client.publish(FTEE1)->wait_for(TIMEOUT);
	
	mqtt::message_ptr FTEE2 = mqtt::make_message(TOPIC3_2, F_T_EE_2);
		FTEE2->set_qos(QOS);
		client.publish(FTEE2)->wait_for(TIMEOUT);
	
	mqtt::message_ptr FTEE3 = mqtt::make_message(TOPIC3_3, F_T_EE_3);
		FTEE3->set_qos(QOS);
		client.publish(FTEE3)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE4 = mqtt::make_message(TOPIC3_4, F_T_EE_4);
		FTEE4->set_qos(QOS);
		client.publish(FTEE4)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE5 = mqtt::make_message(TOPIC3_5, F_T_EE_5);
		FTEE5->set_qos(QOS);
		client.publish(FTEE5)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE6 = mqtt::make_message(TOPIC3_6, F_T_EE_6);
		FTEE6->set_qos(QOS);
		client.publish(FTEE6)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE7 = mqtt::make_message(TOPIC3_7, F_T_EE_7);
		FTEE7->set_qos(QOS);
		client.publish(FTEE7)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE8 = mqtt::make_message(TOPIC3_8, F_T_EE_8);
		FTEE8->set_qos(QOS);
		client.publish(FTEE8)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE9 = mqtt::make_message(TOPIC3_9, F_T_EE_9);
		FTEE9->set_qos(QOS);
		client.publish(FTEE9)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE10 = mqtt::make_message(TOPIC3_10, F_T_EE_10);
		FTEE10->set_qos(QOS);
		client.publish(FTEE10)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE11 = mqtt::make_message(TOPIC3_11, F_T_EE_11);
		FTEE11->set_qos(QOS);
		client.publish(FTEE11)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE12 = mqtt::make_message(TOPIC3_12, F_T_EE_12);
		FTEE12->set_qos(QOS);
		client.publish(FTEE12)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE13 = mqtt::make_message(TOPIC3_13, F_T_EE_13);
		FTEE13->set_qos(QOS);
		client.publish(FTEE13)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE14 = mqtt::make_message(TOPIC3_14, F_T_EE_14);
		FTEE14->set_qos(QOS);
		client.publish(FTEE14)->wait_for(TIMEOUT);

	mqtt::message_ptr FTEE15 = mqtt::make_message(TOPIC3_15, F_T_EE_15);
		FTEE15->set_qos(QOS);
		client.publish(FTEE15)->wait_for(TIMEOUT);	


	
	mqtt::message_ptr FTNE0 = mqtt::make_message(TOPIC32_0, F_T_NE_0);
		FTNE0->set_qos(QOS);
		client.publish(FTNE0)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE1 = mqtt::make_message(TOPIC32_1, F_T_NE_0);
		FTNE1->set_qos(QOS);
		client.publish(FTNE1)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE2 = mqtt::make_message(TOPIC32_2, F_T_NE_2);
		FTNE2->set_qos(QOS);
		client.publish(FTNE2)->wait_for(TIMEOUT);		

	mqtt::message_ptr FTNE3 = mqtt::make_message(TOPIC32_3, F_T_NE_3);
		FTNE3->set_qos(QOS);
		client.publish(FTNE3)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE4 = mqtt::make_message(TOPIC32_4, F_T_NE_4);
		FTNE4->set_qos(QOS);
		client.publish(FTNE4)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE5 = mqtt::make_message(TOPIC32_5, F_T_NE_5);
		FTNE5->set_qos(QOS);
		client.publish(FTNE5)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE6 = mqtt::make_message(TOPIC32_6, F_T_NE_6);
		FTNE6->set_qos(QOS);
		client.publish(FTNE6)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE7 = mqtt::make_message(TOPIC32_7, F_T_NE_7);
		FTNE7->set_qos(QOS);
		client.publish(FTNE7)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE8 = mqtt::make_message(TOPIC32_8, F_T_NE_8);
		FTNE8->set_qos(QOS);
		client.publish(FTNE8)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE9 = mqtt::make_message(TOPIC32_9, F_T_NE_9);
		FTNE9->set_qos(QOS);
		client.publish(FTNE9)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE10 = mqtt::make_message(TOPIC32_10, F_T_NE_10);
		FTNE10->set_qos(QOS);
		client.publish(FTNE10)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE11 = mqtt::make_message(TOPIC32_11, F_T_NE_11);
		FTNE11->set_qos(QOS);
		client.publish(FTNE11)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE12 = mqtt::make_message(TOPIC32_12, F_T_NE_12);
		FTNE12->set_qos(QOS);
		client.publish(FTNE12)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE13 = mqtt::make_message(TOPIC32_13, F_T_NE_13);
		FTNE13->set_qos(QOS);
		client.publish(FTNE13)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE14 = mqtt::make_message(TOPIC32_14, F_T_NE_14);
		FTNE14->set_qos(QOS);
		client.publish(FTNE14)->wait_for(TIMEOUT);	

	mqtt::message_ptr FTNE15 = mqtt::make_message(TOPIC32_15, F_T_NE_15);
		FTNE15->set_qos(QOS);
		client.publish(FTNE15)->wait_for(TIMEOUT);			
	



	mqtt::message_ptr NETEE0 = mqtt::make_message(TOPIC33_0, NE_T_EE_0);
		NETEE0->set_qos(QOS);
		client.publish(NETEE0)->wait_for(TIMEOUT);	
    
	mqtt::message_ptr NETEE1 = mqtt::make_message(TOPIC33_1, NE_T_EE_1);
		NETEE1->set_qos(QOS);
		client.publish(NETEE1)->wait_for(TIMEOUT);	
    
	mqtt::message_ptr NETEE2 = mqtt::make_message(TOPIC33_2, NE_T_EE_2);
		NETEE2->set_qos(QOS);
		client.publish(NETEE2)->wait_for(TIMEOUT);	

    mqtt::message_ptr NETEE3 = mqtt::make_message(TOPIC33_3, NE_T_EE_3);
		NETEE3->set_qos(QOS);
		client.publish(NETEE3)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE4 = mqtt::make_message(TOPIC33_4, NE_T_EE_4);
		NETEE4->set_qos(QOS);
		client.publish(NETEE4)->wait_for(TIMEOUT);			

	mqtt::message_ptr NETEE5 = mqtt::make_message(TOPIC33_5, NE_T_EE_5);
		NETEE5->set_qos(QOS);
		client.publish(NETEE5)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE6 = mqtt::make_message(TOPIC33_6, NE_T_EE_6);
		NETEE6->set_qos(QOS);
		client.publish(NETEE6)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE7 = mqtt::make_message(TOPIC33_7, NE_T_EE_7);
		NETEE7->set_qos(QOS);
		client.publish(NETEE7)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE8 = mqtt::make_message(TOPIC33_8, NE_T_EE_8);
		NETEE8->set_qos(QOS);
		client.publish(NETEE8)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE9 = mqtt::make_message(TOPIC33_9, NE_T_EE_9);
		NETEE9->set_qos(QOS);
		client.publish(NETEE9)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE10 = mqtt::make_message(TOPIC33_10, NE_T_EE_10);
		NETEE10->set_qos(QOS);
		client.publish(NETEE10)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE11 = mqtt::make_message(TOPIC33_11, NE_T_EE_11);
		NETEE11->set_qos(QOS);
		client.publish(NETEE11)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE12 = mqtt::make_message(TOPIC33_12, NE_T_EE_12);
		NETEE12->set_qos(QOS);
		client.publish(NETEE12)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE13 = mqtt::make_message(TOPIC33_13, NE_T_EE_13);
		NETEE13->set_qos(QOS);
		client.publish(NETEE13)->wait_for(TIMEOUT);

	mqtt::message_ptr NETEE14= mqtt::make_message(TOPIC33_14, NE_T_EE_14);
		NETEE14->set_qos(QOS);
		client.publish(NETEE14)->wait_for(TIMEOUT);	

	mqtt::message_ptr NETEE15= mqtt::make_message(TOPIC33_15, NE_T_EE_15);
		NETEE15->set_qos(QOS);
		client.publish(NETEE15)->wait_for(TIMEOUT);	




    mqtt::message_ptr EETK0= mqtt::make_message(TOPIC5_0, EE_T_K_0);
		EETK0->set_qos(QOS);
		client.publish(EETK0)->wait_for(TIMEOUT);	

    mqtt::message_ptr EETK1= mqtt::make_message(TOPIC5_1, EE_T_K_1);
		EETK1->set_qos(QOS);
		client.publish(EETK1)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK2= mqtt::make_message(TOPIC5_2, EE_T_K_2);
		EETK2->set_qos(QOS);
		client.publish(EETK2)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK3= mqtt::make_message(TOPIC5_3, EE_T_K_3);
		EETK3->set_qos(QOS);
		client.publish(EETK3)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK4= mqtt::make_message(TOPIC5_4, EE_T_K_4);
		EETK4->set_qos(QOS);
		client.publish(EETK4)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK5 = mqtt::make_message(TOPIC5_5, EE_T_K_5);
		EETK5->set_qos(QOS);
		client.publish(EETK5)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK6 = mqtt::make_message(TOPIC5_6, EE_T_K_6);
		EETK6->set_qos(QOS);
		client.publish(EETK6)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK7= mqtt::make_message(TOPIC5_7, EE_T_K_7);
		EETK7->set_qos(QOS);
		client.publish(EETK7)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK8= mqtt::make_message(TOPIC5_8, EE_T_K_8);
		EETK8->set_qos(QOS);
		client.publish(EETK8)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK9= mqtt::make_message(TOPIC5_9, EE_T_K_9);
		EETK9->set_qos(QOS);
		client.publish(EETK9)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK10= mqtt::make_message(TOPIC5_10, EE_T_K_10);
		EETK10->set_qos(QOS);
		client.publish(EETK10)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK11 = mqtt::make_message(TOPIC5_11, EE_T_K_11);
		EETK11->set_qos(QOS);
		client.publish(EETK11)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK12 = mqtt::make_message(TOPIC5_12, EE_T_K_12);
		EETK12->set_qos(QOS);
		client.publish(EETK12)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK13= mqtt::make_message(TOPIC5_13, EE_T_K_13);
		EETK13->set_qos(QOS);
		client.publish(EETK13)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK14 = mqtt::make_message(TOPIC5_14, EE_T_K_14);
		EETK14->set_qos(QOS);
		client.publish(EETK14)->wait_for(TIMEOUT);	

	mqtt::message_ptr EETK15= mqtt::make_message(TOPIC5_15, EE_T_K_15);
		EETK15->set_qos(QOS);
		client.publish(EETK15)->wait_for(TIMEOUT);


	mqtt::message_ptr mee= mqtt::make_message(TOPIC4, m_ee_0);
		mee->set_qos(QOS);
		client.publish(mee)->wait_for(TIMEOUT);



	mqtt::message_ptr Iee0= mqtt::make_message(TOPIC10_0, I_ee_0);
		Iee0->set_qos(QOS);
		client.publish(Iee0)->wait_for(TIMEOUT);

	mqtt::message_ptr Iee1= mqtt::make_message(TOPIC10_1, I_ee_1);
		Iee1->set_qos(QOS);
		client.publish(Iee1)->wait_for(TIMEOUT);

	mqtt::message_ptr Iee2= mqtt::make_message(TOPIC10_2, I_ee_2);
		Iee2->set_qos(QOS);
		client.publish(Iee2)->wait_for(TIMEOUT);

	mqtt::message_ptr Iee3 = mqtt::make_message(TOPIC10_3, I_ee_3);
		Iee3->set_qos(QOS);
		client.publish(Iee3)->wait_for(TIMEOUT);

	mqtt::message_ptr Iee4= mqtt::make_message(TOPIC10_4, I_ee_4);
		Iee4->set_qos(QOS);
		client.publish(Iee4)->wait_for(TIMEOUT);

	mqtt::message_ptr Iee5= mqtt::make_message(TOPIC10_5, I_ee_5);
		Iee5->set_qos(QOS);
		client.publish(Iee5)->wait_for(TIMEOUT);

	mqtt::message_ptr Iee6= mqtt::make_message(TOPIC10_6, I_ee_6);
		Iee6->set_qos(QOS);
		client.publish(Iee6)->wait_for(TIMEOUT);

	mqtt::message_ptr Iee7= mqtt::make_message(TOPIC10_7, I_ee_7);
		Iee7->set_qos(QOS);
		client.publish(Iee7)->wait_for(TIMEOUT);

	mqtt::message_ptr Iee8= mqtt::make_message(TOPIC10_8, I_ee_8);
		Iee8->set_qos(QOS);
		client.publish(Iee8)->wait_for(TIMEOUT);		





    mqtt::message_ptr FxCee0= mqtt::make_message(TOPIC9_0, F_x_Cee_0);
		FxCee0->set_qos(QOS);
		client.publish(FxCee0)->wait_for(TIMEOUT); 

	mqtt::message_ptr FxCee1= mqtt::make_message(TOPIC9_1, F_x_Cee_1);
		FxCee1->set_qos(QOS);
		client.publish(FxCee1)->wait_for(TIMEOUT); 

	mqtt::message_ptr FxCee2= mqtt::make_message(TOPIC9_2, F_x_Cee_2);
		FxCee2->set_qos(QOS);
		client.publish(FxCee2)->wait_for(TIMEOUT); 	



	mqtt::message_ptr mload= mqtt::make_message(TOPIC6, m_load_0);
		mload->set_qos(QOS);
		client.publish(mload)->wait_for(TIMEOUT);





	mqtt::message_ptr Iload0= mqtt::make_message(TOPIC13_0, I_load_0);
		Iload0->set_qos(QOS);
		client.publish(Iload0)->wait_for(TIMEOUT); 
	
	mqtt::message_ptr Iload1= mqtt::make_message(TOPIC13_1, I_load_1);
		Iload1->set_qos(QOS);
		client.publish(Iload1)->wait_for(TIMEOUT); 

	mqtt::message_ptr Iload2 = mqtt::make_message(TOPIC13_2, I_load_2);
		Iload2->set_qos(QOS);
		client.publish(Iload2)->wait_for(TIMEOUT); 

	mqtt::message_ptr Iload3 = mqtt::make_message(TOPIC13_3, I_load_3);
		Iload3->set_qos(QOS);
		client.publish(Iload3)->wait_for(TIMEOUT); 

	mqtt::message_ptr Iload4 = mqtt::make_message(TOPIC13_4, I_load_4);
		Iload4->set_qos(QOS);
		client.publish(Iload4)->wait_for(TIMEOUT); 

	mqtt::message_ptr Iload5 = mqtt::make_message(TOPIC13_5, I_load_5);
		Iload5->set_qos(QOS);
		client.publish(Iload5)->wait_for(TIMEOUT); 

	mqtt::message_ptr Iload6 = mqtt::make_message(TOPIC13_6, I_load_6);
		Iload6->set_qos(QOS);
		client.publish(Iload6)->wait_for(TIMEOUT); 

	mqtt::message_ptr Iload7 = mqtt::make_message(TOPIC13_7, I_load_7);
		Iload7->set_qos(QOS);
		client.publish(Iload7)->wait_for(TIMEOUT); 

	mqtt::message_ptr Iload8= mqtt::make_message(TOPIC13_8, I_load_8);
		Iload8->set_qos(QOS);
		client.publish(Iload8)->wait_for(TIMEOUT); 






	mqtt::message_ptr FxCload0= mqtt::make_message(TOPIC12_0, F_x_Cload_0);
		FxCload0->set_qos(QOS);
		client.publish(FxCload0)->wait_for(TIMEOUT); 

	mqtt::message_ptr FxCload1= mqtt::make_message(TOPIC12_1, F_x_Cload_1);
		FxCload1->set_qos(QOS);
		client.publish(FxCload1)->wait_for(TIMEOUT); 

	mqtt::message_ptr FxCload2= mqtt::make_message(TOPIC12_2, F_x_Cload_2);
		FxCload2->set_qos(QOS);
		client.publish(FxCload2)->wait_for(TIMEOUT); 	




	mqtt::message_ptr mtotal= mqtt::make_message(TOPIC7, m_total_0);
		mtotal->set_qos(QOS);
		client.publish(mtotal)->wait_for(TIMEOUT);






	mqtt::message_ptr Itotal0= mqtt::make_message(TOPIC14_0, I_total_0);
		Itotal0->set_qos(QOS);
		client.publish(Itotal0)->wait_for(TIMEOUT); 

	mqtt::message_ptr Itotal1= mqtt::make_message(TOPIC14_1, I_total_1);
		Itotal1->set_qos(QOS);
		client.publish(Itotal1)->wait_for(TIMEOUT); 

	mqtt::message_ptr Itotal2= mqtt::make_message(TOPIC14_2, I_total_2);
		Itotal2->set_qos(QOS);
		client.publish(Itotal2)->wait_for(TIMEOUT); 

	mqtt::message_ptr Itotal3= mqtt::make_message(TOPIC14_3, I_total_3);
		Itotal3->set_qos(QOS);
		client.publish(Itotal3)->wait_for(TIMEOUT); 

	mqtt::message_ptr Itotal4= mqtt::make_message(TOPIC14_4, I_total_4);
		Itotal4->set_qos(QOS);
		client.publish(Itotal4)->wait_for(TIMEOUT); 

	mqtt::message_ptr Itotal5= mqtt::make_message(TOPIC14_5, I_total_5);
		Itotal5->set_qos(QOS);
		client.publish(Itotal5)->wait_for(TIMEOUT); 

	mqtt::message_ptr Itotal6= mqtt::make_message(TOPIC14_6, I_total_6);
		Itotal6->set_qos(QOS);
		client.publish(Itotal6)->wait_for(TIMEOUT); 

	mqtt::message_ptr Itotal7= mqtt::make_message(TOPIC14_7, I_total_7);
		Itotal7->set_qos(QOS);
		client.publish(Itotal7)->wait_for(TIMEOUT); 

	mqtt::message_ptr Itotal8= mqtt::make_message(TOPIC14_8, I_total_8);
		Itotal8->set_qos(QOS);
		client.publish(Itotal8)->wait_for(TIMEOUT); 			





	


	
	mqtt::message_ptr FxCtotal0= mqtt::make_message(TOPIC11_0, F_x_Ctotal_0);
		FxCtotal0->set_qos(QOS);
		client.publish(FxCtotal0)->wait_for(TIMEOUT); 

	mqtt::message_ptr FxCtotal1= mqtt::make_message(TOPIC11_1, F_x_Ctotal_1);
		FxCtotal1->set_qos(QOS);
		client.publish(FxCtotal1)->wait_for(TIMEOUT); 

	mqtt::message_ptr FxCtotal2= mqtt::make_message(TOPIC11_2, F_x_Ctotal_2);
		FxCtotal2->set_qos(QOS);
		client.publish(FxCtotal2)->wait_for(TIMEOUT); 	

							






	mqtt::message_ptr ELBOW0= mqtt::make_message(TOPIC8_0, elbow_0);
		ELBOW0->set_qos(QOS);
		client.publish(ELBOW0)->wait_for(TIMEOUT); 

	mqtt::message_ptr ELBOW1= mqtt::make_message(TOPIC8_1, elbow_1);
		ELBOW1->set_qos(QOS);
		client.publish(ELBOW1)->wait_for(TIMEOUT); 







	mqtt::message_ptr ELBOWD0= mqtt::make_message(TOPIC34_0, elbow_d_0);
		ELBOWD0->set_qos(QOS);
		client.publish(ELBOWD0)->wait_for(TIMEOUT); 

	mqtt::message_ptr ELBOWD1= mqtt::make_message(TOPIC34_1, elbow_d_1);
		ELBOWD1->set_qos(QOS);
		client.publish(ELBOWD1)->wait_for(TIMEOUT); 






	mqtt::message_ptr ELBOWC0= mqtt::make_message(TOPIC35_0, elbow_c_0);
		ELBOWC0->set_qos(QOS);
		client.publish(ELBOWC0)->wait_for(TIMEOUT); 

	mqtt::message_ptr ELBOWC1= mqtt::make_message(TOPIC35_1, elbow_c_1);
		ELBOWC1->set_qos(QOS);
		client.publish(ELBOWC1)->wait_for(TIMEOUT); 
    



	mqtt::message_ptr DELBOWC0= mqtt::make_message(TOPIC36_0, delbow_c_0);
		DELBOWC0->set_qos(QOS);
		client.publish(DELBOWC0)->wait_for(TIMEOUT); 

	mqtt::message_ptr DELBOWC1= mqtt::make_message(TOPIC36_1, delbow_c_1);
		DELBOWC1->set_qos(QOS);
		client.publish(DELBOWC1)->wait_for(TIMEOUT); 



    mqtt::message_ptr DDELBOWC0= mqtt::make_message(TOPIC37_0, ddelbow_c_0);
		DDELBOWC0->set_qos(QOS);
		client.publish(DDELBOWC0)->wait_for(TIMEOUT); 

	mqtt::message_ptr DDELBOWC1= mqtt::make_message(TOPIC37_1, ddelbow_c_1);
		DDELBOWC1->set_qos(QOS);
		client.publish(DDELBOWC1)->wait_for(TIMEOUT); 
	


	mqtt::message_ptr TAUJD0= mqtt::make_message(TOPIC17_0, tau_J_d_0);
		TAUJD0->set_qos(QOS);
		client.publish(TAUJD0)->wait_for(TIMEOUT); 

	mqtt::message_ptr TAUJD1= mqtt::make_message(TOPIC17_1, tau_J_d_1);
		TAUJD1->set_qos(QOS);
		client.publish(TAUJD1)->wait_for(TIMEOUT); 

	mqtt::message_ptr TAUJD2= mqtt::make_message(TOPIC17_2, tau_J_d_2);
		TAUJD2->set_qos(QOS);
		client.publish(TAUJD2)->wait_for(TIMEOUT); 

	mqtt::message_ptr TAUJD3= mqtt::make_message(TOPIC17_3, tau_J_d_3);
		TAUJD3->set_qos(QOS);
		client.publish(TAUJD3)->wait_for(TIMEOUT); 

	mqtt::message_ptr TAUJD4= mqtt::make_message(TOPIC17_4, tau_J_d_4);
		TAUJD4->set_qos(QOS);
		client.publish(TAUJD4)->wait_for(TIMEOUT); 

	mqtt::message_ptr TAUJD5= mqtt::make_message(TOPIC17_5, tau_J_d_5);
		TAUJD5->set_qos(QOS);
		client.publish(TAUJD5)->wait_for(TIMEOUT); 

	mqtt::message_ptr TAUJD6= mqtt::make_message(TOPIC17_6, tau_J_d_6);
		TAUJD6->set_qos(QOS);
		client.publish(TAUJD6)->wait_for(TIMEOUT); 
	
	  
	  
	  
	  
	  
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
