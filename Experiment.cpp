
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
const string PERSIST_DIR			{ "./persist" };

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
array< double, 7 > dtau_J = robot_state.dtau_J;
array< double, 7 > q = robot_state.q; 
array< double, 7 > q_d = robot_state.q_d;
array< double, 7 > dq = robot_state.dq;
array< double, 7 > dq_d = robot_state.dq_d;
array< double, 7 > ddq_d = robot_state.ddq_d;
array< double, 7 > joint_contact = robot_state.joint_contact;
array< double, 6 > cartesian_contact = robot_state.cartesian_contact;
array< double, 7 > joint_collision = robot_state.joint_collision;
array< double, 6 > cartesian_collision = robot_state.cartesian_collision;
array< double, 7 > tau_ext_hat_filtered = robot_state.tau_ext_hat_filtered;
array< double, 6 > O_F_ext_hat_K = robot_state.O_F_ext_hat_K;
array< double, 6 > K_F_ext_hat_K = robot_state.K_F_ext_hat_K;
array< double, 6 > O_dP_EE_d = robot_state.O_dP_EE_d;
array< double, 16 > O_T_EE_c = robot_state.O_T_EE_c;
array< double, 6 > O_dP_EE_c = robot_state.O_dP_EE_c;
array< double, 6 > O_ddP_EE_c = robot_state.O_ddP_EE_c;
array< double, 7 > theta = robot_state.theta;
array< double, 7 > dtheta = robot_state.dtheta;
array< double, 7 > tau_J = robot_state.tau_J;


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

double dtau_J0 = dtau_J.data()[0];
double dtau_J1 = dtau_J.data()[1];
double dtau_J2 = dtau_J.data()[2];
double dtau_J3 = dtau_J.data()[3];
double dtau_J4 = dtau_J.data()[4];
double dtau_J5 = dtau_J.data()[5];
double dtau_J6 = dtau_J.data()[6];

double q0 = q.data()[0];
double q1 = q.data()[1];
double q2 = q.data()[2];
double q3 = q.data()[3];
double q4 = q.data()[4];
double q5 = q.data()[5];
double q6 = q.data()[6];

double q_d0 = q_d.data()[0];
double q_d1 = q_d.data()[1];
double q_d2 = q_d.data()[2];
double q_d3 = q_d.data()[3];
double q_d4 = q_d.data()[4];
double q_d5 = q_d.data()[5];
double q_d6 = q_d.data()[6];

double dq0 = dq.data()[0];
double dq1 = dq.data()[1];
double dq2 = dq.data()[2];
double dq3 = dq.data()[3];
double dq4 = dq.data()[4];
double dq5 = dq.data()[5];
double dq6 = dq.data()[6];

double dq_d0 = dq_d.data()[0];
double dq_d1 = dq_d.data()[1];
double dq_d2 = dq_d.data()[2];
double dq_d3 = dq_d.data()[3];
double dq_d4 = dq_d.data()[4];
double dq_d5 = dq_d.data()[5];
double dq_d6 = dq_d.data()[6];

double ddq_d0 = ddq_d.data()[0];
double ddq_d1 = ddq_d.data()[1];
double ddq_d2 = ddq_d.data()[2];
double ddq_d3 = ddq_d.data()[3];
double ddq_d4 = ddq_d.data()[4];
double ddq_d5 = ddq_d.data()[5];
double ddq_d6 = ddq_d.data()[6];

double joint_contact0 = joint_contact.data()[0];
double joint_contact1 = joint_contact.data()[1];
double joint_contact2 = joint_contact.data()[2];
double joint_contact3 = joint_contact.data()[3];
double joint_contact4 = joint_contact.data()[4];
double joint_contact5 = joint_contact.data()[5];
double joint_contact6 = joint_contact.data()[6];

double cartesian_contact0 = cartesian_contact.data()[0];
double cartesian_contact1 = cartesian_contact.data()[1];
double cartesian_contact2 = cartesian_contact.data()[2];
double cartesian_contact3 = cartesian_contact.data()[3];
double cartesian_contact4 = cartesian_contact.data()[4];
double cartesian_contact5 = cartesian_contact.data()[5];

double joint_collision0 = joint_collision.data()[0];
double joint_collision1 = joint_collision.data()[1];
double joint_collision2 = joint_collision.data()[2];
double joint_collision3 = joint_collision.data()[3];
double joint_collision4 = joint_collision.data()[4];
double joint_collision5 = joint_collision.data()[5];
double joint_collision6 = joint_collision.data()[6];

double cartesian_collision0 = cartesian_collision.data()[0];
double cartesian_collision1 = cartesian_collision.data()[1];
double cartesian_collision2 = cartesian_collision.data()[2];
double cartesian_collision3 = cartesian_collision.data()[3];
double cartesian_collision4 = cartesian_collision.data()[4];
double cartesian_collision5 = cartesian_collision.data()[5];

double tau_ext_hat_filtered0 = tau_ext_hat_filtered.data()[0];
double tau_ext_hat_filtered1 = tau_ext_hat_filtered.data()[1];
double tau_ext_hat_filtered2 = tau_ext_hat_filtered.data()[2];
double tau_ext_hat_filtered3 = tau_ext_hat_filtered.data()[3];
double tau_ext_hat_filtered4 = tau_ext_hat_filtered.data()[4];
double tau_ext_hat_filtered5 = tau_ext_hat_filtered.data()[5];
double tau_ext_hat_filtered6 = tau_ext_hat_filtered.data()[6];

double O_F_ext_hat_K0 = O_F_ext_hat_K.data()[0];
double O_F_ext_hat_K1 = O_F_ext_hat_K.data()[1];
double O_F_ext_hat_K2 = O_F_ext_hat_K.data()[2];
double O_F_ext_hat_K3 = O_F_ext_hat_K.data()[3];
double O_F_ext_hat_K4 = O_F_ext_hat_K.data()[4];
double O_F_ext_hat_K5 = O_F_ext_hat_K.data()[5];

double K_F_ext_hat_K0 = K_F_ext_hat_K.data()[0];
double K_F_ext_hat_K1 = K_F_ext_hat_K.data()[1];
double K_F_ext_hat_K2 = K_F_ext_hat_K.data()[2];
double K_F_ext_hat_K3 = K_F_ext_hat_K.data()[3];
double K_F_ext_hat_K4 = K_F_ext_hat_K.data()[4];
double K_F_ext_hat_K5 = K_F_ext_hat_K.data()[5];

double O_dP_EE_d0 = O_dP_EE_d.data()[0];
double O_dP_EE_d1 = O_dP_EE_d.data()[1];
double O_dP_EE_d2 = O_dP_EE_d.data()[2];
double O_dP_EE_d3 = O_dP_EE_d.data()[3];
double O_dP_EE_d4 = O_dP_EE_d.data()[4];
double O_dP_EE_d5 = O_dP_EE_d.data()[5];

double O_T_EE_c0 = O_T_EE_c.data()[0];
double O_T_EE_c1 = O_T_EE_c.data()[1];
double O_T_EE_c2 = O_T_EE_c.data()[2];
double O_T_EE_c3 = O_T_EE_c.data()[3];
double O_T_EE_c4 = O_T_EE_c.data()[4];
double O_T_EE_c5 = O_T_EE_c.data()[5];
double O_T_EE_c6 = O_T_EE_c.data()[6];
double O_T_EE_c7 = O_T_EE_c.data()[7];
double O_T_EE_c8 = O_T_EE_c.data()[8];
double O_T_EE_c9 = O_T_EE_c.data()[9];
double O_T_EE_c10 = O_T_EE_c.data()[10];
double O_T_EE_c11 = O_T_EE_c.data()[11];
double O_T_EE_c12 = O_T_EE_c.data()[12];
double O_T_EE_c13 = O_T_EE_c.data()[13];
double O_T_EE_c14 = O_T_EE_c.data()[14];
double O_T_EE_c15 = O_T_EE_c.data()[15];

double O_dP_EE_c0 = O_dP_EE_c.data()[0];
double O_dP_EE_c1 = O_dP_EE_c.data()[1];
double O_dP_EE_c2 = O_dP_EE_c.data()[2];
double O_dP_EE_c3 = O_dP_EE_c.data()[3];
double O_dP_EE_c4 = O_dP_EE_c.data()[4];
double O_dP_EE_c5 = O_dP_EE_c.data()[5];
double O_dP_EE_c6 = O_dP_EE_c.data()[6];
double O_dP_EE_c7 = O_dP_EE_c.data()[7];
double O_dP_EE_c8 = O_dP_EE_c.data()[8];
double O_dP_EE_c9 = O_dP_EE_c.data()[9];
double O_dP_EE_c10 = O_dP_EE_c.data()[10];
double O_dP_EE_c11 = O_dP_EE_c.data()[11];
double O_dP_EE_c12 = O_dP_EE_c.data()[12];
double O_dP_EE_c13 = O_dP_EE_c.data()[13];
double O_dP_EE_c14 = O_dP_EE_c.data()[14];
double O_dP_EE_c15 = O_dP_EE_c.data()[15];

double O_ddP_EE_c0 = O_ddP_EE_c.data()[0];
double O_ddP_EE_c1 = O_ddP_EE_c.data()[1];
double O_ddP_EE_c2 = O_ddP_EE_c.data()[2];
double O_ddP_EE_c3 = O_ddP_EE_c.data()[3];
double O_ddP_EE_c4 = O_ddP_EE_c.data()[4];
double O_ddP_EE_c5 = O_ddP_EE_c.data()[5];
double O_ddP_EE_c6 = O_ddP_EE_c.data()[6];
double O_ddP_EE_c7 = O_ddP_EE_c.data()[7];
double O_ddP_EE_c8 = O_ddP_EE_c.data()[8];
double O_ddP_EE_c9 = O_ddP_EE_c.data()[9];
double O_ddP_EE_c10 = O_ddP_EE_c.data()[10];
double O_ddP_EE_c11 = O_ddP_EE_c.data()[11];
double O_ddP_EE_c12 = O_ddP_EE_c.data()[12];
double O_ddP_EE_c13 = O_ddP_EE_c.data()[13];
double O_ddP_EE_c14 = O_ddP_EE_c.data()[14];
double O_ddP_EE_c15 = O_ddP_EE_c.data()[15];

double tau_J0 = tau_J.data()[0];
double tau_J1 = tau_J.data()[1];
double tau_J2 = tau_J.data()[2];
double tau_J3 = tau_J.data()[3];
double tau_J4 = tau_J.data()[4];
double tau_J5 = tau_J.data()[5];
double tau_J6 = tau_J.data()[6];

double theta0 = theta.data()[0];
double theta1 = theta.data()[1];
double theta2 = theta.data()[2];
double theta3 = theta.data()[3];
double theta4 = theta.data()[4];
double theta5 = theta.data()[5];
double theta6 = theta.data()[6];

double dtheta0 = dtheta.data()[0];
double dtheta1 = dtheta.data()[1];
double dtheta2 = dtheta.data()[2];
double dtheta3 = dtheta.data()[3];
double dtheta4 = dtheta.data()[4];
double dtheta5 = dtheta.data()[5];
double dtheta6 = dtheta.data()[6];

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
std::string O_T_EE__d_1 = std::to_string(O_T_EE_d1);
std::string O_T_EE__d_2 = std::to_string(O_T_EE_d2);
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

std::string I_load_0 = std::to_string(I_load0);
std::string I_load_1 = std::to_string(I_load1);
std::string I_load_2 = std::to_string(I_load2);
std::string I_load_3 = std::to_string(I_load3);
std::string I_load_4 = std::to_string(I_load4);
std::string I_load_5 = std::to_string(I_load5);
std::string I_load_6 = std::to_string(I_load6);
std::string I_load_7 = std::to_string(I_load7);
std::string I_load_8 = std::to_string(I_load8);

const int  QOS = 1;



const auto TIMEOUT = std::chrono::seconds(10);


int main(int argc, char** argv) {
  string	address  = (argc > 1) ? string(argv[2]) : DFLT_SERVER_ADDRESS,
			clientID = (argc > 2) ? string(argv[3]) : CLIENT_ID;

	mqtt::async_client client(address, clientID, PERSIST_DIR);
	

	auto connOpts = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
		.finalize();
  
  
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  
  try {
		
		mqtt::token_ptr conntok = client.connect(connOpts);
		
		conntok->wait();
		
    for(;;){
		
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

    }
	}
	catch (const mqtt::exception& exc) {
		cerr << exc.what() << endl;
		return 1;
	}

  return 0;

}
