#include <iostream>
#include <stdlib.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "HTTPRequest.hpp"

struct franka::RobotState robot_state;

using namespace std;

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

	







int main(int argc, char** argv) {
  franka::Robot robot(argv[1]);
  
  try {
		

    //for(;;){


	
    
	
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
    robot.read([&count, &O_T_EE, &O_T_EE_d, &F_T_EE, &F_T_NE, &NE_T_EE, &EE_T_K, &m_ee, &I_ee, &F_x_Cee, &m_load, &I_load, &F_x_Cload, &m_total, &I_total, &F_x_Ctotal, &elbow, &elbow_d, &elbow_c, &delbow_c, &ddelbow_c, &tau_J_d](const franka::RobotState& robot_state) {
      

      using namespace std;

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
	  


    try
    {

         http::Request request{"http://localhost:2000"};
         const string OTEE0 = O_T_EE_0;
         const auto response = request.send("POST", OTEE0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request2{"http://localhost:2000"};
         const string OTEE1 = O_T_EE_1;
         const auto response2 = request2.send("POST", OTEE1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request3{"http://localhost:2000"};
         const string OTEE2 = O_T_EE_2;
         const auto response3 = request3.send("POST", OTEE2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request4{"http://localhost:2000"};
         const string OTEE3 = O_T_EE_3;
         const auto response4 = request4.send("POST", OTEE3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request5{"http://localhost:2000"};
         const string OTEE4 = O_T_EE_4;
         const auto response5 = request5.send("POST", OTEE4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request6{"http://localhost:2000"};
         const string OTEE5 = O_T_EE_5;
         const auto response6 = request6.send("POST", OTEE5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request7{"http://localhost:2000"};
         const string OTEE6 = O_T_EE_6;
         const auto response7 = request7.send("POST", OTEE6, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request8{"http://localhost:2000"};
         const string OTEE7 = O_T_EE_7;
         const auto response8 = request8.send("POST", OTEE7, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request9{"http://localhost:2000"};
         const string OTEE8 = O_T_EE_8;
         const auto response9 = request9.send("POST", OTEE8, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		http::Request request10{"http://localhost:2000"};
         const string OTEE9 = O_T_EE_9;
         const auto response10 = request10.send("POST", OTEE9, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request11{"http://localhost:2000"};
         const string OTEE10 = O_T_EE_10;
         const auto response11 = request11.send("POST", OTEE10, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request12{"http://localhost:2000"};
         const string OTEE11 = O_T_EE_11;
         const auto response12 = request12.send("POST", OTEE11, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request13{"http://localhost:2000"};
         const string OTEE12 = O_T_EE_12;
         const auto response13 = request13.send("POST", OTEE12, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request14{"http://localhost:2000"};
         const string OTEE13 = O_T_EE_13;
         const auto response14 = request14.send("POST", OTEE13, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request15{"http://localhost:2000"};
         const string OTEE14 = O_T_EE_14;
         const auto response15 = request15.send("POST", OTEE14, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request16{"http://localhost:2000"};
         const string OTEE15 = O_T_EE_15;
         const auto response16 = request16.send("POST", OTEE15, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });

         
		 
		 
		 http::Request request17{"http://localhost:2000"};
         const string OTEEd0 = O_T_EE_d_0;
         const auto response17 = request17.send("POST", OTEEd0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request18{"http://localhost:2000"};
         const string OTEEd1 = O_T_EE_d_1;
         const auto response18 = request18.send("POST", OTEEd1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request19{"http://localhost:2000"};
         const string OTEEd2 = O_T_EE_d_2;
         const auto response19 = request19.send("POST", OTEEd2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request20{"http://localhost:2000"};
         const string OTEEd3 = O_T_EE_d_3;
         const auto response20 = request20.send("POST", OTEEd3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request21{"http://localhost:2000"};
         const string OTEEd4 = O_T_EE_d_4;
         const auto response21 = request21.send("POST", OTEEd4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request22{"http://localhost:2000"};
         const string OTEEd5 = O_T_EE_d_5;
         const auto response22 = request22.send("POST", OTEEd5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request23{"http://localhost:2000"};
         const string OTEEd6 = O_T_EE_d_6;
         const auto response23 = request23.send("POST", OTEEd6, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request24{"http://localhost:2000"};
         const string OTEEd7 = O_T_EE_d_7;
         const auto response24 = request24.send("POST", OTEEd7, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request25{"http://localhost:2000"};
         const string OTEEd8 = O_T_EE_d_8;
         const auto response25 = request25.send("POST", OTEEd8, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		http::Request request26{"http://localhost:2000"};
         const string OTEEd9 = O_T_EE_d_9;
         const auto response26 = request26.send("POST", OTEEd9, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request27{"http://localhost:2000"};
         const string OTEEd10 = O_T_EE_d_10;
         const auto response27 = request27.send("POST", OTEEd10, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request28{"http://localhost:2000"};
         const string OTEEd11 = O_T_EE_d_11;
         const auto response28 = request28.send("POST", OTEEd11, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request29{"http://localhost:2000"};
         const string OTEEd12 = O_T_EE_d_12;
         const auto response29 = request29.send("POST", OTEEd12, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request30{"http://localhost:2000"};
         const string OTEEd13 = O_T_EE_d_13;
         const auto response30 = request30.send("POST", OTEEd13, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request31{"http://localhost:2000"};
         const string OTEEd14 = O_T_EE_d_14;
         const auto response31 = request31.send("POST", OTEEd14, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request32{"http://localhost:2000"};
         const string OTEEd15 = O_T_EE_d_15;
         const auto response32 = request32.send("POST", OTEEd15, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });


         http::Request request33{"http://localhost:2000"};
         const string FTEE0 = F_T_EE_0;
         const auto response33 = request33.send("POST", FTEE0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request34{"http://localhost:2000"};
         const string FTEE1 = F_T_EE_1;
         const auto response34 = request34.send("POST", FTEE1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request35{"http://localhost:2000"};
         const string FTEE2 = F_T_EE_2;
         const auto response35 = request35.send("POST", FTEE2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request36{"http://localhost:2000"};
         const string FTEE3 = F_T_EE_3;
         const auto response36 = request36.send("POST", FTEE3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request37{"http://localhost:2000"};
         const string FTEE4 = F_T_EE_4;
         const auto response37 = request37.send("POST", FTEE4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request38{"http://localhost:2000"};
         const string FTEE5 = F_T_EE_5;
         const auto response38 = request38.send("POST", FTEE5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request39{"http://localhost:2000"};
         const string FTEE6 = F_T_EE_6;
         const auto response39 = request39.send("POST", FTEE6, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request40{"http://localhost:2000"};
         const string FTEE7 = F_T_EE_7;
         const auto response40 = request40.send("POST", FTEE7, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request41{"http://localhost:2000"};
         const string FTEE8 = F_T_EE_8;
         const auto response41 = request41.send("POST", FTEE8, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		http::Request request42{"http://localhost:2000"};
         const string FTEE9 = F_T_EE_9;
         const auto response42 = request42.send("POST", FTEE9, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request43{"http://localhost:2000"};
         const string FTEE10 = F_T_EE_10;
         const auto response43 = request43.send("POST", FTEE10, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request44{"http://localhost:2000"};
         const string FTEE11 = F_T_EE_11;
         const auto response44 = request44.send("POST", FTEE11, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request45{"http://localhost:2000"};
         const string FTEE12 = F_T_EE_12;
         const auto response45 = request45.send("POST", FTEE12, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request46{"http://localhost:2000"};
         const string FTEE13 = F_T_EE_13;
         const auto response46 = request46.send("POST", FTEE13, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request47{"http://localhost:2000"};
         const string FTEE14 = F_T_EE_14;
         const auto response47 = request47.send("POST", FTEE14, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request48{"http://localhost:2000"};
         const string FTEE15 = F_T_EE_15;
         const auto response48 = request48.send("POST", FTEE15, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });


         http::Request request49{"http://localhost:2000"};
         const string FTNE0 = F_T_NE_0;
         const auto response49 = request49.send("POST", FTNE0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request50{"http://localhost:2000"};
         const string FTNE1 = F_T_NE_1;
         const auto response50 = request50.send("POST", FTNE1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request51{"http://localhost:2000"};
         const string FTNE2 = F_T_NE_2;
         const auto response51 = request51.send("POST", FTNE2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request52{"http://localhost:2000"};
         const string FTNE3 = F_T_NE_3;
         const auto response52 = request52.send("POST", FTNE3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request53{"http://localhost:2000"};
         const string FTNE4 = F_T_NE_4;
         const auto response53 = request53.send("POST", FTNE4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request54{"http://localhost:2000"};
         const string FTNE5 = F_T_NE_5;
         const auto response54 = request54.send("POST", FTNE5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request55{"http://localhost:2000"};
         const string FTNE6 = F_T_NE_6;
         const auto response55 = request55.send("POST", FTNE6, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request56{"http://localhost:2000"};
         const string FTNE7 = F_T_NE_7;
         const auto response56 = request56.send("POST", FTNE7, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request57{"http://localhost:2000"};
         const string FTNE8 = F_T_NE_8;
         const auto response57 = request57.send("POST", FTNE8, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		http::Request request58{"http://localhost:2000"};
         const string FTNE9 = F_T_NE_9;
         const auto response58 = request58.send("POST", FTNE9, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request59{"http://localhost:2000"};
         const string FTNE10 = F_T_NE_10;
         const auto response59 = request59.send("POST", FTNE10, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request60{"http://localhost:2000"};
         const string FTNE11 = F_T_NE_11;
         const auto response60 = request60.send("POST", FTNE11, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request61{"http://localhost:2000"};
         const string FTNE12 = F_T_NE_12;
         const auto response61 = request61.send("POST", FTNE12, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request62{"http://localhost:2000"};
         const string FTNE13 = F_T_NE_13;
         const auto response62 = request62.send("POST", FTNE13, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request63{"http://localhost:2000"};
         const string FTNE14 = F_T_NE_14;
         const auto response63 = request63.send("POST", FTNE14, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request64{"http://localhost:2000"};
         const string FTNE15 = F_T_NE_15;
         const auto response64 = request64.send("POST", FTNE15, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });

         
		 
		 
		 
		 
		 http::Request request65{"http://localhost:2000"};
         const string NETEE0 = NE_T_EE_0;
         const auto response65 = request65.send("POST", NETEE0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request66{"http://localhost:2000"};
         const string NETEE1 = NE_T_EE_1;
         const auto response66 = request66.send("POST", NETEE1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request67{"http://localhost:2000"};
         const string NETEE2 = NE_T_EE_2;
         const auto response67 = request67.send("POST", NETEE2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request68{"http://localhost:2000"};
         const string NETEE3 = NE_T_EE_3;
         const auto response68 = request68.send("POST", NETEE3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request69{"http://localhost:2000"};
         const string NETEE4 = NE_T_EE_4;
         const auto response69 = request69.send("POST", NETEE4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request70{"http://localhost:2000"};
         const string NETEE5 = NE_T_EE_5;
         const auto response70 = request70.send("POST", NETEE5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request71{"http://localhost:2000"};
         const string NETEE6 = NE_T_EE_6;
         const auto response71 = request71.send("POST", NETEE6, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request72{"http://localhost:2000"};
         const string NETEE7 = NE_T_EE_7;
         const auto response72 = request72.send("POST", NETEE7, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request73{"http://localhost:2000"};
         const string NETEE8 = NE_T_EE_8;
         const auto response73 = request73.send("POST", NETEE8, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		http::Request request74{"http://localhost:2000"};
         const string NETEE9 = NE_T_EE_9;
         const auto response74 = request74.send("POST", NETEE9, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request75{"http://localhost:2000"};
         const string NETEE10 = NE_T_EE_10;
         const auto response75 = request75.send("POST", NETEE10, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request76{"http://localhost:2000"};
         const string NETEE11 = NE_T_EE_11;
         const auto response76 = request76.send("POST", NETEE11, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request77{"http://localhost:2000"};
         const string NETEE12 = NE_T_EE_12;
         const auto response77 = request77.send("POST", NETEE12, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request78{"http://localhost:2000"};
         const string NETEE13 = NE_T_EE_13;
         const auto response78 = request78.send("POST", NETEE13, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request79{"http://localhost:2000"};
         const string NETEE14 = NE_T_EE_14;
         const auto response79 = request79.send("POST", NETEE14, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  http::Request request80{"http://localhost:2000"};
         const string NETEE15 = NE_T_EE_15;
         const auto response80 = request80.send("POST", NETEE15, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		  
		  
		  
		  
		  http::Request request81{"http://localhost:2000"};
         const string EETK0 = EE_T_K_0;
         const auto response81 = request81.send("POST", EETK0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
         http::Request request82{"http://localhost:2000"};
         const string EETK1 = EE_T_K_1;
         const auto response82 = request82.send("POST", EETK1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request83{"http://localhost:2000"};
         const string EETK2 = EE_T_K_2;
         const auto response83 = request83.send("POST", EETK2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request84{"http://localhost:2000"};
         const string EETK3 = EE_T_K_3;
         const auto response84 = request84.send("POST", EETK3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
         http::Request request85{"http://localhost:2000"};
         const string EETK4 = EE_T_K_4;
         const auto response85 = request85.send("POST", EETK4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		http::Request request86{"http://localhost:2000"};
         const string EETK5 = EE_T_K_5;
         const auto response86 = request86.send("POST", EETK5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request87{"http://localhost:2000"};
         const string EETK6 = EE_T_K_6;
         const auto response87 = request87.send("POST", EETK6, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request88{"http://localhost:2000"};
         const string EETK7 = EE_T_K_7;
         const auto response88 = request88.send("POST", EETK7, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request89{"http://localhost:2000"};
         const string EETK8 = EE_T_K_8;
         const auto response89 = request89.send("POST", EETK8, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request90{"http://localhost:2000"};
         const string EETK9 = EE_T_K_9;
         const auto response90 = request90.send("POST", EETK9, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request92{"http://localhost:2000"};
         const string EETK10 = EE_T_K_10;
         const auto response92 = request92.send("POST", EETK10, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request93{"http://localhost:2000"};
         const string EETK11 = EE_T_K_11;
         const auto response93 = request93.send("POST", EETK11, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request94{"http://localhost:2000"};
         const string EETK12 = EE_T_K_12;
         const auto response94 = request94.send("POST", EETK12, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request95{"http://localhost:2000"};
         const string EETK13 = EE_T_K_13;
         const auto response95 = request95.send("POST", EETK13, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request96{"http://localhost:2000"};
         const string EETK14 = EE_T_K_14;
         const auto response96 = request96.send("POST", EETK14, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request97{"http://localhost:2000"};
         const string EETK15 = EE_T_K_15;
         const auto response97 = request97.send("POST", EETK15, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });



		 http::Request request291{"http://localhost:2000"};
         const string MEE0= m_ee_0;
         const auto response291 = request291.send("POST", MEE0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });



		 http::Request request98{"http://localhost:2000"};
         const string IEE0 = I_ee_0;
         const auto response98 = request98.send("POST", IEE0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request99{"http://localhost:2000"};
         const string IEE1 = I_ee_1;
         const auto response99 = request99.send("POST", IEE1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request100{"http://localhost:2000"};
         const string IEE2 = I_ee_2;
         const auto response100 = request100.send("POST", IEE2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request101{"http://localhost:2000"};
         const string IEE3 = I_ee_3;
         const auto response101 = request101.send("POST", IEE3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request102{"http://localhost:2000"};
         const string IEE4 = I_ee_4;
         const auto response102 = request102.send("POST", IEE4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request103{"http://localhost:2000"};
         const string IEE5 = I_ee_5;
         const auto response103 = request103.send("POST", IEE5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request104{"http://localhost:2000"};
         const string IEE6 = I_ee_6;
         const auto response104 = request104.send("POST", IEE6, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request105{"http://localhost:2000"};
         const string IEE7 = I_ee_7;
         const auto response105 = request105.send("POST", IEE7, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request106{"http://localhost:2000"};
         const string IEE8 = I_ee_8;
         const auto response106 = request106.send("POST", IEE8, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });







		 http::Request request107{"http://localhost:2000"};
         const string FXCEE0 = F_x_Cee_0;
         const auto response107 = request107.send("POST", FXCEE0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request108{"http://localhost:2000"};
         const string FXCEE1 = F_x_Cee_1;
         const auto response108 = request108.send("POST", FXCEE1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request109{"http://localhost:2000"};
         const string FXCEE2 = F_x_Cee_2;
         const auto response109 = request109.send("POST", FXCEE2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 



		 http::Request request292{"http://localhost:2000"};
         const string MLOAD= m_load_0;
         const auto response292 = request292.send("POST", MLOAD, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });




		 http::Request request108s{"http://localhost:2000"};
         const string ILOAD0 = I_load_0;
         const auto response108s = request108s.send("POST", ILOAD0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request109s{"http://localhost:2000"};
         const string ILOAD1 = I_load_1;
         const auto response109s = request109s.send("POST", ILOAD1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request110s{"http://localhost:2000"};
         const string ILOAD2 = I_load_2;
         const auto response110s = request110s.send("POST", ILOAD2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request111{"http://localhost:2000"};
         const string ILOAD3 = I_load_3;
         const auto response111 = request111.send("POST", ILOAD3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request112{"http://localhost:2000"};
         const string ILOAD4 = I_load_4;
         const auto response112 = request112.send("POST", ILOAD4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request113{"http://localhost:2000"};
         const string ILOAD5 = I_load_5;
         const auto response113 = request113.send("POST", ILOAD5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request114{"http://localhost:2000"};
         const string ILOAD6 = I_load_6;
         const auto response114 = request114.send("POST", ILOAD6, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request115{"http://localhost:2000"};
         const string ILOAD7 = I_load_7;
         const auto response115 = request115.send("POST", ILOAD7, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request116{"http://localhost:2000"};
         const string ILOAD8 = I_load_8;
         const auto response116 = request116.send("POST", ILOAD8, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 



		
		



		 http::Request request117{"http://localhost:2000"};
         const string FXCLOAD0 = F_x_Cload_0;
         const auto response117 = request117.send("POST", FXCLOAD0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request118{"http://localhost:2000"};
         const string FXCLOAD1 = F_x_Cload_1;
         const auto response118 = request118.send("POST", FXCLOAD1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request119{"http://localhost:2000"};
         const string FXCLOAD2 = F_x_Cload_2;
         const auto response119 = request119.send("POST", FXCLOAD2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 


		 http::Request request293{"http://localhost:2000"};
         const string MTOTAL= m_total_0;
         const auto response293 = request293.send("POST", MTOTAL, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });



		 http::Request request120{"http://localhost:2000"};
         const string ITOTAL0 = I_total_0;
         const auto response120 = request120.send("POST", ITOTAL0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request121{"http://localhost:2000"};
         const string ITOTAL1 = I_total_1;
         const auto response121 = request121.send("POST", ITOTAL1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request122{"http://localhost:2000"};
         const string ITOTAL2 = I_total_2;
         const auto response122 = request122.send("POST", ITOTAL2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request123{"http://localhost:2000"};
         const string ITOTAL3 = I_total_3;
         const auto response123 = request123.send("POST", ITOTAL3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request124{"http://localhost:2000"};
         const string ITOTAL4 = I_total_4;
         const auto response124 = request124.send("POST", ITOTAL4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request125{"http://localhost:2000"};
         const string ITOTAL5 = I_total_5;
         const auto response125 = request125.send("POST", ITOTAL5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request126{"http://localhost:2000"};
         const string ITOTAL6 = I_total_6;
         const auto response126 = request126.send("POST", ITOTAL6, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request127{"http://localhost:2000"};
         const string ITOTAL7 = I_total_7;
         const auto response127 = request127.send("POST", ITOTAL7, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request128{"http://localhost:2000"};
         const string ITOTAL8 = I_total_8;
         const auto response128 = request128.send("POST", ITOTAL8, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 








		 http::Request request129{"http://localhost:2000"};
         const string FXCTOTAL0 = F_x_Ctotal_0;
         const auto response129 = request129.send("POST", FXCTOTAL0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request130{"http://localhost:2000"};
         const string FXCTOTAL1 = F_x_Ctotal_1;
         const auto response130 = request130.send("POST", FXCTOTAL1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request131{"http://localhost:2000"};
         const string FXCTOTAL2 = F_x_Ctotal_2;
         const auto response131 = request131.send("POST", FXCTOTAL2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 










		 http::Request request132{"http://localhost:2000"};
         const string ELBOW0 = elbow_0;
         const auto response132 = request132.send("POST", ELBOW0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request133{"http://localhost:2000"};
         const string ELBOW1 = elbow_1;
         const auto response133 = request133.send("POST", ELBOW1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 




		 http::Request request134{"http://localhost:2000"};
         const string ELBOWD0 = elbow_d_0;
         const auto response134 = request134.send("POST", ELBOWD0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request135{"http://localhost:2000"};
         const string ELBOWD1 = elbow_d_1;
         const auto response135 = request135.send("POST", ELBOWD1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 




		 http::Request request136{"http://localhost:2000"};
         const string ELBOWC0 = elbow_c_0;
         const auto response136 = request136.send("POST", ELBOWC0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request137{"http://localhost:2000"};
         const string ELBOWC1 = elbow_c_1;
         const auto response137 = request137.send("POST", ELBOWC1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 





		 http::Request request138{"http://localhost:2000"};
         const string DELBOWC0 = delbow_c_0;
         const auto response138 = request138.send("POST", DELBOWC0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		 http::Request request139{"http://localhost:2000"};
         const string DELBOWC1 = delbow_c_1;
         const auto response139 = request139.send("POST", DELBOWC1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 





		 http::Request request140{"http://localhost:2000"};
         const string DDELBOWC0 = ddelbow_c_0;
         const auto response140 = request140.send("POST", DDELBOWC0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 
		  http::Request request141{"http://localhost:2000"};
         const string DDELBOWC1 = ddelbow_c_1;
         const auto response141 = request141.send("POST", DDELBOWC1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 }); 

		

		 http::Request request156{"http://localhost:2000"};
         const string TAUJD0 = tau_J_d_0;
         const auto response156 = request156.send("POST", TAUJD0, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request157{"http://localhost:2000"};
         const string TAUJD1 = tau_J_d_1;
         const auto response157 = request157.send("POST", TAUJD1, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request158{"http://localhost:2000"};
         const string TAUJD2 = tau_J_d_2;
         const auto response158 = request158.send("POST", TAUJD2, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request159{"http://localhost:2000"};
         const string TAUJD3 = tau_J_d_3;
         const auto response159 = request159.send("POST", TAUJD3, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request160{"http://localhost:2000"};
         const string TAUJD4 = tau_J_d_4;
         const auto response160 = request160.send("POST", TAUJD4, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request161{"http://localhost:2000"};
         const string TAUJD5 = tau_J_d_5;
         const auto response161 = request161.send("POST", TAUJD5, {
           {"Content-Type", "application/x-www-form-urlencoded"}
		 });
		 http::Request request162{"http://localhost:2000"};
         const string TAUJD6 = tau_J_d_6;
         const auto response162 = request162.send("POST", TAUJD6, {
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
