#include <franka/robot.h>
#include <franka/robot_state.h>
#include <iostream>
#include <stdlib.h>
#include <franka/exception.h>

#include <iostream>
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

const string TOPIC { "Hey" };
const string TOPIC1 { "O_T_EE" };
const string TOPIC2 { "O_T_EE_d" };                    
const string TOPIC3 { "F_T_EE" };                       
const string TOPIC4 { "m_ee" };                        
const string TOPIC5 { "EE_T_K" };
const string TOPIC6 { "m_load" };
const string TOPIC7 { "m_total" };
const string TOPIC8 { "elbow" };
const string TOPIC9 { "F_x_Cee" };
const string TOPIC10 { "I_ee" };
const string TOPIC11 { "F_x_Ctotal" };
const string TOPIC12 { "F_x_Cload" };
const string TOPIC13 { "I_load" };
const string TOPIC14 { "I_total" };
const string TOPIC15 { "elbow_d" };
const string TOPIC16 { "tau_J" };
const string TOPIC17 { "tau_J_d" };
const string TOPIC18 { "dtau_J" };
const string TOPIC19 { "q" };
const string TOPIC20 { "dq" };
const string TOPIC21 { "q_d" };
const string TOPIC22 { "dq_d" };
const string TOPIC23 { "joint_contact" };
const string TOPIC24 { "cartesian_contact" };
const string TOPIC25 { "joint_collision" };
const string TOPIC26 { "cartesian_collision" };
const string TOPIC27 { "tau_ext_hat_filtered" };
const string TOPIC28 { "O_F_ext_hat_K" };
const string TOPIC29 { "K_F_ext_hat_K" };
const string TOPIC30 { "theta" };
const string TOPIC31 { "dtheta" };
const string TOPIC32 { "F_T_NE"};
const string TOPIC33 { "NE_T_EE" };
const string TOPIC34 { "elbow_d" };
const string TOPIC35 { "elbow_c" };
const string TOPIC36 { "delbow_c" };
const string TOPIC37 { "ddelbow_c" };
const string TOPIC38 { "ddq_d" };
const string TOPIC39 { "O_dP_EE_d" };
const string TOPIC40 { "O_T_EE_c" };
const string TOPIC41 { "O_dP_EE_c" };

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



const char* PAYLOAD1 = "Hello World!";
const char* PAYLOAD3 = "22";
const char* PAYLOAD4 = "Daten";



const char* LWT_PAYLOAD = "Last will and testament.";

const int  QOS = 1;

const auto TIMEOUT = std::chrono::seconds(10);

/////////////////////////////////////////////////////////////////////////////

/**
 * A callback class for use with the main MQTT client.
 */
class callback : public virtual mqtt::callback
{
public:
	void connection_lost(const string& cause) override {
		cout << "\nConnection lost" << endl;
		if (!cause.empty())
			cout << "\tcause: " << cause << endl;
	}

	void delivery_complete(mqtt::delivery_token_ptr tok) override {
		cout << "\tDelivery complete for token: "
			<< (tok ? tok->get_message_id() : -1) << endl;
	}
};

/////////////////////////////////////////////////////////////////////////////

/**
 * A base action listener.
 */
class action_listener : public virtual mqtt::iaction_listener
{
protected:
	void on_failure(const mqtt::token& tok) override {
		cout << "\tListener failure for token: "
			<< tok.get_message_id() << endl;
	}

	void on_success(const mqtt::token& tok) override {
		cout << "\tListener success for token: "
			<< tok.get_message_id() << endl;
	}
};

/////////////////////////////////////////////////////////////////////////////

/**
 * A derived action listener for publish events.
 */
class delivery_action_listener : public action_listener
{
	atomic<bool> done_;

	void on_failure(const mqtt::token& tok) override {
		action_listener::on_failure(tok);
		done_ = true;
	}

	void on_success(const mqtt::token& tok) override {
		action_listener::on_success(tok);
		done_ = true;
	}

public:
	delivery_action_listener() : done_(false) {}
	bool is_done() const { return done_; }
};

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	
franka::Robot robot(argv[1]);

    try{
        for(;;){
            franka::RobotState state = robot.readOnce();{
                 cout << state << endl;
            };
         }
        }catch (franka::Exception const& e){
         cout << e.what() << endl;
         return -1;
    }
    
    
    // A client that just publishes normally doesn't need a persistent
	// session or Client ID unless it's using persistence, then the local
	// library requires an ID to identify the persistence files.

	string	address  = (argc > 1) ? string(argv[1]) : DFLT_SERVER_ADDRESS,
			clientID = (argc > 2) ? string(argv[2]) : CLIENT_ID;

	cout << "Initializing for server '" << address << "'..." << endl;
	mqtt::async_client client(address, clientID, PERSIST_DIR);

	callback cb;
	client.set_callback(cb);

	auto connOpts = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
		.finalize();

    auto connOpts1 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC1, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts2 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC2, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts3 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC3, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts4 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC4, LWT_PAYLOAD, QOS))
		.finalize();    
    auto connOpts5 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC5, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts6 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC6, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts7 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC7, LWT_PAYLOAD, QOS))
		.finalize();    
    auto connOpts8 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC8, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts9 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC9, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts10 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC10, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts11 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC11, LWT_PAYLOAD, QOS))
		.finalize(); 
    auto connOpts12 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC12, LWT_PAYLOAD, QOS))
		.finalize();    
    auto connOpts13 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC13, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts14 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC14, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts15 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC15, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts16 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC16, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts17 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC17, LWT_PAYLOAD, QOS))
		.finalize(); 
    auto connOpts18 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC18, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts19 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC19, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts20 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC20, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts21 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC21, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts22 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC22, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts23 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC23, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts24 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC24, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts25 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC25, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts26 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC26, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts27 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC27, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts28 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC28, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts29 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC29, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts30 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC30, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts31 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC31, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts32 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC32, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts33 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC33, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts34 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC34, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts35 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC35, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts36 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC36, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts37 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC37, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts38 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC38, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts39 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC39, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts40 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC40, LWT_PAYLOAD, QOS))
		.finalize();
    auto connOpts41 = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC41, LWT_PAYLOAD, QOS))
		.finalize();



	cout << "  ...OK" << endl;

	try {
		cout << "\nConnecting..." << endl;
		mqtt::token_ptr conntok = client.connect(connOpts);
		cout << "Waiting for the connection..." << endl;
		conntok->wait();
		cout << "  ...OK" << endl;

		// First use a message pointer.

		cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg->set_qos(QOS);
		client.publish(pubmsg)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg1 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg->set_qos(QOS);
		client.publish(pubmsg)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg2 = mqtt::make_message(TOPIC2, O_T_EE_d);
		pubmsg2->set_qos(QOS);
		client.publish(pubmsg2)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg3 = mqtt::make_message(TOPIC3, F_T_EE);
		pubmsg3->set_qos(QOS);
		client.publish(pubmsg3)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg4 = mqtt::make_message(TOPIC4, m_ee);
		pubmsg4->set_qos(QOS);
		client.publish(pubmsg4)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg5 = mqtt::make_message(TOPIC5, EE_T_K);
		pubmsg5->set_qos(QOS);
		client.publish(pubmsg5)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg6 = mqtt::make_message(TOPIC6, m_load);
		pubmsg6->set_qos(QOS);
		client.publish(pubmsg6)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg7 = mqtt::make_message(TOPIC7, m_total);
		pubmsg7->set_qos(QOS);
		client.publish(pubmsg7)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg8 = mqtt::make_message(TOPIC8, elbow);
		pubmsg8->set_qos(QOS);
		client.publish(pubmsg8)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg9 = mqtt::make_message(TOPIC9, F_x_Cee);
		pubmsg9->set_qos(QOS);
		client.publish(pubmsg9)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg10 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg10->set_qos(QOS);
		client.publish(pubmsg10)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg11 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg11->set_qos(QOS);
		client.publish(pubmsg11)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg12 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg12->set_qos(QOS);
		client.publish(pubmsg12)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg13 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg13->set_qos(QOS);
		client.publish(pubmsg13)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg14 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg14->set_qos(QOS);
		client.publish(pubmsg14)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg15 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg15->set_qos(QOS);
		client.publish(pubmsg15)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg16 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg16->set_qos(QOS);
		client.publish(pubmsg16)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg17 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg17->set_qos(QOS);
		client.publish(pubmsg17)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg18 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg18->set_qos(QOS);
		client.publish(pubmsg18)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg19 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg19->set_qos(QOS);
		client.publish(pubmsg19)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg20 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg20->set_qos(QOS);
		client.publish(pubmsg20)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg21 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg21->set_qos(QOS);
		client.publish(pubmsg21)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg22 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg22->set_qos(QOS);
		client.publish(pubmsg22)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg23 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg23->set_qos(QOS);
		client.publish(pubmsg23)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg24 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg24->set_qos(QOS);
		client.publish(pubmsg24)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg25 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg25->set_qos(QOS);
		client.publish(pubmsg25)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg26 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg26->set_qos(QOS);
		client.publish(pubmsg26)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg27 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg27->set_qos(QOS);
		client.publish(pubmsg27)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg28 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg28->set_qos(QOS);
		client.publish(pubmsg28)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg29 = mqtt::make_message(TOPIC1, O_T_EE);
		pubmsg29->set_qos(QOS);
		client.publish(pubmsg29)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg30 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg30->set_qos(QOS);
		client.publish(pubmsg30)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg31 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg31->set_qos(QOS);
		client.publish(pubmsg31)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg32 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg32->set_qos(QOS);
		client.publish(pubmsg32)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg33 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg33->set_qos(QOS);
		client.publish(pubmsg33)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg34 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg34->set_qos(QOS);
		client.publish(pubmsg34)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg35 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg35->set_qos(QOS);
		client.publish(pubmsg35)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg36 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg36->set_qos(QOS);
		client.publish(pubmsg36)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg37 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg37->set_qos(QOS);
		client.publish(pubmsg37)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg38 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg38->set_qos(QOS);
		client.publish(pubmsg38)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg39 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg39->set_qos(QOS);
		client.publish(pubmsg39)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg40 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg40->set_qos(QOS);
		client.publish(pubmsg40)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

        cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg41 = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg41->set_qos(QOS);
		client.publish(pubmsg41)->wait_for(TIMEOUT);
		cout << "  ...OK" << endl;

		// Now try with itemized publish.

		cout << "\nSending next message..." << endl;
		mqtt::delivery_token_ptr pubtok;
		pubtok = client.publish(TOPIC, PAYLOAD3, strlen(PAYLOAD3), QOS, false);
		cout << "  ...with token: " << pubtok->get_message_id() << endl;
		cout << "  ...for message with " << pubtok->get_message()->get_payload().size()
			<< " bytes" << endl;
		pubtok->wait_for(TIMEOUT);
		cout << "  Topic out" << endl;
        
        
		// Now try with a listener

		cout << "\nSending next message..." << endl;
		action_listener listener;
		pubmsg = mqtt::make_message(TOPIC, PAYLOAD3);
		pubtok = client.publish(pubmsg, nullptr, listener);
		pubtok->wait();
		cout << "  ...OK" << endl;

		// Finally try with a listener, but no token

		cout << "\nSending final message..." << endl;
		delivery_action_listener deliveryListener;
		pubmsg = mqtt::make_message(TOPIC, PAYLOAD4);
		client.publish(pubmsg, nullptr, deliveryListener);

		while (!deliveryListener.is_done()) {
			this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		cout << "OK" << endl;

		// Double check that there are no pending tokens

		auto toks = client.get_pending_delivery_tokens();
		if (!toks.empty())
			cout << "Error: There are pending delivery tokens!" << endl;

		// Disconnect
		cout << "\nDisconnecting..." << endl;
		client.disconnect()->wait();
		cout << "  ...OK" << endl;
	}
	catch (const mqtt::exception& exc) {
		cerr << exc.what() << endl;
		return 1;
	}

 	return 0;
}