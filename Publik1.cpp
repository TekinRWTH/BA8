#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include "paho.mqtt.cpp/src/mqtt/async_client.h"


using namespace std;

const string DFLT_SERVER_ADDRESS	{ "tcp://localhost:1883" };
const string CLIENT_ID				{ "paho_cpp_async_publish" };
const string PERSIST_DIR			{ "./persist" };

const string TOPIC { "hello" };

const char* PAYLOAD1 = "Hello World!";
const char* LWT_PAYLOAD = "Last will and testament.";

const int  QOS = 1;

const auto TIMEOUT = std::chrono::seconds(10);

int main(int argc, char* argv[])
{

	string	address  = (argc > 1) ? string(argv[1]) : DFLT_SERVER_ADDRESS,
			clientID = (argc > 2) ? string(argv[2]) : CLIENT_ID;

	mqtt::async_client client(address, clientID, PERSIST_DIR);
	

	auto connOpts = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
		.finalize();

	

	try {
		
		mqtt::token_ptr conntok = client.connect(connOpts);
		
		conntok->wait();
		
		mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg->set_qos(QOS);
		client.publish(pubmsg)->wait_for(TIMEOUT);

	}
	catch (const mqtt::exception& exc) {
		cerr << exc.what() << endl;
		return 1;
	}

 	return 0;
}