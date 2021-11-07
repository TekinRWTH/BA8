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
const char* PAYLOAD2 = "Hi there!";
const char* PAYLOAD3 = "Is anyone listening?";
const char* PAYLOAD4 = "Someone is always listening.";

const char* LWT_PAYLOAD = "Last will and testament.";

const int  QOS = 1;

const auto TIMEOUT = std::chrono::seconds(10);

int main(int argc, char* argv[])
{
	// A client that just publishes normally doesn't need a persistent
	// session or Client ID unless it's using persistence, then the local
	// library requires an ID to identify the persistence files.

	string	address  = (argc > 1) ? string(argv[1]) : DFLT_SERVER_ADDRESS,
			clientID = (argc > 2) ? string(argv[2]) : CLIENT_ID;

	//cout << "Initializing for server '" << address << "'..." << endl;
	mqtt::async_client client(address, clientID, PERSIST_DIR);
	

	auto connOpts = mqtt::connect_options_builder()
		.clean_session()
		.will(mqtt::message(TOPIC, LWT_PAYLOAD, QOS))
		.finalize();

	//cout << "  ...OK" << endl;

	try {
		//cout << "\nConnecting..." << endl;
		mqtt::token_ptr conntok = client.connect(connOpts);
		//cout << "Waiting for the connection..." << endl;
		conntok->wait();
		//cout << "  ...OK" << endl;

		// First use a message pointer.

		//cout << "\nSending message..." << endl;
		mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, PAYLOAD1);
		pubmsg->set_qos(QOS);
		client.publish(pubmsg)->wait_for(TIMEOUT);
		//cout << "  ...OK" << endl;

		// Now try with itemized publish.

		//cout << "\nSending next message..." << endl;
		mqtt::delivery_token_ptr pubtok;
		pubtok = client.publish(TOPIC, PAYLOAD2, strlen(PAYLOAD2), QOS, false);
		//cout << "  ...with token: " << pubtok->get_message_id() << endl;
		//cout << "  ...for message with " << pubtok->get_message()->get_payload().size()
			//<< " bytes" << endl;
		pubtok->wait_for(TIMEOUT);
		//cout << "  ...OK" << endl;

		
		pubmsg = mqtt::make_message(TOPIC, PAYLOAD3);
		
		pubtok->wait();
		//cout << "  ...OK" << endl;

		// Finally try with a listener, but no token

		//cout << "\nSending final message..." << endl;
		
		pubmsg = mqtt::make_message(TOPIC, PAYLOAD4);	
	}
	catch (const mqtt::exception& exc) {
		cerr << exc.what() << endl;
		return 1;
	}

 	return 0;
}