#include <random>
#include <string>
#include <thread>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include "paho.mqtt.cpp/src/mqtt/async_client.h"
#include "mosquitto.h"


int main(int argc, char* argv[])
{
    const std::string TOPIC { "hello" };
    const std::string PAYLOAD1 { "Hello World!" };

    const char* PAYLOAD2 = "Hi there!";
    const std::string CLIENT_ID { "8883" };
    const std::string ADDRESS { "tcp://127.0.0.1:1883"};
    
    const int QOS = 1;
    // Client erstellen

    mqtt::async_client cli(ADDRESS, CLIENT_ID);

    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    try {
        // Client verbinden

        cli.connect(connOpts);

        // Publish pointer

        //auto msg = mqtt::make_message(TOPIC, PAYLOAD1);
        //msg->set_qos(QOS);

        //cli.publish(msg);

        // Publish

        //cli.publish(TOPIC, PAYLOAD2, strlen(PAYLOAD2), 0, false);

        // Disconnect

        //cli.disconnect();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << "Error: " << exc.what() << " ["
            << exc.get_reason_code() << "]" << std::endl;
        return 1;
    }

    return 0;
}