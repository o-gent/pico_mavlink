#include "settings.h"
#include "sensors.h"
#include "commands.h"
#include "receive.h"


rtos::Thread heartbeat_thread;
rtos::Thread rangefinder1_thread;
rtos::Thread recieve_mavlink;

/*
Setup comms links, start threads
*/
void setup()
{
    Serial2.begin(57600);
    Serial.begin(9600);
    Serial.println("started");

    mav_request_data();
    gps2raw_request();

    heartbeat_thread.start(heartbeat_update);
    rangefinder1_thread.start(rangefinder1_update);
    recieve_mavlink.start(receive_update);
}


/*
Our loop does nothing as all the activities are in threads
*/
void loop()
{
    rtos::ThisThread::sleep_for(1000);
}