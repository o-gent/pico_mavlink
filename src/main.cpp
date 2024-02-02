#include "settings.h"
#include "sensors.h"
#include "commands.h"
#include "receive.h"

TaskHandle_t heartbeat_thread;
TaskHandle_t rangefinder1_thread;
TaskHandle_t rangefinder2_thread;
TaskHandle_t recieve_mavlink;

SerialPIO Serial3(14,15);
SerialPIO Serial4(16,17);

/*
Setup comms links, start threads
*/
void setup()
{
    Serial.begin(9600);
    Serial.println("started");
    
    
    Serial1.begin(56700);
    Serial1.println("hi");

    Serial2.begin(57600);

    Serial3.begin(57600);

    Serial4.begin(57600);



    xTaskCreate(
        heartbeat_update, /* Task function. */
        "record",         /* name of task. */
        1000,            /* Stack size of task */
        NULL,             /* parameter of the task */
        3,                /* priority of the task */
        &heartbeat_thread /* Task handle to keep track of created task */
    );

    xTaskCreate(
        rangefinder1_update, /* Task function. */
        "rangefinder",       /* name of task. */
        1000,               /* Stack size of task */
        NULL,                /* parameter of the task */
        1,                   /* priority of the task */
        &rangefinder1_thread /* Task handle to keep track of created task */
    );

    xTaskCreate(
        rangefinder2_update, /* Task function. */
        "rangefinder2",       /* name of task. */
        1000,               /* Stack size of task */
        NULL,                /* parameter of the task */
        1,                   /* priority of the task */
        &rangefinder2_thread /* Task handle to keep track of created task */
    );

    xTaskCreate(
        receive_update,  /* Task function. */
        "recieve",       /* name of task. */
        1000,           /* Stack size of task */
        NULL,            /* parameter of the task */
        2,               /* priority of the task */
        &recieve_mavlink /* Task handle to keep track of created task */
    );
}

/*
Our loop does nothing as all the activities are in threads
*/
void loop()
{   
    mav_request_data();
    gps2raw_request();
    vTaskDelay(10000);
}