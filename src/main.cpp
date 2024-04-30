#include "mavnode.h"

TaskHandle_t heartbeat_thread;
TaskHandle_t rangefinder1_thread;
TaskHandle_t rangefinder2_thread;
TaskHandle_t recieve_mavlink;

SerialPIO Serial3(14,15);
SerialPIO Serial4(16,17);

MavNode *mavnode = new MavNode();

/*
Setup comms links, start threads
*/
void setup()
{   
    
    pinMode(LED_BUILTIN, OUTPUT);

    for (int i = 0;  i < 6; i++) {
        digitalWrite(LED_BUILTIN, LOW);     delay(50);
        digitalWrite(LED_BUILTIN, HIGH);    delay(50);
    }
    
    Serial.begin(57600);
    
    SERIAL_MAVLINK.begin(57600);

    SERIAL_SENSOR_1.begin(57600);

    SERIAL_SENSOR_2.begin(57600);

    SERIAL_MAVFWD.begin(57600);

    EEPROM.begin(512);

    param_init(mavnode);

    rp2040.wdt_begin(1000);

    xTaskCreate(
        (TaskFunction_t)heartbeat_update, /* Task function. */
        "record",         /* name of task. */
        1000,            /* Stack size of task */
        mavnode,             /* parameter of the task */
        3,                /* priority of the task */
        &heartbeat_thread /* Task handle to keep track of created task */
    );

    // xTaskCreate(
    //     (TaskFunction_t)rangefinder1_update, /* Task function. */
    //     "rangefinder",       /* name of task. */
    //     1000,               /* Stack size of task */
    //     mavnode,                /* parameter of the task */
    //     1,                   /* priority of the task */
    //     &rangefinder1_thread /* Task handle to keep track of created task */
    // );

    xTaskCreate(
        (TaskFunction_t)servo1_update, /* Task function. */
        "rangefinder2",       /* name of task. */
        1000,               /* Stack size of task */
        mavnode,                /* parameter of the task */
        1,                   /* priority of the task */
        &rangefinder2_thread /* Task handle to keep track of created task */
    );

    xTaskCreate(
        (TaskFunction_t)receive_update,  /* Task function. */
        "recieve",       /* name of task. */
        10000,           /* Stack size of task */
        mavnode,            /* parameter of the task */
        2,               /* priority of the task */
        &recieve_mavlink /* Task handle to keep track of created task */
    );
}

/*
Our loop does nothing as all the activities are in threads
*/
void loop()
{   
    // mav_request_data(mavnode);
    // gps2raw_request(mavnode);
    vTaskDelay(500);
    // Serial.println("loop");
    rp2040.wdt_reset();
}


