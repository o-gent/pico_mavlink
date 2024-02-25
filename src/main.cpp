#include "mavnode.h"

TaskHandle_t heartbeat_thread;
TaskHandle_t rangefinder1_thread;
TaskHandle_t rangefinder2_thread;
TaskHandle_t recieve_mavlink;

SerialPIO Serial3(14,15);
SerialPIO Serial4(16,17);

MavNode *mavnode;

void start_task_heartbeat(void* _this){ ((MavNode*)_this)->heartbeat_update(); }
void start_task_rangefinder1_update(void* _this){ ((MavNode*)_this)->rangefinder1_update(); }
void start_task_rangefinder2_update(void* _this){ ((MavNode*)_this)->rangefinder2_update(); }
void start_task_receive_update(void* _this){ ((MavNode*)_this)->receive_update(); }


/*
Setup comms links, start threads
*/
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.begin(9600);
    Serial.println("started");
    
    SERIAL_MAVLINK.begin(57600);

    SERIAL_SENSOR_1.begin(56700);

    SERIAL_SENSOR_2.begin(57600);

    SERIAL_MAVFWD.begin(57600);

    EEPROM.begin(512);

    mavnode->param_init();
    xSemaphoreTake(mavnode->parameterSemaphore, (TickType_t)10);
    xSemaphoreGive(mavnode->parameterSemaphore);

    xTaskCreate(
        start_task_heartbeat, /* Task function. */
        "record",         /* name of task. */
        1000,            /* Stack size of task */
        &mavnode,             /* parameter of the task */
        3,                /* priority of the task */
        &heartbeat_thread /* Task handle to keep track of created task */
    );

    xTaskCreate(
        start_task_rangefinder1_update, /* Task function. */
        "rangefinder",       /* name of task. */
        1000,               /* Stack size of task */
        &mavnode,                /* parameter of the task */
        1,                   /* priority of the task */
        &rangefinder1_thread /* Task handle to keep track of created task */
    );

    xTaskCreate(
        start_task_rangefinder2_update, /* Task function. */
        "rangefinder2",       /* name of task. */
        1000,               /* Stack size of task */
        &mavnode,                /* parameter of the task */
        1,                   /* priority of the task */
        &rangefinder2_thread /* Task handle to keep track of created task */
    );

    xTaskCreate(
        start_task_receive_update,  /* Task function. */
        "recieve",       /* name of task. */
        10000,           /* Stack size of task */
        &mavnode,            /* parameter of the task */
        2,               /* priority of the task */
        &recieve_mavlink /* Task handle to keep track of created task */
    );
}

/*
Our loop does nothing as all the activities are in threads
*/
void loop()
{   
    mavnode->mav_request_data();
    mavnode->gps2raw_request();
    vTaskDelay(10000);
}


