#include <Arduino.h>
#include "ros_led.h"
#include "led_control.h"
RosLed ros_led ;


void setup() {
    led_setup();
    ros_led.initialize();
    ros_led.subscriber_define();
    ros_led.executors_start();
}

void loop() {
    ros_led.start_receiving_msgs();
}