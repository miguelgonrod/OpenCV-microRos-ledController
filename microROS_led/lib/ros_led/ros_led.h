#ifndef ROS_LED_H
#define ROS_LED_H

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>


class RosLed{
    public:
        RosLed();
        void initialize();
        void subscriber_define();
        static void led_status_callback(const void *msg_recv);
        void start_receiving_msgs();
        void executors_start();
    private:

};

#endif