#include "ros_led.h"
#include "led_control.h"

rcl_subscription_t led_status_sub;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

RosLed::RosLed(){
}

void RosLed::initialize(){
    Serial.begin(115200);
    Serial.println("ROS Led node started");

    // Adding Wifi
    IPAddress agent_ip(192, 168, 190, 23); // change this line to your computer IP
    size_t agent_port = 8888; // Don't change this port unless you know what you are doing and you have 8888 port already in use

    char ssid[] = "WIFI name"; // change this line with your wifi name
    char psk[]= "Password"; // change this line with your password

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    // set_microros_serial_transports(Serial);  // uncomment this line if you want to use serial instead of wifi

    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "led_status_sub", "", &support);
}


void RosLed::executors_start(){
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &led_status_sub, &msg,&RosLed::led_status_callback, ON_NEW_DATA);

  Serial.println("Executors Started");
}
void RosLed::subscriber_define(){

    rclc_subscription_init_default(
    &led_status_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/led_status"); // define subcriber topic and datatype

}

void RosLed::led_status_callback(const void *msg_recv){
    const std_msgs__msg__Int32 * recieved_data = (const std_msgs__msg__Int32 *) msg_recv ;
    int status_received = recieved_data->data;

    Serial.print(status_received);

    if(status_received == 0) {
        Serial.println("Turn off");
        turnOff();
    } else if(status_received == 1) {
        Serial.println("Turn on");
        turnOn();
    }
    else{
        Serial.println("Not valid status");
    }
}
void RosLed::start_receiving_msgs(){
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        delay(100);
}