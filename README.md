# OpenCV-microRos-ledController

## Table of contents
* [Description](#description)
* [Demostration](#demostration)
* [Technologies](#technologies)
* [Flash Esp32](#flash-esp32)
* [Setup](#setup)
* [Run](#run)
* [Licence](#licence)


## Description
Explore the fusion of computer vision and robotics with this ROS Humble repository. Leveraging the power of OpenCV, this project features a custom node designed to detect hand gestures and communicate with an ESP32 microcontroller via WiFi using Micro-ROS. The node publishes to a specific topic, allowing seamless integration with the ESP32, enabling the control of an LED to illuminate when an open hand gesture is detected and turn off when the hand is closed. Dive into the intersection of image processing, robotics, and IoT with this practical example of gesture-controlled hardware.

## Demostration

https://github.com/miguelgonrod/OpenCV-microRos-ledController/assets/49737722/1d689c20-c75e-4cc9-b7db-c38e9a0c3101


## Technologies
This project was created with:
* python: 3.10.12
* openCv
* ROS: Humble
* Micro-ros: esp32
* Arduino C++


## Flash Esp32
This image is for reference to the next steps:

![image](https://github.com/miguelgonrod/OpenCV-microRos-ledController/assets/49737722/1cb9e118-9004-4f53-9843-e682e02711fb)


To flash the esp32 microcontroller with the Micro-Ros code you need to have vsCode with the plugin PlatformIO, first you need to clone the repo and move the microROS_led to your Micro-Ros/src workspace.
To create your Micro-Ros workspace you can follow the oficial guide: https://micro.ros.org/docs/tutorials/core/first_application_linux/
```
$ git clone https://github.com/miguelgonrod/OpenCV-microRos-ledController
$ mv OpenCv-microRos-ledController/microROS_led ~/microros_ws/src
$ cd ~/microros_ws && colcon build
$ code src/microROS_led
```
now you have to plug your esp32 to the computer and give permissions to the /dev/ttyUSB# port:
```
$ sudo chmod 777 /dev/ttyUSB0
```

If this step gives you errors or it doesn't flash in further steps you can look up for your ttyUSB number using:
```
$ ls /dev/ttyUSB*
```
Next step is to modify your code to make esp32 connect to your network and your computer, in the ros_led.cpp file you have to change your ip in the line 20 (if you don't know how to get your ip use the next command):
```
$ ifconfig
```
Now change your wifi SSID in line 23 and your wifi password in line 24, remenber to connect you computer to the same network you specified in the code.


Now you have to hold BOOT button (you can see wich button it is in the previous image) while clicking the flas button in vscode (this button only shows up when you have PlatformIO extension installed):

![vscode](https://github.com/miguelgonrod/OpenCV-microRos-ledController/assets/49737722/691014fe-b4bf-4477-9031-df654b96beb3)

If you can see a message that says "[success]" you are ready to go, in any other case create an issue to help you.

Physical connections:
![image](https://github.com/miguelgonrod/OpenCV-microRos-ledController/assets/49737722/c51a2ce9-372b-493d-b2e7-c23894305c52)

Cathode is connected to GND pin and anode is connected to pin "2" (D2 if you have newer versions)


## Setup
To run this node you need to have ros humble installed and a workspace:
```
$ git clone https://github.com/miguelgonrod/OpenCV-microRos-ledController
$ mv OpenCv-microRos-ledController/opencv_led ~/ros2_ws/src
$ cd ~/ros2_ws && colcon build
$ source install/setup.bash
```

Now to run the node you have to install the required dependencies:
```
$ cd ~/ros2_ws/src/opencv_led
$ pip3 install -r requirements.txt
```


## Run
To run the entire project you need to follow the next steps.

In one terminal you are going to create the agent that connects to the Micro-ros topic (remember to be connected to the same network as the esp32):
```
$ cd ~/microros_ws
$ source install/setup.bash
$ ros2 run micro_ros_setup create_agent_ws.sh
$ ros2 run micro_ros_setup build_agent.sh
$ ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```
Now push the EN button in the esp32 (look the previous image to locate the button) if the terminal starts to give a lot of messages you are connected to the Micro-Ros topic

To enable the ros2 humble node open a new terminal and run:
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 launch opencv_led opencv_led.launch.py
```
If this node gives errors first check if you installed the requirements, if you have, try to change the camera number in the fingerCounter.py file in line 81, if this still gives error create an issue to help you.


## Licence
OpenCV-microRos-ledController is available under the BSD-3-Clause license. See the LICENSE file for more details.
