#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int32


import cv2

import os, sys, time

modules = os.path.join(get_package_share_directory('opencv_led'), 'modules')
sys.path.append(modules)

import handTrackingModule as htm

class ledStatusPublisherNode(Node):
    def __init__(self):
        super().__init__('led_status_publisher')
        
        self.publisher_ = self.create_publisher(Int32, "led_status", 10)
        self.get_logger().info("led status publisher started")
    
    def publishStatus(self, number):
        msg = Int32()
        msg.data = number
        self.publisher_.publish(msg)
        self.get_logger().info("Published: %d" % msg.data)

class fingerCounter():
    def __init__(self):
        self.tipIds_ = [4, 8, 12, 16, 20] 
        self.detector_ = htm.handDetector(detectionCon=0.75, maxHands=1)
        self.totalFingers_ = 1
        self.actStatus_ = 0
        self.myPublisher_ = ledStatusPublisherNode()

    def getTotalFingers(self) -> int:
        return self.totalFingers_
    
    def getActStatus(self) -> int:
        return self.actStatus_
    
    def setActStatus(self, actNumber: int):
        self.actStatus_ = actNumber
    
    def countFingers(self, frame): # This code is programmed to be used with the right hand
        frame = self.detector_.findHands(frame)
        lmList = self.detector_.findPosition(frame, draw=False)
        if len(lmList) != 0:
            fingers = []
            
            # Thumb
            if lmList[self.tipIds_[0]][1] < lmList[self.tipIds_[0] - 1][1]:
                fingers.append(1)
                    
            else:
                fingers.append(0)
            
            # Other 4 fingers
            for id in range (1, 5):
                if lmList[self.tipIds_[id]][2] < lmList[self.tipIds_[id] - 2][2]:
                    fingers.append(1)
                    
                else:
                    fingers.append(0)
            
            self.totalFingers_ = fingers.count(1)
        
        return frame
            
    def publish(self, number):
        self.myPublisher_.publishStatus(number)


def main(args = None):
    rclpy.init(args=args)
    wCam, hCam = 640, 480
    cam = cv2.VideoCapture(2) # camera ID (Default: 0)
    cam.set(3, wCam)
    cam.set(4, hCam)
        
    if not cam.isOpened():
        sys.exit("Video source not found....")

    cTime = 0
    pTime = 0
    
    counter = fingerCounter()
    
    while True:
        ret, frame = cam.read()
        if ret == False: break
        
        frame = counter.countFingers(frame)
        
        if counter.getTotalFingers() == 0 and counter.getTotalFingers() != counter.getActStatus():
            counter.publish(0)
            counter.setActStatus(counter.getTotalFingers())
        
        elif counter.getTotalFingers() == 5 and counter.getTotalFingers() != counter.getActStatus():
            counter.publish(1)
            counter.setActStatus(counter.getTotalFingers())
        
        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime
            
        cv2.putText(frame, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        
        cv2.imshow("Hand Recognition", frame)
        
        if cv2.waitKey(10) == 13:
            break   
    
    cv2.destroyAllWindows()
    cam.release()
    rclpy.shutdown()



if __name__ == "__main__":
    main()