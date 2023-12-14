#!/usr/bin/env python3
import cv2
import mediapipe as mp
import time

class handDetector():
    def __init__(self, mode = False, maxHands = 2, complexity = 1, detectionCon = 0.5, trackCon = 0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.complexity = complexity
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.complexity, self.detectionCon, self.trackCon)
        
        self.mpDraw = mp.solutions.drawing_utils
        
        self.results = None
        
    def findHands(self, frame, draw = True):
        frame = cv2.flip(frame, 1)
        imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results =  self.hands.process(imgRGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(frame, handLms, self.mpHands.HAND_CONNECTIONS)
        
        return frame
    
    def findPosition(self, frame, handNo = 0, draw = True):
        lmList = []
        
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = frame.shape
                cx, cy = int(lm.x * w), int (lm.y * h)

                lmList.append([id, cx, cy])
                
                if draw:
                    cv2.circle(frame, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
            
        return lmList

    
def main():
    pTime = 0
    cTime = 0
    
    cam = cv2.VideoCapture(2)
    detector = handDetector()

    while True:
        ret, frame = cam.read()
        
        frame = detector.findHands(frame)
        
        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime
        
        cv2.putText(frame, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        
        cv2.imshow("Hand Recognizer", frame)
        cv2.waitKey(1)
    
if __name__ == "__main__":
    main()