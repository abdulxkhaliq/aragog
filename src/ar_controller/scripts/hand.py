#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import cv2
import mediapipe
import time

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

class HandGesturePublisher(Node):

    def __init__(self):
        super().__init__('hand_gesture_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(0.033, self.timer_callback)  # ~30fps

        self.ctime = 0
        self.ptime = 0

        self.cap = cv2.VideoCapture(0)

        self.medhands = mediapipe.solutions.hands
        self.hands = self.medhands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.draw = mediapipe.solutions.drawing_utils

    def timer_callback(self):
        settings = saveTerminalSettings()
        success, img = self.cap.read()
        if not success:
            self.get_logger().warn('Could not read frame from camera')
            return

        img = cv2.flip(img, 1)
        imgrgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        res = self.hands.process(imgrgb)

        lmlist = []
        tipids = [4, 8, 12, 16, 20]  # list of all landmarks of the tips of fingers
##
        if res.multi_hand_landmarks:
            for handlms in res.multi_hand_landmarks:
                for id,lm in enumerate(handlms.landmark):                
                    h,w,c= img.shape
                    cx,cy=int(lm.x * w) , int(lm.y * h)
                    lmlist.append([id,cx,cy])
                if len(lmlist) != 0 and len(lmlist)==21:
                    fingerlist=[]
                    
                    #thumb and dealing with flipping of hands
                    if lmlist[12][1] > lmlist[20][1]:
                        if lmlist[tipids[0]][1] > lmlist[tipids[0]-1][1]:
                            fingerlist.append(1)
                        else:
                            fingerlist.append(0)
                    else:
                        if lmlist[tipids[0]][1] < lmlist[tipids[0]-1][1]:
                            fingerlist.append(1)
                        else:
                            fingerlist.append(0)
                    
                    #others
                    for id in range (1,5):
                        if lmlist[tipids[id]][2] < lmlist[tipids[id]-2][2]:
                            fingerlist.append(1)
                        else:
                            fingerlist.append(0)
                    
                    
                    if len(fingerlist)!=0:
                        fingercount=fingerlist.count(1)
                        if fingercount == 1:
                            linear_x = 0.5
                            linear_z = 0.0
                            angular_z = 0.0
                        elif fingercount == 2:
                            linear_x = -0.5
                            linear_z = 0.0
                            angular_z = 0.0
                        elif fingercount == 3:
                            linear_x = 0.0
                            linear_z = 0.0
                            angular_z = 0.5
                        elif fingercount == 4:
                            linear_x = 0.0
                            linear_z = 0.0
                            angular_z = -0.5
                        elif fingercount == 5:
                            linear_x = 0.0
                            linear_z = 0.5
                            angular_z = 0.0
                        elif fingercount == 0:
                            linear_x = 0.0
                            linear_z = -0.5
                            angular_z = 0.0
                        
                        key = getKey(settings)
                        if key == 'p':
                            twist_msg = Twist()
                            twist_msg.linear.x = linear_x
                            twist_msg.linear.z = linear_z
                            twist_msg.angular.z = angular_z
                            self.publisher_.publish(twist_msg)
                    
                cv2.putText(img,str(fingercount),(25,430),cv2.FONT_HERSHEY_PLAIN,6,(0,0,0),5)
##
                    

                # change color of points and lines
                self.draw.draw_landmarks(img, handlms, self.medhands.HAND_CONNECTIONS,
                                         self.draw.DrawingSpec(color=(0, 255, 204), thickness=2, circle_radius=2),
                                         self.draw.DrawingSpec(color=(0, 0, 0), thickness=2, circle_radius=3))

        # fps counter
        self.ctime = time.time()
        fps = 1 / (self.ctime - self.ptime)
        self.ptime = self.ctime

        # fps display
        cv2.putText(img, f'FPS:{str(int(fps))}', (0, 12), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)

        cv2.imshow("hand gestures", img)

        if cv2.waitKey(1) == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    
    rclpy.init(args=args)

    hand_gesture_publisher = HandGesturePublisher()

    rclpy.spin(hand_gesture_publisher)

    # Destroy the node explicitly
    hand_gesture_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()