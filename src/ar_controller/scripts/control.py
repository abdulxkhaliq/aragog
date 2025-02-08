#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.wait_for_message
import rclpy.waitable
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import numpy as np
import math
from inverse_kinematic import *

#todo1 : 
#clone launch.py and remove joint_publisher_gui

class JointStatePublisher(Node):
    publish_joint_angles = True
    def __init__(self):
        super().__init__('joint_state_publisherr')
        self.get_logger().info("publishing to joints")
        rate = self.create_rate(10)
        self.joint_names = ['j_c1_rf', 'j_thigh_rf', 'j_tibia_rf',
						   'j_c1_lf', 'j_thigh_lf', 'j_tibia_lf',
						   'j_c1_lr', 'j_thigh_lr', 'j_tibia_lr',
						   'j_c1_rr', 'j_thigh_rr', 'j_tibia_rr']  #joint names
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # initial position
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.increment = 0.01
        self.thigh_raise = -1.0
        self.thigh_ground = -1.33

    def move(self,v1,v2,o,t):
        '''
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        '''
          
        self.i = 0

        self.get_logger().info("Received velocities")
            #self.get_logger().info(str(f"linearX = {v1}, linearY = {v2}, linearZ = {t}, angularZ = {o}"))
            #recieving velocities..

        if t>0:                                             
            self.stand()
            

        if t<0:
            self.sit()

        if v1>0.0:           
            self.forward()

        if v1<0.0:
            self.backward()
            
        if o>0.0:
            self.left()
            
        if o<0.0:
            self.right()
          
        #self.joint_state_pub.publish(joint_state_msg)

    def cmd_vel_callback(self, msg):
        v1 = msg.linear.x
        v2 = msg.linear.y
        o = msg.angular.z
        t = msg.linear.z

        self.get_logger().info(f"Receiving velocities: linearX={v1}, linearY={v2}, AngularZ={o}, LinearZ={t}")

        self.move(v1, v2, o, t)


    def stand(self):
        self.get_logger().info("standing")
        self.joint_positions = [-0.42, -1.33, 0.99, 0.42, -1.33, 0.99, -0.42, -1.33, 0.99, 0.42, -1.33, 0.99]
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        self.joint_state_pub.publish(joint_state_msg)
        #timer_period = 0.05
        #self.create_timer(timer_period, self.update_and_publish)


    def sit(self):
        self.get_logger().info("initial")
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        self.joint_state_pub.publish(joint_state_msg)
        #timer_period = 0.05
        #self.create_timer(timer_period, self.update_and_publish)


    def forward(self):

        self.get_logger().info("forward motion")
        count = 0
        while(count < 6):
            #rf lr rise 
            if count == 0:
                self.joint_positions = [
                    self.joint_positions[0], self.thigh_raise, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.thigh_raise, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                        
                
            
            #rf lr forward
            if count == 1:
                time.sleep(1)
                self.joint_positions = [
                    -1.0, self.thigh_raise, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    0.1, self.thigh_raise, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)                

            #rf lr touch ground
            if count == 2:
                time.sleep(1)
                self.joint_positions = [
                    -1.0, self.thigh_ground, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    0.1, self.thigh_ground, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)

            #lf rr rise
            if count == 3:
                time.sleep(1)
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.thigh_raise, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.thigh_raise, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)

            #lf rr forward
            if count == 4:
                time.sleep(1)
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    1.0, self.thigh_raise, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    0.1, self.thigh_raise, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)

            #lf rr touch ground
            if count == 5:
                time.sleep(1)
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    1.0, self.thigh_ground, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    0.1, self.thigh_ground, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
            count += 1
            time.sleep(1)
        self.stand()
        #timer_period = 0.05   
        #self.create_timer(timer_period, self.new_update_and_publish)
        

    def backward(self):
        self.get_logger().info("backward motion")
        count = 0
        while(count < 6):
            # rf lr rise
            if count == 0:
                self.joint_positions = [
                    self.joint_positions[0], self.thigh_raise, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.thigh_raise, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            
            # rf lr back
            if count == 1:
                self.joint_positions = [
                    -0.1, self.thigh_raise, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    -1.0, self.thigh_raise, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)


            # rf lr touch ground
            if count == 2:
                self.joint_positions = [
                    -0.1, self.thigh_ground, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    -1.0, self.thigh_ground, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)


            # lf rr rise
            if count == 3:
                time.sleep(1)
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.thigh_raise, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.thigh_raise, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)

            # lf rr back
            if count == 4:
                time.sleep(1)
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    -0.1, self.thigh_raise, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    1.0, self.thigh_raise, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)

            # lf rr touch ground
            if count == 5:
                time.sleep(1)
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    -0.1, self.thigh_ground, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    1.0, self.thigh_ground, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)

            count += 1
            time.sleep(1)
        self.stand()


    def right(self):
        self.get_logger().info("move right")
        count = 0
        while (count < 12):
            right = -0.42
            if count == 0:
                #rf rise
                self.joint_positions = [
                    self.joint_positions[0], self.thigh_raise, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 1:
                #rf right
                self.joint_positions = [
                    self.joint_positions[0] - right, self.thigh_raise, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 2:
                #rf down
                self.joint_positions = [
                    self.joint_positions[0], self.thigh_ground, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 3:
                #rr up
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.thigh_raise, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 4:
                #rr right
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9] - right, self.thigh_raise, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 5:
                #rr down
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.thigh_ground, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 6:
                #lr up
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.thigh_raise, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 7:
                #lr right
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6] - right, self.thigh_raise, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 8:
                #lr down
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.thigh_ground, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 9:
                #rf up
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.thigh_raise, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 10:
                #rf right
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3] - right, self.thigh_raise, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 11:
                #rr down
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.thigh_ground, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            
            count += 1
            time.sleep(1)
        self.stand()

    def left(self):
        self.get_logger().info("move left")
        count = 0
        while (count < 12):
            left = 0.42
            if count == 0:
                #rf rise
                self.joint_positions = [
                    self.joint_positions[0], self.thigh_raise, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 1:
                #rf left
                self.joint_positions = [
                    self.joint_positions[0] - left, self.thigh_raise, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 2:
                #rf down
                self.joint_positions = [
                    self.joint_positions[0], self.thigh_ground, self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 3:
                #rr up
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.thigh_raise, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 4:
                #rr left
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9] - left, self.thigh_raise, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 5:
                #rr down
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.thigh_ground, self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 6:
                #lr up
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.thigh_raise, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 7:
                #lr left
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6] - left, self.thigh_raise, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 8:
                #lr down
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.joint_positions[4], self.joint_positions[5],
                    self.joint_positions[6], self.thigh_ground, self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 9:
                #rf up
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.thigh_raise, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 10:
                #rf left
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3] - left, self.thigh_raise, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            if count == 11:
                #rr down
                self.joint_positions = [
                    self.joint_positions[0], self.joint_positions[1], self.joint_positions[2],
                    self.joint_positions[3], self.thigh_ground, self.joint_positions[5],
                    self.joint_positions[6], self.joint_positions[7], self.joint_positions[8],
                    self.joint_positions[9], self.joint_positions[10], self.joint_positions[11],
                    ]
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.joint_positions
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(1)
            
            count += 1
            time.sleep(1)
        self.stand()

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisherr = JointStatePublisher()
    rclpy.spin(joint_state_publisherr)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
