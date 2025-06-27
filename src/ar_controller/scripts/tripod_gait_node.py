import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import numpy as np
import time

# link length in meters

L1 = 0.05  # coxa
L2 = 0.1   # thigh
L3 = 0.1   # tibia

#joints
JOINT_NAMES = [
    'j_c1_rf', 'j_thigh_rf', 'j_tibia_rf',
    'j_c1_lf', 'j_thigh_lf', 'j_tibia_lf',
    'j_c1_lr', 'j_thigh_lr', 'j_tibia_lr',
    'j_c1_rr', 'j_thigh_rr', 'j_tibia_rr'
]

#intial pos
FOOT_POSITIONS = {
    'leg1': np.array([ 0.10,  0.10, -0.15]),
    'leg2': np.array([ 0.10, -0.10, -0.15]),
    'leg3': np.array([-0.10,  0.10, -0.15]),
    'leg4': np.array([-0.10, -0.10, -0.15])
}

class TripodGaitNode(Node):
    def __init__(self):
        super().__init__('tripod_gait_node')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.update_loop)

        self.current_twist = Twist()
        self.phase = 0
        self.phase_duration = 0.4  # sec
        self.last_phase_time = time.time()

    def cmd_vel_callback(self, msg):
        self.current_twist = msg

    def update_loop(self):
        now = time.time()
        if now - self.last_phase_time > self.phase_duration:
            self.phase = (self.phase + 1) % 6  # 6 leg cycle
            self.last_phase_time = now

        leg_angles = []

        for i, leg_name in enumerate(['leg1', 'leg2', 'leg3', 'leg4']):
            swing = self.is_leg_in_swing(leg_name, self.phase)
            new_pos = self.compute_leg_trajectory(leg_name, swing)
            angles = self.inverse_kinematics(new_pos)
            leg_angles.extend(angles)

        joint_msg = JointState()
        joint_msg.name = JOINT_NAMES
        joint_msg.position = leg_angles
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_msg)

    def is_leg_in_swing(self, leg_name, phase):
        tripod_1 = ['leg1', 'leg4']
        tripod_2 = ['leg2', 'leg3']
        if phase % 2 == 0:
            return leg_name in tripod_1
        else:
            return leg_name in tripod_2

    def compute_leg_trajectory(self, leg_name, swing):
        base_pos = FOOT_POSITIONS[leg_name].copy()
        step_length = self.current_twist.linear.x * 0.1  # scale down

        if swing:
            base_pos[0] += step_length / 2
            base_pos[2] += 0.02 * math.sin((time.time() - self.last_phase_time) / self.phase_duration * math.pi)
        else:
            base_pos[0] -= step_length / 2

        return base_pos

    def inverse_kinematics(self, pos):
        x, y, z = pos
        theta1 = math.atan2(y, x)
        dx = math.sqrt(x**2 + y**2) - L1
        dz = -z
        D = (dx**2 + dz**2 - L2**2 - L3**2) / (2 * L2 * L3)

        D = min(1.0, max(-1.0, D))  # clamp
        theta3 = math.acos(D)
        theta2 = math.atan2(dz, dx) - math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))

        return [theta1, theta2, -theta3]

def main(args=None):
    rclpy.init(args=args)
    node = TripodGaitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
