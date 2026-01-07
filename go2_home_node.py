import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Go2Home(Node):
    def __init__(self):
        super().__init__('go2_home_node')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_group_effort_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.2, self.move_to_home)

    def move_to_home(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint'
        ]

        point = JointTrajectoryPoint()
        # Initial Stand Pose: Hip=0.0, Thigh=0.7, Knee=-1.5
        point.positions = [
            0.2, 0.7, -1.5, 
            -0.2, 0.7, -1.5, 
            0.2, 0.9, -1.5, 
            -0.2, 0.9, -1.5]
        point.time_from_start.sec = 0.2 # Move slowly to avoid physics instability
        
        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info('Sent Stand Initial Pose command')
        # self.timer.cancel() # Only run once

def main():
    rclpy.init()
    node = Go2Home()
    rclpy.spin(node)
    rclpy.shutdown()