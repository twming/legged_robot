import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Go2TeleopBridge(Node):
    def __init__(self):
        super().__init__('go2_teleop_bridge')
        
        # Subscribe to teleop commands
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publish to controller
        self.publisher = self.create_publisher(JointTrajectory, '/joint_group_effort_controller/joint_trajectory', 10)
        
        self.timer = self.create_timer(0.02, self.publish_gait)
        self.start_time = self.get_clock().now()
        
        # Movement state
        self.linear_x = 0.0
        self.angular_z = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def publish_gait(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = JointTrajectory()
        msg.joint_names = [
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint'
        ]

        # Use velocity to scale the movement
        freq = 2.0 if self.linear_x != 0 else 0.0
        amp = 0.4 * self.linear_x
        
        # Sine wave phases
        phase_a = math.sin(2 * math.pi * freq * t)
        phase_b = math.sin(2 * math.pi * freq * t + math.pi)

        point = JointTrajectoryPoint()
        # If linear_x is 0, it just holds the standing pose (0.7, -1.2)
        stand_p = 0.7
        stand_k = -1.2

        point.positions = [
            0.0, stand_p + (amp * phase_a), stand_k + (amp * phase_a),
            0.0, stand_p + (amp * phase_b), stand_k + (amp * phase_b),
            0.0, stand_p + (amp * phase_b), stand_k + (amp * phase_b),
            0.0, stand_p + (amp * phase_a), stand_k + (amp * phase_a)
        ]
        
        point.time_from_start.nanosec = 100000000
        msg.points = [point]
        self.publisher.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(Go2TeleopBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()