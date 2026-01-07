import rclpy
from rclpy.node import Node
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Go2Gallop(Node):
    def __init__(self):
        super().__init__('go2_gallop_node')
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_group_effort_controller/joint_trajectory', 
            10)
        
        # High frequency for a fast gait
        self.timer = self.create_timer(0.01, self.publish_gallop_gait)
        self.start_time = self.get_clock().now()
        self.get_logger().info('Go2 Gallop Node Started - Watch for liftoff!')

    def publish_gallop_gait(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        
        msg = JointTrajectory()
        msg.joint_names = [
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint'
        ]

        # Gallop Parameters
        frequency = 1.0    # Galloping is much faster than trotting
        base_pitch = 0.6   # Neutral upper leg angle
        base_knee = -1.1   # Neutral knee angle
        amplitude = 0.6    # Large range of motion for speed
        
        # Front legs move together
        phase_front = math.sin(2 * math.pi * frequency * t)
        # Rear legs move together, delayed by 210 degrees (3.66 rad)
        phase_rear = math.sin(2 * math.pi * frequency * t - 3.66)

        point = JointTrajectoryPoint()
        
        # Calculate positions
        front_upper = base_pitch + (amplitude * phase_front)
        front_lower = base_knee + (amplitude * phase_front)
        
        rear_upper = base_pitch + (amplitude * phase_rear)
        rear_lower = base_knee + (amplitude * phase_rear)

        point.positions = [
            0.0, front_upper, front_lower, # LF
            0.0, front_upper, front_lower, # RF
            0.0, rear_upper,  rear_lower,  # LH
            0.0, rear_upper,  rear_lower   # RH
        ]
        
        # Time to reach target (very short for high-speed gait)
        point.time_from_start.nanosec = 20000000 # 20ms
        
        msg.points = [point]
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = Go2Gallop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()