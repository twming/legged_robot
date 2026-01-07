import rclpy
from rclpy.node import Node
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Go2Trot(Node):
    def __init__(self):
        super().__init__('go2_trot_node')
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_group_effort_controller/joint_trajectory', 
            10)
        
        self.timer = self.create_timer(0.02, self.publish_trot_gait)
        self.start_time = self.get_clock().now()
        self.get_logger().info('Go2 Optimized Trot Node Started')

    def publish_trot_gait(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        
        msg = JointTrajectory()
        msg.joint_names = [
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint'
        ]

        # --- GAIT PARAMETERS ---
        frequency = 1.2  # Slightly faster for better momentum
        f_pitch_base = 0.7 
        h_pitch_base = 0.8 # Higher to fix backward lean
        knee_base = -1.4   # Deeper crouch for stability
        
        pitch_amp = 0.4    # How far the leg swings
        knee_amp = 0.5     # How much the foot lifts
        
        # Diagonal A: LF and RH
        # Diagonal B: RF and LH
        wave_a = math.sin(2 * math.pi * frequency * t)
        wave_b = math.sin(2 * math.pi * frequency * t + math.pi)

        # Foot lift wave (lifting only, no "stabbing" into the ground)
        lift_a = max(0, wave_a) 
        lift_b = max(0, wave_b)

        point = JointTrajectoryPoint()
        
        # Schema: [Hip (Roll), Pitch (Upper), Knee (Lower)]
        # We invert the knee_amp relative to lift to pull the foot UP
        point.positions = [
            # LF (Phase A)
            0.0,  f_pitch_base + (pitch_amp * wave_a), knee_base + (knee_amp * lift_a),
            # RF (Phase B)
            0.0, f_pitch_base + (pitch_amp * wave_b), knee_base + (knee_amp * lift_b),
            # LH (Phase B)
            0.0,  h_pitch_base + (pitch_amp * wave_b), knee_base + (knee_amp * lift_b),
            # RH (Phase A)
            0.0, h_pitch_base + (pitch_amp * wave_a), knee_base + (knee_amp * lift_a)
        ]

        # Use velocities to help the controller converge smoother
        point.time_from_start.nanosec = 100000000 # 0.05 s
        msg.points = [point]
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = Go2Trot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()