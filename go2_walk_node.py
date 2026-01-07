import rclpy
from rclpy.node import Node
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Go2Walk(Node):
    def __init__(self):
        super().__init__('go2_walk_node')
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_group_effort_controller/joint_trajectory', 
            10)
        
        self.timer = self.create_timer(0.02, self.control_loop)
        self.start_time = self.get_clock().now()
        
        # Gait Settings
        self.stand_time = 2.0  # Seconds to reach standing pose
        self.walk_freq = 0.8   # Walking is slower/more stable than trotting
        self.step_height = 0.35 
        self.step_length = 0.25

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        msg = JointTrajectory()
        msg.joint_names = [
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint'
        ]

        # Base Stance Offsets
        f_pitch = 0.7 
        h_pitch = 0.85 # Higher rear to prevent the backward lean
        knee = -1.4 

        point = JointTrajectoryPoint()

        if elapsed < self.stand_time:
            # PHASE 1: STANDING
            point.positions = [
                0.1, f_pitch, knee, -0.1, f_pitch, knee,
                0.1, h_pitch, knee, -0.1, h_pitch, knee
            ]
        else:
            # PHASE 2: WALKING GAIT (Crawl)
            t = elapsed - self.stand_time
            # Each leg moves with a 90-degree phase shift (0.25 offset)
            phases = [
                math.sin(2 * math.pi * self.walk_freq * t),         # LF
                math.sin(2 * math.pi * self.walk_freq * t + 1.57),  # RH
                math.sin(2 * math.pi * self.walk_freq * t + 3.14),  # RF
                math.sin(2 * math.pi * self.walk_freq * t + 4.71)   # LH
            ]
            
            # Helper to calculate lift only (preventing ground stabbing)
            lifts = [max(0, p) for p in phases]

            point.positions = [
                0.1,  f_pitch + (self.step_length * phases[0]), knee + (self.step_height * lifts[0]),
                -0.1, f_pitch + (self.step_length * phases[2]), knee + (self.step_height * lifts[2]),
                0.1,  h_pitch + (self.step_length * phases[3]), knee + (self.step_height * lifts[3]),
                -0.1, h_pitch + (self.step_length * phases[1]), knee + (self.step_height * lifts[1])
            ]

        point.time_from_start.nanosec = 40000000 # 0.04s for smoother interpolation
        msg.points = [point]
        self.publisher.publish(msg)