import math
from dataclasses import dataclass
import numpy as np
from geometry import SE2, se2, se2_from_linear_angular, SE2_from_xytheta, translation_from_SE2, angle_from_SE2

# @dataclass
class Configuration:
    pose: SE2
    velocity: se2

    def __init__(self, pose, velocity):
        self.pose = pose
        self.velocity = velocity


class DuckiebotKinematics:
    radius: float
    baseline: float
    max_number_ticks_encoder: int
    pose_space: SE2
    configuration: Configuration

    def __init__(self, radius: float, baseline: float, pose: SE2) -> None:
        self.radius = radius
        self.baseline = baseline
        self.max_number_ticks_encoder = 135
        self.pose_space = SE2
        self.configuration = self.initialize_config(pose)

    def initialize_config(self, pose: SE2) -> Configuration:
        vel = self.pose_space.algebra.zero()
        configuration = Configuration(pose, vel)
        return configuration

    def step(self, number_ticks_left, number_ticks_right, dt) -> None:
        d_l = 2*np.pi*self.radius*number_ticks_left / self.max_number_ticks_encoder
        d_r = 2*np.pi*self.radius*number_ticks_right / self.max_number_ticks_encoder
        d = (d_l + d_r)/2
        delta_theta = (d_r - d_l) / (self.baseline)
        pose1 = self.configuration.pose
        translation1 = translation_from_SE2(pose1)
        theta1 = angle_from_SE2(pose1)
        theta2 = theta1 + delta_theta
        x2 = translation1[0] + d*math.cos(theta2)
        y2 = translation1[1] + d*math.sin(theta2)
        pose2 = SE2_from_xytheta([x2, y2, theta2])
        self.configuration.pose = pose2
        v_l = d_l / dt
        v_r = d_l / dt
        linear_vel2 = (v_l + v_r)/2
        theta_dot = (v_r - v_l) / (2*self.baseline)
        linear_velocity = np.array([linear_vel2, 0])
        velocity2 = se2_from_linear_angular(linear_velocity, theta_dot)
        self.configuration.velocity = velocity2
