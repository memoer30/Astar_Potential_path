#!/usr/bin/env python3

import numpy as np
import math
from geometry_msgs.msg import Twist

def laser_to_cartesian(scan):
    """
    Converts laser scan data to 2D Cartesian points in the robot's frame.
    Filters out distant and very close readings to focus on relevant obstacles.
    """
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges = np.array(scan.ranges)
    
    # Focus only on very close obstacles directly in front
    mask = np.isfinite(ranges) & (ranges > 0.2) & (ranges < 1.0) & \
           (np.abs(angles) < math.pi / 6)  # 30 degrees total (15 degrees each side)
           
    xs = ranges[mask] * np.cos(angles[mask])
    ys = ranges[mask] * np.sin(angles[mask])
    return np.stack((xs, ys), axis=-1)

def compute_repulsive_force(points, d0=0.5, k_rep=0.1):
    """
    Computes a weaker repulsive force to prevent deadlock situations.
    Uses smaller influence distance and weaker magnitude.
    """
    if len(points) == 0:
        return np.zeros(2)

    total_force = np.zeros(2)
    
    # Only consider the closest obstacles to avoid force conflicts
    if len(points) > 0:
        distances = [np.linalg.norm(p) for p in points]
        min_dist_idx = np.argmin(distances)
        closest_point = points[min_dist_idx]
        d = distances[min_dist_idx]
        
        if d < d0:
            # Much weaker repulsive force to prevent getting stuck
            rep_magnitude = k_rep * (1.0/d - 1.0/d0)  # Linear instead of quadratic
            rep_force = rep_magnitude * (-closest_point / d)
            total_force = rep_force

    # Limit the magnitude of the force to prevent extreme velocities  
    force_magnitude = np.linalg.norm(total_force)
    if force_magnitude > 0.8:  # Much lower max force to prevent conflicts
        total_force = total_force * (0.8 / force_magnitude)
        
    return total_force


def compute_attractive_force(current_position, goal_position, k_att=0.6):
    """
    Computes a weaker attractive force to prevent force conflicts.
    Reduces the pull toward waypoints when in obstacle avoidance mode.
    """
    force = k_att * (np.array(goal_position) - np.array(current_position))
    
    # Limit attractive force magnitude to prevent overwhelming repulsive forces
    force_magnitude = np.linalg.norm(force)
    if force_magnitude > 0.5:  # Cap attractive force
        force = force * (0.5 / force_magnitude)
    
    return force

def force_to_cmd(force_vector, current_yaw, max_speed=0.15, max_angular=0.6):
    """
    Converts a 2D force vector into a Twist command for the robot.
    Even more conservative speeds to prevent getting stuck.
    """
    force_magnitude = np.linalg.norm(force_vector)
    if force_magnitude < 0.05:  # Lower threshold for more responsive movement
        return Twist()

    desired_angle = np.arctan2(force_vector[1], force_vector[0])
    angle_error = math.atan2(math.sin(desired_angle - current_yaw), 
                            math.cos(desired_angle - current_yaw))

    # More conservative movement to prevent conflicts
    linear_vel = max_speed * max(0.3, 1 - 1.0 * abs(angle_error) / math.pi)

    # Reduced angular response to prevent oscillations
    angular_vel = 1.2 * angle_error
    
    # Clip velocities to their maximum allowed values
    cmd = Twist()
    cmd.linear.x = np.clip(linear_vel, 0.0, max_speed)
    cmd.angular.z = np.clip(angular_vel, -max_angular, max_angular)
    
    return cmd