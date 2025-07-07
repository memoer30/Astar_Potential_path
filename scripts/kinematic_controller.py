#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist

def compute_tracking_cmd(current_pose, goal_pose, k_rho=0.6, k_alpha=1.5, k_beta=-0.2, max_linear=0.25, max_angular=0.7):
    """
    Compute tracking command with balanced parameters for smooth path following
    
    Args:
        current_pose: (x, y, theta) current robot pose
        goal_pose: (x, y) goal position
        k_rho: proportional gain for distance
        k_alpha: proportional gain for heading error  
        k_beta: proportional gain for final orientation
        max_linear: maximum linear velocity (reduced for stability)
        max_angular: maximum angular velocity (reduced for stability)
    """
    x, y, theta = current_pose
    xg, yg = goal_pose

    dx = xg - x
    dy = yg - y

    rho = np.hypot(dx, dy)
    
    # Normalize angles to [-pi, pi]
    alpha = math.atan2(dy, dx) - theta
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    
    beta = -theta - alpha
    beta = math.atan2(math.sin(beta), math.cos(beta))

    # Reduce gains when very close to goal to prevent oscillations
    distance_factor = min(1.0, rho / 0.3)  # Reduced threshold for smoother approach
    
    # Compute velocities
    v = k_rho * rho * distance_factor
    omega = k_alpha * alpha * distance_factor + k_beta * beta
    
    # Apply velocity limits
    v = max(-max_linear, min(max_linear, v))
    omega = max(-max_angular, min(max_angular, omega))
    
    # Reduce linear velocity when turning to prevent overshooting
    if abs(alpha) > math.pi/4:  # If heading error > 45 degrees
        v = v * 0.5
    
    cmd = Twist()
    cmd.linear.x = max(0, v)  # Only forward motion
    cmd.angular.z = omega

    return cmd, rho