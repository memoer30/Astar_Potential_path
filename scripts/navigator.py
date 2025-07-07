#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from global_planner import a_star
from kinematic_controller import compute_tracking_cmd
from potential_fields import laser_to_cartesian, compute_repulsive_force, force_to_cmd, compute_attractive_force

class Navigator:
    def __init__(self):
        rospy.init_node("navigator")

        # --- MODIFIED: Initialize all attributes FIRST ---
        self.waypoints = []
        self.goal = None
        self.pending_goal = None
        self.robot_pose = (0, 0, 0)
        self.scan = None
        self.grid = None
        self.map_info = None
        
        self.in_obstacle_mode = False
        self.obstacle_enter_threshold = 0.3   # Even closer threshold to minimize avoidance
        self.obstacle_exit_threshold = 0.45   # Quick exit from avoidance mode
        
        # Add oscillation detection
        self.last_obstacle_mode_time = rospy.Time.now()
        self.obstacle_mode_duration_limit = rospy.Duration(5.0)  # Shorter timeout to escape deadlocks faster
        
        # Add deadlock detection for potential fields
        self.last_robot_pose_in_avoidance = None
        self.avoidance_deadlock_threshold = 0.1  # Minimum movement required in avoidance mode
        
        self.last_progress_time = rospy.Time.now()
        self.last_robot_position_for_stuck_check = (-2.0, 0.0)
        self.stuck_check_interval = rospy.Duration(12.0)  # Wait much longer before considering stuck
        self.stuck_distance_threshold = 0.3  # Require even more movement to not be considered stuck
        self.in_recovery_mode = False
        
        # --- Now, create publishers and subscribers ---
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.listener = tf.TransformListener()

        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo("Navigator initialized. Waiting for map and goal...")

    def goal_callback(self, msg):
        rospy.loginfo(f"Goal received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        
        if self.map_info is None or self.grid is None:
            rospy.logwarn("Map not available yet. Storing goal to process once map is loaded.")
            self.pending_goal = msg
            return
            
        self.process_goal(msg)

    def process_goal(self, goal_msg):
        """
        Helper function to process a PoseStamped goal message into grid coordinates and plan a path.
        """
        goal_x = int((goal_msg.pose.position.x - self.map_info.origin.position.x) / self.map_info.resolution)
        goal_y = int((goal_msg.pose.position.y - self.map_info.origin.position.y) / self.map_info.resolution)
        
        goal_x = max(0, min(goal_x, self.map_info.width - 1))
        goal_y = max(0, min(goal_y, self.map_info.height - 1))
        
        self.goal = (goal_y, goal_x)
        rospy.loginfo(f"New goal set: world({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}) -> grid{self.goal}")
        self.plan_global_path()


    def map_callback(self, msg):
        if self.map_info is not None:
            return
            
        rospy.loginfo("Map callback triggered!")
        try:
            width = msg.info.width
            height = msg.info.height
            self.map_info = msg.info
            data = np.array(msg.data).reshape((height, width))
            
            raw_grid = np.where(data < 50, 0, 1)
            
            safety_radius = 6  # Reduced from 8 to allow more feasible paths
            self.grid = self.inflate_obstacles(raw_grid, safety_radius)
            
            rospy.loginfo(f"Map loaded: {width}x{height}, safety margin: {safety_radius * msg.info.resolution:.2f}m")
            
            if self.pending_goal is not None:
                rospy.loginfo("Processing pending goal now that map is available.")
                self.process_goal(self.pending_goal)
                self.pending_goal = None

        except Exception as e:
            rospy.logerr(f"Failed to process map: {e}")
            self.map_info = None
            self.grid = None

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        self.robot_pose = (position.x, position.y, yaw)

        current_pos = np.array([position.x, position.y])
        last_pos = np.array(self.last_robot_position_for_stuck_check)
        if np.linalg.norm(current_pos - last_pos) > self.stuck_distance_threshold:
            self.last_progress_time = rospy.Time.now()
            self.last_robot_position_for_stuck_check = (position.x, position.y)
            if self.in_recovery_mode:
                rospy.loginfo("Robot has moved, exiting recovery mode.")
                self.in_recovery_mode = False

    def scan_callback(self, msg):
        self.scan = msg

    def inflate_obstacles(self, grid, radius):
        from scipy import ndimage
        try:
            structure = ndimage.generate_binary_structure(2, 1)
            inflated = ndimage.binary_dilation(grid, structure=structure, iterations=radius)
            return inflated.astype(int)
        except ImportError:
            rospy.logwarn("scipy not available, using simple obstacle inflation")
            return grid

    def plan_global_path(self):
        if self.grid is None or self.goal is None or self.map_info is None:
            rospy.logwarn("Cannot plan path: Missing grid, goal, or map info.")
            return
            
        start_x = int((self.robot_pose[0] - self.map_info.origin.position.x) / self.map_info.resolution)
        start_y = int((self.robot_pose[1] - self.map_info.origin.position.y) / self.map_info.resolution)
        start = (start_y, start_x)
        
        rospy.loginfo(f"Planning new path from grid {start} to {self.goal}")
        
        try:
            path_grid = a_star(self.grid, start, self.goal)
            if path_grid:
                self.waypoints = []
                for i, grid_point in enumerate(path_grid):
                    if i % 5 == 0 or i == len(path_grid) - 1:  # Balanced waypoint density
                        world_x = grid_point[1] * self.map_info.resolution + self.map_info.origin.position.x
                        world_y = grid_point[0] * self.map_info.resolution + self.map_info.origin.position.y
                        self.waypoints.append((world_x, world_y))
                
                rospy.loginfo(f"Path found with {len(self.waypoints)} waypoints.")
                self.publish_path()
            else:
                rospy.logwarn("Path planning failed. Trying intermediate goal approach...")
                self.try_intermediate_goal()
        except Exception as e:
            rospy.logerr(f"Path planning failed: {e}")
            self.try_intermediate_goal()

    def try_intermediate_goal(self):
        """
        When direct path planning fails, try to find an intermediate reachable point
        closer to the goal and navigate there first.
        """
        if self.goal is None or self.map_info is None:
            return
            
        rospy.loginfo("Trying to find intermediate reachable goal...")
        
        start_x = int((self.robot_pose[0] - self.map_info.origin.position.x) / self.map_info.resolution)
        start_y = int((self.robot_pose[1] - self.map_info.origin.position.y) / self.map_info.resolution)
        start = (start_y, start_x)
        
        goal_y, goal_x = self.goal
        
        # Try points along the line from robot to goal, starting from closer points
        for fraction in [0.3, 0.5, 0.7]:
            intermediate_x = int(start_x + fraction * (goal_x - start_x))
            intermediate_y = int(start_y + fraction * (goal_y - start_y))
            
            # Ensure the intermediate point is within map bounds
            intermediate_x = max(0, min(intermediate_x, self.map_info.width - 1))
            intermediate_y = max(0, min(intermediate_y, self.map_info.height - 1))
            
            intermediate = (intermediate_y, intermediate_x)
            
            try:
                # Use the same safe grid for intermediate goals
                path_grid = a_star(self.grid, start, intermediate)
                if path_grid:
                    self.waypoints = []
                    for i, grid_point in enumerate(path_grid):
                        if i % 5 == 0 or i == len(path_grid) - 1:  # Same waypoint density
                            world_x = grid_point[1] * self.map_info.resolution + self.map_info.origin.position.x
                            world_y = grid_point[0] * self.map_info.resolution + self.map_info.origin.position.y
                            self.waypoints.append((world_x, world_y))
                    
                    rospy.loginfo(f"Found intermediate path with {len(self.waypoints)} waypoints (fraction {fraction}).")
                    self.publish_path()
                    return
            except Exception as e:
                continue
        
        rospy.logwarn("Could not find any reachable intermediate goal. Robot may need manual intervention.")
        self.waypoints = []

    def publish_path(self):
        if not self.waypoints: return
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def control_loop(self, event):
        if self.scan is None or not self.waypoints or self.in_recovery_mode:
            if not self.waypoints:
                 rospy.loginfo_throttle(5, "Waiting for a goal to generate waypoints...")
            return

        if rospy.Time.now() - self.last_progress_time > self.stuck_check_interval:
            rospy.logwarn("Robot appears to be stuck! Entering recovery mode.")
            self.in_recovery_mode = True
            self.cmd_pub.publish(Twist())
            rospy.Timer(rospy.Duration(0.1), self.execute_recovery, oneshot=True)
            return

        min_range = min(self.scan.ranges)
        
        if self.in_obstacle_mode and min_range > self.obstacle_exit_threshold:
            self.in_obstacle_mode = False
            rospy.loginfo("Exiting obstacle avoidance mode.")
        elif not self.in_obstacle_mode and min_range < self.obstacle_enter_threshold:
            self.in_obstacle_mode = True
            self.last_obstacle_mode_time = rospy.Time.now()
            self.last_robot_pose_in_avoidance = self.robot_pose[:2]  # Store position when entering avoidance
            rospy.loginfo("Entering obstacle avoidance mode.")
        
        # Check if robot has been in obstacle avoidance mode too long OR is stuck
        if self.in_obstacle_mode:
            time_in_avoidance = rospy.Time.now() - self.last_obstacle_mode_time
            
            # Check for deadlock: robot hasn't moved much in avoidance mode
            if (self.last_robot_pose_in_avoidance is not None and 
                time_in_avoidance > rospy.Duration(2.0)):  # Check after 2 seconds
                current_pos = np.array(self.robot_pose[:2])
                last_pos = np.array(self.last_robot_pose_in_avoidance)
                movement = np.linalg.norm(current_pos - last_pos)
                
                if movement < self.avoidance_deadlock_threshold:
                    rospy.logwarn("Robot deadlocked in obstacle avoidance! Forcing exit and replan.")
                    self.in_obstacle_mode = False
                    self.plan_global_path()
                    return
            
            # Time-based timeout
            if time_in_avoidance > self.obstacle_mode_duration_limit:
                rospy.logwarn("Robot stuck in obstacle avoidance too long. Forcing path replan.")
                self.in_obstacle_mode = False
                self.plan_global_path()
                return

        if self.in_obstacle_mode:
            points = laser_to_cartesian(self.scan)
            rep_force = compute_repulsive_force(points)
            
            current_goal_wp = self.waypoints[0]
            att_force = compute_attractive_force(self.robot_pose[:2], current_goal_wp)
            
            # Smart force combination to prevent deadlocks
            rep_magnitude = np.linalg.norm(rep_force)
            att_magnitude = np.linalg.norm(att_force)
            
            if rep_magnitude > 0.02:  # If there's significant repulsive force
                # Check if forces are opposing each other (deadlock situation)
                if rep_magnitude > 0.05 and att_magnitude > 0.05:
                    force_dot = np.dot(rep_force, att_force) / (rep_magnitude * att_magnitude)
                    if force_dot < -0.5:  # Forces are opposing (angle > 120 degrees)
                        # Prioritize obstacle avoidance, minimal attraction
                        total_force = rep_force + 0.1 * att_force
                    else:
                        # Forces are aligned or perpendicular, normal combination
                        total_force = rep_force + 0.3 * att_force
                else:
                    total_force = rep_force + 0.2 * att_force
            else:
                # No significant obstacles, follow attraction
                total_force = att_force
                
            # Emergency stop if forces are canceling each other out
            total_force_magnitude = np.linalg.norm(total_force)
            if (rep_magnitude > 0.05 and att_magnitude > 0.05 and 
                total_force_magnitude < 0.03):  # Forces are canceling out
                rospy.logwarn("Force deadlock detected! Stopping and replanning.")
                self.cmd_pub.publish(Twist())  # Stop robot
                self.in_obstacle_mode = False
                self.plan_global_path()
                return
                
            cmd = force_to_cmd(total_force, self.robot_pose[2])
            rospy.loginfo_throttle(1, f"AVOIDANCE: min_range={min_range:.2f}m, rep_mag={rep_magnitude:.2f}")
        else:
            goal_wp = self.waypoints[0]
            cmd, rho = compute_tracking_cmd(self.robot_pose, goal_wp)
            rospy.loginfo_throttle(1, f"TRACKING: distance to waypoint={rho:.2f}m")

            if rho < 0.25:  # More forgiving waypoint acceptance
                self.waypoints.pop(0)
                if not self.waypoints:
                    rospy.loginfo("Goal reached!")
                    self.cmd_pub.publish(Twist())
                    return
                else:
                    rospy.loginfo(f"Waypoint reached, {len(self.waypoints)} remaining.")
        
        self.cmd_pub.publish(cmd)

    def execute_recovery(self, event):
        rospy.loginfo("Executing recovery: Backing up and turning...")
        
        # First, back up
        recovery_cmd = Twist()
        recovery_cmd.linear.x = -0.12
        end_time = rospy.Time.now() + rospy.Duration(2.5)  # Back up longer
        
        rate = rospy.Rate(10)
        while rospy.Time.now() < end_time:
            self.cmd_pub.publish(recovery_cmd)
            rate.sleep()
            
        # Then turn to find a new direction
        recovery_cmd = Twist()
        recovery_cmd.angular.z = 0.4  # Turn in place
        end_time = rospy.Time.now() + rospy.Duration(3.0)  # Turn for 3 seconds
        
        while rospy.Time.now() < end_time:
            self.cmd_pub.publish(recovery_cmd)
            rate.sleep()
            
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.5)
        
        rospy.loginfo("Recovery maneuver complete. Attempting to replan global path.")
        self.plan_global_path()
        
        self.last_progress_time = rospy.Time.now()
        self.last_robot_position_for_stuck_check = self.robot_pose[:2]
        self.in_recovery_mode = False

if __name__ == "__main__":
    try:
        Navigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass