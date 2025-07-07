# 🧭 assignment_turtlebot3

A ROS1 package for autonomous navigation of TurtleBot3 using a **custom A\* global path planner**, **potential field-based local obstacle avoidance**, and a **differential-drive kinematic controller**. Built entirely from scratch in Python — no ROS Navigation Stack used.


---

## 📁 Package Structure

---

## 🚀 Features

✅ A* global path planner  
✅ Potential fields for local obstacle avoidance  
✅ Kinematic controller for smooth motion  
✅ Full Python-based implementation  
✅ Compatible with TurtleBot3 and Gazebo  
✅ RViz visualization and map loading

---

## 📦 Dependencies

Make sure you have the following ROS1 packages installed (tested on **ROS Noetic**):

- `rospy`
- `geometry_msgs`
- `nav_msgs`
- `tf`
- `sensor_msgs`
- `std_msgs`
- `turtlebot3_description`
- `turtlebot3_gazebo`
- `turtlebot3_navigation` *(optional, only for model support)*

Install missing dependencies with:

```bash
rosdep install --from-paths src --ignore-src -r -y

cd ~/catkin_ws
catkin_make
source devel/setup.bash

roslaunch assignment_turtlebot3 navigation.launch



