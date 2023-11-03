# turtlebot3_localization-2D Kalman Filter

## Special thanks to **Ebrahim Abdelghafar** for creating this package. You can find his GitHub profile [here](https://github.com/ebrahimabdelghfar).



# Project
This project focuses on localization for the TurtleBot3 robot using ROS (Robot Operating System). It provides the necessary packages and configuration for accurate robot localization in a simulated environment, achieved through the implementation of 2D Kalman filter.

## Prerequisites

Before getting started, ensure you have the following prerequisites installed on your system:

- ROS (Robot Operating System) of the desired distribution (e.g., Melodic, Noetic, Humble, etc.)
- TurtleBot3 ROS messages: `sudo apt-get install ros-<distro>-turtlebot3-msgs`
- Gazebo for ROS simulation: `sudo apt-get install ros-<distro>-gazebo*`
- TurtleBot3 ROS package: `sudo apt-get install ros-<distro>-turtlebot3`
- TurtleBot3 teleoperation: `sudo apt-get install ros-<distro>-turtlebot3-teleop`
- TF transformations library: `sudo apt-get install ros-<distro>-tf*`
- Python 3 transforms3d library: `sudo pip3 install transforms3d`
- ROS plotJuggler for data visualization: `sudo apt-get install ros-<distro>-plotjuggler*`

Make sure to replace `<distro>` with your specific ROS distribution (e.g., Melodic, Noetic, etc.).

## Environment Configuration

Follow these steps to set up your environment:

1. Create a directory for your project:

   ```bash
   mkdir turtlebot3_localization
   cd turtlebot3_localization
   ```
2. Download the source files for this project into your workspace:

   ```bash
   git clone https://github.com/MostafaELFEEL/turtlebot3_localization.git
   ```
3. Building the package:
   ```bash
   colcon build
   ```
4. Running the code:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```
   
   ```bash
   ros2 run kalman_filter Noise
   ```
   
   ```bash
   ros2 run kalman_filter KalmanFilter
   ```
   
   **Use Plotjuggler to graph the ground truth, Noise, and Estimated pos**
   ```bash
   ros2 run plotjuggler plotjuggler
   ```

   **Move the robot with teleop**
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

## Results:
| Before Filter | After Filter |
| --- | --- |
| <img src="https://github.com/MostafaELFEEL/turtlebot3_localization/assets/106331831/c5e22376-b113-4604-851f-079827c749ad" width="400" /> | <img src="https://github.com/MostafaELFEEL/turtlebot3_localization/assets/106331831/0dffc1ca-9dbe-4f54-b0b0-1f0533abf680" width="400" /> |




   
   
   

