# ROS2 Bag Upgrade Tool
Upgrade recorded bag from older version of ros2 (e.g. dashing) to newer version of ros2 (e.g. foxy)

For example, a bag recorded in a dashing environment can be re-recorded in a foxy environment. 

For now, only works with one PointCloud2 and one Image topic - use as template to customize to any number of messages.


### Installation

1. Clone the repo to workspace
   ```sh
   cd ~/ros2_ws/src/
   ```
   ```sh
   git clone https://github.com/nhma20/rosbag2_upgrader.git
   ```
2. Colcon build package
   ```sh
   cd ~/ros2_ws/
   ```
   ```sh
   colcon build --packages-select rosbag2_upgrader
   ```


<!-- USAGE EXAMPLES -->
## Usage

0. Plug in IWR6843AOPEVM, make sure CLI and data ports match (default /dev/ttyUSB0 and /dev/ttyUSB1)
1. Run ros package (make sure /opt/ros/foxy/setup.bash and <ros2_workspace>/install/setup.bash are sourced)

   With parameters (ros2 run rosbag2_upgrader upgrade <bag_path> <cam_topic> <mmw_topic> <new_bag_dir>):
   ```sh 
   ros2 run rosbag2_upgrader upgrade /home/nm/Downloads/ros_bags/sim_topics /cable_camera/image_raw /iwr6843_pcl /home/nm/ros2_ws/test_bag/

   ```
   
## Dependencies  

Depends on the following Python3 packages:
- rosbags (pip install rosbags)
- cv_bridge (sudo apt-get install ros-$ROS_DISTRO-cv-bridge)
