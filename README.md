# rosbag2_upgrader
Play ros2 bag from older ROS2 version in newer ROS2 version

For example, a bag recorded in a dashing environment can be replayed in a foxy environment.


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
1. Run ros package (make sure /opt/ros/dashing/setup.bash and <ros2_workspace>/install/setup.bash are sourced)

   With parameters (ros2 run rosbag2_upgrader upgrade <bag_path> <cam_topic> <mmw_topic>):
   ```sh
   ros2 run rosbag2_upgrader upgrade /home/nm/Downloads/PL_MAPPER_bags/sim_topics /cable_camera/image_raw /iwr6843_pcl
   ```
