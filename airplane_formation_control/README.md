# Airport Robot Gazebo
这是集成了小车手动控制、DLIO建图和定位、飞机手动控制的版本，下一个目标是在gazebo和rviz中集成小车和飞机，然后实现编队控制
## 环境

1. Ubuntu 20.04
2. Ros neotic
3. Gazebo 9.0


## 依赖包

`sudo apt update` \
`sudo apt install ros-neotic-ros-controllers` \
`sudo apt install ros-neotic-velodyne-simulator` \
`sudo apt install ros-neotic-key-teleop` 

## 运行指令

`roslaunch airport_robot_gazebo airport_robot.launch`

`rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel _forward_rate:=2.0 _backward_rate:=2.0 _rotation_rate:=1.0`

旋转激光雷达：
`roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=true pointcloud_topic:=/robot0/points_raw imu_topic:=/robot0/imu robot_namespace:=robot0`

`roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=true pointcloud_topic:=/robot1/points_raw imu_topic:=/robot1/processed_imu`

`roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=true pointcloud_topic:=/robot2/points_raw imu_topic:=/robot2/processed_imu`


双激光雷达：
`roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=true pointcloud_topic:=/robot0/fused_points imu_topic:=/robot0/processed_imu`


`rosservice call /robot/dlio_map/save_pcd 0.05 "/home/shenlu/airplane_ws/src/airplane_formation_control/config"`


