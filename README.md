# message_connect

root@lite3:/home/user/ros2_ws/src/livox_ros_driver2/launch_ROS2# ros2 launch msg_MID360.launch.py ouput_data_type:=0
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-08-21-09-27-32-040078-lite3-21659
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [livox_ros_driver2_node-1]: process started with pid [21660]
[livox_ros_driver2_node-1] [INFO] [1755768452.156843878] [livox_lidar_publisher2]: Livox Ros Driver2 Version: 1.2.4
[livox_ros_driver2_node-1] [INFO] [1755768452.157678273] [livox_lidar_publisher2]: Data Source is raw lidar.
[livox_ros_driver2_node-1] [INFO] [1755768452.157725826] [livox_lidar_publisher2]: Config file : /home/user/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
[livox_ros_driver2_node-1] LdsLidar *GetInstance
[livox_ros_driver2_node-1] config lidar type: 8
[livox_ros_driver2_node-1] successfully parse base config, counts: 1
[livox_ros_driver2_node-1] [INFO] [1755768452.163388886] [livox_lidar_publisher2]: Init lds lidar success!
[livox_ros_driver2_node-1] GetFreeIndex key:livox_lidar_50357258.
[livox_ros_driver2_node-1] Init queue, real query size:16.
[livox_ros_driver2_node-1] Lidar[0] storage queue size: 10
[livox_ros_driver2_node-1] set pcl data type, handle: 50357258, data type: 1
[livox_ros_driver2_node-1] set scan pattern, handle: 50357258, scan pattern: 0
[livox_ros_driver2_node-1] begin to change work mode to 'Normal', handle: 50357258
[livox_ros_driver2_node-1] successfully set data type, handle: 50357258, set_bit: 2
[livox_ros_driver2_node-1] successfully set pattern mode, handle: 50357258, set_bit: 0
[livox_ros_driver2_node-1] successfully set lidar attitude, ip: 10.100.0.3
[livox_ros_driver2_node-1] successfully change work mode, handle: 50357258
[livox_ros_driver2_node-1] successfully enable Livox Lidar imu, ip: 10.100.0.3
[livox_ros_driver2_node-1] [INFO] [1755768455.164144475] [livox_lidar_publisher2]: livox/imu publish use imu format
[livox_ros_driver2_node-1] [INFO] [1755768455.168650474] [livox_lidar_publisher2]: livox/lidar publish use PointCloud2 format

