# message_connect

```
Finished <<< costmap_converter [2min 43s]
Starting >>> teb_local_planner
[Processing: nav2_behaviors, nav2_controller, nav2_smac_planner, teb_local_planner]                                            
[Processing: nav2_behaviors, nav2_controller, nav2_smac_planner, teb_local_planner]                                            
--- stderr: teb_local_planner                                                                                                  
CMake Warning:
  Manually-specified variables were not used by the project:

    HUMBLE_ROS
    ROS_EDITION


In file included from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/vertex_pose.h:52,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/timed_elastic_band.h:57,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/timed_elastic_band.cpp:39:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In member function ‘void teb_local_planner::PoseSE2::averageInPlace(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:282:19: error: ‘average_angle’ is not a member of ‘g2o’
  282 |     _theta = g2o::average_angle(pose1._theta, pose2._theta);
      |                   ^~~~~~~~~~~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In static member function ‘static teb_local_planner::PoseSE2 teb_local_planner::PoseSE2::average(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:295:66: error: ‘average_angle’ is not a member of ‘g2o’
  295 |     return PoseSE2( (pose1._position + pose2._position)/2 , g2o::average_angle(pose1._theta, pose2._theta) );
      |                                                                  ^~~~~~~~~~~~~
In file included from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/robot_footprint_model.h:43,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_config.h:48,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:46,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:42:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In member function ‘void teb_local_planner::PoseSE2::averageInPlace(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:282:19: error: ‘average_angle’ is not a member of ‘g2o’
  282 |     _theta = g2o::average_angle(pose1._theta, pose2._theta);
      |                   ^~~~~~~~~~~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In static member function ‘static teb_local_planner::PoseSE2 teb_local_planner::PoseSE2::average(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:295:66: error: ‘average_angle’ is not a member of ‘g2o’
  295 |     return PoseSE2( (pose1._position + pose2._position)/2 , g2o::average_angle(pose1._theta, pose2._theta) );
      |                                                                  ^~~~~~~~~~~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/src/recovery_behaviors.cpp: In member function ‘bool teb_local_planner::FailureDetector::detect(double, double)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/src/recovery_behaviors.cpp:102:26: error: ‘sign’ is not a member of ‘g2o’
  102 |         if ( i>0 && g2o::sign(buffer_[i].omega) != g2o::sign(buffer_[i-1].omega) )
      |                          ^~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/src/recovery_behaviors.cpp:102:57: error: ‘sign’ is not a member of ‘g2o’
  102 |         if ( i>0 && g2o::sign(buffer_[i].omega) != g2o::sign(buffer_[i-1].omega) )
      |                                                         ^~~~
gmake[2]: *** [CMakeFiles/teb_local_planner.dir/build.make:132: CMakeFiles/teb_local_planner.dir/src/recovery_behaviors.cpp.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
In file included from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/robot_footprint_model.h:43,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_config.h:48,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:46,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:50,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In member function ‘void teb_local_planner::PoseSE2::averageInPlace(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:282:19: error: ‘average_angle’ is not a member of ‘g2o’
  282 |     _theta = g2o::average_angle(pose1._theta, pose2._theta);
      |                   ^~~~~~~~~~~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In static member function ‘static teb_local_planner::PoseSE2 teb_local_planner::PoseSE2::average(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:295:66: error: ‘average_angle’ is not a member of ‘g2o’
  295 |     return PoseSE2( (pose1._position + pose2._position)/2 , g2o::average_angle(pose1._theta, pose2._theta) );
      |                                                                  ^~~~~~~~~~~~~
In file included from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/robot_footprint_model.h:43,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_config.h:48,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_config.cpp:39:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In member function ‘void teb_local_planner::PoseSE2::averageInPlace(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:282:19: error: ‘average_angle’ is not a member of ‘g2o’
  282 |     _theta = g2o::average_angle(pose1._theta, pose2._theta);
      |                   ^~~~~~~~~~~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In static member function ‘static teb_local_planner::PoseSE2 teb_local_planner::PoseSE2::average(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:295:66: error: ‘average_angle’ is not a member of ‘g2o’
  295 |     return PoseSE2( (pose1._position + pose2._position)/2 , g2o::average_angle(pose1._theta, pose2._theta) );
      |                                                                  ^~~~~~~~~~~~~
In file included from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:65,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:42:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h: In member function ‘virtual void teb_local_planner::EdgeKinematicsDiffDrive::linearizeOplus()’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h:132:30: error: ‘sign’ is not a member of ‘g2o’
  132 |     double dev_nh_abs = g2o::sign( ( cos(conf1->theta())+cos(conf2->theta()) ) * deltaS[1] -
      |                              ^~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h:132:30: note: suggested alternatives:
In file included from /usr/include/eigen3/Eigen/Core:380,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_config.h:45,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:46,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:42:
/usr/include/eigen3/Eigen/src/Core/GlobalFunctions.h:100:3: note:   ‘Eigen::sign’
  100 |   EIGEN_ARRAY_DECLARE_GLOBAL_UNARY(sign,scalar_sign_op,sign (or 0),\sa ArrayBase::sign)
      |   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/kdl/frames.hpp:129,
                 from /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp:54,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:51,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/robot_footprint_model.h:43,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_config.h:48,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:46,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:42:
/usr/include/kdl/utilities/utility.h:251:15: note:   ‘KDL::sign’
  251 | inline double sign(double arg) {
      |               ^~~~
In file included from /usr/include/spdlog/fmt/fmt.h:25,
                 from /usr/include/spdlog/common.h:36,
                 from /usr/include/spdlog/spdlog.h:12,
                 from /usr/local/include/g2o/stuff/logger.h:34,
                 from /usr/local/include/g2o/core/block_solver.hpp:33,
                 from /usr/local/include/g2o/core/block_solver.h:204,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:55,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:42:
/usr/include/fmt/core.h:2024:11: note:   ‘fmt::v8::sign’
 2024 | namespace sign {
      |           ^~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp: In member function ‘void teb_local_planner::TebOptimalPlanner::extractVelocity(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&, double, double&, double&, double&) const’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:1116:24: error: ‘sign’ is not a member of ‘g2o’
 1116 |     vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
      |                        ^~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:1116:24: note: suggested alternatives:
In file included from /usr/include/eigen3/Eigen/Core:380,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_config.h:45,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:46,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:42:
/usr/include/eigen3/Eigen/src/Core/GlobalFunctions.h:100:3: note:   ‘Eigen::sign’
  100 |   EIGEN_ARRAY_DECLARE_GLOBAL_UNARY(sign,scalar_sign_op,sign (or 0),\sa ArrayBase::sign)
      |   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/kdl/frames.hpp:129,
                 from /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp:54,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:51,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/robot_footprint_model.h:43,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_config.h:48,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:46,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:42:
/usr/include/kdl/utilities/utility.h:251:15: note:   ‘KDL::sign’
  251 | inline double sign(double arg) {
      |               ^~~~
In file included from /usr/include/spdlog/fmt/fmt.h:25,
                 from /usr/include/spdlog/common.h:36,
                 from /usr/include/spdlog/spdlog.h:12,
                 from /usr/local/include/g2o/stuff/logger.h:34,
                 from /usr/local/include/g2o/core/block_solver.hpp:33,
                 from /usr/local/include/g2o/core/block_solver.h:204,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:55,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/optimal_planner.cpp:42:
/usr/include/fmt/core.h:2024:11: note:   ‘fmt::v8::sign’
 2024 | namespace sign {
      |           ^~~~
gmake[2]: *** [CMakeFiles/teb_local_planner.dir/build.make:76: CMakeFiles/teb_local_planner.dir/src/timed_elastic_band.cpp.o] Error 1
gmake[2]: *** [CMakeFiles/teb_local_planner.dir/build.make:146: CMakeFiles/teb_local_planner.dir/src/teb_config.cpp.o] Error 1
In file included from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:65,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:50,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h: In member function ‘virtual void teb_local_planner::EdgeKinematicsDiffDrive::linearizeOplus()’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h:132:30: error: ‘sign’ is not a member of ‘g2o’
  132 |     double dev_nh_abs = g2o::sign( ( cos(conf1->theta())+cos(conf2->theta()) ) * deltaS[1] -
      |                              ^~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h:132:30: note: suggested alternatives:
In file included from /usr/include/kdl/frames.hpp:129,
                 from /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp:54,
                 from /home/user/ros2_ws/install/nav2_costmap_2d/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_publisher.hpp:55,
                 from /home/user/ros2_ws/install/nav2_costmap_2d/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp:48,
                 from /home/user/ros2_ws/install/nav2_core/include/nav2_core/controller.hpp:42,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:47,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/usr/include/kdl/utilities/utility.h:251:15: note:   ‘KDL::sign’
  251 | inline double sign(double arg) {
      |               ^~~~
In file included from /usr/include/eigen3/Eigen/Core:380,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_config.h:45,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:46,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:50,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/usr/include/eigen3/Eigen/src/Core/GlobalFunctions.h:100:3: note:   ‘Eigen::sign’
  100 |   EIGEN_ARRAY_DECLARE_GLOBAL_UNARY(sign,scalar_sign_op,sign (or 0),\sa ArrayBase::sign)
      |   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/spdlog/fmt/fmt.h:25,
                 from /usr/include/spdlog/common.h:36,
                 from /usr/include/spdlog/spdlog.h:12,
                 from /usr/local/include/g2o/stuff/logger.h:34,
                 from /usr/local/include/g2o/core/block_solver.hpp:33,
                 from /usr/local/include/g2o/core/block_solver.h:204,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:55,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:50,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/usr/include/fmt/core.h:2024:11: note:   ‘fmt::v8::sign’
 2024 | namespace sign {
      |           ^~~~
In file included from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/planner_interface.h:49,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/homotopy_class_planner.h:58,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/homotopy_class_planner.cpp:39:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In member function ‘void teb_local_planner::PoseSE2::averageInPlace(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:282:19: error: ‘average_angle’ is not a member of ‘g2o’
  282 |     _theta = g2o::average_angle(pose1._theta, pose2._theta);
      |                   ^~~~~~~~~~~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h: In static member function ‘static teb_local_planner::PoseSE2 teb_local_planner::PoseSE2::average(const teb_local_planner::PoseSE2&, const teb_local_planner::PoseSE2&)’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:295:66: error: ‘average_angle’ is not a member of ‘g2o’
  295 |     return PoseSE2( (pose1._position + pose2._position)/2 , g2o::average_angle(pose1._theta, pose2._theta) );
      |                                                                  ^~~~~~~~~~~~~
In file included from /opt/ros/humble/include/pluginlib/pluginlib/class_loader_imp.hpp:57,
                 from /opt/ros/humble/include/pluginlib/pluginlib/class_loader.hpp:334,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:42,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/home/user/ros2_ws/install/costmap_converter/include/costmap_converter/costmap_converter_interface.h: In member function ‘void costmap_converter::BaseCostmapToPolygons::startWorker(rclcpp::GenericRate<std::chrono::_V2::system_clock>::SharedPtr, nav2_costmap_2d::Costmap2D*, bool)’:
/home/user/ros2_ws/install/costmap_converter/include/costmap_converter/costmap_converter_interface.h:197:41: warning: too many arguments for format [-Wformat-extra-args]
  197 |         RCLCPP_DEBUG(nh_->get_logger(), "costmap_converter", "Spinning up a thread for the CostmapToPolygons plugin");
      |                                         ^~~~~~~~~~~~~~~~~~~
/home/user/ros2_ws/install/costmap_converter/include/costmap_converter/costmap_converter_interface.h: In member function ‘void costmap_converter::BaseCostmapToDynamicObstacles::loadStaticCostmapConverterPlugin(const string&, rclcpp::Node::SharedPtr)’:
/home/user/ros2_ws/install/costmap_converter/include/costmap_converter/costmap_converter_interface.h:313:32: warning: format ‘%s’ expects argument of type ‘char*’, but argument 5 has type ‘const string’ {aka ‘const std::__cxx11::basic_string<char>’} [-Wformat=]
  313 |       RCLCPP_INFO(getLogger(), "CostmapToDynamicObstacles: underlying costmap conversion plugin for static obstacles %s loaded.", plugin_name);
      |                                ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:68,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/home/user/ros2_ws/install/costmap_converter/include/costmap_converter/costmap_converter_interface.h:313:119: note: format string is defined here
  313 |       RCLCPP_INFO(getLogger(), "CostmapToDynamicObstacles: underlying costmap conversion plugin for static obstacles %s loaded.", plugin_name);
      |                                                                                                                      ~^
      |                                                                                                                       |
      |                                                                                                                       char*
/home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp: In member function ‘double teb_local_planner::TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double, double, double, double) const’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:942:26: error: ‘sign’ is not a member of ‘g2o’
  942 |     radius = double(g2o::sign(radius)) * min_turning_radius;
      |                          ^~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:942:26: note: suggested alternatives:
In file included from /usr/include/kdl/frames.hpp:129,
                 from /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp:54,
                 from /home/user/ros2_ws/install/nav2_costmap_2d/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_publisher.hpp:55,
                 from /home/user/ros2_ws/install/nav2_costmap_2d/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp:48,
                 from /home/user/ros2_ws/install/nav2_core/include/nav2_core/controller.hpp:42,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:47,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/usr/include/kdl/utilities/utility.h:251:15: note:   ‘KDL::sign’
  251 | inline double sign(double arg) {
      |               ^~~~
In file included from /usr/include/eigen3/Eigen/Core:380,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_config.h:45,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:46,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:50,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/usr/include/eigen3/Eigen/src/Core/GlobalFunctions.h:100:3: note:   ‘Eigen::sign’
  100 |   EIGEN_ARRAY_DECLARE_GLOBAL_UNARY(sign,scalar_sign_op,sign (or 0),\sa ArrayBase::sign)
      |   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/spdlog/fmt/fmt.h:25,
                 from /usr/include/spdlog/common.h:36,
                 from /usr/include/spdlog/spdlog.h:12,
                 from /usr/local/include/g2o/stuff/logger.h:34,
                 from /usr/local/include/g2o/core/block_solver.hpp:33,
                 from /usr/local/include/g2o/core/block_solver.h:204,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:55,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:50,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/usr/include/fmt/core.h:2024:11: note:   ‘fmt::v8::sign’
 2024 | namespace sign {
      |           ^~~~
In file included from /usr/include/boost/lexical_cast/detail/inf_nan.hpp:34,
                 from /usr/include/boost/lexical_cast/detail/converter_lexical_streams.hpp:63,
                 from /usr/include/boost/lexical_cast/detail/converter_lexical.hpp:54,
                 from /usr/include/boost/lexical_cast/try_lexical_convert.hpp:44,
                 from /usr/include/boost/lexical_cast.hpp:32,
                 from /usr/include/boost/math/tools/convert_from_string.hpp:15,
                 from /usr/include/boost/math/constants/constants.hpp:14,
                 from /usr/include/boost/math/special_functions/gamma.hpp:24,
                 from /usr/include/boost/math/special_functions/detail/bessel_jy.hpp:14,
                 from /usr/include/boost/math/special_functions/bessel.hpp:20,
                 from /usr/include/boost/math/special_functions/airy.hpp:12,
                 from /usr/include/boost/math/special_functions.hpp:15,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/h_signature.h:42,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/homotopy_class_planner.hpp:40,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/homotopy_class_planner.h:589,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h:51,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/teb_local_planner_ros.cpp:39:
/usr/include/boost/math/special_functions/sign.hpp:163:12: note:   ‘boost::math::sign’
  163 | inline int sign BOOST_NO_MACRO_EXPAND(const T& z)
      |            ^~~~
In file included from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:65,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/homotopy_class_planner.h:61,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/homotopy_class_planner.cpp:39:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h: In member function ‘virtual void teb_local_planner::EdgeKinematicsDiffDrive::linearizeOplus()’:
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h:132:30: error: ‘sign’ is not a member of ‘g2o’
  132 |     double dev_nh_abs = g2o::sign( ( cos(conf1->theta())+cos(conf2->theta()) ) * deltaS[1] -
      |                              ^~~~
/home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/g2o_types/edge_kinematics.h:132:30: note: suggested alternatives:
In file included from /usr/include/kdl/frames.hpp:129,
                 from /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp:54,
                 from /home/user/ros2_ws/install/nav2_costmap_2d/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_publisher.hpp:55,
                 from /home/user/ros2_ws/install/nav2_costmap_2d/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp:48,
                 from /opt/ros/humble/include/dwb_core/trajectory_critic.hpp:44,
                 from /opt/ros/humble/include/dwb_critics/base_obstacle.hpp:42,
                 from /opt/ros/humble/include/dwb_critics/obstacle_footprint.hpp:39,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/planner_interface.h:45,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/homotopy_class_planner.h:58,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/homotopy_class_planner.cpp:39:
/usr/include/kdl/utilities/utility.h:251:15: note:   ‘KDL::sign’
  251 | inline double sign(double arg) {
      |               ^~~~
In file included from /usr/include/eigen3/Eigen/Core:380,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/pose_se2.h:44,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/planner_interface.h:49,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/homotopy_class_planner.h:58,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/homotopy_class_planner.cpp:39:
/usr/include/eigen3/Eigen/src/Core/GlobalFunctions.h:100:3: note:   ‘Eigen::sign’
  100 |   EIGEN_ARRAY_DECLARE_GLOBAL_UNARY(sign,scalar_sign_op,sign (or 0),\sa ArrayBase::sign)
      |   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/spdlog/fmt/fmt.h:25,
                 from /usr/include/spdlog/common.h:36,
                 from /usr/include/spdlog/spdlog.h:12,
                 from /usr/local/include/g2o/stuff/logger.h:34,
                 from /usr/local/include/g2o/core/block_solver.hpp:33,
                 from /usr/local/include/g2o/core/block_solver.h:204,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/optimal_planner.h:55,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/include/teb_local_planner/homotopy_class_planner.h:61,
                 from /home/user/ros2_ws/src/navigation2/teb_local_planner/src/homotopy_class_planner.cpp:39:
/usr/include/fmt/core.h:2024:11: note:   ‘fmt::v8::sign’
 2024 | namespace sign {
      |           ^~~~
gmake[2]: *** [CMakeFiles/teb_local_planner.dir/build.make:90: CMakeFiles/teb_local_planner.dir/src/optimal_planner.cpp.o] Error 1
gmake[2]: *** [CMakeFiles/teb_local_planner.dir/build.make:160: CMakeFiles/teb_local_planner.dir/src/homotopy_class_planner.cpp.o] Error 1
gmake[2]: *** [CMakeFiles/teb_local_planner.dir/build.make:174: CMakeFiles/teb_local_planner.dir/src/teb_local_planner_ros.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/teb_local_planner.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< teb_local_planner [1min 26s, exited with code 2]
Aborted  <<< nav2_behaviors [4min 31s]                                                                                                     
Aborted  <<< nav2_controller [4min 38s]                                                                                                   
Aborted  <<< nav2_smac_planner [5min 53s]                                             

Summary: 26 packages finished [21min 5s]
  1 package failed: teb_local_planner

```

