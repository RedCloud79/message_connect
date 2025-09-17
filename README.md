# message_connect

```
--- stderr: autoware_pose_initializer       
CMake Warning:
  Manually-specified variables were not used by the project:

    HUMBLE_ROS
    ROS_EDITION


In file included from /home/user/ros2_ws/src/autoware_core/localization/autoware_pose_initializer/src/pose_initializer_core.cpp:23:
/home/user/ros2_ws/src/autoware_core/localization/autoware_pose_initializer/src/stop_check_module.hpp:18:10: fatal error: autoware/motion_utils/vehicle/vehicle_state_checker.hpp: No such file or directory
   18 | #include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/autoware_pose_initializer.dir/build.make:76: CMakeFiles/autoware_pose_initializer.dir/src/pose_initializer_core.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:178: CMakeFiles/autoware_pose_initializer.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< autoware_pose_initializer [13.2s, exited with code 2]


```

