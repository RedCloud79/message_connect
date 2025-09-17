# message_connect

```
--- stderr: autoware_motion_utils      
In this package, headers install destination is set to `include` by ament_auto_package. It is recommended to install `include/autoware_motion_utils` instead and will be the default behavior of ament_auto_package from ROS 2 Kilted Kaiju. On distributions before Kilted, ament_auto_package behaves the same way when you use USE_SCOPED_HEADER_INSTALL_DIR option.
CMake Warning:
  Manually-specified variables were not used by the project:

    HUMBLE_ROS
    ROS_EDITION


/home/user/ros2_ws/src/autoware_core/common/autoware_motion_utils/src/resample/resample.cpp:17:10: fatal error: autoware/interpolation/linear_interpolation.hpp: No such file or directory
   17 | #include "autoware/interpolation/linear_interpolation.hpp"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/autoware_motion_utils.dir/build.make:118: CMakeFiles/autoware_motion_utils.dir/src/resample/resample.cpp.o] Error 1
gmake[2]: *** Waiting for unfinished jobs....
/home/user/ros2_ws/src/autoware_core/common/autoware_motion_utils/src/trajectory/interpolation.cpp:17:10: fatal error: autoware/interpolation/linear_interpolation.hpp: No such file or directory
   17 | #include "autoware/interpolation/linear_interpolation.hpp"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/autoware_motion_utils.dir/build.make:146: CMakeFiles/autoware_motion_utils.dir/src/trajectory/interpolation.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:157: CMakeFiles/autoware_motion_utils.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< autoware_motion_utils [32.7s, exited with code 2]

```

