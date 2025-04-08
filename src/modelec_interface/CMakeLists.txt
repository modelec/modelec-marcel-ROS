cmake_minimum_required(VERSION 3.8)
project(modelec_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Odometry/OdometryPos.msg"
  "msg/Odometry/OdometrySpeed.msg"
  "msg/Odometry/OdometryToF.msg"
  "msg/Odometry/OdometryWaypointReach.msg"
  "msg/Odometry/OdometryAddWaypoint.msg"
  "msg/Odometry/OdometryStart.msg"
  "msg/Odometry/PID/OdometryPid.msg"
  "msg/PCA9685Servo.msg"
  "msg/ServoMode.msg"
  "msg/Solenoid.msg"
  "msg/Button.msg"
  "srv/Alim/AlimOut.srv"
  "srv/Alim/AlimIn.srv"
  "srv/Alim/AlimTemp.srv"
  "srv/Alim/AlimBau.srv"
  "srv/Alim/AlimEmg.srv"
  "srv/Odometry/OdometryPosition.srv"
  "srv/Odometry/OdometrySpeed.srv"
  "srv/Odometry/OdometryToF.srv"
  "srv/Odometry/OdometryStart.srv"
  "srv/Odometry/OdometryAddWaypoint.srv"
  "srv/Odometry/PID/OdometryGetPid.srv"
  "srv/Odometry/PID/OdometrySetPid.srv"
  "srv/AddServoMotor.srv"
  "srv/AddSolenoid.srv"
  "srv/Tirette.srv"
  "srv/AddButton.srv"
  "srv/Button.srv"
  "srv/AddSerialListener.srv"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
