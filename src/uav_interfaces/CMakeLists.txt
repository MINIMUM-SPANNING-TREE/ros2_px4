cmake_minimum_required(VERSION 3.8)
project(uav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 声明消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UavState.msg"
  "msg/UavPose.msg"
  "msg/UavHealth.msg"
  "msg/LslidarInformation.msg"
  "msg/LslidarPacket.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TrackDetection.msg"
  "msg/TrackDetectionArray.msg"
  "srv/Takeoff.srv"
  "srv/Land.srv"
  "srv/Move.srv"
  "srv/Arm.srv"
  "srv/SetMode.srv"
  "srv/Rtl.srv"
  "srv/AngleDistortionCorrection.srv"
  "srv/FrameRate.srv"
  "srv/InvalidData.srv"
  "srv/IpAndPort.srv"
  "srv/MotorControl.srv"
  "srv/MotorSpeed.srv"
  "srv/PowerControl.srv"
  "srv/RfdRemoval.srv"
  "srv/StandbyMode.srv"
  "srv/TailRemoval.srv"
  "srv/TimeMode.srv"
  "srv/NavSetGoal.srv"
  "srv/NavAddWaypoint.srv"
  "srv/NavStart.srv"
  "srv/NavStop.srv"
  "srv/NavGetStatus.srv"
  "srv/NavClearWaypoints.srv"
  "srv/MoveRelative.srv"
  "srv/SetYaw.srv"
  "srv/SetVelocity.srv"
  "srv/SetMaxSpeed.srv"
  "srv/EmergencyLand.srv"
  "srv/EmergencyStop.srv"
  "srv/TrackTarget.srv"
  DEPENDENCIES std_msgs builtin_interfaces
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
