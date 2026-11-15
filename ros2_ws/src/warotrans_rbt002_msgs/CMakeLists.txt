cmake_minimum_required(VERSION 3.8)
project(warotrans_rbt002_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorSetpoints.msg"
  "msg/RrcLiteStatus.msg"
  "msg/ImuRaw.msg"
  "msg/EncoderFeedback.msg"
  "msg/UltrasonicRaw.msg"
  "msg/MotorState.msg"
  "msg/SafetyState.msg"
  "srv/SetMode.srv"
  "srv/CalibrateMotor.srv"
  "srv/SetSpeedLimit.srv"
  DEPENDENCIES std_msgs
)

ament_package()
