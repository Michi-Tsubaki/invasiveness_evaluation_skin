cmake_minimum_required(VERSION 3.8)
project(invasiveness_evaluation_skin)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Int32MultiArrayWithHeader.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  scripts/iip_butterworth_filter.py
  scripts/iip_calibration.py
  scripts/iip_strain_heatmap.py
  scripts/iip_pain_visualizer.py
  scripts/iip_publisher.py
  scripts/iip_serial_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  launch/IIP_bringup.launch.py
  launch/IIP_bringup_butterworth.launch.py
  launch/IIP_heatmap.launch.xml
  launch/IIP_pain_arrows.launch.xml
  launch/IIP_pain_plot.launch.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  config/iip_heatmap.rviz
  config/iip_pain_arrows.rviz
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES arduino_sketch/IIPArduinoReadSerial/IIPArduinoReadSerial.ino
  DESTINATION share/${PROJECT_NAME}/arduino_sketch/IIPArduinoReadSerial
)

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
