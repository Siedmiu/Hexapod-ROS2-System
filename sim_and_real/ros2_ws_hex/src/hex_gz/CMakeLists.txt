################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(hex_gz)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch worlds rviz scripts_m scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES scripts/kwadrat.txt
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/one_leg.py
  scripts/initial_positions_publisher.py
  scripts/setup.py
  scripts/tri_gate.py
  scripts/tri_gate_contact_sensor.py
  scripts/imu_processer.py
  scripts/wifi_conect.py
  scripts/imu_data.py
  scripts/bi_gate.py
  scripts/wave_gate.py
  scripts/wave_gate_square.py
  scripts/ripple_gate.py
  scripts/rotation.py
  scripts/point_to_point.py
  scripts/ws_run.py
  scripts/dance.py
  scripts/imu_topic_data_plotter.py
  scripts/hi.py
  scripts/move_2_legs.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
