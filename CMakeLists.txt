cmake_minimum_required(VERSION 3.5)
project(LiDAR_simulator_project)

# find dependencies
find_package(ament_cmake REQUIRED)

#############
## Include ##
#############

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(FILES
  bridge.yaml
  params.yaml
  rviz_config.rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY simulator/
  DESTINATION share/${PROJECT_NAME}/simulator
)

install(DIRECTORY sdf/
  DESTINATION share/${PROJECT_NAME}/sdf
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY tracks/
  DESTINATION share/${PROJECT_NAME}/tracks
)

ament_package()