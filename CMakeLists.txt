cmake_minimum_required(VERSION 2.8.3)
project(sensorblue_data_collection)

find_package(catkin REQUIRED COMPONENTS
  rosduct
  std_msgs
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/sensorblue_action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir scripts )
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}/)
endforeach(dir)
