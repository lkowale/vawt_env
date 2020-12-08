include_directories(
  include/
)

# VawtRobot Hardware Interface
add_library(vawt_robot_hw_interface
  src/vawt_robot_hw_interface.cpp
)
target_link_libraries(vawt_robot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(vawt_robot_hw_main src/vawt_robot_hw_main.cpp)
target_link_libraries(vawt_robot_hw_main
  vawt_robot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    vawt_robot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    vawt_robot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/vawt_robot_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
