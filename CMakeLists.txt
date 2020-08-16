cmake_minimum_required(VERSION 3.0.2)
project(system_health_monitor)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  FILES
  SystemNodeHealth.msg
  NodeHealth.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
  ${PROJECT_NAME} 
  src/system_health_monitor_node.cpp
  src/SystemHealthMonitor.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)


# install(TARGETS ${PROJECT_NAME} system_health_monitor_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
