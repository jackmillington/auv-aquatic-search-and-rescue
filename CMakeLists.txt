cmake_minimum_required(VERSION 3.0.2)
project(auv_search_rescue)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  message_generation
  roscpp
  roslib
  std_msgs
)

add_service_files(
  FILES
  UpdateGrid.srv
  Sensor.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    message_runtime
    roscpp
    roslib
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(update_grid src/update_grid.cpp)
target_link_libraries(update_grid ${catkin_LIBRARIES})
add_dependencies(update_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(search_rescue_search src/search_rescue_search.cpp)
target_link_libraries(search_rescue_search ${catkin_LIBRARIES})
add_dependencies(search_rescue_search ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(search_rescue_rescue src/search_rescue_rescue.cpp)
target_link_libraries(search_rescue_rescue ${catkin_LIBRARIES})
add_dependencies(search_rescue_rescue ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(search_rescue_manager src/search_rescue_manager.cpp)
target_link_libraries(search_rescue_manager ${catkin_LIBRARIES})
add_dependencies(search_rescue_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
