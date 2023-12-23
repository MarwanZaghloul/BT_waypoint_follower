cmake_minimum_required(VERSION 3.8)
project(bt_nav_client)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(behaviortree_cpp_v3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  behaviortree_cpp_v3
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base
)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp behaviortree_cpp_v3 actionlib actionlib_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/bt_action_client.cpp
  # Add other source files if needed
)

add_executable(move_base_bt_client src/bt_action_client.cpp)
target_link_libraries(move_base_bt_client  behaviortree_cpp_v3 ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME} move_base_bt_client
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

