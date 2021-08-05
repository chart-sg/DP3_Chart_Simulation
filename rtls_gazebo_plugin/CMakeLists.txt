cmake_minimum_required(VERSION 3.5)

project(rtls_gazebo_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(${CMAKE_PROJECT_NAME} SHARED rtls_gazebo_plugin.cc)
target_link_libraries(${CMAKE_PROJECT_NAME} ${GAZEBO_LIBRARIES})

install(
  TARGETS ${CMAKE_PROJECT_NAME}
  DESTINATION lib)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
  
ament_package()