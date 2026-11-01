cmake_minimum_required(VERSION 3.0.2)
project(collab_planner)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(LAPACK REQUIRED) 

option(WITH_HSL "Compile solver with coinhsl" ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(HSL)
if(NOT ${HSL_FOUND})
  set(WITH_HSL OFF)
endif()

if(WITH_HSL)
  add_definitions(-DWITH_HSL)
endif()

include_directories(
  ./
  include
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  KeyboardInput.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_executable(keyboard_input_server keyboard_input/keyboard_input_server.cpp)
add_executable(keyboard_input keyboard_input/keyboard_input_client.cpp)

add_dependencies(keyboard_input_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(keyboard_input ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(keyboard_input_server ${catkin_LIBRARIES})
target_link_libraries(keyboard_input ${catkin_LIBRARIES})

catkin_package(
 INCLUDE_DIRS .
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
 DEPENDS Boost
)

set(SRC
  "common/math/aabox2d.cc"
  "common/math/box2d.cc"
  "common/math/line_segment2d.cc"
  "common/math/linear_interpolation.cc"
  "common/math/math_utils.cc"
  "common/math/polygon2d.cc"
  "common/math/trajectory1d.cc"
  "common/math/vec2d.cc"
  "common/util/point_cloud.cc"
  "common/util/color.cc"
  "trajectory_nlp/collab_traj_nlp.cc"
  "trajectory_nlp/collab_traj_optimizer.cpp"
  visualization_plot.cpp)

add_library(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PUBLIC
  ./
  include/nlohmann/
  ${catkin_INCLUDE_DIRS})

find_library(CASADI_LIBRARY casadi PATHS /usr/local/lib)
target_link_libraries(${PROJECT_NAME} ${CASADI_LIBRARY} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node collab_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
