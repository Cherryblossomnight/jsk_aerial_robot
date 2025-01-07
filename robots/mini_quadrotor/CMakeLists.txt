cmake_minimum_required(VERSION 3.0.2)
project(mini_quadrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  aerial_robot_control
  aerial_robot_model
  mujoco_ros_control
  spinal
  eigen_conversions
  tf_conversions
  std_srvs
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include test
  LIBRARIES quad_robot_model 
  CATKIN_DEPENDS roscpp aerial_robot_msgs aerial_robot_control aerial_robot_model spinal eigen_conversions tf_conversions std_srvs
)

# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  test
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(quad_robot_model
  src/quad_robot_model.cpp)
target_link_libraries(quad_robot_model ${catkin_LIBRARIES})

add_library(quad_controller_pluginlib
  src/quad_impedance_controller.cpp)
target_link_libraries(quad_controller_pluginlib quad_robot_model ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

add_dependencies(quad_controller_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/ test/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY config scripts plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()

