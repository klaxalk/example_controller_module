cmake_minimum_required(VERSION 3.1.2)
project(controller_module)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  mrs_msgs
  nav_msgs
  std_srvs
  nodelet
  mrs_uav_managers
  mrs_lib
  dynamic_reconfigure
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

generate_dynamic_reconfigure_options(
  cfg/controller_params.cfg
  )

catkin_package(
  CATKIN_DEPENDS roscpp nodelet mrs_msgs mrs_uav_managers mrs_lib std_srvs dynamic_reconfigure
  DEPENDS Eigen
  LIBRARIES ControllerModule
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# wrapper

add_library(ControllerModule
  src/controller_module.cpp
  )

add_dependencies(ControllerModule
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ControllerModule
  ${catkin_LIBRARIES}
  )
