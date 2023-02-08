#
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(basalt_ros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED core imgproc calib3d highgui)

find_package(cereal REQUIRED)
find_package(Sophus REQUIRED)
find_package(basalt-headers REQUIRED)
find_package(basalt REQUIRED)

find_package(realsense2 QUIET)  # optional

# Do not pick up the system EIGEN3 since basalt is built with
# its own version of Eigen and you cannot mix and match Eigen versions.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# hopefully insisting on "MODULE" below causes system Eigen to be ignored
find_package(EIGEN3 MODULE REQUIRED)


set(ROS2_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "nav_msgs"
  "sensor_msgs"
  "message_filters"
  "image_transport"
  "tf2"
  "tf2_ros"
  "basalt_vio_msgs"
  "cv_bridge"
  "std_msgs")

foreach(pkg ${ROS2_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Flags used for CHECK_CXX_SOURCE_COMPILES
set(CMAKE_REQUIRED_FLAGS "-Wno-error")

# save flags passed by user
set(BASALT_PASSED_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")                                      # cmake default: "-O3 -DNDEBUG"
set(CMAKE_CXX_FLAGS_CIDEBUG  "-O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # CI version with no debug symbols
set(CMAKE_CXX_FLAGS_CIRELWITHDEBINFO "-O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # CI version with no debug symbols

# CRUCIAL: must match how basalt is compiled or you will get a segfault on startup!
set(MARCH_FLAGS "-march=native")

# base set of compile flags
set(BASALT_CXX_FLAGS "${MARCH_FLAGS} -Wall -Wextra -Werror -Wno-error=unused-parameter -ftemplate-backtrace-limit=0")

# OpenMP option and compile flags
option(USE_OPENMP "Use OpenMP (e.g. for parallel computation in Eigen)" ${USE_OPENMP_DEFAULT})
if(USE_OPENMP)
  message(STATUS "OpenMP Enabled")
  set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -fopenmp")
else()
  message(STATUS "OpenMP Disabled")
endif()

# setup combined compiler flags
set(CMAKE_CXX_FLAGS "-std=c++17 ${BASALT_CXX_FLAGS} ${BASALT_PASSED_CXX_FLAGS}")

#
# utils library
#
add_library(vio_utils SHARED
  src/utils.cpp)
target_link_libraries(vio_utils basalt::basalt)
ament_target_dependencies(vio_utils ${ROS2_DEPENDENCIES})
target_include_directories(vio_utils PUBLIC include)

#
# frontend library
#
add_library(vio_frontend SHARED
  src/vio_frontend.cpp
  src/image_subscriber.cpp
  src/optical_flow_publisher.cpp)

target_link_libraries(vio_frontend basalt::basalt Eigen3::Eigen TBB::tbb vio_utils)
ament_target_dependencies(vio_frontend ${ROS2_DEPENDENCIES})
target_include_directories(vio_frontend PUBLIC include)
rclcpp_components_register_nodes(vio_frontend "basalt_ros::VIOFrontEndNode")

#
# frontend node
#
add_executable(vio_frontend_node
  src/vio_frontend_node.cpp)
target_link_libraries(vio_frontend_node vio_frontend)


#
# backend library
#
add_library(vio_backend SHARED
  src/vio_backend.cpp
  src/vio_publisher.cpp
  src/imu_subscriber.cpp
  src/imu_publisher.cpp
  src/optical_flow_subscriber.cpp)

target_link_libraries(vio_backend basalt::basalt Eigen3::Eigen TBB::tbb vio_utils)
ament_target_dependencies(vio_backend ${ROS2_DEPENDENCIES})
target_include_directories(vio_backend PUBLIC include)

rclcpp_components_register_nodes(vio_backend "basalt_ros::VIOBackEndNode")

#
# backend node
#
add_executable(vio_backend_node
  src/vio_backend_node.cpp)
target_link_libraries(vio_backend_node vio_backend)

#
# test subscriber
#
add_executable(test_subscriber
  src/test_subscriber.cpp
  src/imu_subscriber.cpp)

target_link_libraries(test_subscriber basalt::basalt Eigen3::Eigen TBB::tbb)
ament_target_dependencies(test_subscriber ${ROS2_DEPENDENCIES})
target_include_directories(test_subscriber PRIVATE include)

add_executable(viz_flow_node
  src/viz_flow_node.cpp)

ament_target_dependencies(viz_flow_node ${ROS2_DEPENDENCIES})


# --- realsense tools ---
if(realsense2_FOUND)
  add_executable(get_calibration src/get_calibration.cpp)
  target_link_libraries(get_calibration basalt::basalt basalt::basalt_rs_t265
    ${TBB_LIBRARIES} realsense2::realsense2)
  install(TARGETS get_calibration
    DESTINATION lib/${PROJECT_NAME})
endif()

# the shared libraries go into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  vio_backend
  vio_frontend
  vio_utils
  DESTINATION lib)

install(TARGETS
  vio_frontend_node
  vio_backend_node
  viz_flow_node
  test_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_xmllint()
endif()

ament_package()
