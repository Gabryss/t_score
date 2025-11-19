cmake_minimum_required(VERSION 3.8)
project(RoughSense)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-std=c++17 -O3 -g -ggdb)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Import ROS related packages
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Import PCL library
find_package(PCL 1.3 REQUIRED)
find_package(pcl_conversions REQUIRED)

# Import OpenCV library
find_package(OpenCV REQUIRED)

# Manually set Liquid-DSP include directory and library
# Adjust these paths if they differ on your system.
set(LIQUID_INCLUDE_DIRS "/usr/include/liquid")
set(LIQUID_LIBRARIES "/usr/lib/x86_64-linux-gnu/libliquid.so")

# Optionally print Liquid-DSP info for debugging
message(STATUS "Liquid-DSP include dirs: ${LIQUID_INCLUDE_DIRS}")
message(STATUS "Liquid-DSP libraries: ${LIQUID_LIBRARIES}")

include_directories(
                    include
                    include/RoughSense
                    ${PCL_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS}
                    ${LIQUID_INCLUDE_DIRS}
                    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



### Ros CMakelist ###
add_library(Roughness src/Roughness.cpp)
add_library(Ransac src/Ransac.cpp)
add_library(MapCreator src/MapCreator.cpp)
add_library(Dsp src/Dsp.cpp)
add_library(DynamicGrid src/DynamicGrid.cpp)



add_executable(roughness_node src/ROSWrapper.cpp)
ament_target_dependencies(roughness_node rclcpp tf2 tf2_ros sensor_msgs nav2_msgs std_msgs pcl_conversions)



target_link_libraries(roughness_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_SYSTEM_LIBRARY}
  ${LIQUID_LIBRARIES}
  Roughness
  Ransac
  MapCreator
  Dsp
  DynamicGrid
)

#Find the executable name
install(TARGETS
  roughness_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Only for test purposes
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
