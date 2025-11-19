cmake_minimum_required(VERSION 3.8)
project(t_score)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set compile options
add_compile_options(-std=c++17 -O3 -g -ggdb)



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

include_directories(
                    include
                    include/t_score
                    ${PCL_INCLUDE_DIRS} 
                    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### Ros CMakelist ###
add_library(TScore src/TScore.cpp)
add_library(GridManager src/GridManager.cpp)


add_executable(t_score_node src/ROSWrapper.cpp)
ament_target_dependencies(t_score_node rclcpp tf2 tf2_ros sensor_msgs nav2_msgs std_msgs pcl_conversions)

target_link_libraries(t_score_node
  ${PCL_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  TScore
  GridManager
)

#Find the executable name
install(TARGETS
  t_score_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

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
