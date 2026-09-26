cmake_minimum_required(VERSION 3.10)
project(pgo)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Keep legacy dependency lookup behavior for newer CMake versions.
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 OLD)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

# Also propagate defaults to dependency scripts invoked via find_dependency().
set(CMAKE_POLICY_DEFAULT_CMP0144 OLD)
set(CMAKE_POLICY_DEFAULT_CMP0167 OLD)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(interface REQUIRED)
find_package(yaml-cpp REQUIRED)



include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/scancontext
)

set(SRC_LIST src/pgos/commons.cpp
             src/pgos/simple_pgo.cpp
             third_party/scancontext/Scancontext.cpp)

add_executable(pgo_node src/pgo_node.cpp ${SRC_LIST})
ament_target_dependencies(pgo_node rclcpp std_msgs sensor_msgs nav_msgs message_filters pcl_conversions tf2_ros geometry_msgs visualization_msgs interface)
target_link_libraries(pgo_node ${PCL_LIBRARIES} gtsam yaml-cpp)

add_executable(pcd_map_publisher src/pcd_map_publisher.cpp)
ament_target_dependencies(pcd_map_publisher rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pcd_map_publisher ${PCL_LIBRARIES})

add_executable(pcd_to_nav2_map src/pcd_to_nav2_map.cpp)
target_link_libraries(pcd_to_nav2_map ${PCL_LIBRARIES})

add_executable(rebuild_filtered_map src/rebuild_filtered_map.cpp)
target_link_libraries(rebuild_filtered_map ${PCL_LIBRARIES})

install(TARGETS pgo_node pcd_map_publisher pcd_to_nav2_map rebuild_filtered_map DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/pcd_to_nav2_map_gui DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})


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
