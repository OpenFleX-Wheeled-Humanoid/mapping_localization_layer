cmake_minimum_required(VERSION 3.8)
project(icp_registration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/scancontext
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/icp_registration.cpp
  third_party/scancontext/Scancontext.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN icp::IcpNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
