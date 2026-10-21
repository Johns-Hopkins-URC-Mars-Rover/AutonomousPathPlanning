cmake_minimum_required(VERSION 3.8)
project(rover_imu_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Core: no ROS dependency.
add_library(imu_fusion_core
  src/imu_transform.cpp
  src/imu_ekf.cpp
  src/imu_initializer.cpp
  src/imu_fusion.cpp
)
target_compile_features(imu_fusion_core PUBLIC cxx_std_17)
target_include_directories(imu_fusion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_fusion_core PUBLIC Eigen3::Eigen)

add_executable(zed_imu_node src/zed_imu_node.cpp)
target_compile_features(zed_imu_node PRIVATE cxx_std_17)
target_link_libraries(zed_imu_node imu_fusion_core)
ament_target_dependencies(zed_imu_node rclcpp sensor_msgs tf2_ros tf2_eigen)

add_executable(imu_fusion_node src/imu_fusion_node.cpp)
target_compile_features(imu_fusion_node PRIVATE cxx_std_17)
target_link_libraries(imu_fusion_node imu_fusion_core)
ament_target_dependencies(imu_fusion_node rclcpp sensor_msgs diagnostic_msgs tf2_ros tf2_eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS imu_fusion_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS zed_imu_node imu_fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_imu_transform test/test_imu_transform.cpp)
  target_link_libraries(test_imu_transform imu_fusion_core)

  ament_add_gtest(test_imu_ekf test/test_imu_ekf.cpp)
  target_link_libraries(test_imu_ekf imu_fusion_core)

  ament_add_gtest(test_imu_fusion test/test_imu_fusion.cpp)
  target_link_libraries(test_imu_fusion imu_fusion_core)
endif()

ament_package()
