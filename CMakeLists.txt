cmake_minimum_required(VERSION 3.8)
project(portable_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(GTSAM REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(sense_hat_node src/sense_hat_node.cpp src/icm20948.cpp)
add_executable(static_transform_node src/static_transform_node.cpp)
add_executable(lps22hb_node src/lps22hb_node.cpp src/lps22hb.cpp)
add_executable(gtsam_slam_node src/gtsam_slam_node.cpp)
target_include_directories(sense_hat_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(lps22hb_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(gtsam_slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${GTSAM_INCLUDE_DIR})

target_compile_features(sense_hat_node PUBLIC c_std_23 cxx_std_20)
target_compile_features(lps22hb_node PUBLIC c_std_23 cxx_std_20)
target_compile_features(gtsam_slam_node PUBLIC cxx_std_20)
target_compile_options(gtsam_slam_node PRIVATE -DEIGEN_DONT_VECTORIZE)

#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(sense_hat_node
    "rclcpp"
    "sensor_msgs"
    "std_srvs"
)
ament_target_dependencies(static_transform_node
    "rclcpp"
    "tf2_ros"
)
ament_target_dependencies(lps22hb_node
    "rclcpp"
    "sensor_msgs"
    "std_msgs"
)
ament_target_dependencies(gtsam_slam_node
    "rclcpp"
    "sensor_msgs"
    "nav_msgs"
    "tf2_ros"
)
target_link_libraries(gtsam_slam_node gtsam)
# Copy the launch files to the install folder
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Copy the config files to the install folder
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  )

install(TARGETS sense_hat_node static_transform_node lps22hb_node gtsam_slam_node
  DESTINATION lib/${PROJECT_NAME})

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
