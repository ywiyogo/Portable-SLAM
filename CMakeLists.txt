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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(sense_hat_node src/sense_hat_node.cpp src/icm20948.cpp)
target_include_directories(sense_hat_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(sense_hat_node PUBLIC c_std_23 cxx_std_20)  

#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(sense_hat_node
    "rclcpp"
    "sensor_msgs"
    "std_srvs"
)
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

install(TARGETS sense_hat_node
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
