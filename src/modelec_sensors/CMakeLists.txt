cmake_minimum_required(VERSION 3.8)
project(modelec_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(modelec_interfaces REQUIRED)
find_package(modelec_utils REQUIRED)
find_library(WIRINGPI_LIB wiringPi)

add_executable(lidar_controller src/lidar_controller.cpp)
ament_target_dependencies(lidar_controller rclcpp std_msgs sensor_msgs modelec_interfaces)
target_link_libraries(lidar_controller modelec_utils::modelec_utils)
target_include_directories(lidar_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_executable(tirette_controller ../modelec_sensors/src/tirette_controller.cpp)
ament_target_dependencies(tirette_controller rclcpp std_msgs modelec_interfaces)
target_link_libraries(tirette_controller ${WIRINGPI_LIB} modelec_utils::modelec_utils)
target_include_directories(tirette_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS
        lidar_controller
        DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
        DESTINATION include/
)
ament_package()
