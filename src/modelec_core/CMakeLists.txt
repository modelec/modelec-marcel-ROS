cmake_minimum_required(VERSION 3.8)
project(modelec_core)

# Enable all warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(modelec_interfaces REQUIRED)
find_package(modelec_utils REQUIRED)
find_library(WIRINGPI_LIB wiringPi)

add_executable(gamecontroller_listener src/gamecontroller_listener.cpp)
ament_target_dependencies(gamecontroller_listener rclcpp std_msgs sensor_msgs modelec_interfaces)
target_link_libraries(gamecontroller_listener modelec_utils::utils)
target_include_directories(gamecontroller_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(solenoid_controller src/solenoid_controller.cpp)
ament_target_dependencies(solenoid_controller rclcpp modelec_interfaces)
target_link_libraries(solenoid_controller ${WIRINGPI_LIB} modelec_utils::utils)
target_include_directories(solenoid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(arm_controller src/arm_controller.cpp)
ament_target_dependencies(arm_controller rclcpp std_msgs modelec_interfaces)
target_link_libraries(arm_controller ${WIRINGPI_LIB} modelec_utils::utils)
target_include_directories(arm_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(button_gpio_controller src/button_gpio_controller.cpp)
ament_target_dependencies(button_gpio_controller rclcpp std_msgs modelec_interfaces)
target_link_libraries(button_gpio_controller ${WIRINGPI_LIB} modelec_utils::utils)
target_include_directories(button_gpio_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(speed_result src/speed_result.cpp)
ament_target_dependencies(speed_result rclcpp std_msgs sensor_msgs modelec_interfaces)
target_link_libraries(speed_result modelec_utils::utils)
target_include_directories(speed_result PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Enable testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS
  gamecontroller_listener
  solenoid_controller
  arm_controller
  button_gpio_controller
  speed_result
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Finalize package
ament_package()
