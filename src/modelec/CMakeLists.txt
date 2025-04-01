cmake_minimum_required(VERSION 3.8)
project(modelec)

# Enable all warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(modelec_interface REQUIRED) 
find_library(WIRINGPI_LIB wiringPi)

# USB Arduino Listener Node
add_executable(serial_listener src/multiple_serial_listener.cpp)
ament_target_dependencies(serial_listener rclcpp std_msgs modelec_interface)
target_link_libraries(serial_listener Boost::system)
target_include_directories(serial_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# USB Arduino Logic Processor Node
add_executable(odometry_logic_processor src/odometry_logic_processor.cpp)
ament_target_dependencies(odometry_logic_processor rclcpp std_msgs modelec_interface)
target_include_directories(odometry_logic_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# USB Arduino Logic Processor Node
add_executable(pcb_alim_interface src/pcb_alim_interface.cpp)
ament_target_dependencies(pcb_alim_interface rclcpp std_msgs modelec_interface)
target_include_directories(pcb_alim_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# PCA9685 Listener Node
add_executable(pca9685_controller src/pca9685_controller.cpp)
ament_target_dependencies(pca9685_controller rclcpp std_msgs modelec_interface)
target_link_libraries(pca9685_controller ${WIRINGPI_LIB})
target_include_directories(pca9685_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(gamecontroller_listener src/gamecontroller_listener.cpp)
ament_target_dependencies(gamecontroller_listener rclcpp std_msgs sensor_msgs modelec_interface)
target_include_directories(gamecontroller_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(move_controller src/move_controller.cpp)
ament_target_dependencies(move_controller rclcpp std_msgs sensor_msgs modelec_interface)
target_include_directories(move_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(tirette_controller src/tirette_controller.cpp)
ament_target_dependencies(tirette_controller rclcpp std_msgs modelec_interface)
target_link_libraries(tirette_controller ${WIRINGPI_LIB})
target_include_directories(tirette_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(solenoid_controller src/solenoid_controller.cpp)
ament_target_dependencies(solenoid_controller rclcpp modelec_interface)
target_link_libraries(solenoid_controller ${WIRINGPI_LIB})
target_include_directories(solenoid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(arm_controller src/arm_controller.cpp)
ament_target_dependencies(arm_controller rclcpp std_msgs modelec_interface)
target_link_libraries(arm_controller ${WIRINGPI_LIB})
target_include_directories(arm_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(button_gpio_controller src/button_gpio_controller.cpp)
ament_target_dependencies(button_gpio_controller rclcpp std_msgs modelec_interface)
target_link_libraries(button_gpio_controller ${WIRINGPI_LIB})
target_include_directories(button_gpio_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(lidar_controller src/lidar_controller.cpp)
ament_target_dependencies(lidar_controller rclcpp std_msgs sensor_msgs modelec_interface)
target_include_directories(lidar_controller PUBLIC
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
  serial_listener
  odometry_logic_processor
  pca9685_controller
  gamecontroller_listener
  move_controller
  tirette_controller
  solenoid_controller
  arm_controller
  button_gpio_controller
  pcb_alim_interface
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
