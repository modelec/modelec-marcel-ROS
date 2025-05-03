cmake_minimum_required(VERSION 3.8)
project(modelec_com)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(modelec_interfaces REQUIRED)
find_package(modelec_utils REQUIRED)
find_library(WIRINGPI_LIB wiringPi)

# USB Arduino Listener Node
add_executable(serial_listener src/multiple_serial_listener.cpp)
ament_target_dependencies(serial_listener rclcpp std_msgs modelec_interfaces)
target_link_libraries(serial_listener Boost::system modelec_utils::utils)
target_include_directories(serial_listener PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# USB Arduino Logic Processor Node
add_executable(pcb_odo_interface src/pcb_odo_interface.cpp)
ament_target_dependencies(pcb_odo_interface rclcpp std_msgs modelec_interfaces)
target_link_libraries(pcb_odo_interface modelec_utils::utils)
target_include_directories(pcb_odo_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# USB Arduino Logic Processor Node
add_executable(pcb_alim_interface src/pcb_alim_interface.cpp)
ament_target_dependencies(pcb_alim_interface rclcpp std_msgs modelec_interfaces)
target_link_libraries(pcb_alim_interface modelec_utils::utils)
target_include_directories(pcb_alim_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# PCA9685 Listener Node
add_executable(pca9685_controller src/pca9685_controller.cpp)
ament_target_dependencies(pca9685_controller rclcpp std_msgs modelec_interfaces)
target_link_libraries(pca9685_controller ${WIRINGPI_LIB} modelec_utils::utils)
target_include_directories(pca9685_controller PUBLIC
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
        pcb_alim_interface
        pcb_odo_interface
        pca9685_controller
        DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
        DESTINATION include/
)

ament_package()
