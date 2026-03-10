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
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_index_cpp REQUIRED)
find_package(fmt)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(${LIBCAMERA_INCLUDE_DIRS})
link_directories(${LIBCAMERA_LIBRARY_DIRS})
add_definitions(${LIBCAMERA_CFLAGS_OTHER})

find_package(modelec_interfaces REQUIRED)
find_package(modelec_utils REQUIRED)

# PCB Odometry Node
add_executable(pcb_odo_interface src/pcb_odo_interface.cpp src/serial_listener.cpp)
ament_target_dependencies(pcb_odo_interface rclcpp std_msgs sensor_msgs modelec_interfaces ament_index_cpp)
target_link_libraries(pcb_odo_interface Boost::system modelec_utils::utils modelec_utils::config)
target_include_directories(pcb_odo_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# PCB Action Node
add_executable(pcb_action_interface src/pcb_action_interface.cpp src/serial_listener.cpp)
ament_target_dependencies(pcb_action_interface rclcpp std_msgs modelec_interfaces ament_index_cpp)
target_link_libraries(pcb_action_interface Boost::system modelec_utils::utils modelec_utils::config fmt::fmt)
target_include_directories(pcb_action_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Color Detector Node
add_executable(color_detector
  src/color_detector.cpp
)

ament_target_dependencies(color_detector
  rclcpp
  std_msgs
  std_srvs
  modelec_interfaces
  ament_index_cpp
)

target_link_libraries(color_detector
  ${OpenCV_LIBS}
  modelec_utils::utils
  modelec_utils::config
  ${LIBCAMERA_LIBRARIES}
  cam2opencv
)

target_include_directories(color_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
        pcb_odo_interface
        pcb_action_interface
        color_detector
        DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
        DESTINATION include/
)

ament_package()
