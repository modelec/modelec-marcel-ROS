cmake_minimum_required(VERSION 3.8)
project(modelec_gui)

# Enable all warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(modelec_interface REQUIRED)
find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)

# Add the executable
add_executable(modelec_gui src/modelec_gui.cpp src/main.cpp include/modelec_gui/modelec_gui.hpp)
ament_target_dependencies(modelec_gui
  rclcpp
  std_msgs
  modelec_interface
)
target_link_libraries(modelec_gui
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
)

target_include_directories(modelec_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS modelec_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Package finalization
ament_package()
