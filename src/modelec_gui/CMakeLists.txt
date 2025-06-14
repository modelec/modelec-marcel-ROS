cmake_minimum_required(VERSION 3.8)
project(modelec_gui)

# Enable all warnings
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(modelec_interfaces REQUIRED)
find_package(modelec_utils REQUIRED)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        Svg
        REQUIRED)

# Add the executable
add_executable(modelec_gui
        resource.qrc

        src/main.cpp
        src/modelec_gui.cpp
        src/pages/odo_page.cpp
        src/pages/home_page.cpp
        src/pages/test_map_page.cpp
        src/pages/map_page.cpp
        src/pages/action_page.cpp
        src/pages/alim_page.cpp
        src/widget/action_widget.cpp

        include/modelec_gui/modelec_gui.hpp
        include/modelec_gui/pages/odo_page.hpp
        include/modelec_gui/pages/home_page.hpp
        include/modelec_gui/pages/test_map_page.hpp
        include/modelec_gui/pages/map_page.hpp
        include/modelec_gui/pages/action_page.hpp
        include/modelec_gui/pages/alim_page.hpp
        include/modelec_gui/widget/action_widget.hpp
)

ament_target_dependencies(modelec_gui
        rclcpp
        std_msgs
        std_srvs
        modelec_interfaces
        ament_index_cpp
)
target_link_libraries(modelec_gui
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::Svg
        modelec_utils::utils
        modelec_utils::config
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
