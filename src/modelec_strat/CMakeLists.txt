cmake_minimum_required(VERSION 3.8)
project(modelec_strat)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(modelec_interfaces REQUIRED)
find_package(modelec_utils REQUIRED)

add_executable(strat_fsm src/strat_fms.cpp src/navigation_helper.cpp src/pathfinding.cpp src/action_executor.cpp src/mission_manager.cpp src/missions/prepare_concert_mission.cpp src/missions/promotion_mission.cpp src/missions/go_home_mission.cpp src/obstacle/obstacle.cpp src/obstacle/column.cpp src/deposite_zone.cpp)
ament_target_dependencies(strat_fsm rclcpp std_msgs std_srvs modelec_interfaces ament_index_cpp)
target_link_libraries(strat_fsm modelec_utils::utils modelec_utils::config)
target_include_directories(strat_fsm PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_executable(enemy_manager src/enemy_manager.cpp)
ament_target_dependencies(enemy_manager rclcpp std_msgs sensor_msgs std_srvs modelec_interfaces ament_index_cpp)
target_link_libraries(enemy_manager modelec_utils::utils modelec_utils::config)
target_include_directories(enemy_manager PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(TARGETS
        strat_fsm
        enemy_manager
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY data/
        DESTINATION share/${PROJECT_NAME}/data
        FILES_MATCHING PATTERN "*.xml"
)

ament_package()
