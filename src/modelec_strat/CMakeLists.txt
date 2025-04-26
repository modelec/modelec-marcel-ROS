cmake_minimum_required(VERSION 3.8)
project(modelec_strat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(rclcpp REQUIRED)

find_package(modelec_interfaces REQUIRED)
find_package(modelec_utils REQUIRED)

add_executable(strat_fsm src/strat_fms.cpp src/navigation_helper.cpp src/pathfinding.cpp src/action_executor.cpp src/mission_manager.cpp)
ament_target_dependencies(strat_fsm rclcpp std_msgs std_srvs modelec_interfaces)
target_link_libraries(strat_fsm modelec_utils::modelec_utils tinyxml2::tinyxml2)
target_include_directories(strat_fsm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  strat_fsm
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
