cmake_minimum_required(VERSION 3.8)
project(modelec_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

find_package(tinyxml2 REQUIRED)

# === modelec::config ===
add_library(config
        src/config.cpp
)

target_include_directories(config PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(config PUBLIC tinyxml2::tinyxml2)

# === modelec::utils ===
add_library(utils
        src/utils.cpp
        src/point.cpp
)

target_include_directories(utils PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Si utils dépend de config, décommente :
# target_link_libraries(${PROJECT_NAME} PUBLIC modelec_config)

# === Install headers ===
install(DIRECTORY include/
        DESTINATION include/
)

# === Install and export config lib ===
install(TARGETS config
        EXPORT configTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(EXPORT configTargets
        FILE configTargets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/config
)

# === Install and export utils lib ===
install(TARGETS utils
        EXPORT utilsTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(EXPORT utilsTargets
        FILE utilsTargets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/utils
)

# === Export metadata for ament ===
ament_export_targets(utilsTargets)
ament_export_targets(configTargets)
ament_export_include_directories(include)

ament_package()
