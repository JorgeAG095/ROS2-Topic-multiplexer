cmake_minimum_required(VERSION 3.8)
project(pixkit_multiplexer VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(YAML_CONFIG_PATH ${CMAKE_SOURCE_DIR}/config/multiplexer_config.yaml)

add_executable(pixkit_multiplexer src/mux.cpp include/pixkit_multiplexer/typeFunctions.cpp)

ament_target_dependencies(pixkit_multiplexer
  rclcpp 
  std_msgs
  geometry_msgs)

target_include_directories(pixkit_multiplexer PRIVATE ${CMAKE_SOURCE_DIR}/config)
target_include_directories(pixkit_multiplexer PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_definitions(pixkit_multiplexer PRIVATE YAML_CONFIG_PATH="${YAML_CONFIG_PATH}")

target_link_libraries(pixkit_multiplexer yaml-cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
