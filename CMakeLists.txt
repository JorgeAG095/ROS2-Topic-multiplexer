cmake_minimum_required(VERSION 3.8)
project(multiplexer_node VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(MULTIPLEXER_CONFIG_PATH ${CMAKE_SOURCE_DIR}/config/multiplexer_config.yaml)
set(LOCK_CONFIG_PATH ${CMAKE_SOURCE_DIR}/config/locks_config.yaml)

add_executable(multiplexer_node src/mux.cpp include/multiplexer_node/typeFunctions.cpp)

ament_target_dependencies(multiplexer_node
  rclcpp 
  std_msgs
  geometry_msgs)

target_include_directories(multiplexer_node PRIVATE ${CMAKE_SOURCE_DIR}/config)
target_include_directories(multiplexer_node PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_compile_definitions(multiplexer_node PRIVATE MULTIPLEXER_CONFIG_PATH="${MULTIPLEXER_CONFIG_PATH}")
target_compile_definitions(multiplexer_node PRIVATE LOCK_CONFIG_PATH="${LOCK_CONFIG_PATH}")

target_link_libraries(multiplexer_node yaml-cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
