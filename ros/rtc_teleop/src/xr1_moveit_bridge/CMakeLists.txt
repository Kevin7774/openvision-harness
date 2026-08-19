cmake_minimum_required(VERSION 3.16)
project(xr1_moveit_bridge VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
# MoveIt 2.12.4 on this Jazzy image is built against Ubuntu's OctoMap 1.9.
# Resolve that ABI before moveit_core to avoid loading ROS OctoMap 1.10 too.
find_package(octomap 1.9 CONFIG REQUIRED PATHS /usr/share/octomap NO_DEFAULT_PATH)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)

function(xr1_moveit_enable_warnings target_name)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

add_library(xr1_moveit_validator_core STATIC
  src/validator.cpp
)
target_include_directories(xr1_moveit_validator_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(xr1_moveit_validator_core PUBLIC cxx_std_17)
target_link_libraries(xr1_moveit_validator_core nlohmann_json::nlohmann_json)
ament_target_dependencies(xr1_moveit_validator_core
  geometry_msgs
  moveit_core
  moveit_msgs
  shape_msgs
  srdfdom
  urdf
)
xr1_moveit_enable_warnings(xr1_moveit_validator_core)

add_executable(xr1_moveit_validator src/main.cpp)
target_compile_features(xr1_moveit_validator PRIVATE cxx_std_17)
target_link_libraries(xr1_moveit_validator PRIVATE xr1_moveit_validator_core)
xr1_moveit_enable_warnings(xr1_moveit_validator)

install(
  TARGETS xr1_moveit_validator_core
  EXPORT export_xr1_moveit_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS xr1_moveit_validator RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

set(no_exceptions_script "${CMAKE_CURRENT_SOURCE_DIR}/../astrabot_rtc/cmake/no_exceptions_check.cmake")
if(EXISTS "${no_exceptions_script}")
  add_custom_target(xr1_moveit_no_exceptions_check
    COMMAND ${CMAKE_COMMAND} -DROOT=${CMAKE_CURRENT_SOURCE_DIR} -P ${no_exceptions_script}
    COMMENT "Checking handwritten C++ exception keywords"
    VERBATIM
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(xr1_moveit_validator_contract_test test/validator_contract_test.cpp)
  target_link_libraries(xr1_moveit_validator_contract_test xr1_moveit_validator_core)
  xr1_moveit_enable_warnings(xr1_moveit_validator_contract_test)
endif()

ament_export_targets(export_xr1_moveit_bridge HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core)
ament_package()
