cmake_minimum_required(VERSION 3.16)
project(astrabot_teleop LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(astrabot_data_interfaces REQUIRED)
find_package(astrabot_rtc REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Protobuf REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TeleopCommand.msg"
  "msg/TeleopSessionStatus.msg"
  "srv/AcquireControlOwner.srv"
  "srv/RenewControlOwner.srv"
  "srv/ReleaseControlOwner.srv"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime geometry_msgs)

set(TELEOP_PROTO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/proto/TeleopFrame.proto")
protobuf_generate_cpp(TELEOP_PROTO_SOURCES TELEOP_PROTO_HEADERS "${TELEOP_PROTO_FILE}")

add_library(astrabot_teleop_proto STATIC
  ${TELEOP_PROTO_SOURCES}
  ${TELEOP_PROTO_HEADERS}
)
target_include_directories(astrabot_teleop_proto
  PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(astrabot_teleop_proto PUBLIC protobuf::libprotobuf)
target_compile_features(astrabot_teleop_proto PUBLIC cxx_std_17)

add_library(astrabot_teleop_protocol STATIC
  src/common/status.cpp
  src/protocol/crc32.cpp
  src/protocol/data_channel_contract.cpp
  src/protocol/teleop_frame_codec.cpp
  src/protocol/frame_validator.cpp
)
target_include_directories(astrabot_teleop_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(astrabot_teleop_protocol
  PUBLIC
    astrabot_teleop_proto
)
target_compile_features(astrabot_teleop_protocol PUBLIC cxx_std_17)

add_library(astrabot_teleop_core STATIC
  src/adapter/legacy_mpc_command_encoder.cpp
  src/config/teleop_config.cpp
  src/grant/grant_expiry.cpp
  src/grant/grant_verifier.cpp
  src/session/control_owner_lease_tracker.cpp
  src/session/run_context_generation_tracker.cpp
  src/session/teleop_status_report_scheduler.cpp
  src/session/teleop_state_machine.cpp
  src/session/session_registry.cpp
  src/safety/motion_watchdog.cpp
  src/safety/command_limiter.cpp
  src/mapping/arm_origin_mapper.cpp
  src/mapping/command_mapper.cpp
)
target_include_directories(astrabot_teleop_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(astrabot_teleop_core
  PUBLIC
    astrabot_teleop_protocol
    nlohmann_json::nlohmann_json
)
target_compile_features(astrabot_teleop_core PUBLIC cxx_std_17)
target_compile_definitions(astrabot_teleop_core PUBLIC JSON_NOEXCEPTION)
if(OpenSSL_FOUND)
  target_compile_definitions(astrabot_teleop_core PUBLIC ASTRABOT_TELEOP_HAS_OPENSSL=1)
  target_link_libraries(astrabot_teleop_core PUBLIC OpenSSL::Crypto)
else()
  target_compile_definitions(astrabot_teleop_core PUBLIC ASTRABOT_TELEOP_HAS_OPENSSL=0)
  message(WARNING "OpenSSL was not found: astrabot_teleop will compile but all grants fail closed")
endif()

add_executable(astrabot_teleop_node
  src/main.cpp
  src/runtime/teleop_node.cpp
  src/runtime/teleop_status_report_client.cpp
)
target_include_directories(astrabot_teleop_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# ament_target_dependencies 在 Jazzy 内部使用 plain signature；该 target 的 link 调用必须保持一致。
target_link_libraries(astrabot_teleop_node astrabot_teleop_core)
target_compile_features(astrabot_teleop_node PRIVATE cxx_std_17)
ament_target_dependencies(astrabot_teleop_node
  rclcpp
  geometry_msgs
  std_msgs
  astrabot_data_interfaces
  astrabot_rtc
)
rosidl_get_typesupport_target(teleop_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(astrabot_teleop_node "${teleop_typesupport_target}")

foreach(TELEOP_TARGET IN ITEMS astrabot_teleop_protocol astrabot_teleop_core astrabot_teleop_node)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${TELEOP_TARGET} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endforeach()

add_custom_target(no_exceptions_check
  COMMAND ${CMAKE_COMMAND}
    -DROOT=${CMAKE_CURRENT_SOURCE_DIR}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/no_exceptions_check.cmake
  COMMENT "Checking astrabot_teleop handwritten code for exceptions"
)

install(TARGETS
  astrabot_teleop_protocol
  astrabot_teleop_core
  astrabot_teleop_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch proto docs systemd DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # 团队标准以仓库 .clang-format 和 no_exceptions_check 为准，避免 ROS 默认格式器重复且冲突地改写代码。
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(teleop_protocol_test test/teleop_protocol_test.cpp)
  if(TARGET teleop_protocol_test)
    target_include_directories(teleop_protocol_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(teleop_protocol_test astrabot_teleop_protocol)
  endif()

  ament_add_gtest(teleop_core_test test/teleop_core_test.cpp)
  if(TARGET teleop_core_test)
    target_include_directories(teleop_core_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(teleop_core_test astrabot_teleop_core)
  endif()

  ament_add_gtest(command_mapper_safety_test test/command_mapper_safety_test.cpp)
  if(TARGET command_mapper_safety_test)
    target_include_directories(command_mapper_safety_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(command_mapper_safety_test astrabot_teleop_core)
  endif()

  ament_add_gtest(legacy_mpc_command_encoder_test test/legacy_mpc_command_encoder_test.cpp)
  if(TARGET legacy_mpc_command_encoder_test)
    target_include_directories(legacy_mpc_command_encoder_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(legacy_mpc_command_encoder_test astrabot_teleop_core)
  endif()

  ament_add_gtest(control_owner_lease_tracker_test test/control_owner_lease_tracker_test.cpp)
  if(TARGET control_owner_lease_tracker_test)
    target_include_directories(control_owner_lease_tracker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(control_owner_lease_tracker_test astrabot_teleop_core)
  endif()

  ament_add_gtest(run_context_generation_tracker_test test/run_context_generation_tracker_test.cpp)
  if(TARGET run_context_generation_tracker_test)
    target_include_directories(run_context_generation_tracker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(run_context_generation_tracker_test astrabot_teleop_core)
  endif()

  ament_add_gtest(teleop_status_report_scheduler_test test/teleop_status_report_scheduler_test.cpp)
  if(TARGET teleop_status_report_scheduler_test)
    target_include_directories(teleop_status_report_scheduler_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(teleop_status_report_scheduler_test astrabot_teleop_core)
  endif()

  ament_add_gtest(grant_verifier_test test/grant_verifier_test.cpp)
  if(TARGET grant_verifier_test)
    target_include_directories(grant_verifier_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(grant_verifier_test astrabot_teleop_core)
  endif()

  ament_add_gtest(teleop_interface_contract_test test/teleop_interface_contract_test.cpp)
  if(TARGET teleop_interface_contract_test)
    target_include_directories(teleop_interface_contract_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(teleop_interface_contract_test astrabot_teleop_core)
    ament_target_dependencies(teleop_interface_contract_test astrabot_data_interfaces)
    target_compile_definitions(teleop_interface_contract_test
      PRIVATE ASTRABOT_TELEOP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
  endif()

  ament_add_gtest(teleop_node_ros_integration_test
    test/teleop_node_ros_integration_test.cpp
    src/runtime/teleop_node.cpp
    src/runtime/teleop_status_report_client.cpp
  )
  if(TARGET teleop_node_ros_integration_test)
    target_include_directories(teleop_node_ros_integration_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(teleop_node_ros_integration_test astrabot_teleop_core "${teleop_typesupport_target}")
    ament_target_dependencies(teleop_node_ros_integration_test
      rclcpp
      geometry_msgs
      std_msgs
      astrabot_data_interfaces
      astrabot_rtc
    )
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(teleop_node_ros_integration_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()
  endif()
endif()

ament_package()
