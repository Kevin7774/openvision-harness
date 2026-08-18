cmake_minimum_required(VERSION 3.16)
project(astrabot_rtc VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ASTRABOT_RTC_ENABLE_LIBDATACHANNEL_TECH_GATE
  "Build the compile/runtime API gate against an installed LibDataChannel package" OFF)
option(ASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND
  "Build the production libdatachannel PeerConnection/DataChannel backend" OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)

set(message_files
  "msg/RtcDataPacket.msg"
  "msg/RtcPeerEvent.msg"
)
set(service_files
  "srv/AuthorizeDataChannel.srv"
  "srv/CloseRtcPeer.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  ${service_files}
)

function(astrabot_rtc_enable_warnings target_name)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

add_library(astrabot_rtc_protocol SHARED
  src/protocol/data_channel_contract.cpp
)
target_include_directories(astrabot_rtc_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(astrabot_rtc_protocol PUBLIC cxx_std_17)
astrabot_rtc_enable_warnings(astrabot_rtc_protocol)

add_library(astrabot_rtc_core SHARED
  src/config/rtc_config.cpp
  src/runtime/media_rate_tracker.cpp
  src/session/clock.cpp
  src/session/data_channel_router.cpp
  src/session/session_registry.cpp
)
target_include_directories(astrabot_rtc_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(astrabot_rtc_core PUBLIC cxx_std_17)
target_link_libraries(astrabot_rtc_core PUBLIC astrabot_rtc_protocol)
astrabot_rtc_enable_warnings(astrabot_rtc_core)

add_library(astrabot_rtc_media SHARED
  src/media/ffmpeg_h264_encoder.cpp
  src/media/h264_encoder_worker.cpp
  src/media/latest_frame_buffer.cpp
)
target_include_directories(astrabot_rtc_media
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(astrabot_rtc_media PUBLIC cxx_std_17)
target_link_libraries(astrabot_rtc_media PRIVATE PkgConfig::FFMPEG)
astrabot_rtc_enable_warnings(astrabot_rtc_media)

set(astrabot_rtc_webrtc_sources
  src/transport/webrtc/disabled_webrtc_transport.cpp
  src/transport/webrtc/peer_reconnect_tracker.cpp
  src/transport/webrtc/webrtc_transport_factory.cpp
)
if(ASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND)
  find_package(LibDataChannel REQUIRED CONFIG)
  find_package(nlohmann_json REQUIRED)
  list(APPEND astrabot_rtc_webrtc_sources
    src/transport/webrtc/libdatachannel_signaling.cpp
    src/transport/webrtc/libdatachannel_webrtc_transport.cpp
  )
endif()

add_library(astrabot_rtc_webrtc SHARED ${astrabot_rtc_webrtc_sources})
target_include_directories(astrabot_rtc_webrtc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(astrabot_rtc_webrtc PUBLIC cxx_std_17)
target_link_libraries(astrabot_rtc_webrtc
  PUBLIC
    astrabot_rtc_core
    astrabot_rtc_media
)
if(ASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND)
  target_compile_definitions(astrabot_rtc_webrtc PRIVATE ASTRABOT_RTC_HAS_LIBDATACHANNEL=1)
  target_link_libraries(astrabot_rtc_webrtc
    PRIVATE
      LibDataChannel::LibDataChannel
      nlohmann_json::nlohmann_json
  )
endif()
astrabot_rtc_enable_warnings(astrabot_rtc_webrtc)

add_executable(astrabot_rtc_node
  src/main.cpp
  src/runtime/rtc_node.cpp
  src/signaling/ros_signaling_adapter.cpp
)
target_include_directories(astrabot_rtc_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(astrabot_rtc_node PRIVATE cxx_std_17)
target_link_libraries(astrabot_rtc_node
  astrabot_rtc_core
  astrabot_rtc_media
  astrabot_rtc_protocol
  astrabot_rtc_webrtc
)
ament_target_dependencies(astrabot_rtc_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
rosidl_get_typesupport_target(astrabot_rtc_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(astrabot_rtc_node "${astrabot_rtc_typesupport_target}")
astrabot_rtc_enable_warnings(astrabot_rtc_node)

install(
  TARGETS
    astrabot_rtc_protocol
    astrabot_rtc_core
    astrabot_rtc_media
    astrabot_rtc_webrtc
  EXPORT export_astrabot_rtc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS astrabot_rtc_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch systemd DESTINATION share/${PROJECT_NAME})
if(ASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND)
  set(astrabot_rtc_capability_directory "${CMAKE_CURRENT_BINARY_DIR}/capabilities")
  file(MAKE_DIRECTORY "${astrabot_rtc_capability_directory}")
  file(WRITE "${astrabot_rtc_capability_directory}/libdatachannel.enabled"
    "backend=libdatachannel\nsoname=libdatachannel.so.0.24\n")
  install(FILES "${astrabot_rtc_capability_directory}/libdatachannel.enabled"
    DESTINATION share/${PROJECT_NAME}/capabilities)
endif()

if(ASTRABOT_RTC_ENABLE_LIBDATACHANNEL_TECH_GATE)
  find_package(LibDataChannel REQUIRED CONFIG)
  add_executable(astrabot_rtc_libdatachannel_tech_gate
    test/libdatachannel_tech_gate.cpp
  )
  target_compile_features(astrabot_rtc_libdatachannel_tech_gate PRIVATE cxx_std_17)
  target_link_libraries(astrabot_rtc_libdatachannel_tech_gate PRIVATE LibDataChannel::LibDataChannel)
  astrabot_rtc_enable_warnings(astrabot_rtc_libdatachannel_tech_gate)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(rtc_config_test test/rtc_config_test.cpp)
  target_link_libraries(rtc_config_test astrabot_rtc_core)
  target_include_directories(rtc_config_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  astrabot_rtc_enable_warnings(rtc_config_test)

  ament_add_gtest(session_registry_test test/session_registry_test.cpp)
  target_link_libraries(session_registry_test astrabot_rtc_core)
  target_include_directories(session_registry_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  astrabot_rtc_enable_warnings(session_registry_test)

  ament_add_gtest(data_channel_router_test test/data_channel_router_test.cpp)
  target_link_libraries(data_channel_router_test astrabot_rtc_core astrabot_rtc_protocol)
  target_include_directories(data_channel_router_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  astrabot_rtc_enable_warnings(data_channel_router_test)

  ament_add_gtest(media_rate_tracker_test test/media_rate_tracker_test.cpp)
  target_link_libraries(media_rate_tracker_test astrabot_rtc_core)
  target_include_directories(media_rate_tracker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  astrabot_rtc_enable_warnings(media_rate_tracker_test)

  ament_add_gtest(media_buffers_test test/media_buffers_test.cpp)
  target_link_libraries(media_buffers_test astrabot_rtc_media)
  target_include_directories(media_buffers_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  astrabot_rtc_enable_warnings(media_buffers_test)

  ament_add_gtest(ffmpeg_h264_encoder_test test/ffmpeg_h264_encoder_test.cpp)
  target_link_libraries(ffmpeg_h264_encoder_test astrabot_rtc_media)
  target_include_directories(ffmpeg_h264_encoder_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  astrabot_rtc_enable_warnings(ffmpeg_h264_encoder_test)

  ament_add_gtest(h264_encoder_worker_test test/h264_encoder_worker_test.cpp)
  target_link_libraries(h264_encoder_worker_test astrabot_rtc_media)
  target_include_directories(h264_encoder_worker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  astrabot_rtc_enable_warnings(h264_encoder_worker_test)

  ament_add_gtest(disabled_webrtc_transport_test test/disabled_webrtc_transport_test.cpp)
  target_link_libraries(disabled_webrtc_transport_test astrabot_rtc_webrtc)
  target_include_directories(disabled_webrtc_transport_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  astrabot_rtc_enable_warnings(disabled_webrtc_transport_test)

  ament_add_gtest(peer_reconnect_tracker_test test/peer_reconnect_tracker_test.cpp)
  target_link_libraries(peer_reconnect_tracker_test astrabot_rtc_webrtc)
  target_include_directories(peer_reconnect_tracker_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  astrabot_rtc_enable_warnings(peer_reconnect_tracker_test)

  ament_add_gtest(rtc_interface_contract_test test/rtc_interface_contract_test.cpp)
  target_compile_definitions(rtc_interface_contract_test PRIVATE RTC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  astrabot_rtc_enable_warnings(rtc_interface_contract_test)

  if(ASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND)
    ament_add_gtest(libdatachannel_signaling_test test/libdatachannel_signaling_test.cpp)
    target_link_libraries(libdatachannel_signaling_test
      astrabot_rtc_webrtc
      nlohmann_json::nlohmann_json
    )
    target_include_directories(libdatachannel_signaling_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    astrabot_rtc_enable_warnings(libdatachannel_signaling_test)

    ament_add_gtest(libdatachannel_transport_test test/libdatachannel_transport_test.cpp)
    target_link_libraries(libdatachannel_transport_test
      astrabot_rtc_webrtc
      LibDataChannel::LibDataChannel
      nlohmann_json::nlohmann_json
    )
    target_include_directories(libdatachannel_transport_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    astrabot_rtc_enable_warnings(libdatachannel_transport_test)
  endif()

  if(ASTRABOT_RTC_ENABLE_LIBDATACHANNEL_TECH_GATE)
    add_test(NAME libdatachannel_tech_gate COMMAND astrabot_rtc_libdatachannel_tech_gate)
  endif()
endif()

set(NO_EXCEPTIONS_CHECK_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/no_exceptions_check.cmake")
add_custom_target(no_exceptions_check
  COMMAND ${CMAKE_COMMAND}
    -DROOT=${CMAKE_CURRENT_SOURCE_DIR}
    -DCHECK_DIRS=src\;include
    -DEXCLUDE_DIRS=build\;install\;third_party\;generated\;test
    -P ${NO_EXCEPTIONS_CHECK_SCRIPT}
  COMMENT "Checking handwritten C++ exception keywords"
  VERBATIM
)

ament_export_targets(export_astrabot_rtc HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
