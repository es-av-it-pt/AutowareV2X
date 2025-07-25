cmake_minimum_required(VERSION 3.5)
project(autoware_v2x)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
set(VANETZA_INSTALL ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
find_package(Vanetza REQUIRED)
find_package(GeographicLib 1.37 REQUIRED)
find_package(Boost COMPONENTS thread program_options REQUIRED)
find_package(etsi_its_cam_ts_coding REQUIRED)
find_package(etsi_its_cam_ts_conversion REQUIRED)
find_package(etsi_its_msgs_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(Protobuf REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/src
  /opt/ros/humble/include/etsi_its_msgs_utils
)

set(SOURCES
  src/v2x_node.cpp
  src/v2x_app.cpp
  src/application.cpp
  src/cpm_application.cpp
  src/cam_application.cpp
  src/cube_evk_link.cpp
  src/ethernet_device.cpp
  src/link_layer.cpp
  src/raw_socket_link.cpp
  src/router_context.cpp
  src/dcc_passthrough.cpp
  src/time_trigger.cpp
  src/positioning.cpp
  src/security.cpp
)

if(DEFINED BUILD_COHDA)
  include_directories(
    /v2x/cohda-headers
  )
  list(APPEND SOURCES src/cohda_link.cpp)
  add_compile_definitions(BUILD_COHDA)
endif()

ament_auto_add_library(autoware_v2x SHARED ${SOURCES})

protobuf_generate(TARGET autoware_v2x PROTOS src/cube_evk_radio.proto)

target_link_libraries(autoware_v2x
  Vanetza::vanetza
  ${GeographicLib_LIBRARIES}
  Boost::thread
  Boost::program_options
  sqlite3
  etsi_its_cam_ts_coding::etsi_its_cam_ts_coding
  etsi_its_cam_ts_conversion::etsi_its_cam_ts_conversion
  protobuf::libprotobuf
  gps
)

rclcpp_components_register_node(autoware_v2x
  PLUGIN "v2x::V2XNode"
  EXECUTABLE autoware_v2x_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
