cmake_minimum_required(VERSION 3.16.3)
project(flexiv_ddk-examples)

# Show verbose build info
SET(CMAKE_VERBOSE_MAKEFILE ON)

message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

# Example list
set(EXAMPLE_LIST
  basics1_display_robot_states
  basics2_display_robot_actions
  basics3_display_plan_info
  basics4_display_primitive_states
  basics5_display_server_time
  basics6_display_system_status
  basics7_display_configuration_score
)


# Find flexiv_ddk library
find_package(flexiv_ddk REQUIRED)
if(flexiv_ddk_FOUND)
  message(STATUS "Found flexiv_ddk: ${flexiv_ddk_DIR}, version:${flexiv_ddk_VERSION}")
endif()

# Build all selected examples
foreach(example ${EXAMPLE_LIST})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} flexiv::flexiv_ddk)

  # C++20 required by the grouped DDK data API.
  set_target_properties(${example} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON)
endforeach()
