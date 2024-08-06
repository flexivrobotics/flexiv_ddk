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
  basics1_display_joint_states
  basics2_display_cartesian_states
  basics3_display_plan_info
  basics4_display_primitive_states
)


# Find flexiv_ddk INTERFACE library
find_package(flexiv_ddk REQUIRED)

# Build all selected examples
foreach(example ${EXAMPLE_LIST})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} flexiv::flexiv_ddk)

  # C++17 required
  set_target_properties(${example} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
endforeach()
