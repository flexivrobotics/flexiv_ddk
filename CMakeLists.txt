cmake_minimum_required(VERSION 3.16.3)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_ddk VERSION 1.3.0)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo")

# Set C++ library according to OS and processor architecture
message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(DDK_LIB "libflexiv_ddk.x86_64-linux-gnu.a")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "AMD64")
    set(DDK_LIB "flexiv_ddk.win_amd64.lib")
  else()
    message(FATAL_ERROR "Windows with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
endif()

# Parse expected hash of the library file
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/${DDK_LIB}.sha256 EXPECTED_HASH)

# Download the library file from GitHub 
set(DDK_LIB_URL https://github.com/flexivrobotics/flexiv_ddk/releases/download/v1.3/${DDK_LIB})
message(STATUS "Downloading ${DDK_LIB_URL} ...")
file(DOWNLOAD ${DDK_LIB_URL} ${CMAKE_CURRENT_BINARY_DIR}/${DDK_LIB}
     SHOW_PROGRESS
     TLS_VERIFY ON
     EXPECTED_HASH SHA256=${EXPECTED_HASH}
     STATUS status)
list(GET status 0 code)
if(code EQUAL 0)
  message(STATUS "Download finished")
else()
  message(FATAL_ERROR "Download failed: ${status}")
endif()

# ===================================================================
#      PROJECT DEPENDENCIES
# ===================================================================
# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
if(Threads_FOUND)
  message(STATUS "Found Threads: HAVE_PTHREAD = ${THREADS_HAVE_PTHREAD_ARG}")
endif()

# Eigen3
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  message(STATUS "Found Eigen3: ${Eigen3_DIR}")
endif()

# spdlog
find_package(spdlog REQUIRED)
if(spdlog_FOUND)
  message(STATUS "Found spdlog: ${spdlog_DIR}")
endif()

# Fast-DDS (Fast-RTPS)
set(OPENSSL_USE_STATIC_LIBS ON)
find_package(fastrtps REQUIRED)
if(fastrtps_FOUND)
  message(STATUS "Found fastrtps: ${fastrtps_DIR}")
endif()

# ===================================================================
#      INSTALL C++ LIBRARY
# ===================================================================
# Create static library target with dummy source
add_library(${PROJECT_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/dummy.cpp)

# Create an alias of the target with flexiv namespace
add_library(flexiv::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link to dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
  Threads::Threads
  Eigen3::Eigen
  spdlog::spdlog
  fastrtps
  fastcdr
)

# Use moderate compiler warning option
if(CMAKE_HOST_UNIX)
  target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)
else()
  target_compile_options(${PROJECT_NAME} PUBLIC /W1)
endif()

# Install the library
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivInstallLibrary.cmake)
FlexivInstallLibrary()