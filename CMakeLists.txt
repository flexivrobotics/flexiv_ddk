cmake_minimum_required(VERSION 3.22.1)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_ddk VERSION 2.1)

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
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(DDK_LIB "libflexiv_ddk.aarch64-linux-gnu.a")
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
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/${DDK_LIB}.sha256 EXPECTED_HASH_RAW)
string(STRIP "${EXPECTED_HASH_RAW}" EXPECTED_HASH)
string(LENGTH "${EXPECTED_HASH}" HASH_LEN)
if(NOT HASH_LEN EQUAL 64)
    message(FATAL_ERROR "Invalid SHA256 hash length: ${HASH_LEN}. Expected 64. Value: '${EXPECTED_HASH}'")
endif()

# Download the library file from GitHub 
set(DDK_LIB_URL https://github.com/flexivrobotics/flexiv_ddk/releases/download/v2.1/${DDK_LIB})
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

# spdlog
find_package(spdlog REQUIRED)
if(spdlog_FOUND)
  message(STATUS "Found spdlog: ${spdlog_DIR}")
endif()

# Zenoh transport
find_package(zenohc REQUIRED)
if(zenohc_FOUND)
  message(STATUS "Found zenohc: ${zenohc_DIR}")
endif()

# zenohcxx
find_package(zenohcxx REQUIRED)
if(zenohcxx_FOUND)
  message(STATUS "Found zenohcxx: ${zenohcxx_DIR}")
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
  spdlog::spdlog
  zenohcxx::zenohc
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