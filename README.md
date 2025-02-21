# Flexiv DDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_ddk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Flexiv DDK (Data Distribution Kit) is an auxiliary tool that enables the users to obtain realtime data from the robot, including system status, robot states and commands, primitive states, plan info, etc.


## Compatibility Overview

| **Supported OS**           | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------------- | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu 20.04/22.04) | x86_64                  | C++                    | build-essential           |
| Windows 10/11              | x86_64                  | C++                    | MSVC v14.2+               |

## Quick Start

The **C++** DDK libraries are packed into a unified modern CMake project named ``flexiv_ddk``, which can be configured and installed via CMake on all supported OS.

### Note

* You might need to turn off your computer's firewall or whitelist the DDK programs to be able to establish connection with the robot.

### Install on Linux

1. In a new Terminal, install compiler kit, CMake (with GUI):

       sudo apt install build-essential cmake cmake-qt-gui -y

2. Choose a directory for installing DDK library and all its dependencies. This directory can be under system path or not, depending on whether you want DDK to be globally discoverable by CMake. For example, a new folder named ``ddk_install`` under the home directory.
3. In a new Terminal, run the provided script to compile and install all C++ dependencies to the installation directory chosen in step 2:

       cd flexiv_ddk/thirdparty
       bash build_and_install_dependencies.sh ~/ddk_install

4. In a new Terminal, use CMake to configure the ``flexiv_ddk`` project:

       cd flexiv_ddk
       mkdir build && cd build
       cmake .. -DCMAKE_INSTALL_PREFIX=~/ddk_install

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` sets the CMake variable that specifies the path of the installation directory. The configuration process can also be done using CMake GUI.

5. Install DDK:

       cd flexiv_ddk/build
       cmake --build . --target install --config Release

   DDK is installed to the path specified by ``CMAKE_INSTALL_PREFIX``, which may or may not be globally discoverable by CMake.

### Install on Windows

1. Install compiler kit: Download and install Microsoft Visual Studio 2019 (MSVC v14.2) or above. Choose "Desktop development with C++" under the *Workloads* tab during installation. You only need to keep the following components for the selected workload:
   * MSVC ... C++ x64/x86 build tools (Latest)
   * C++ CMake tools for Windows
   * Windows 10 SDK or Windows 11 SDK, depending on your actual Windows version
2. Install CMake (with GUI): Download ``cmake-3.x.x-windows-x86_64.msi`` from [CMake download page](https://cmake.org/download/) and install the msi file. The minimum required version is 3.16.3. **Add CMake to system PATH** when prompted, so that ``cmake`` and ``cmake-gui`` command can be used from Command Prompt or a bash emulator.
3. Install bash emulator: Download and install [Git for Windows](https://git-scm.com/download/win/), which comes with a bash emulator Git Bash.
4. Within the bash emulator, the rest are identical to steps 2 and below in [Install on Linux](#install-on-linux).

### Link to DDK from a user program

After DDK is installed, it can be found as a CMake library and linked to by other CMake projects. Use the provided examples project for instance::

    cd flexiv_ddk/example
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=~/ddk_install
    cmake --build . --config Release -j 4

NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` tells the user project's CMake where to find the installed DDK library. The instruction above applies to all supported OS.

### Run example programs

To run a compiled example C++ program:

    cd flexiv_ddk/example/build
    ./<program_name> [robot_serial_number]

For example:

    ./basics1_display_joint_states Rizon4s-123456

## API Documentation

The API documentation of a specified release can be generated manually using Doxygen. For example, on Linux:

    sudo apt install doxygen-latex graphviz
    cd flexiv_ddk
    git checkout <release_tag>
    doxygen doc/Doxyfile.in

The generated API documentation is under ``flexiv_ddk/doc/html/`` directory. Open any html file with your browser to view it.
