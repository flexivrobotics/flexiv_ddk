#!/bin/sh
# Build and install all dependencies of flexiv_ddk.
echo ">>>>> Start: flexiv_ddk/thirdparty/build_and_install_dependencies.sh <<<<<"

# Absolute path of this script
export SCRIPT_DIR="$(dirname $(readlink -f $0))"
set -e

# Check script arguments
if [ "$#" -lt 1 ]; then
    echo "Error: invalid script argument"
    echo "Required argument: [install_directory_path]"
    echo "    install_directory_path: directory to install all dependencies, should be the same as the install directory of flexiv_ddk"
    echo "Optional arguments: [num_parallel_jobs]"
    echo "    num_parallel_jobs: number of parallel jobs used to build, use 4 if not specified"
    exit
fi

# Get dependencies install directory from script argument, should be the same as the install directory of flexiv_ddk
export INSTALL_DIR=$1
echo "Dependencies will be installed to: $INSTALL_DIR"

# Use specified number for parallel build jobs, otherwise use 4
if [ -n "$2" ] ;then
    export NUM_JOBS=$2
else
    export NUM_JOBS=4
fi
echo "Number of parallel build jobs: $NUM_JOBS"

# Set shared cmake arguments
SHARED_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release \
                   -DBUILD_SHARED_LIBS=ON \
                   -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
                   -DCMAKE_PREFIX_PATH=$INSTALL_DIR \
                   -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
                   -DBUILD_TESTING=OFF"

# OS type
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS_NAME="Linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS_NAME="Darwin"
    echo "Darwin is not supported yet."
    exit
else
    OS_NAME="Windows"
fi

export OS_NAME
export SHARED_CMAKE_ARGS

# Clone all dependencies in a subfolder
mkdir -p cloned && cd cloned

# Build and install all dependencies to INSTALL_DIR
bash $SCRIPT_DIR/scripts/install_eigen.sh
bash $SCRIPT_DIR/scripts/install_spdlog.sh
bash $SCRIPT_DIR/scripts/install_tinyxml2.sh
bash $SCRIPT_DIR/scripts/install_foonathan_memory.sh
bash $SCRIPT_DIR/scripts/install_Fast-CDR.sh
bash $SCRIPT_DIR/scripts/install_Fast-DDS.sh

echo ">>>>> Finished: flexiv_ddk/thirdparty/build_and_install_dependencies.sh <<<<<"
