#!/bin/bash
set -e
echo "Installing foonathan_memory"

# Repo name
REPO=memory
# Use a specific version
VER_TAG=v0.7-3

# Clone source code
if [ ! -d $REPO ] ; then
  git clone https://github.com/foonathan/$REPO.git --branch $VER_TAG
  cd $REPO
else
  cd $REPO
  git checkout $VER_TAG
fi

# Configure CMake
rm -rf build && mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS -DBUILD_SHARED_LIBS=ON -DFOONATHAN_MEMORY_BUILD_EXAMPLES=OFF -DFOONATHAN_MEMORY_BUILD_TESTS=OFF -DFOONATHAN_MEMORY_BUILD_TOOLS=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed foonathan_memory"
