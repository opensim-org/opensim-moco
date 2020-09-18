#/bin/bash

NUM_BUILD_JOBS=${NUM_BUILD_JOBS:-$(nproc)}

git submodule update --init
mkdir ../moco_dependencies_build
cd ../moco_dependencies_build
cmake ../opensim-moco/dependencies
make --jobs "${NUM_BUILD_JOBS}" ipopt
make --jobs "${NUM_BUILD_JOBS}"
cd ..
mkdir build
cd build
cmake ../opensim-moco 
make --jobs "${NUM_BUILD_JOBS}"

