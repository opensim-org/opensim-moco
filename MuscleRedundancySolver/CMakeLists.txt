project(MuscleRedundancySolver)
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 11)
#find_package(Simbody 3.6 REQUIRED)
find_package(OpenSim 4.0 REQUIRED)
link_directories(C:/Users/Nick/repos/opensim/opensim_dependencies_install/BTK/lib/btk-0.4dev)

add_subdirectory(MuscleRedundancySolver)
add_subdirectory(tests)