# This file should be used as a template for creating new projects using the CMake tools
# This project has the following features
#    - GTSAM linking
#    - Unit tests via CppUnitLite
#    - Scripts
#    - Automatic MATLAB wrapper generation

###################################################################################
# To create your own project, replace "example" with the actual name of your project
cmake_minimum_required(VERSION 2.6)

# cmake_policy(SET CMP0048 OLD)
project(SLAM CXX C )

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts
# include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation


# Ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

find_package(nlohmann_json  REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

###################################################################################
# Build static library from common sources
set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
file (GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS "src/lib/*.cpp")
file (GLOB_RECURSE LIB_HEADERS CONFIGURE_DEPENDS "src/lib/*.h")

set (LIB_INCLUDE_DIRS "")
foreach (_headerFile ${Foo_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND Foo_INCLUDE_DIRS ${_dir})
endforeach()

list (REMOVE_DUPLICATES LIB_INCLUDE_DIRS)

add_executable(MAIN src/main.cpp)
target_include_directories(MAIN PRIVATE ${Foo_INCLUDE_DIRS})


###################################################################################
# Build tests (CMake tracks the dependecy to link with GTSAM through our project's static library)
# gtsamAddTestsGlob("tests" "tests/test*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build scripts (CMake tracks the dependecy to link with GTSAM through our project's static library)
# gtsamAddExamplesGlob("src/*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build MATLAB wrapper (CMake tracks the dependecy to link with GTSAM through our project's static library)
# wrap_and_install_library("example.h" "${CONVENIENCE_LIB_NAME}" "" "")