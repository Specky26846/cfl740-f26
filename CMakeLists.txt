####
# This sets up the build system for the 'CubesatRef' project, including
# components and deployments from project.cmake. In addition, it imports the core F Prime components.
####

cmake_minimum_required(VERSION 3.24.2)
project(CubesatRef C CXX)

###
# F' Core Setup
# This includes all of the F prime core components, and imports the make-system.
###
include("${CMAKE_CURRENT_LIST_DIR}/lib/fprime/cmake/FPrime.cmake")
# NOTE: register custom targets between these two lines
fprime_setup_included_code()

# Find the OpenCV libraries
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Build the AprilTag library
function(add_apriltag_library)
    set(BUILD_SHARED_LIBS OFF)
    set(BUILD_EXAMPLES OFF)
    set(BUILD_PYTHON_WRAPPER OFF)

    add_subdirectory(lib/apriltag)
    set_target_properties(apriltag PROPERTIES C_EXTENSIONS ON)

    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(apriltag PRIVATE -Wno-pedantic)
    endif()
endfunction()

add_apriltag_library()


# This includes project-wide objects
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/CubesatRef")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/fprimecubesat/")
