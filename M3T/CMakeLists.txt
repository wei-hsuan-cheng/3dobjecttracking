cmake_minimum_required(VERSION 3.5)

# A Multi-Body Multi-Modality Multi-Camera 3D Tracker (M3T)
# =============================================================================
project(M3T)


# Options
# =============================================================================
# Defaults OFF so `colcon build` works out of the box on machines without the
# Azure Kinect / RealSense SDKs or gtest. Enable explicitly if available.
option(BUILD_TESTING "Build tests" OFF)
option(USE_AZURE_KINECT "Use Azure Kinect" OFF)
option(USE_REALSENSE "Use RealSense D435" OFF)
option(USE_GTEST "Use gtest" ${BUILD_TESTING})
option(M3T_BUILD_EXAMPLES "Build M3T example executables" OFF)
option(M3T_BUILD_DOCUMENTATION "Build Doxygen documentation" OFF)


# Libraries
# =============================================================================

# Build static on Windows
if (WIN32)
    set(GLEW_STATIC true)
    set(OpenCV_STATIC true)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -MTd")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -MT")
endif ()

# Used libraries
find_package(OpenMP REQUIRED)
find_package(Eigen3 3.3.2 REQUIRED)
find_package(GLEW REQUIRED)
if (UNIX)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif ()
find_package(OpenGL REQUIRED)
find_package(glfw3 3.1.2 REQUIRED)
find_package(OpenCV 4.3.0 REQUIRED COMPONENTS core imgproc highgui imgcodecs calib3d features2d OPTIONAL_COMPONENTS xfeatures2d cudafeatures2d)

if (USE_AZURE_KINECT)
    find_package(k4a 1.3.0 REQUIRED)
endif ()

if (USE_REALSENSE)
    find_package(realsense2 REQUIRED)
endif ()

find_package(OpenMP)
if (OPENMP_FOUND)
    if (WIN32)
        set(CMAKE_CXX_FLAGS_RELEASE
                "${CMAKE_CXX_FLAGS_RELEASE} -openmp:experimental")
        set(CMAKE_C_FLAGS_RELEASE
                "${CMAKE_C_FLAGS_RELEASE} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS_RELEASE
                "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS
                "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    else ()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS
                "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endIf ()
endif ()


# Compiler Settings
# =============================================================================

# Default settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-march HAS_MARCH)
check_cxx_compiler_flag(-mtune HAS_MTUNE)
if (HAS_MARCH)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif ()
if (HAS_MTUNE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")
endif ()

# Debug settings
check_cxx_compiler_flag(-ggdb HAS_GGDB)
check_cxx_compiler_flag(-Z7 HAS_Z7)
check_cxx_compiler_flag(-ftrapv HAS_FTRAPV)
check_cxx_compiler_flag(-Od HAS_OD)
if (HAS_GGDB)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb")
elseif (HAS_Z7)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Z7")
endif ()
if (HAS_FTRAPV)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftrapv")
endif ()
if (HAS_OD)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Od")
endif ()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

# Release settings
check_cxx_compiler_flag(-Ob3 HAS_OB3)
check_cxx_compiler_flag(-O3 HAS_O3)
check_cxx_compiler_flag(-Ob2 HAS_OB2)
check_cxx_compiler_flag(-O2 HAS_O2)
if (HAS_OB3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob3")
elseif (HAS_O3)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
elseif (HAS_OB2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ob2")
elseif (HAS_O2)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")


# Definitions
# =============================================================================
if (USE_AZURE_KINECT)
    add_definitions( -DUSE_AZURE_KINECT=TRUE )
endif ()
if (USE_REALSENSE)
    add_definitions( -DUSE_REALSENSE=TRUE )
endif ()
if(OpenCV_CUDA_VERSION)
    add_definitions( -DUSE_CUDA=TRUE )
endif ()
if(TARGET opencv_xfeatures2d)
    add_definitions( -DUSE_XFEATURES2D=TRUE )
endif ()
if (USE_GTEST)
    add_definitions( -DUSE_GTEST=TRUE )
endif ()


# Directories
# =============================================================================
add_subdirectory(src)
if (USE_GTEST)
    add_subdirectory(test)
endif ()
if (M3T_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()
if (M3T_BUILD_DOCUMENTATION)
    add_subdirectory(doc)
endif ()


# Colcon / ROS package resources
# =============================================================================
#
# M3T remains a plain CMake library, but colcon discovers it through package.xml.
# Install the package marker so downstream ROS packages can resolve this
# colcon-built library through the ament index. Runtime-generated models must
# not be written here.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/m3t_package_marker" "")
install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/m3t_package_marker"
    DESTINATION share/ament_index/resource_index/packages
    RENAME m3t)
install(FILES package.xml DESTINATION share/m3t)
