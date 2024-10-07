# CMakeLists.txt of the project "digraph"
cmake_minimum_required(VERSION 3.22)

# =======================================================
# vcpkg Configuration (must be before project())
# =======================================================
get_filename_component(VCPKG_ROOT "$ENV{VCPKG_ROOT}" ABSOLUTE)
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

option(BUILD_TESTING "Install test dependencies")
if(BUILD_TESTING)
    list(APPEND VCPKG_MANIFEST_FEATURES "test")
endif()

# ========================================
# Project Configuration
# ========================================
project(digraph
        DESCRIPTION "A library for creating and analyzing directed graphs with string node IDs.")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Directories for build output and installation
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ========================================
# Add component(s)
# ========================================
add_library(digraph INTERFACE)
target_include_directories(digraph INTERFACE include)

# =====================================
# Test Configuration
# =====================================
if(BUILD_TESTING)
    include(CTest)
    find_package(GTest CONFIG REQUIRED)
    include(GoogleTest)
	
	# Code coverage is only supported by GCC
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
        set(DCMAKE_C_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    endif()
	
    # Add test executable(s)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)
endif()
