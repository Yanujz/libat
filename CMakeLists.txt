cmake_minimum_required(VERSION 3.20)

# ---------------------------------------------------------------------------
# Git commit hash (best-effort; falls back to "unknown")
# ---------------------------------------------------------------------------
find_package(Git QUIET)
if(Git_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE  LIBAT_GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE  _git_result
    )
    if(NOT _git_result EQUAL 0)
        set(LIBAT_GIT_HASH "unknown")
    endif()
else()
    set(LIBAT_GIT_HASH "unknown")
endif()

# ---------------------------------------------------------------------------
# Project
# ---------------------------------------------------------------------------
project(libat
    VERSION     1.1.0
    DESCRIPTION "High-performance C++23 header-only AT command parser"
    HOMEPAGE_URL "https://github.com/yanujz/libat"
    LANGUAGES   CXX
)

# ---------------------------------------------------------------------------
# Build options
# ---------------------------------------------------------------------------
option(LIBAT_BUILD_TESTS    "Build the libat test suite"   ON)
option(LIBAT_BUILD_EXAMPLES "Build the libat examples"     ON)

# ---------------------------------------------------------------------------
# Interface (header-only) library
# ---------------------------------------------------------------------------
add_library(libat INTERFACE)
add_library(at::libat ALIAS libat)

target_include_directories(libat INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(libat INTERFACE cxx_std_23)

# std::generator (<generator>) requires GCC ≥ 14 or MSVC ≥ 19.38.
# Apple/LLVM libc++ does not yet ship <generator>; emit a helpful diagnostic.
if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang")
    include(CheckIncludeFileCXX)
    set(CMAKE_REQUIRED_FLAGS "-std=c++23")
    check_include_file_cxx("generator" _LIBAT_HAS_GENERATOR)
    if(NOT _LIBAT_HAS_GENERATOR)
        message(WARNING
            "libat: the <generator> header is not available with ${CMAKE_CXX_COMPILER_ID} "
            "${CMAKE_CXX_COMPILER_VERSION}. "
            "Use GCC ≥ 14 (e.g. -DCMAKE_CXX_COMPILER=g++-15) to build this project."
        )
    endif()
endif()

# ---------------------------------------------------------------------------
# Generate version header
# ---------------------------------------------------------------------------
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/include/at/version.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/include/at/version.hpp"
    @ONLY
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(LIBAT_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
        GIT_SHALLOW    TRUE
    )
    # Prevent GoogleTest from overriding our compiler/linker settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(BUILD_GMOCK             OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST           OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    include(GoogleTest)

    add_executable(libat_tests tests/test_parser.cpp)
    target_link_libraries(libat_tests PRIVATE GTest::gtest_main at::libat)
    target_compile_features(libat_tests PRIVATE cxx_std_23)

    gtest_discover_tests(libat_tests)
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------
if(LIBAT_BUILD_EXAMPLES)
    add_executable(basic_modem examples/basic_modem.cpp)
    target_link_libraries(basic_modem PRIVATE at::libat)
    target_compile_features(basic_modem PRIVATE cxx_std_23)
endif()

