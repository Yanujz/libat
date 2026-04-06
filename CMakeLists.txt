# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2025 yanujz

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
    VERSION     1.2.0
    DESCRIPTION "High-performance C++23 header-only AT command parser"
    HOMEPAGE_URL "https://github.com/yanujz/libat"
    LANGUAGES   CXX
)

# Define project license (SPDX identifier)
set(PROJECT_LICENSE "GPL-3.0-or-later")

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
# On all Clang builds the include is skipped at compile-time via the
# !defined(__clang__) guard in parser.hpp; tokenizer::stream() will not
# be compiled but all other APIs are fully available.
if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang")
    message(STATUS
        "libat: building with ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} — "
        "tokenizer::stream() (std::generator coroutine) is not available on Clang builds."
    )
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

# ---------------------------------------------------------------------------
# Custom targets (developer convenience, replaces Makefile)
# ---------------------------------------------------------------------------
if(NOT TARGET libat_help)
    add_custom_target(libat_help
        COMMAND ${CMAKE_COMMAND} -E echo "libat build targets:"
        COMMAND ${CMAKE_COMMAND} -E echo "  cmake --build . --target libat_tests  # Run test suite"
        COMMAND ${CMAKE_COMMAND} -E echo "  cmake --build . --target libat_license # Add license headers"
        COMMAND ${CMAKE_COMMAND} -E echo "  cmake --build . --target libat_check   # Check license headers"
    )
endif()

if(NOT TARGET libat_license AND LIBAT_BUILD_TESTS)
    find_program(ADDLICENSE addlicense)
    if(ADDLICENSE)
        add_custom_target(libat_license
            COMMAND ${ADDLICENSE} -c "Yanujz" -l gpl3 -s
                ${CMAKE_SOURCE_DIR}/include/at/*.hpp
                ${CMAKE_SOURCE_DIR}/include/at/*.in
                ${CMAKE_SOURCE_DIR}/tests/*.cpp
                ${CMAKE_SOURCE_DIR}/examples/*.cpp
                ${CMAKE_SOURCE_DIR}/CMakeLists.txt
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            COMMENT "Adding/updating GPL-3.0 license headers..."
        )
    endif()
endif()

if(NOT TARGET libat_check)
    find_program(ADDLICENSE addlicense)
    if(ADDLICENSE)
        add_custom_target(libat_check
            COMMAND ${ADDLICENSE} -check -c "Yanujz" -l gpl3 -s
                ${CMAKE_SOURCE_DIR}/include/at/*.hpp
                ${CMAKE_SOURCE_DIR}/include/at/*.in
                ${CMAKE_SOURCE_DIR}/tests/*.cpp
                ${CMAKE_SOURCE_DIR}/examples/*.cpp
                ${CMAKE_SOURCE_DIR}/CMakeLists.txt
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            COMMENT "Checking GPL-3.0 license headers..."
        )
    else()
        message(WARNING "addlicense not found. Install with: go install github.com/google/addlicense@latest")
    endif()
endif()
