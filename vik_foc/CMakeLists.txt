cmake_minimum_required(VERSION 3.16)

# ============================================================
# Project Version
# ============================================================

set(PROJECT_VER "1.0.5")

# Hardware Version
set(HW_VERSION "HW_V1.0")

# 1.0.2 -> 1_0_2
string(REPLACE "." "_" PROJECT_VER_UNDERSCORE "${PROJECT_VER}")

# Project Name
set(VIK_PROJECT_NAME "vik_foc_v${PROJECT_VER_UNDERSCORE}")

# ============================================================
# Git Information
# ============================================================

execute_process(
    COMMAND git rev-parse --short=8 HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE GIT_RESULT
)

execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_STATUS_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(NOT GIT_RESULT EQUAL 0)

    set(GIT_COMMIT "unknown")
    set(GIT_DESCRIBE "unknown")
    set(GIT_STATUS "unknown")

else()

    if(GIT_STATUS_RAW STREQUAL "")
        set(GIT_STATUS "clean")
    else()
        set(GIT_STATUS "dirty")
    endif()

endif()

# ============================================================
# Generate Build Information Header
# 必须在 ESP-IDF project() 解析 main/CMakeLists.txt 之前生成
# ============================================================

file(MAKE_DIRECTORY
    "${CMAKE_BINARY_DIR}/generated"
)

configure_file(
    "${CMAKE_SOURCE_DIR}/version_info.h.in"
    "${CMAKE_BINARY_DIR}/generated/version_info.h"
    @ONLY
)

# ============================================================
# ESP-IDF
# ============================================================

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ============================================================
# Minimal Build
# ============================================================

idf_build_set_property(MINIMAL_BUILD ON)

# ============================================================
# Project Name
# ============================================================

project(${VIK_PROJECT_NAME})