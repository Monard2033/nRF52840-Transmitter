cmake_minimum_required(VERSION 3.20.0)
set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ProMicro.overlay" CACHE STRING "")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Transmitter)

target_sources(app PRIVATE src/main.c)
