# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lsm6dsl)

FILE(GLOB app_sources CONFIGURE_DEPENDS src/*.c)
target_sources(app PRIVATE
  ${app_sources}
  src/HARnet/network.c
  src/HARnet/network_data.c
)

target_include_directories(app PRIVATE
  src/HARnet
  src/HARnet/Inc
)

target_link_libraries(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/HARnet/Lib/NetworkRuntime1200_CM4_GCC.a
)
