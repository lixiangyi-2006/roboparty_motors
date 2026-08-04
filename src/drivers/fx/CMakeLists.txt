# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 lixiangyi-2006

AUX_SOURCE_DIRECTORY(. SOURCE_LIST_FX_MOTORS)
add_library(fx_motors
        STATIC
        ${SOURCE_LIST_FX_MOTORS}
)
target_include_directories(fx_motors
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fx_motors PUBLIC ${PUBLIC_DEPENDENCIES} motors_canfd)
