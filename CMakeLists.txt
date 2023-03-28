#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)
list(APPEND ZEPHYR_EXTRA_MODULES "C:/Nordic/peripheral_uart/ubxlib" )

###set(PM_STATIC_YML_FILE "C:/Nordic/peripheral_uart/pm-static.yml")

macro(app_set_runner_args)
  board_runner_args(dfu-util "--dfuse-modifiers=force:mass-erase")
  board_runner_args(pyocd "--erase")
  board_runner_args(nrfjprog "--erase")
endmacro()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)


# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
)

target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/fs/nvs)

# Include UART ASYNC API adapter
target_sources_ifdef(CONFIG_BT_NUS_UART_ASYNC_ADAPTER app PRIVATE
  src/uart_async_adapter.c
)

target_sources(app PRIVATE includes/accessories/special.c )
target_sources(app PRIVATE includes/Protobuf/data_def_v0_pb.c )
target_sources(app PRIVATE includes/Protobuf/pb_common.c )
target_sources(app PRIVATE includes/Protobuf/pb_decode.c )
target_sources(app PRIVATE includes/Protobuf/pb_encode.c )

# NORDIC SDK APP END

zephyr_library_include_directories(.)
