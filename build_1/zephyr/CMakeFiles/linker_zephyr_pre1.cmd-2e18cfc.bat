@echo off
cd /D C:\Nordic\peripheral_uart\build_1\zephyr || (set FAIL_LINE=2& goto :ABORT)
C:\ncs\toolchains\v2.3.0-rc1\opt\zephyr-sdk\arm-zephyr-eabi\bin\arm-zephyr-eabi-gcc.exe -x assembler-with-cpp -undef -MD -MF linker_zephyr_pre1.cmd.dep -MT linker_zephyr_pre1.cmd -D_LINKER -D_ASMLANGUAGE -imacros C:/Nordic/peripheral_uart/build_1/zephyr/include/generated/autoconf.h -IC:/ncs/v2.3.0-rc1/zephyr/include -IC:/Nordic/peripheral_uart/build_1/zephyr/include/generated -IC:/ncs/v2.3.0-rc1/zephyr/soc/arm/nordic_nrf/nrf53 -IC:/ncs/v2.3.0-rc1/zephyr/lib/libc/newlib/include -IC:/ncs/v2.3.0-rc1/zephyr/soc/arm/nordic_nrf/common/. -IC:/ncs/v2.3.0-rc1/zephyr/subsys/bluetooth -IC:/ncs/v2.3.0-rc1/zephyr/subsys/settings/include -IC:/ncs/v2.3.0-rc1/nrf/include -IC:/ncs/v2.3.0-rc1/nrf/include/tfm -IC:/ncs/v2.3.0-rc1/nrf/tests/include -IC:/Nordic/peripheral_uart/build_1/tfm/generated/interface/include -IC:/ncs/v2.3.0-rc1/modules/hal/cmsis/CMSIS/Core/Include -IC:/ncs/v2.3.0-rc1/modules/hal/nordic/nrfx -IC:/ncs/v2.3.0-rc1/modules/hal/nordic/nrfx/drivers/include -IC:/ncs/v2.3.0-rc1/modules/hal/nordic/nrfx/mdk -IC:/ncs/v2.3.0-rc1/zephyr/modules/hal_nordic/nrfx/. -IC:/Nordic/peripheral_uart/build_1/modules/libmetal/libmetal/lib/include -IC:/ncs/v2.3.0-rc1/modules/lib/open-amp/open-amp/lib/include -IC:/ncs/v2.3.0-rc1/modules/debug/segger/SEGGER -IC:/ncs/v2.3.0-rc1/modules/debug/segger/Config -IC:/ncs/v2.3.0-rc1/zephyr/modules/segger/. -IC:/ncs/v2.3.0-rc1/modules/crypto/tinycrypt/lib/include -IC:/Nordic/peripheral_uart/ubxlib/common/at_client/api -IC:/Nordic/peripheral_uart/ubxlib/common/error/api -IC:/Nordic/peripheral_uart/ubxlib/common/assert/api -IC:/Nordic/peripheral_uart/ubxlib/common/location/api -IC:/Nordic/peripheral_uart/ubxlib/common/mqtt_client/api -IC:/Nordic/peripheral_uart/ubxlib/common/http_client/api -IC:/Nordic/peripheral_uart/ubxlib/common/security/api -IC:/Nordic/peripheral_uart/ubxlib/common/sock/api -IC:/Nordic/peripheral_uart/ubxlib/common/ubx_protocol/api -IC:/Nordic/peripheral_uart/ubxlib/common/spartn/api -IC:/Nordic/peripheral_uart/ubxlib/common/utils/api -IC:/Nordic/peripheral_uart/ubxlib/port/platform/common/debug_utils/api -IC:/Nordic/peripheral_uart/ubxlib -IC:/Nordic/peripheral_uart/ubxlib/cfg -IC:/Nordic/peripheral_uart/ubxlib/common/type/api -IC:/Nordic/peripheral_uart/ubxlib/port/api -IC:/Nordic/peripheral_uart/ubxlib/common/network/api -IC:/Nordic/peripheral_uart/ubxlib/common/device/api -IC:/Nordic/peripheral_uart/ubxlib/common/short_range/api -IC:/Nordic/peripheral_uart/ubxlib/ble/api -IC:/Nordic/peripheral_uart/ubxlib/wifi/api -IC:/Nordic/peripheral_uart/ubxlib/cell/api -IC:/Nordic/peripheral_uart/ubxlib/gnss/api -IC:/Nordic/peripheral_uart/ubxlib/common/lib_common/api -IC:/Nordic/peripheral_uart/ubxlib/common/at_client/src -IC:/Nordic/peripheral_uart/ubxlib/common/assert/src -IC:/Nordic/peripheral_uart/ubxlib/common/location/src -IC:/Nordic/peripheral_uart/ubxlib/common/mqtt_client/src -IC:/Nordic/peripheral_uart/ubxlib/common/http_client/src -IC:/Nordic/peripheral_uart/ubxlib/common/security/src -IC:/Nordic/peripheral_uart/ubxlib/common/sock/src -IC:/Nordic/peripheral_uart/ubxlib/common/ubx_protocol/src -IC:/Nordic/peripheral_uart/ubxlib/common/spartn/src -IC:/Nordic/peripheral_uart/ubxlib/common/utils/src -IC:/Nordic/peripheral_uart/ubxlib/port/platform/common/debug_utils/src -IC:/Nordic/peripheral_uart/ubxlib/port/platform/common/event_queue -IC:/Nordic/peripheral_uart/ubxlib/port/platform/common/mutex_debug -IC:/Nordic/peripheral_uart/ubxlib/port/platform/common/log_ram -IC:/Nordic/peripheral_uart/ubxlib/common/network/src -IC:/Nordic/peripheral_uart/ubxlib/common/device/src -IC:/Nordic/peripheral_uart/ubxlib/common/short_range/src -IC:/Nordic/peripheral_uart/ubxlib/ble/src -IC:/Nordic/peripheral_uart/ubxlib/wifi/src -IC:/Nordic/peripheral_uart/ubxlib/cell/src -IC:/Nordic/peripheral_uart/ubxlib/gnss/src -IC:/Nordic/peripheral_uart/ubxlib/port/platform/zephyr/cfg -IC:/Nordic/peripheral_uart/ubxlib/port/platform/zephyr/src -IC:/Nordic/peripheral_uart/ubxlib/port/clib -IC:/ncs/v2.3.0-rc1/zephyr/include/zephyr -IC:/Nordic/peripheral_uart/build_1/tfm/install/interface/include -D__GCC_LINKER_CMD__ -DUSE_PARTITION_MANAGER=1 -DLINKER_ZEPHYR_PREBUILT -E C:/ncs/v2.3.0-rc1/zephyr/soc/arm/nordic_nrf/nrf53/linker.ld -P -o linker_zephyr_pre1.cmd || (set FAIL_LINE=3& goto :ABORT)
C:\ncs\toolchains\v2.3.0-rc1\opt\bin\cmake.exe -E cmake_transform_depfile Ninja gccdepfile C:/Nordic/peripheral_uart C:/ncs/v2.3.0-rc1/zephyr C:/Nordic/peripheral_uart/build_1 C:/Nordic/peripheral_uart/build_1/zephyr C:/Nordic/peripheral_uart/build_1/zephyr/linker_zephyr_pre1.cmd.dep C:/Nordic/peripheral_uart/build_1/CMakeFiles/d/20881f7b589bc224edec46f8037c3cc036642812e10262eeafb6f39178b4cfd6.d || (set FAIL_LINE=4& goto :ABORT)
goto :EOF

:ABORT
set ERROR_CODE=%ERRORLEVEL%
echo Batch file failed at line %FAIL_LINE% with errorcode %ERRORLEVEL%
exit /b %ERROR_CODE%