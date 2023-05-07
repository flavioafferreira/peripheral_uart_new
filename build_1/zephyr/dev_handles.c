#include <zephyr/device.h>
#include <zephyr/toolchain.h>

/* 1 : /soc/peripheral@40000000/clock@5000:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_80[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 2 : /soc/peripheral@40000000/gpio@842800:
 * Supported:
 *    - /soc/peripheral@40000000/spi@a000/sx1276@0
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_19[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 14, DEVICE_HANDLE_ENDS };

/* 3 : /soc/peripheral@40000000/gpio@842500:
 * Supported:
 *    - /soc/peripheral@40000000/spi@a000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_10[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 12, DEVICE_HANDLE_ENDS };

/* 4 : /entropy_bt_hci:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_5[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 5 : /psa-rng:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_6[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 6 : /soc/peripheral@40000000/uart@b000:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_122[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 7 : /soc/peripheral@40000000/uart@8000:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_120[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 8 : /ipc/ipc0:
 * Direct Dependencies:
 *    - /soc/peripheral@40000000/mbox@2a000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_30[] = { 11, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 9 : /soc/peripheral@40000000/adc@e000:
 * Direct Dependencies:
 *    - /soc/peripheral@40000000/adc@e000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_128[] = { 9, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 10 : /soc/peripheral@40000000/flash-controller@39000:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_135[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 11 : /soc/peripheral@40000000/mbox@2a000:
 * Supported:
 *    - /ipc/ipc0
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_29[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 8, DEVICE_HANDLE_ENDS };

/* 12 : /soc/peripheral@40000000/spi@a000:
 * Direct Dependencies:
 *    - /soc/peripheral@40000000/gpio@842500
 * Supported:
 *    - /soc/peripheral@40000000/spi@a000/sx1276@0
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_153[] = { 3, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, 14, DEVICE_HANDLE_ENDS };

/* 13 : /soc/peripheral@40000000/qspi@2b000/mx25r6435f@0:
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_146[] = { DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };

/* 14 : /soc/peripheral@40000000/spi@a000/sx1276@0:
 * Direct Dependencies:
 *    - /soc/peripheral@40000000/gpio@842800
 *    - /soc/peripheral@40000000/spi@a000
 */
const Z_DECL_ALIGN(device_handle_t) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_dts_ord_154[] = { 2, 12, DEVICE_HANDLE_SEP, DEVICE_HANDLE_SEP, DEVICE_HANDLE_ENDS };
