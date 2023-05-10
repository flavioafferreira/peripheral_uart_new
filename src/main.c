/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 * Modified: FlavioFerreira rfid.flavio@gmail.com
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

#include "uart_async_adapter.h"

// Special Routines
#include "includes/accessories/variables.h"
#include "includes/accessories/special.h"

// Encoder & Decoder Protobuf
#include "includes/Protobuf/pb.h"
#include "includes/Protobuf/pb_common.h"
#include "includes/Protobuf/pb_decode.h"
#include "includes/Protobuf/pb_encode.h"

#include <soc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pm_config.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>

// #include <dk_buttons_and_leds.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>


#include <zephyr/lorawan/lorawan.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/random/rand32.h>



// ublox module gnss
// https://github.com/u-blox/ubxlib
// https://github.com/u-blox/ubxlib/tree/master/gnss  - examples
// #define U_CFG_APP_PIN_CELL_TX 41
// #define U_CFG_APP_PIN_CELL_RX 40
#define U_CFG_APP_PIN_CELL_TXD 45 // P1.09
#define U_CFG_APP_PIN_CELL_RXD 44 // P1.08

#include "ubxlib.h"
#include "u_cfg_app_platform_specific.h"

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_LED_BLINK_INTERVAL 1000

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

// LEDS and Buttons and Digital

// BUTTONS
#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button_sw0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data_sw0;

#define SW1_NODE DT_ALIAS(sw1)
static const struct gpio_dt_spec button_sw1 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0});
static struct gpio_callback button_cb_data_sw1;

#define SW2_NODE DT_ALIAS(sw2)
static const struct gpio_dt_spec button_sw2 = GPIO_DT_SPEC_GET_OR(SW2_NODE, gpios, {0});
static struct gpio_callback button_cb_data_sw2;

#define SW3_NODE DT_ALIAS(sw3)
static const struct gpio_dt_spec button_sw3 = GPIO_DT_SPEC_GET_OR(SW3_NODE, gpios, {0});
static struct gpio_callback button_cb_data_sw3;

#define BUTTON1_ADR &button_sw0
#define BUTTON2_ADR &button_sw1
#define BUTTON3_ADR &button_sw2
#define BUTTON4_ADR &button_sw3

#define BUTTON1 button_sw0
#define BUTTON2 button_sw1
#define BUTTON3 button_sw2
#define BUTTON4 button_sw3

#define BUTTON1_CB &button_cb_data_sw0
#define BUTTON2_CB &button_cb_data_sw1
#define BUTTON3_CB &button_cb_data_sw2
#define BUTTON4_CB &button_cb_data_sw3

// DIGITAL INPUT

#define DIG_0_NODE DT_ALIAS(dg0)
static const struct gpio_dt_spec digital_dig0 = GPIO_DT_SPEC_GET_OR(DIG_0_NODE, gpios, {0});
static struct gpio_callback digital_cb_data_dig0;

#define DIG_1_NODE DT_ALIAS(dg1)
static const struct gpio_dt_spec digital_dig1 = GPIO_DT_SPEC_GET_OR(DIG_1_NODE, gpios, {0});
static struct gpio_callback digital_cb_data_dig1;

#define DIG_2_NODE DT_ALIAS(dg2)
static const struct gpio_dt_spec digital_dig2 = GPIO_DT_SPEC_GET_OR(DIG_2_NODE, gpios, {0});
static struct gpio_callback digital_cb_data_dig2;

#define DIG_0_ADR &digital_dig0
#define DIG_1_ADR &digital_dig1
#define DIG_2_ADR &digital_dig2

#define DIG_0 digital_dig0
#define DIG_1 digital_dig1
#define DIG_2 digital_dig2

#define DIG_0_CB &digital_cb_data_dig0
#define DIG_1_CB &digital_cb_data_dig1
#define DIG_2_CB &digital_cb_data_dig2

// LEDS
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec pin_test_led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec pin_test_led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

#define LED2_NODE DT_ALIAS(led2)
static const struct gpio_dt_spec pin_test_led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

#define LED3_NODE DT_ALIAS(led3)
static const struct gpio_dt_spec pin_test_led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

#define LED1 &pin_test_led0
#define LED2 &pin_test_led1
#define LED3 &pin_test_led2
#define LED4 &pin_test_led3

#define CON_STATUS_LED LED1
#define RUN_STATUS_LED LED2

// MUTEX DEFINE
struct k_mutex ad_ready;

// ADC
int flag = 0; // used to print once the results

int16_t buf_adc;
volatile int16_t adc_value[8];
volatile int16_t digital_value[8];

struct adc_sequence sequence = {
	.buffer = &buf_adc,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(buf_adc),
};


#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(soc, peripheral_40000000, adc_e000), io_channels,
						 DT_SPEC_AND_COMMA)};

// FLASH

#ifndef DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL
#define DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL ""
#endif

static const struct device *flash_device =
	DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_flash_controller));

// NVS deploy
struct nvs_fs fs;
struct flash_pages_info info;

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)
/*
#define ADDRESS_ID 1
#define KEY_ID 2
#define BOOT_POSITION 3
#define STRING_ID 4
#define LONG_ID 5
*/
uint32_t button2_counter = 0U, button2_counter_his;

int err = 0;
uint8_t start_send = 0;

// variable to measure time
int64_t time_stamp;
int64_t time_stamp_begin;
int64_t time_stamp_end;

//gps
Gnss position;

void turn_off_all_leds(void);

// SEMAPHORES FOR THREADS
static K_SEM_DEFINE(ble_init_ok, 0, 1);
static K_SEM_DEFINE(send_proto, 0, 1);
static K_SEM_DEFINE(save_memory, 0, 1);
static K_SEM_DEFINE(button_test, 0, 1);
static K_SEM_DEFINE(button_3, 0, 1);
static K_SEM_DEFINE(lorawan_tx, 0, 1);
uint8_t lorawan_reconnect=0;
uint32_t lorawan_reconnect_cnt=0;

// MUTEX FOR AD CONVERSION
// K_MUTEX_DEFINE(ad_ready)

// CIRCULAR BUFFER
extern uint32_t C_Buffer_Free_Position;
extern uint32_t C_Buffer_Current_Position;
// extern uint32_t C_Buffer_Alarm_Free_Position;
// extern uint32_t C_Buffer_Alarm_Current_Position;

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

// UART PORT DEFINITION ON app.overlay

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static const struct device *uart_2 = DEVICE_DT_GET(DT_NODELABEL(uart2));

static struct k_work_delayable uart_work;
static struct k_work_delayable uart_work_2;

struct uart_data_t
{
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

struct uart_data_t *buf_extra;
uint32_t buff_extra_index=0;
uint32_t buff_marker=0;

// static struct uart_data_t *last_buf2;
// uint8_t reserved_memory=0;

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static K_FIFO_DEFINE(fifo_uart2_tx_data);
static K_FIFO_DEFINE(fifo_uart2_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
static const struct device *const async_adapter_2;

#endif


//LORAWAN 
#define DEFAULT_RADIO_NODE DT_NODELABEL(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_RADIO_NODE, okay), "No default LoRa radio specified in DT");
#define DEFAULT_RADIO DT_LABEL(DEFAULT_RADIO_NODE)

/* Customize based on network configuration */
#define LORAWAN_DEV_EUI			{0x70, 0xB3, 0xD5, 0x7E, 0xD8, 0x00, 0x13, 0x44} //LITTLE ENDIAN  msb
#define LORAWAN_JOIN_EUI        {0x60, 0x81, 0xF9, 0x62, 0x41, 0x65, 0x5D, 0x0B}
#define LORAWAN_APP_KEY	        {0x10, 0xF4, 0xCD, 0x51, 0x20, 0x52, 0x7A, 0x9E, 0x14, 0x75, 0x0A, 0xA4, 0x7F, 0x54, 0x46, 0x0B}
#define LORAWAN_DEV_EUI_HELIUM  {0x60, 0x81, 0xF9, 0x07, 0x40, 0x35, 0x0D, 0x69} //msb
#define LORAWAN_JOIN_EUI_HELIUM {0x60, 0x81, 0xF9, 0x82, 0xBD, 0x7F, 0x80, 0xD5} //msb
#define LORAWAN_APP_KEY_HELIUM  {0xE0, 0x07, 0x38, 0x87, 0xAF, 0x4F, 0x16, 0x6E, 0x8E, 0x52, 0xD3, 0x27, 0x0F, 0x2E, 0x64, 0x6F}
#define DELAY K_MSEC(10000)
#define MAX_DATA_LEN 10
char data_test[] =  { 0X00 , 0X01 ,
                      0X00 , 0X02 , 
					  0X00 , 0X00 , 0X00 , 0X01 ,
					  0X00 , 0X00 , 0X00 , 0X01 };


const struct device *lora_dev;
struct lorawan_join_config join_cfg;



uint8_t dev_eui[] = LORAWAN_DEV_EUI_HELIUM;
uint8_t join_eui[] = LORAWAN_JOIN_EUI_HELIUM;
uint8_t app_key[] = LORAWAN_APP_KEY_HELIUM;


static void dl_callback(uint8_t port, bool data_pending,
			int16_t rssi, int8_t snr,
			uint8_t len, const uint8_t *data)
{
	LOG_INF("Port %d, Pending %d, RSSI %ddB, SNR %ddBm", port, data_pending, rssi, snr);
	if (data) {
		LOG_HEXDUMP_INF(data, len, "Payload: ");
	}
}

struct lorawan_downlink_cb downlink_cb = {
	.port = LW_RECV_PORT_ANY,
	.cb = dl_callback
};


static void lorwan_datarate_changed(enum lorawan_datarate dr)
{
	uint8_t unused, max_size;

	lorawan_get_payload_sizes(&unused, &max_size);
	LOG_INF("New Datarate: DR_%d, Max Payload %d", dr, max_size);
}



// UART

void uart2_tx(uint8_t Name[])
{
	struct uart_data_t *buf;
	buf = k_malloc(sizeof(*buf));
	uint16_t size = sizeof(Name);

	uint8_t i = 0;
	while (i < sizeof(Name))
	{
		buf->data[i] = Name[i];
		i++;
	}
	buf->len = (sizeof(Name) - 1);
	uart_tx(uart_2, buf->data, buf->len, SYS_FOREVER_MS);
	k_free(buf);
}

void blink(struct gpio_dt_spec *led, uint8_t times)
{
	uint8_t i = 0;
	while (i < times)
	{
		gpio_pin_set_dt(led, ON);
		gpio_pin_set_dt(led, OFF);
		i++;
	}
}

static void uart_cb_2(const struct device *dev, struct uart_event *evt, void *user_data)
{

	ARG_UNUSED(dev);
	static bool disable_req;
	struct uart_data_t *buf2;
	uint8_t i = 0;
	switch (evt->type)
	{

	case UART_RX_RDY:
		buf2 = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf2->len += evt->data.rx.len;
	
	   //START WITH $ CHARACTER
        if(buf2->data[buf2->len - 1]==0x24  && buff_marker==0){
			buf_extra = k_malloc(sizeof(*buf_extra));
			buff_extra_index=0;
			buff_marker=1;
			blink(LED3,2);
		}
   
       //STOP WITH 0X0A
        if(buff_marker==1 && (buff_extra_index<(sizeof(*buf2)-1)) ){
		    buf_extra->data[buff_extra_index++]=buf2->data[buf2->len - 1];
			if(buf2->data[buf2->len - 1]==0x0A){
			   buf_extra->data[buff_extra_index++] = 0x00;
			   buf_extra->len = buff_extra_index;
			   if(buf_extra->len>0) {
				 k_fifo_put(&fifo_uart2_rx_data, buf_extra); // TRANSFER TO FIFO
				 k_free(buf_extra);
			   }
			   buff_marker=0;
			   blink(LED4,2);
			}
		} 
 		
		break;

	case UART_RX_DISABLED:

		// blink(LED4,4);
		buf2 = k_malloc(sizeof(*buf2)); // THE SIZE IS 92 BYTES
		if (buf2)
		{
			buf2->len = 0;
		}
		else
		{
			k_work_reschedule(&uart_work_2, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		buf2->len = 0;
		uart_rx_enable(uart_2, buf2->data, sizeof(buf2->data), UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		buf2 = k_malloc(sizeof(*buf2));
		buf2->len = 0;
		uart_rx_buf_rsp(uart_2, buf2->data, sizeof(buf2->data));
		break;

	case UART_RX_BUF_RELEASED:

		buf2 = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data);
		if (buf2->len > 0)
		{
			k_free(buf2);
		}

		break;
	}
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type)
	{
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
			(!evt->data.tx.buf))
		{
			return;
		}

		if (aborted_buf){
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data);
			aborted_buf = NULL;
			aborted_len = 0;
		}
		else
		{
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf)
		{
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS))
		{
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;

		if (disable_req)
		{
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
			(evt->data.rx.buf[buf->len - 1] == '\r'))
		{
			disable_req = true;
			uart_rx_disable(uart);

			// start_send=1;
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf)
		{
			buf->len = 0;
		}
		else
		{
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf)
		{
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		}
		else
		{
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
						   data);

		if (buf->len > 0)
		{
			k_fifo_put(&fifo_uart_rx_data, buf);
		}
		else
		{
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf)
		{
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data);

		uart_tx(uart, &buf->data[aborted_len], buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;
	buf = k_malloc(sizeof(*buf));
	if (buf)
	{
		buf->len = 0;
	}
	else
	{
		LOG_WRN("Not able to allocate UART_1 receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static void uart_2_work_handler(struct k_work *item)
{
	struct uart_data_t *buf2;
	buf2 = k_malloc(sizeof(*buf2)); // SIZE IS 92 BYTES
	if (buf2)
	{
		buf2->len = 0;
	}
	else
	{
		LOG_WRN("Not able to allocate UART_2 receive buffer");
		k_work_reschedule(&uart_work_2, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart_2, buf2->data, sizeof(buf2->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
		(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	uart_irq_rx_enable(uart);

	if (!device_is_ready(uart))
	{
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK))
	{
		err = usb_enable(NULL);
		if (err && (err != -EALREADY))
		{
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx)
	{
		rx->len = 0;
	}
	else
	{
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart))
	{
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err)
	{
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL))
	{
		LOG_INF("Wait for DTR");
		while (true)
		{
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr)
			{
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err)
		{
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err)
		{
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx)
	{
		pos = snprintf(tx->data, sizeof(tx->data),
					   "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data)))
		{
			k_free(tx);
			printf("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	}
	else
	{
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err)
	{
		printf("Cannot display welcome message (err: %d)", err);
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
}

static int uart_2_init(void)
{

	uart_irq_rx_enable(uart_2);

	struct uart_data_t *rx_uart2;
	struct uart_data_t *tx_uart2;

	if (!device_is_ready(uart_2))
	{
		return -ENODEV;
	}

	rx_uart2 = k_malloc(sizeof(*rx_uart2));
	rx_uart2->len = 0;
	k_work_init_delayable(&uart_work_2, uart_2_work_handler);

	uart_callback_set(uart_2, uart_cb_2, NULL);
	uart_rx_enable(uart_2, rx_uart2->data, sizeof(rx_uart2->data), UART_WAIT_FOR_RX);

	return 0;
}

// BLUETOOTH

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err)
	{
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);

	gpio_pin_set_dt(CON_STATUS_LED, ON);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn)
	{
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;

		gpio_pin_set_dt(CON_STATUS_LED, OFF);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		LOG_INF("Security changed: %s level %u", addr, level);
	}
	else
	{
		LOG_WRN("Security failed: %s level %u err %d", addr,
				level, err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	printf("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;)
	{
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx)
		{
			printf("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size)
		{
			tx->len = tx_data_size;
		}
		else
		{
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r'))
		{
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err)
		{
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	turn_off_all_leds();

	while (true)
	{
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

// HERE THE AUTHENTICATION

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept)
	{
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	}
	else
	{
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	// uint32_t buttons = button_state & has_changed;

	if (auth_conn)
	{
		// if (buttons & KEY_PASSKEY_ACCEPT) {
		//	num_comp_reply(true);
		// }

		// if (buttons & KEY_PASSKEY_REJECT) {
		//	num_comp_reply(false);
		// }
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

// FLASH FUNCTIONS

void flash_init(void)
{

	// struct flash_pages_info info;

	int rc = 0, cnt = 0, cnt_his = 0;
	char buf[16];
	uint8_t key[8], longarray[128];
	// uint32_t button2_counter = 0U, button2_counter_his;

	/* define the nvs file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	3 sectors
	 *	starting at NVS_PARTITION_OFFSET
	 */
	fs.flash_device = NVS_PARTITION_DEVICE;

	if (!device_is_ready(fs.flash_device))
	{
		printk("Flash device %s is not ready\n", fs.flash_device->name);
		return;
	}
	fs.offset = NVS_PARTITION_OFFSET;
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc)
	{
		printk("Unable to get page info\n");
		return;
	}
	fs.sector_size = info.size;
	fs.sector_count = 2048U; // NUMBER OF SECTORS total 0X800000 BYTES

	rc = nvs_mount(&fs);
	if (rc)
	{
		printk("Flash Init failed\n");
		return;
	}

	// Button Counter
	rc = nvs_read(&fs, BOOT_POSITION, &button2_counter, sizeof(button2_counter));
	if (rc > 0)
	{ /* item was found, show it */
		printk("Id: %d, button2_counter: %d\n",
			   BOOT_POSITION, button2_counter);
	}
	else
	{ /* item was not found, add it */
		printk("No Reboot counter found, adding it at id %d\n",
			   BOOT_POSITION);
		(void)nvs_write(&fs, BOOT_POSITION, &button2_counter,
						sizeof(button2_counter));
	}

	// Creates C_Buffer_Current_Position if not exist or retrieve the value stored on memory
	rc = nvs_read(&fs, LOG_POSITION, &C_Buffer_Current_Position, sizeof(C_Buffer_Current_Position));
	if (rc > 0)
	{ /* item was found, show it */
		printk("Id: %d, Current Position: %d\n",
			   LOG_POSITION, C_Buffer_Current_Position);
		if (C_Buffer_Free_Position < CIRCULAR_BUFFER_ELEMENTS)
			C_Buffer_Free_Position = C_Buffer_Current_Position + 1;
		if (C_Buffer_Free_Position == CIRCULAR_BUFFER_ELEMENTS)
			C_Buffer_Free_Position = 0;
	}
	else
	{ /* item was not found, add it */
		printk("Current Position counter found, adding it at id %d\n",
			   LOG_POSITION);
		(void)nvs_write(&fs, LOG_POSITION, &C_Buffer_Current_Position, sizeof(C_Buffer_Current_Position));
	}
}

void flash_test_atom(void)
{
	uint32_t buff_size = 256; // Minimum to be saved
	uint8_t *buf;
	buf = k_malloc(buff_size);

	flash_device = device_get_binding("mx25r6435f@0");

	// size_t page_size = flash_get_write_block_size(flash_device);

	// printf("Result flash_page_size:%d \n", page_size);

	err = flash_read(flash_device, 0, buf, buff_size);
	printf("Result flash_read:%d \n", err);
	printf("valor:%d\n", buf[0]);

	err = flash_erase(flash_device, 0, 4096); // Mininum to be erased
	printf("Result flash_erase:%d \n", err);

	buf[0] = 0xfd;
	err = flash_write(flash_device, 0, buf, buff_size);
	printf("Result flash_write:%d \n", err);

	buf[0] = 0xFF;
	err = flash_read(flash_device, 0, buf, buff_size);
	printf("Result flash_read:%d \n", err);
	printf("valor:%d\n", buf[0]);
}

// PROTOBUFFER FUNCTIONS

uint8_t send_bluetooth(buf_data buf)
{
	uint32_t comprimento = buf.len;
	uint8_t err_code = 0;
	uint8_t BLE_NUS_MAX_DATA_LEN; //=bt_nus_get_mtu(NULL);

	// 18 bytes de MTU - depende de negociacao com o host
	BLE_NUS_MAX_DATA_LEN = 61; // was18

	uint8_t data[BLE_NUS_MAX_DATA_LEN];
	// here we have to send MTU less than BLE_NUS_MAX_DATA_LEN bytes
	uint8_t *packet_data;

	packet_data = k_malloc(BLE_NUS_MAX_DATA_LEN);

	uint16_t small_pkt = 0;
	int k = 0;

	int pacote = 0;
	while (k < comprimento - 1)
	{
		pacote++;
		while (small_pkt < BLE_NUS_MAX_DATA_LEN && k < comprimento)
		{
			data[small_pkt] = buf.data[k];
			*(packet_data + small_pkt) = buf.data[k];

			k++;
			small_pkt++;
		}

		if (bt_nus_send(NULL, packet_data, small_pkt))
		{
			printk("FALHA - ATIVE A RECEPCAO BLUETOOTH ");
		}

		small_pkt = 0;
	}
	k_free(packet_data);
	return err_code;
}

void print_serial(void)
{
	buf_data buf;
	buf = send_array_dd_v0();

	// printk("START");

	int j = 0;
	while (j < buf.len)
	{
		LOG_ERR("%c", buf.data[j]);
		k_sleep(K_MSEC(2));
		j++;
	}
}

void send_protobuf(void)
{

	buf_data buf_proto;
	buf_proto = send_array_dd_v0();
	// print_serial();
	send_bluetooth(buf_proto);
}

// BUTTONS INTERRUPTS CALL BACK

void button_pressed_1(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	// SEND MESSAGE
	k_sem_give(&send_proto);
	gpio_pin_set_dt(LED4, ON);
	printk("Button pressed 1 at %" PRIu32 "\n", k_cycle_get_32());
}

void button_pressed_2(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	// SAVE MEMORY
	k_sem_give(&save_memory);
	gpio_pin_set_dt(LED3, ON);
	printk("Button pressed 2 at %" PRIu32 "\n", k_cycle_get_32());
}

void button_pressed_3(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&button_3);
	printk("Button pressed 3 at %" PRIu32 "\n", k_cycle_get_32());
}

void button_pressed_4(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&button_test);
	printk("Button pressed 4 at %" PRIu32 "\n", k_cycle_get_32());
}

// DIGITAL CALL BACK
void digital_0_call_back(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Digital 0 activated at %" PRIu32 "\n", k_cycle_get_32());
	if (digital_value[0] <= DIGITAL_0_LIMIT)
		digital_value[0]++;
}

void digital_1_call_back(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Digital 1 activated at %" PRIu32 "\n", k_cycle_get_32());
	if (digital_value[1] <= DIGITAL_0_LIMIT)
		digital_value[1]++;
}

void digital_2_call_back(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Digital 2 activated at %" PRIu32 "\n", k_cycle_get_32());
	if (digital_value[2] <= DIGITAL_0_LIMIT)
		digital_value[2]++;
}

// CONFIGURE BUTTONS

void configure_all_buttons(void)
{
	gpio_pin_configure_dt(BUTTON1_ADR, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(BUTTON1_ADR, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(BUTTON1_CB, button_pressed_1, BIT(BUTTON1.pin));
	gpio_add_callback(BUTTON1.port, BUTTON1_CB);
	printk("Set up button at %s pin %d\n", BUTTON1.port->name, BUTTON1.pin);

	gpio_pin_configure_dt(BUTTON2_ADR, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(BUTTON2_ADR, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(BUTTON2_CB, button_pressed_2, BIT(BUTTON2.pin));
	gpio_add_callback(BUTTON2.port, BUTTON2_CB);
	printk("Set up button at %s pin %d\n", BUTTON2.port->name, BUTTON2.pin);

	gpio_pin_configure_dt(BUTTON3_ADR, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(BUTTON3_ADR, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(BUTTON3_CB, button_pressed_3, BIT(BUTTON3.pin));
	gpio_add_callback(BUTTON3.port, BUTTON3_CB);
	printk("Set up button at %s pin %d\n", BUTTON3.port->name, BUTTON3.pin);

	gpio_pin_configure_dt(BUTTON4_ADR, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(BUTTON4_ADR, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(BUTTON4_CB, button_pressed_4, BIT(BUTTON4.pin));
	gpio_add_callback(BUTTON4.port, BUTTON4_CB);
	printk("Set up button at %s pin %d\n", BUTTON4.port->name, BUTTON4.pin);
}

// CONFIGURE DIGITAL INPUTS

void configure_digital_inputs(void)
{

	gpio_pin_configure_dt(DIG_0_ADR, GPIO_INPUT);
	printk("GPIO 1 Pin 4 Value:%d \n", gpio_pin_get_dt(DIG_0_ADR));
	gpio_pin_interrupt_configure_dt(DIG_0_ADR, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(DIG_0_CB, digital_0_call_back, BIT(DIG_0.pin));
	gpio_add_callback(DIG_0.port, DIG_0_CB);
	printk("Set up Digital Input at %s pin %d\n", DIG_0.port->name, DIG_0.pin);

	gpio_pin_configure_dt(DIG_1_ADR, GPIO_INPUT);
	printk("GPIO 1 Pin 5 Value:%d \n", gpio_pin_get_dt(DIG_1_ADR));
	gpio_pin_interrupt_configure_dt(DIG_1_ADR, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(DIG_1_CB, digital_1_call_back, BIT(DIG_1.pin));
	gpio_add_callback(DIG_1.port, DIG_1_CB);
	printk("Set up Digital Input at %s pin %d\n", DIG_1.port->name, DIG_1.pin);

	gpio_pin_configure_dt(DIG_2_ADR, GPIO_INPUT);
	printk("GPIO 1 Pin 6 Value:%d \n", gpio_pin_get_dt(DIG_2_ADR));
	gpio_pin_interrupt_configure_dt(DIG_2_ADR, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(DIG_2_CB, digital_2_call_back, BIT(DIG_2.pin));
	gpio_add_callback(DIG_2.port, DIG_2_CB);
	printk("Set up Digital Input at %s pin %d\n", DIG_2.port->name, DIG_2.pin);
}

void configure_led(void)
{
	gpio_pin_configure_dt(LED1, GPIO_OUTPUT);
	gpio_pin_configure_dt(LED2, GPIO_OUTPUT);
	gpio_pin_configure_dt(LED3, GPIO_OUTPUT);
	gpio_pin_configure_dt(LED4, GPIO_OUTPUT);
}

void turn_off_all_leds(void)
{
	gpio_pin_set_dt(LED1, OFF);
	gpio_pin_set_dt(LED2, OFF);
	gpio_pin_set_dt(LED3, OFF);
	gpio_pin_set_dt(LED4, OFF);
}

void led_on_off(struct gpio_dt_spec led, uint8_t value)
{
	gpio_pin_set_dt(&led, value);
}



// CONFIGURE ADC

void configure_adc(void)
{
	int err;
	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++)
	{
		if (!device_is_ready(adc_channels[i].dev))
		{
			printk("ADC controller device not ready\n");
			return;
		}
		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0)
		{
			printk("Could not setup channel #%d (%d)\n", i, err);
			return;
		}
	}
}

// MAIN

void main(void)
{
	// int ret;
	int blink_status = 0;
	// int led1_status = 0;
	// int led2_status = 0;
	// int led3_status = 0;
	// int led4_status = 0;
	int err = 0;

	// init MUTEX
	k_mutex_init(&ad_ready);

	configure_led();
	turn_off_all_leds();
	configure_all_buttons();
	configure_digital_inputs();
	configure_adc();

	err = uart_init();
	if (err)
	{
		error();
	}


	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED))
	{
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err)
		{
			printk("Failed to register authorization callbacks.\n");
			return;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err)
		{
			printk("Failed to register authorization info callbacks.\n");
			return;
		}
	}

	err = bt_enable(NULL);
	if (err)
	{
		error();
	}

	printf("Bluetooth initialized \n\r");
	printf("Increase the Client MTU to 65 \n\r");
	printf("Press any key to send the Protobuffer \n\r");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err)
	{
		printf("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
						  ARRAY_SIZE(sd));
	if (err)
	{
		printf("Advertising failed to start (err %d)", err);
		return;
	}

	flag = 1; // print ad values once

	k_msleep(300);
	flash_init();


    err = uart_2_init();
	if (err)
	{
		error();
	}
    

	for (;;)
	{

		// uart2_tx("ABC");
		led_on_off(*RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

// THREADS

void lorawan_thread(void)
{

    uint64_t i=0,j=0;
	int ret;

    lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora0));

	if (!device_is_ready(lora_dev)) {
		printk("%s: device not ready.\n\n", lora_dev->name);
		return;
	}

  #if defined(CONFIG_LORAMAC_REGION_EU868)
	ret = lorawan_set_region(LORAWAN_REGION_EU868);
	if (ret < 0) {
		printk("lorawan_set_region failed: %d\n\n", ret);
		return;
	}
  #endif
  

    ret = lorawan_start();
	if (ret < 0) {
		printk("lorawan_start failed: %d\n\n", ret);
		return;
	}

    k_sleep(K_MSEC(500));//500ms

	lorawan_enable_adr( true );

	lorawan_register_downlink_callback(&downlink_cb);
	lorawan_register_dr_changed_callback(lorwan_datarate_changed);


    uint32_t random = sys_rand32_get();
    uint16_t dev_nonce = random & 0x0000FFFF;

	join_cfg.mode = LORAWAN_CLASS_A; //was A
	join_cfg.dev_eui = dev_eui;
	join_cfg.otaa.join_eui = join_eui;
	join_cfg.otaa.app_key = app_key;
	join_cfg.otaa.nwk_key = app_key;
    join_cfg.otaa.dev_nonce = dev_nonce;

	ret=0;
    while(1){
		 printk("Started\n\n");
   	     printk("Joining network over OTAA\n\n");
   

   	 do {
    	ret = lorawan_join(&join_cfg);
    	if (ret < 0) {
	    	printk("lorawan_join_network failed: %d\n\n", ret);
            printk("Sleeping for 10s to try again to join network.\n\n");
            k_sleep(K_MSEC(10000));

          if (ret == -116  || ret == -111  ){
            lorawan_start();
		    lorawan_enable_adr( true );
            lorawan_register_downlink_callback(&downlink_cb);
			lorawan_register_dr_changed_callback(lorwan_datarate_changed);
     		random = sys_rand32_get();
     		dev_nonce = random & 0x0000FFFF;
			join_cfg.mode = LORAWAN_CLASS_A; //was A
			join_cfg.dev_eui = dev_eui;
			join_cfg.otaa.join_eui = join_eui;
			join_cfg.otaa.app_key = app_key;
			join_cfg.otaa.nwk_key = app_key;
    		join_cfg.otaa.dev_nonce = dev_nonce;
		    k_sleep(K_MSEC(10000));
		   }

	    }
      } while ( ret < 0 );
	  printk("Joined OTAA\n\n");
	  lorawan_reconnect=0;
      while (!lorawan_reconnect) {
		  k_sem_take(&lorawan_tx, K_FOREVER);
		  lorawan_tx_data();

	    }
	  //k_sleep(K_MSEC(100));//500ms

     }
		
	
}

void shoot_minute_save_thread(void)
{

	// each one minute this thread will shot.
	uint64_t actual_time = k_uptime_get() / 1000;
	signed int h, m, s, last_minute;
	h = (actual_time / 3600);
	m = (actual_time - (3600 * h)) / 60;
	s = (actual_time - (3600 * h) - (m * 60));
	last_minute = m;

	// time_print ();

	while (1)
	{
		actual_time = k_uptime_get() / 1000;
		h = (actual_time / 3600);
		m = (actual_time - (3600 * h)) / 60;
		s = (actual_time - (3600 * h) - (m * 60));

		if (m == (last_minute + 1))
		{
			last_minute = m;
			if (m == 59)
			{
				last_minute = -1;
			}
			if (h == 24)
			{
				h = 0;
			} // only up to 23:59:59h
			  // START RUN THE MINUTE ROUTINE
			printk("LOG Circular Buffer\n");

			feed_circular_buffer();
			print_current_position_cb(C_Buffer_Current_Position);
			k_sem_give(&lorawan_tx);
			printk(" \n");
		}
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;)
	{
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);

		// SE ATIVAR ABAIXO VAI ENVIAR TODO CARACTERE DIGITADO
		if (bt_nus_send(NULL, buf->data, buf->len))
		{
			printk("Falha aqui- Failed to send data over BLE connection");
		}
		k_free(buf);
	}
}

void send_protobuf_thread(void)
{
	while (1)
	{
		k_sem_take(&send_proto, K_FOREVER);
		send_protobuf();
	}
}

void write_memory_thread(void)
{
	while (1)
	{
		k_sem_take(&save_memory, K_FOREVER);
		flash_button2_counter();
		print_current_position_cb_new(0);
	}
}

void button3_thread(void)
{
	uint32_t i;
	while (1)
	{
		i = 0;
		k_sem_take(&button_3, K_FOREVER);
		while (i <= C_Buffer_Current_Position)
		{
			print_current_position_cb(i);
			i++;
		}
	}
}

void button4_thread(void)
{
	// print ISADORA PENATI FERREIRA
	char Name[] = "ISADORA PENATI FERREIRA";
	uint16_t size = sizeof(Name);

	/*
	uint8_t *packet_data;
	packet_data = k_malloc(7);
	*packet_data     = 0x49;//I
	*(packet_data+1) = 0x53;//S
	*(packet_data+2) = 0x41;//A
	*(packet_data+3) = 0x44;//D
	*(packet_data+4) = 0x4F;//O
	*(packet_data+5) = 0x52;//R
	*(packet_data+6) = 0x41;//A
	*(packet_data+7) = 0x20;//SPACE
	*/

	while (1)
	{
		k_sem_take(&button_test, K_FOREVER);
		bt_nus_send(NULL, Name, size);
	}
	// k_free(packet_data);
}

void adc_thread(void)
{
	int err;

	while (1)
	{
		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++)
		{
			int32_t val_mv;

			(void)adc_sequence_init_dt(&adc_channels[i], &sequence);
			k_mutex_lock(&ad_ready, K_FOREVER);
			err = adc_read(adc_channels[i].dev, &sequence);
			adc_value[adc_channels[i].channel_id] = buf_adc;
			k_mutex_unlock(&ad_ready);

			if (flag == 1)
			{
				printk("- %s, channel %d: ", adc_channels[i].dev->name, adc_channels[i].channel_id);
				printk("%" PRId16, buf_adc);
				val_mv = buf_adc;
				adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
				printk(" = %" PRId32 " mV\n", val_mv);
			}
		}
		flag = 0;
		k_sleep(K_MSEC(100));
	}
}

void gnss_write_thread(void)
{
    uint8_t debug = ON;
	uint8_t value;
	uint32_t i = 0, j = 1, k = 0, h = 0, g = 0, index = 0, bfcnt = 0;
	uint64_t time = k_uptime_get();
	uint8_t state = 0, pkt_init = 0;
	static uint8_t buffer[BUFF_SIZE];

    //http://aprs.gids.nl/nmea/     sentences descriptions
    //const char nmea_id[10] = "$GPGGA"; //capture this sentence
    const char nmea_id[10] = "$GPRMC"; //capture this sentence
	

    static char *field[20];
    char *ret;
    char *token;
    char marker[2]="\n";
    //marker[0]=0x0d;

	uint8_t part[2];

	while (i < BUFF_SIZE)buffer[i++] = 0x20;//space
	i = 0;

	struct uart_data_t *buf2a;
	buf2a = k_malloc(sizeof(*buf2a));
	//
	for (;;)
	{
		buf2a = k_fifo_get(&fifo_uart2_rx_data, K_FOREVER);
		k_fifo_init(&fifo_uart2_rx_data);

		if (buf2a->len > 0)
		{
			k = (buf2a->len);

			i = 0;
			index = 0;
			blink(LED4,2);
           
			while (i < k && pkt_init == 0)
			{
				// printf("%02X ",buf2a->data[i]);
				switch (buf2a->data[i])
				{
				case 0x24: //$
					if (state == 0)state = 1;
					break;
				case 0x47: // G
					if (state == 1)state = 2;

					break;
				case 0x50: // P
					if (state == 2)state = 3;

					break;
				case 0x52: // R
					if (state == 3)state = 4;

					break;
				case 0x4D: // M
					if (state == 4)state = 5;

					break;
				case 0x43: // C
					if (state == 5){
						state = 6;
					    index = i - 5;
					}
					break;
				}
				i++;
			}

			if (state == 6 && pkt_init == 0)
			{
				//printf("BEGIN:\n");
				while (index < k)
				{
					//printf("%c", buf2a->data[index]);
					if (buf2a->data[index]!=0x0D) {
						buffer[bfcnt] = buf2a->data[index];
						bfcnt++;
					}
					index++;
				}
				pkt_init=1;
			}
		}

		buf2a = k_fifo_get(&fifo_uart2_rx_data, K_FOREVER);
		if (buf2a->len > 0)
		{
			if ((pkt_init >= 1) && (bfcnt < BUFF_SIZE)  )
			{
				index = 0;
				while ((index < k)  && (bfcnt < BUFF_SIZE))
				{
					if (buf2a->data[index]!=0x0D) {
						buffer[bfcnt] = buf2a->data[index];
						bfcnt++;
					}
					index++;
				}
				pkt_init++;
			}

			if (bfcnt >= BUFF_SIZE - 1)
			{
                index = 0;
                while(index < bfcnt ){
					//printf("%c",buffer[index]);
				   	index++;
				}
   				ret = strstr(buffer, nmea_id);
   				//printf("The substring is: %s\n", ret);
   				token = strtok(ret, marker);
   				//printf("%s\n", token );
   				i=parse_comma_delimited_str(token, field, 20);
                //debug_print_fields(i,field);
				
				if (i==12){
				  //printf("\nGPS Fixed  :%s\r\n",field[2]); //(0=invalid; 1=GPS fix; 2=Diff. GPS fix)
				  
				  position.gps_fixed=*field[2]-0x40; //char A=0x41 - 0x40 = 1
				  //printf("inteiro %d\n",position.gps_fixed);
				  if (position.gps_fixed==1){  
                   //printf("GPS Fixed  :Yes\n");
				   //printf("Time       :%s\r\n",field[1]);
				   //printf("Date       :%s\r\n",field[9]);
                   //printf("Latitude  N:%s\r\n",field[3]);
                   //printf("Longitude E:%s\r\n",field[5]);
				   position.latitude=atof(field[3]);
				   position.longitude=atof(field[5]);
				   fill_date(field[1],field[9]);
				  }else printf("Not Fixed yet\n");
			    }
				index = 0;
				pkt_init = 0;
				bfcnt = 0;
				state = 0;
				while (index < BUFF_SIZE)buffer[index++] = 0x20;//space
				index=0;
			}
			// printf("j:%d\n",j);
			j++;
		}
	}
}

// THREADS START

K_THREAD_DEFINE(adc_id, 10000, adc_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(circ_buff_id, 10000, button3_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(message_id, 10000, button4_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(memory_save_id, 10000, write_memory_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(send_protobuf_id, 10000, send_protobuf_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(ble_write_thread_id, 10000, ble_write_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(gnss_write_thread_id, STACKSIZE, gnss_write_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(shoot_minute_save_thread_id, STACKSIZE, shoot_minute_save_thread, NULL, NULL, NULL, 9, 0, 0);
K_THREAD_DEFINE(lorawan_thread_id, 8196, lorawan_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
