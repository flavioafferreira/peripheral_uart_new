/* File generated by C:/ncs/v2.3.0-rc1/nrf/scripts/partition_manager_output.py, do not modify */
#ifndef PM_CONFIG_H__
#define PM_CONFIG_H__
#define PM_EXTERNAL_FLASH_OFFSET 0x0
#define PM_EXTERNAL_FLASH_ADDRESS 0x0
#define PM_EXTERNAL_FLASH_END_ADDRESS 0x800000
#define PM_EXTERNAL_FLASH_SIZE 0x800000
#define PM_EXTERNAL_FLASH_NAME external_flash
#define PM_EXTERNAL_FLASH_ID 0
#define PM_external_flash_ID PM_EXTERNAL_FLASH_ID
#define PM_external_flash_IS_ENABLED 1
#define PM_0_LABEL EXTERNAL_FLASH
#define PM_EXTERNAL_FLASH_DEV DT_CHOSEN(nordic_pm_ext_flash)
#define PM_EXTERNAL_FLASH_DEFAULT_DRIVER_KCONFIG CONFIG_PM_EXTERNAL_FLASH_HAS_DRIVER
#define PM_TFM_OFFSET 0x0
#define PM_TFM_ADDRESS 0x0
#define PM_TFM_END_ADDRESS 0x8000
#define PM_TFM_SIZE 0x8000
#define PM_TFM_NAME tfm
#define PM_TFM_ID 1
#define PM_tfm_ID PM_TFM_ID
#define PM_tfm_IS_ENABLED 1
#define PM_1_LABEL TFM
#define PM_TFM_DEV flash_controller
#define PM_TFM_DEFAULT_DRIVER_KCONFIG CONFIG_SOC_FLASH_NRF
#define PM_TFM_SECURE_OFFSET 0x0
#define PM_TFM_SECURE_ADDRESS 0x0
#define PM_TFM_SECURE_END_ADDRESS 0x8000
#define PM_TFM_SECURE_SIZE 0x8000
#define PM_TFM_SECURE_NAME tfm_secure
#define PM_TFM_SECURE_ID 2
#define PM_tfm_secure_ID PM_TFM_SECURE_ID
#define PM_tfm_secure_IS_ENABLED 1
#define PM_2_LABEL TFM_SECURE
#define PM_TFM_SECURE_DEV flash_controller
#define PM_TFM_SECURE_DEFAULT_DRIVER_KCONFIG CONFIG_SOC_FLASH_NRF
#define PM_APP_OFFSET 0x8000
#define PM_APP_ADDRESS 0x8000
#define PM_APP_END_ADDRESS 0xfc000
#define PM_APP_SIZE 0xf4000
#define PM_APP_NAME app
#define PM_APP_ID 3
#define PM_app_ID PM_APP_ID
#define PM_app_IS_ENABLED 1
#define PM_3_LABEL APP
#define PM_APP_DEV flash_controller
#define PM_APP_DEFAULT_DRIVER_KCONFIG CONFIG_SOC_FLASH_NRF
#define PM_TFM_NONSECURE_OFFSET 0x8000
#define PM_TFM_NONSECURE_ADDRESS 0x8000
#define PM_TFM_NONSECURE_END_ADDRESS 0xfc000
#define PM_TFM_NONSECURE_SIZE 0xf4000
#define PM_TFM_NONSECURE_NAME tfm_nonsecure
#define PM_TFM_NONSECURE_ID 4
#define PM_tfm_nonsecure_ID PM_TFM_NONSECURE_ID
#define PM_tfm_nonsecure_IS_ENABLED 1
#define PM_4_LABEL TFM_NONSECURE
#define PM_TFM_NONSECURE_DEV flash_controller
#define PM_TFM_NONSECURE_DEFAULT_DRIVER_KCONFIG CONFIG_SOC_FLASH_NRF
#define PM_NONSECURE_STORAGE_OFFSET 0xfc000
#define PM_NONSECURE_STORAGE_ADDRESS 0xfc000
#define PM_NONSECURE_STORAGE_END_ADDRESS 0xfe000
#define PM_NONSECURE_STORAGE_SIZE 0x2000
#define PM_NONSECURE_STORAGE_NAME nonsecure_storage
#define PM_NONSECURE_STORAGE_ID 5
#define PM_nonsecure_storage_ID PM_NONSECURE_STORAGE_ID
#define PM_nonsecure_storage_IS_ENABLED 1
#define PM_5_LABEL NONSECURE_STORAGE
#define PM_NONSECURE_STORAGE_DEV flash_controller
#define PM_NONSECURE_STORAGE_DEFAULT_DRIVER_KCONFIG CONFIG_SOC_FLASH_NRF
#define PM_SETTINGS_STORAGE_OFFSET 0xfc000
#define PM_SETTINGS_STORAGE_ADDRESS 0xfc000
#define PM_SETTINGS_STORAGE_END_ADDRESS 0xfe000
#define PM_SETTINGS_STORAGE_SIZE 0x2000
#define PM_SETTINGS_STORAGE_NAME settings_storage
#define PM_SETTINGS_STORAGE_ID 6
#define PM_settings_storage_ID PM_SETTINGS_STORAGE_ID
#define PM_settings_storage_IS_ENABLED 1
#define PM_6_LABEL SETTINGS_STORAGE
#define PM_SETTINGS_STORAGE_DEV flash_controller
#define PM_SETTINGS_STORAGE_DEFAULT_DRIVER_KCONFIG CONFIG_SOC_FLASH_NRF
#define PM_EMPTY_0_OFFSET 0xfe000
#define PM_EMPTY_0_ADDRESS 0xfe000
#define PM_EMPTY_0_END_ADDRESS 0x100000
#define PM_EMPTY_0_SIZE 0x2000
#define PM_EMPTY_0_NAME EMPTY_0
#define PM_EMPTY_0_ID 7
#define PM_empty_0_ID PM_EMPTY_0_ID
#define PM_empty_0_IS_ENABLED 1
#define PM_7_LABEL EMPTY_0
#define PM_EMPTY_0_DEV flash_controller
#define PM_EMPTY_0_DEFAULT_DRIVER_KCONFIG CONFIG_SOC_FLASH_NRF
#define PM_OTP_OFFSET 0x0
#define PM_OTP_ADDRESS 0xff8100
#define PM_OTP_END_ADDRESS 0xff83fc
#define PM_OTP_SIZE 0x2fc
#define PM_OTP_NAME otp
#define PM_SRAM_SECURE_OFFSET 0x0
#define PM_SRAM_SECURE_ADDRESS 0x20000000
#define PM_SRAM_SECURE_END_ADDRESS 0x20008000
#define PM_SRAM_SECURE_SIZE 0x8000
#define PM_SRAM_SECURE_NAME sram_secure
#define PM_TFM_SRAM_OFFSET 0x0
#define PM_TFM_SRAM_ADDRESS 0x20000000
#define PM_TFM_SRAM_END_ADDRESS 0x20008000
#define PM_TFM_SRAM_SIZE 0x8000
#define PM_TFM_SRAM_NAME tfm_sram
#define PM_SRAM_NONSECURE_OFFSET 0x8000
#define PM_SRAM_NONSECURE_ADDRESS 0x20008000
#define PM_SRAM_NONSECURE_END_ADDRESS 0x20080000
#define PM_SRAM_NONSECURE_SIZE 0x78000
#define PM_SRAM_NONSECURE_NAME sram_nonsecure
#define PM_SRAM_PRIMARY_OFFSET 0x8000
#define PM_SRAM_PRIMARY_ADDRESS 0x20008000
#define PM_SRAM_PRIMARY_END_ADDRESS 0x20070000
#define PM_SRAM_PRIMARY_SIZE 0x68000
#define PM_SRAM_PRIMARY_NAME sram_primary
#define PM_RPMSG_NRF53_SRAM_OFFSET 0x70000
#define PM_RPMSG_NRF53_SRAM_ADDRESS 0x20070000
#define PM_RPMSG_NRF53_SRAM_END_ADDRESS 0x20080000
#define PM_RPMSG_NRF53_SRAM_SIZE 0x10000
#define PM_RPMSG_NRF53_SRAM_NAME rpmsg_nrf53_sram
#define PM_NUM 8
#define PM_ALL_BY_SIZE "otp EMPTY_0 settings_storage nonsecure_storage tfm tfm_sram sram_secure tfm_secure rpmsg_nrf53_sram sram_primary sram_nonsecure app tfm_nonsecure external_flash"
#define PM_CPUNET_APP_OFFSET 0x0
#define PM_CPUNET_APP_ADDRESS 0x1000000
#define PM_CPUNET_APP_END_ADDRESS 0x1040000
#define PM_CPUNET_APP_SIZE 0x40000
#define PM_CPUNET_APP_NAME app
#define PM_CPUNET_APP_ID 8
#define PM_CPUNET_app_ID PM_APP_ID
#define PM_CPUNET_app_IS_ENABLED 1
#define PM_8_LABEL CPUNET_APP
#define PM_CPUNET_APP_DEV flash_controller
#define PM_CPUNET_APP_DEFAULT_DRIVER_KCONFIG CONFIG_SOC_FLASH_NRF
#define PM_CPUNET_HCI_RPMSG_OFFSET 0x0
#define PM_CPUNET_HCI_RPMSG_ADDRESS 0x1000000
#define PM_CPUNET_HCI_RPMSG_END_ADDRESS 0x1040000
#define PM_CPUNET_HCI_RPMSG_SIZE 0x40000
#define PM_CPUNET_HCI_RPMSG_NAME hci_rpmsg
#define PM_CPUNET_HCI_RPMSG_ID 9
#define PM_CPUNET_hci_rpmsg_ID PM_HCI_RPMSG_ID
#define PM_CPUNET_hci_rpmsg_IS_ENABLED 1
#define PM_9_LABEL CPUNET_HCI_RPMSG
#define PM_CPUNET_HCI_RPMSG_DEV flash_controller
#define PM_CPUNET_HCI_RPMSG_DEFAULT_DRIVER_KCONFIG CONFIG_SOC_FLASH_NRF
#define PM_CPUNET_SRAM_PRIMARY_OFFSET 0x0
#define PM_CPUNET_SRAM_PRIMARY_ADDRESS 0x21000000
#define PM_CPUNET_SRAM_PRIMARY_END_ADDRESS 0x21010000
#define PM_CPUNET_SRAM_PRIMARY_SIZE 0x10000
#define PM_CPUNET_SRAM_PRIMARY_NAME sram_primary
#define PM_CPUNET_NUM 10
#define PM_CPUNET_ALL_BY_SIZE "sram_primary hci_rpmsg app"
#define PM_ADDRESS 0x8000
#define PM_SIZE 0xf4000
#define PM_SRAM_ADDRESS 0x20008000
#define PM_SRAM_SIZE 0x68000
#endif /* PM_CONFIG_H__ */