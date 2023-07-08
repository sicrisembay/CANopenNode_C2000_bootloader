#define BOOTLOADER_RUN_VAL          0x364C71F3
#define BOOTLOADER_JUMP_VAL         0xD489B4BB
#define APPLICATION_RUN_VAL         0x248B2ADC
#define BOOT_FLAG_ADDRESS           (0x00FFFC)  // refer to linker memory definition of BOOT_FLAG
#define BOOT_FLAG_VAL               *(volatile unsigned long *)BOOT_FLAG_ADDRESS
#define CONFIG_TIMER_EXPIRY_MS      (500)

//#define CONFIG_CRYSTAL_20MHZ        1
#define CONFIG_CRYSTAL_30MHZ        1
#define CONFIG_CORE_FREQ_150MHZ     1

/* CAN */
#define CONFIG_USE_CAN_A            1
#define CONFIG_CAN_A_TX_GPIO31      1
#define CONFIG_CAN_A_RX_GPIO30      1
//#define CONFIG_CAN_A_TX_GPIO19      1
//#define CONFIG_CAN_A_RX_GPIO18      1
