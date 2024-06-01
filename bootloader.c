/*
 * \file bootloader.c
 */
#include "autoconf.h"
#include "DSP2833x_Device.h"
#include "CANopen.h"
#include "301/crc16-ccitt.h"
#include "OD.h"
#include "302/CO_Prog_F28335.h"
#include "eeprom.h"

#define BOOT_FLAG_VAL               *(volatile unsigned long *)CONFIG_BOOT_FLAG_ADDRESS
#define CONFIG_TIMER_EXPIRY_MS      (500)

#define NMT_CONTROL   (CO_NMT_control_t)(CO_NMT_STARTUP_TO_OPERATIONAL   \
                      | CO_NMT_ERR_ON_ERR_REG         \
                      | CO_ERR_REG_GENERIC_ERR        \
                      | CO_ERR_REG_COMMUNICATION)

#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL

#if (CO_CONFIG_LEDS)
#define CONCAT(x, y)        x##y
#define CONCAT_L1(x, y)     CONCAT(x, y)
#if ((CONFIG_CANOPEN_LED_RED_GPIO >= 0) && (CONFIG_CANOPEN_LED_RED_GPIO <= 31))
    #define RED_LED_SET     CONCAT_L1(GpioDataRegs.GPASET.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #define RED_LED_CLEAR   CONCAT_L1(GpioDataRegs.GPACLEAR.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #define RED_LED_PUD     CONCAT_L1(GpioCtrlRegs.GPAPUD.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #define RED_LED_DIR     CONCAT_L1(GpioCtrlRegs.GPADIR.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #if (CONFIG_CANOPEN_LED_RED_GPIO <= 15)
        #define RED_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPAMUX1.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 0
    #else
        #define RED_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPAMUX2.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 0
    #endif
#elif ((CONFIG_CANOPEN_LED_RED_GPIO >= 32) && (CONFIG_CANOPEN_LED_RED_GPIO <= 63))
    #define RED_LED_SET     CONCAT_L1(GpioDataRegs.GPBSET.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #define RED_LED_CLEAR   CONCAT_L1(GpioDataRegs.GPBCLEAR.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #define RED_LED_PUD     CONCAT_L1(GpioCtrlRegs.GPBPUD.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #define RED_LED_DIR     CONCAT_L1(GpioCtrlRegs.GPBDIR.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #if (CONFIG_CANOPEN_LED_RED_GPIO <= 47)
        #define RED_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPBMUX1.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 0
    #else
        #define RED_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPBMUX2.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 0
    #endif
#elif ((CONFIG_CANOPEN_LED_RED_GPIO >= 64) && (CONFIG_CANOPEN_LED_RED_GPIO <= 87))
    #define RED_LED_SET     CONCAT_L1(GpioDataRegs.GPCSET.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #define RED_LED_CLEAR   CONCAT_L1(GpioDataRegs.GPCCLEAR.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #define RED_LED_PUD     CONCAT_L1(GpioCtrlRegs.GPCPUD.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #define RED_LED_DIR     CONCAT_L1(GpioCtrlRegs.GPCDIR.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 1
    #if (CONFIG_CANOPEN_LED_RED_GPIO <= 79)
        #define RED_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPCMUX1.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 0
    #else
        #define RED_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPCMUX2.bit.GPIO, CONFIG_CANOPEN_LED_RED_GPIO) = 0
    #endif
#else
    #error "Unsupported Red LED GPIO"
#endif /* CONFIG_CANOPEN_LED_RED_GPIO */

#if ((CONFIG_CANOPEN_LED_GREEN_GPIO >= 0) && (CONFIG_CANOPEN_LED_GREEN_GPIO <= 31))
    #define GREEN_LED_SET     CONCAT_L1(GpioDataRegs.GPASET.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #define GREEN_LED_CLEAR   CONCAT_L1(GpioDataRegs.GPACLEAR.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #define GREEN_LED_PUD     CONCAT_L1(GpioCtrlRegs.GPAPUD.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #define GREEN_LED_DIR     CONCAT_L1(GpioCtrlRegs.GPADIR.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #if (CONFIG_CANOPEN_LED_GREEN_GPIO <= 15)
        #define GREEN_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPAMUX1.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 0
    #else
        #define GREEN_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPAMUX2.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 0
    #endif
#elif ((CONFIG_CANOPEN_LED_GREEN_GPIO >= 32) && (CONFIG_CANOPEN_LED_GREEN_GPIO <= 63))
    #define GREEN_LED_SET     CONCAT_L1(GpioDataRegs.GPBSET.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #define GREEN_LED_CLEAR   CONCAT_L1(GpioDataRegs.GPBCLEAR.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #define GREEN_LED_PUD     CONCAT_L1(GpioCtrlRegs.GPBPUD.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #define GREEN_LED_DIR     CONCAT_L1(GpioCtrlRegs.GPBDIR.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #if (CONFIG_CANOPEN_LED_GREEN_GPIO <= 47)
        #define GREEN_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPBMUX1.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 0
    #else
        #define GREEN_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPBMUX2.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 0
    #endif
#elif ((CONFIG_CANOPEN_LED_GREEN_GPIO >= 64) && (CONFIG_CANOPEN_LED_GREEN_GPIO <= 87))
    #define GREEN_LED_SET     CONCAT_L1(GpioDataRegs.GPCSET.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #define GREEN_LED_CLEAR   CONCAT_L1(GpioDataRegs.GPCCLEAR.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #define GREEN_LED_PUD     CONCAT_L1(GpioCtrlRegs.GPCPUD.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #define GREEN_LED_DIR     CONCAT_L1(GpioCtrlRegs.GPCDIR.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 1
    #if (CONFIG_CANOPEN_LED_GREEN_GPIO <= 79)
        #define GREEN_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPCMUX1.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 0
    #else
        #define GREEN_LED_MUX     CONCAT_L1(GpioCtrlRegs.GPCMUX2.bit.GPIO, CONFIG_CANOPEN_LED_GREEN_GPIO) = 0
    #endif
#else
    #error "Unsupported Green LED GPIO"
#endif /* CONFIG_CANOPEN_LED_GREEN_GPIO */

#endif /* CO_CONFIG_LEDS */

typedef enum {
    MODE_RUN_BOOT,
    MODE_TIMER,
    N_MODE
} mode_t;

/* Data block for main data, which can be stored to EEPROM, if available */
typedef struct {
    uint16_t pendingBitRate;    // Pending CAN bit rate, can be set via LSS
    uint16_t pendingNodeId;     // Pending CANopen NodeId, can be set via LSS slave
} mainStorage_t;

typedef union {
    uint32_t all;
    struct {
        uint32_t len:16;
        uint32_t crc:16;
    } word;
    struct {
        uint32_t b0:8;
        uint32_t b1:8;
        uint32_t b2:8;
        uint32_t b3:8;
    } byte;
} SIGNATURE_T;

extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;

static CO_t * CO = NULL;
static void * CANptr = NULL; /* CAN module address */
static mode_t bootMode = N_MODE;
static uint16_t timerCounter = 0;

static mainStorage_t mainStorage = {
    .pendingBitRate = CONFIG_BITRATE,
#if (CONFIG_NODE_ID > 0x7F)
    .pendingNodeId = CO_LSS_NODE_ID_ASSIGNMENT,
#else
    .pendingNodeId = CONFIG_NODE_ID,
#endif
};

#if ((CO_CONFIG_LSS & CO_CONFIG_LSS_SLAVE) != 0) && CONFIG_HAS_EEPROM
static SIGNATURE_T signature;
#endif

#pragma CODE_SECTION(Device_reset, "ramfuncs");
void Device_reset(void)
{
    /* Tickle dog */
    EALLOW;
    SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;

    /* Enable watchdog */
    EALLOW;
    SysCtrlRegs.WDCR = 0x0000;  /* writing value other than b101 to WDCHK will immediately resets the device */
    EDIS;

    /* Should not reach here */
    while(1);
}


static void configure_core_pll(uint16_t val)
{
    /* Make sure the PLL is not running in limp mode */
    if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
    {
       /*
        * Missing external clock has been detected
        * Replace this line with a call to an appropriate
        * SystemShutdown(); function.
        */
       asm("        ESTOP0");
    }

    /* Change the PLLCR */
    if (SysCtrlRegs.PLLCR.bit.DIV != val)
    {

       EALLOW;
       /* Before setting PLLCR turn off missing clock detect logic */
       SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
       SysCtrlRegs.PLLCR.bit.DIV = val;
       EDIS;

       /*
        * Wait for PLL to lock.
        * During this time the CPU will switch to OSCCLK/2 until
        * the PLL is stable.  Once the PLL is stable the CPU will
        *
        * Wait for the PLL lock bit to be set.
        */
       while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS == 0) {
           /*
            * Note: The watchdog should be fed within
            * the loop via ServiceDog().
            */
       }

       EALLOW;
       SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
       EDIS;
     }
}


/*
 * InitFlash - This function initializes the Flash Control registers
 * This function MUST be executed out of RAM. Executing it
 * out of OTP/Flash will yield unpredictable results
 */
#pragma CODE_SECTION(InitFlashWaitState, "ramfuncs");
static void InitFlashWaitState(void)
{
    EALLOW;

    //
    // Enable Flash Pipeline mode to improve performance
    // of code executed from Flash.
    //
    FlashRegs.FOPT.bit.ENPIPE = 1;

    //
    //                CAUTION
    // Minimum waitstates required for the flash operating
    // at a given CPU rate must be characterized by TI.
    // Refer to the datasheet for the latest information.
    //
#if CONFIG_CORE_FREQ_150MHZ
    //
    // Set the Paged Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.PAGEWAIT = 5;

    //
    // Set the Random Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.RANDWAIT = 5;

    //
    // Set the Waitstate for the OTP
    //
    FlashRegs.FOTPWAIT.bit.OTPWAIT = 8;
#elif CONFIG_CORE_FREQ_100MHZ
    //
    // Set the Paged Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.PAGEWAIT = 5;

    //
    // Set the Random Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.RANDWAIT = 5;

    //
    // Set the Waitstate for the OTP
    //
    FlashRegs.FOTPWAIT.bit.OTPWAIT = 8;
#else
#error "Other clock not supported for now"
#endif
    //
    //                CAUTION
    // ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED
    //
    FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;
    FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;

    EDIS;

    //
    // Force a pipeline flush to ensure that the write to
    // the last register configured occurs before returning.
    //
    asm(" RPT #7 || NOP");
}


static bool_t LSScfgStoreCallback(void *object, uint8_t id, uint16_t bitRate) {
    mainStorage_t * storage = object;
    storage->pendingNodeId = id;
    storage->pendingBitRate = bitRate;
#if CONFIG_HAS_EEPROM
    uint8_t tmpBuf[4];
    SIGNATURE_T tmpSignature;
    // Write LSS data block (node ID and bit rate) to EEPROM
    tmpBuf[0] = bitRate & 0xFF;
    tmpBuf[1] = (bitRate >> 8) & 0xFF;
    tmpBuf[2] = id & 0xFF;
    tmpBuf[3] = 0;
    EEPROM_write(CONFIG_LSS_DATA_BLOCK_ADDR, tmpBuf, 4);
    // Add 20ms delay for EEPROM write cycle time
    for(uint16_t i = 0; i < 20; i++) {
        while(CpuTimer0Regs.TCR.bit.TIF == 0);
        /* Clear flag */
        CpuTimer0Regs.TCR.bit.TIF = 1;
    }
    // Calculate Signature
    tmpSignature.word.crc = (uint16_t)crc16_ccitt(tmpBuf, 4, 0);;
    tmpSignature.word.len = 4;
    tmpBuf[0] = tmpSignature.byte.b0;
    tmpBuf[1] = tmpSignature.byte.b1;
    tmpBuf[2] = tmpSignature.byte.b2;
    tmpBuf[3] = tmpSignature.byte.b3;
    EEPROM_write(CONFIG_LSS_ENTRY_SIGNATURE_ADDR, tmpBuf, 4);
    // Add 20ms delay for EEPROM write cycle time
    for(uint16_t i = 0; i < 20; i++) {
        while(CpuTimer0Regs.TCR.bit.TIF == 0);
        /* Clear flag */
        CpuTimer0Regs.TCR.bit.TIF = 1;
    }
#endif /* CONFIG_HAS_EEPROM */
    return true;
}


#if (CO_CONFIG_LEDS)
static void SetRedLEDState(uint16_t state)
{
#if CONFIG_CANOPEN_LED_RED_ACTIVE_HIGH
    if(state != 0) {
        RED_LED_SET;
    } else {
        RED_LED_CLEAR;
    }
#else
    if(state != 0) {
        RED_LED_CLEAR;
    } else {
        RED_LED_SET;
    }
#endif
}
static void SetGreenLEDState(uint16_t state)
{
#if CANOPEN_LED_GREEN_ACTIVE_HIGH
    if(state != 0) {
        GREEN_LED_SET;
    } else {
        GREEN_LED_CLEAR;
    }
#else
    if(state != 0) {
        GREEN_LED_CLEAR;
    } else {
        GREEN_LED_SET;
    }
#endif
}
#endif /* CO_CONFIG_LEDS */


int main()
{
    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    uint8_t activeNodeId = mainStorage.pendingNodeId; /* Copied from CO_pendingNodeId in the communication reset section */
    union CANTIOC_REG shadow_cantioc;
    union CANRIOC_REG shadow_canrioc;

    /*
     * Disable interrupt
     * Bootloader runs without using any interrupt.
     */
    DINT;
    IER = 0x0000;
    IFR = 0x0000;

    /*
     * Copy ramfuncs section
     */
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
           &RamfuncsLoadEnd - &RamfuncsLoadStart);

    /*
     * Initialize Core Clock
     */
#if CONFIG_CORE_FREQ_150MHZ
  #if CONFIG_CRYSTAL_30MHZ
    configure_core_pll(0xA);
  #elif CONFIG_CRYSTAL_20MHZ
    configure_core_pll(0xF);
  #else
    #error "Unsupported Crytal Frequency"
  #endif
#elif CONFIG_CORE_FREQ_100MHZ
  #if CONFIG_CRYSTAL_20MHZ
    configure_core_pll(0xA);
  #else
    #error "Unsupported Crytal Frequency"
  #endif
#else
  #error "Unsupported Core Clock Frequency"
#endif

    /*
     * Initialize FLASH wait states
     */
    InitFlashWaitState();

    /*
     * Determine Bootloader Mode
     */
    if(!CheckAppCrc()) {
        /* There is no valid App in flash */
        BOOT_FLAG_VAL = 0UL;
        bootMode = MODE_RUN_BOOT;
    } else {
        if(BOOT_FLAG_VAL == CONFIG_APPLICATION_RUN_VAL) {
            /* Run application */
            BOOT_FLAG_VAL = 0UL;
            CheckAppAndJump();
            /*
             * Only reaches here when App is invalid.
             * Run bootloader
             */
            BOOT_FLAG_VAL = 0UL;
            bootMode = MODE_RUN_BOOT;
        } else if(BOOT_FLAG_VAL == CONFIG_BOOTLOADER_RUN_VAL) {
            /* Run bootloader */
            BOOT_FLAG_VAL = 0UL;
            bootMode = MODE_RUN_BOOT;
        } else if(BOOT_FLAG_VAL == CONFIG_BOOTLOADER_JUMP_VAL) {
            /* Bootloader called directly from Application */
            BOOT_FLAG_VAL = 0UL;
            bootMode = MODE_RUN_BOOT;
        } else {
            /* Default is Timer mode */
            bootMode = MODE_TIMER;
            timerCounter = 0;
        }
    }

#if CONFIG_USE_CAN_A
    CANptr = (void *)(&ECanaRegs);
    EALLOW;
    /* Configure IO */
#if CONFIG_CAN_TX_GPIO31
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;     //Enable pull-up for GPIO31 (CANTXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;    // Configure GPIO31 for CANTXA
#elif CONFIG_CAN_TX_GPIO19
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;     // Enable pull-up for CANTXA
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;    // Configure GPIO19 for CANTXA
#endif /* CONFIG_CAN_TX_GPIOXX */
#if CONFIG_CAN_RX_GPIO30
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;     // Enable pull-up for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;    // Configure GPIO30 for CANRXA
#elif CONFIG_CAN_RX_GPIO18
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;     // Enable pull-up for CANRXA
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;   // Asynch qual for CANRXA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;    // Configure GPIO18 for CANRXA
#endif /* CONFIG_CAN_RX_GPIOXX */
    /* Enable CAN-A clock */
    SysCtrlRegs.PCLKCR0.bit.ECANAENCLK = 1;

    shadow_cantioc.all = ECanaRegs.CANTIOC.all;
    shadow_cantioc.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = shadow_cantioc.all;

    shadow_canrioc.all = ECanaRegs.CANRIOC.all;
    shadow_canrioc.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = shadow_canrioc.all;
    EDIS;
#elif CONFIG_USE_CAN_B
    CANptr = (void *)(&ECanbRegs);
    EALLOW;
    /* Configure IO */
#if CONFIG_CAN_TX_GPIO16
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;      // Enable pull-up for GPIO16(CANTXB)
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 2;     // Configure GPIO16 for CANTXB
#else
#error "Invalid CAN-B Tx GPIO"
#endif

#if CONFIG_CAN_RX_GPIO17
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;     // Enable pull-up for GPIO17(CANRXB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;   // Asynch qual for GPIO17 (CANRXB)
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 2;    // Configure GPIO17 for CANRXB
#else
#error "Invalid CAN-B Rx GPIO"
#endif
    /* Enable CAN-B clock */
    SysCtrlRegs.PCLKCR0.bit.ECANBENCLK = 1;

    shadow_cantioc.all = ECanbRegs.CANTIOC.all;
    shadow_cantioc.bit.TXFUNC = 1;
    ECanbRegs.CANTIOC.all = shadow_cantioc.all;

    shadow_canrioc.all = ECanbRegs.CANRIOC.all;
    shadow_canrioc.bit.RXFUNC = 1;
    ECanbRegs.CANRIOC.all = shadow_canrioc.all;
    EDIS;
#else
  #error "Invalid CAN peripheral"
#endif

    /* Allocate CANopen object */
    CO = CO_new(NULL, &heapMemoryUsed);
    if(CO == NULL) {
        while(1);
    }

#if (CONFIG_HAS_EEPROM)
    uint8_t tmpBuf[4];
    uint16_t dataCrc;
    EEPROM_init();
    // Read Signature
    EEPROM_read(CONFIG_LSS_ENTRY_SIGNATURE_ADDR, tmpBuf, 4);
    signature.all = ((uint32_t)tmpBuf[0]) +
            (((uint32_t)tmpBuf[1]) << 8) +
            (((uint32_t)tmpBuf[2]) << 16) +
            (((uint32_t)tmpBuf[3]) << 24);
    // Read main storage
    EEPROM_read(CONFIG_LSS_DATA_BLOCK_ADDR, tmpBuf, 4);
    dataCrc = 0;
    dataCrc = crc16_ccitt(tmpBuf, 4, dataCrc);
    if((dataCrc == signature.word.crc) &&
       (4 == signature.word.len)){
        /* Valid Signature */
        mainStorage.pendingBitRate = ((uint16_t)(tmpBuf[0])) +
                                    (((uint16_t)tmpBuf[1]) << 8);
        mainStorage.pendingNodeId = ((uint16_t)(tmpBuf[2])) +
                                    (((uint16_t)tmpBuf[3]) << 8);
    }
#endif

#if (CO_CONFIG_LEDS)
    EALLOW;
    RED_LED_PUD;
    RED_LED_DIR;
    RED_LED_MUX;
    GREEN_LED_PUD;
    GREEN_LED_DIR;
    GREEN_LED_MUX;
    EDIS;
#endif

    while(reset != CO_RESET_APP) {
        /* CANopen communication reset - initialize CANopen objects *******************/
        CO->CANmodule->CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);
        /* Initialize CAN */
        err = CO_CANinit(CO, CANptr, mainStorage.pendingBitRate);
        if(err != CO_ERROR_NO) {
            while(1);
        }

#if (CO_CONFIG_LSS & CO_CONFIG_LSS_SLAVE)
        /*
         * Use C2000 Unique Device Number
         * Refer to SPRACD0B
         */
        uint16_t uid_lsw = *(((uint16_t *)CONFIG_C2000_UID_LSW_ADDR));
        uint32_t uid_msw = *(((uint16_t *)CONFIG_C2000_UID_MSW_ADDR));
        OD_PERSIST_COMM.x1018_identity.serialNumber = (uid_msw << 16) | uid_lsw;

        CO_LSS_address_t lssAddress = {
            .identity = {
                 .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                 .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                 .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                 .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
            }
        };

        err = CO_LSSinit(CO, &lssAddress, &(mainStorage.pendingNodeId), &(mainStorage.pendingBitRate));
        if(err != CO_ERROR_NO) {
            while(1);
        }
#endif /* CO_CONFIG_LSS & CO_CONFIG_LSS_SLAVE */

        activeNodeId = mainStorage.pendingNodeId;
        uint32_t errInfo = 0;

        err = CO_CANopenInit(CO,                /* CANopen object */
                             NULL,              /* alternate NMT */
                             NULL,              /* alternate em */
                             OD,                /* Object dictionary */
                             OD_STATUS_BITS,    /* Optional OD_statusBits */
                             NMT_CONTROL,       /* CO_NMT_control_t */
                             FIRST_HB_TIME,     /* firstHBTime_ms */
                             SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                             SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                             SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
                             activeNodeId,
                             &errInfo);

        if((err != CO_ERROR_NO)
#if (CO_CONFIG_LSS & CO_CONFIG_LSS_SLAVE)
                && (err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS))
#endif
        {
            while(1);
        }

#if (CO_CONFIG_LSS & CO_CONFIG_LSS_SLAVE)
        CO_LSSslave_initCfgStoreCallback(CO->LSSslave, &mainStorage, LSScfgStoreCallback);
#endif

#if (CO_CONFIG_LEDS)
        CO_LEDs_init(CO->LEDs);
#endif

#if (CO_CONFIG_PROG & CO_CONFIG_PROG_ENABLE)
        err = CO_Prog_F28335_init(CO->CANmodule);
#endif

        /* Start CAN */
        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;

        /* Initialize Timer */
        CpuTimer0Regs.TCR.bit.TSS = 1;
        CpuTimer0Regs.TCR.bit.FREE = 0;
        CpuTimer0Regs.TCR.bit.SOFT = 0;
        CpuTimer0Regs.TCR.bit.TIE = 0;
#if CONFIG_CORE_FREQ_150MHZ
        CpuTimer0Regs.PRD.all = 150000;
#elif CONFIG_CORE_FREQ_100MHZ
        CpuTimer0Regs.PRD.all = 100000;
#else
  #error "Unsupported Core Clock Frequency"
#endif
        CpuTimer0Regs.TPR.all = 0;
        CpuTimer0Regs.TPRH.all = 0;
        CpuTimer0Regs.TCR.bit.TRB = 1;
        CpuTimer0Regs.TCR.bit.TIF = 1;
        CpuTimer0Regs.TCR.bit.TSS = 0;

        if(CO->nodeIdUnconfigured == true) {
            /* Does not have valid node ID */
            bootMode = MODE_RUN_BOOT;
            timerCounter = 0;
        }

        while(reset == CO_RESET_NOT) {
            extern void CO_CANpacket_process(CO_CANmodule_t *CANmodule);
            CO_CANpacket_process(CO->CANmodule);

            if(CpuTimer0Regs.TCR.bit.TIF) {
                /* Clear flag */
                CpuTimer0Regs.TCR.bit.TIF = 1;

                if(bootMode == MODE_TIMER) {
                    if(CO->SDOserver->CANrxNew) {
                        /* Disable Timer mode when any SDO is received */
                        bootMode = MODE_RUN_BOOT;
                        timerCounter = 0;
                    } else {
                        timerCounter++;
                        if(timerCounter >= CONFIG_TIMER_EXPIRY_MS) {
                            BOOT_FLAG_VAL = CONFIG_APPLICATION_RUN_VAL;
                            Device_reset();
                        }
                    }
                }

                uint32_t timeDifference_us = 1000;
                reset = CO_process(CO, false, timeDifference_us, NULL);
#if (CO_CONFIG_LEDS)
                SetRedLEDState(CO_LED_RED(CO->LEDs, CO_LED_CANopen));
                SetGreenLEDState(CO_LED_GREEN(CO->LEDs, CO_LED_CANopen));
#endif /* CO_CONFIG_LEDS */
            }
        }
    }

    Device_reset();
    return 0;
}
