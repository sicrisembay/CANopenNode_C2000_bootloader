/*
 * \file bootloader.c
 */
#include "conf.h"
#include "DSP2833x_Device.h"
#include "CANopen.h"
#include "OD.h"
#include "302/CO_Prog_F28335.h"


#define NMT_CONTROL   CO_NMT_STARTUP_TO_OPERATIONAL   \
                      | CO_NMT_ERR_ON_ERR_REG         \
                      | CO_ERR_REG_GENERIC_ERR        \
                      | CO_ERR_REG_COMMUNICATION

#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL

typedef enum {
    MODE_RUN_BOOT,
    MODE_TIMER,
    N_MODE
} mode_t;

extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;

static CO_t * CO = NULL;
static void * CANptr = NULL; /* CAN module address */
static mode_t bootMode = N_MODE;
static uint16_t timerCounter = 0;

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


int main()
{
    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    uint8_t pendingNodeId = 10; /* read from dip switches or nonvolatile memory, configurable by LSS slave */
    uint8_t activeNodeId = 10; /* Copied from CO_pendingNodeId in the communication reset section */
    uint16_t pendingBitRate = 1000;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */
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
        if(BOOT_FLAG_VAL == APPLICATION_RUN_VAL) {
            /* Run application */
            BOOT_FLAG_VAL = 0UL;
            CheckAppAndJump();
            /*
             * Only reaches here when App is invalid.
             * Run bootloader
             */
            BOOT_FLAG_VAL = 0UL;
            bootMode = MODE_RUN_BOOT;
        } else if(BOOT_FLAG_VAL == BOOTLOADER_RUN_VAL) {
            /* Run bootloader */
            BOOT_FLAG_VAL = 0UL;
            bootMode = MODE_RUN_BOOT;
        } else if(BOOT_FLAG_VAL == BOOTLOADER_JUMP_VAL) {
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
#if CONFIG_CAN_A_TX_GPIO31
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;     //Enable pull-up for GPIO31 (CANTXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;    // Configure GPIO31 for CANTXA
#elif CONFIG_CAN_A_TX_GPIO19
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;     // Enable pull-up for CANTXA
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;    // Configure GPIO19 for CANTXA
#endif /* CONFIG_CAN_A_TX_GPIOXX */
#if CONFIG_CAN_A_RX_GPIO30
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;     // Enable pull-up for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;    // Configure GPIO30 for CANRXA
#elif CONFIG_CAN_A_RX_GPIO18
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;     // Enable pull-up for CANRXA
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;   // Asynch qual for CANRXA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3;    // Configure GPIO18 for CANRXA
#endif /* CONFIG_CAN_A_RX_GPIOXX */
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
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;      // Enable pull-up for GPIO16(CANTXB)
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 2;     // Configure GPIO16 for CANTXB
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;     // Enable pull-up for GPIO17(CANRXB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;   // Asynch qual for GPIO17 (CANRXB)
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 2;    // Configure GPIO17 for CANRXB
    /* Enable CAN-A clock */
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

    while(reset != CO_RESET_APP) {
        /* CANopen communication reset - initialize CANopen objects *******************/
        CO->CANmodule->CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);
        /* Initialize CAN */
        err = CO_CANinit(CO, CANptr, pendingBitRate);
        if(err != CO_ERROR_NO) {
            while(1);
        }

        activeNodeId = pendingNodeId;
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
        if(err != CO_ERROR_NO) {
            while(1);
        }

#if (CO_CONFIG_PDO & (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE))
        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        if(err != CO_ERROR_NO) {
            while(1);
        }
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
                            BOOT_FLAG_VAL = APPLICATION_RUN_VAL;
                            Device_reset();
                        }
                    }
                }

                if(!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
                    uint32_t timeDifference_us = 1000;
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
                    CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
                    CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif
                    reset = CO_process(CO, false, timeDifference_us, NULL);
                }
            }
        }
    }

    Device_reset();
    return 0;
}
