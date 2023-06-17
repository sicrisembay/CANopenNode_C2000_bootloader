/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== TMS320F28335.cmd ========
 *  Define the memory block start/length for the F28335
 */

/*
 *  PAGE 0 will be used to organize program sections
 *  PAGE 1 will be used to organize data sections
 *
 *  Notes:
 *        Memory blocks on F28335 are uniform (ie same
 *        physical memory) in both PAGE 0 and PAGE 1.
 *        That is the same memory region should not be
 *        defined for both PAGE 0 and PAGE 1.
 *        Doing so will result in corruption of program
 *        and/or data.
 *
 *        L0/L1/L2 and L3 memory blocks are mirrored - that is
 *        they can be accessed in high memory or low memory.
 *        For simplicity only one instance is used in this
 *        linker file.
 */

MEMORY
{
PAGE 0:    /* Program Memory */

    FLASH       : origin = 0x300000, length = 0x03FF80     /* on-chip FLASH */
    CSM_RSVD    : origin = 0x33FF80, length = 0x000076     /* Program with all 0x0000 when CSM is in use. */
    BEGIN       : origin = 0x33FFF6, length = 0x000002     /* Used for "boot to Flash" bootloader mode. */
    CSM_PWL     : origin = 0x33FFF8, length = 0x000008     /* CSM password locations in FLASH */
    OTP         : origin = 0x380400, length = 0x000400     /* on-chip OTP */
    ADC_CAL     : origin = 0x380080, length = 0x000009     /* ADC_cal function in Reserved memory */

    IQTABLES    : origin = 0x3FE000, length = 0x000b50     /* IQ Math Tables in Boot ROM */
    IQTABLES2   : origin = 0x3FEB50, length = 0x00008c     /* IQ Math Tables in Boot ROM */
    FPUTABLES   : origin = 0x3FEBDC, length = 0x0006A0     /* FPU Tables in Boot ROM */
    ROM         : origin = 0x3FF27C, length = 0x000D44     /* Boot ROM */
    RESET       : origin = 0x3FFFC0, length = 0x000002     /* part of boot ROM  */
    VECTORS     : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM  */

PAGE 1 :   /* Data Memory */

    RESERVED    : origin = 0x000000, length = 0x000002
    M01SARAM    : origin = 0x000002, length = 0x0007FE     /* on-chip RAM block M0, M1 */
    PIEVECT     : origin = 0xD00,    length = 0x100
    L07SARAM    : origin = 0x008000, length = 0x008000     /* on-chip RAM block L0-L7 */

    DEV_EMU     : origin = 0x000880, length = 0x000180     /* device emulation registers */
    FLASH_REGS  : origin = 0x000A80, length = 0x000060     /* FLASH registers */
    CSM         : origin = 0x000AE0, length = 0x000010     /* code security module registers */
    ADC_MIRROR  : origin = 0x000B00, length = 0x000010     /* ADC Results register mirror */
    XINTF       : origin = 0x000B20, length = 0x000020     /* external interface registers */
    CPU_TIMER0  : origin = 0x000C00, length = 0x000008     /* CPU Timer0 registers */
    CPU_TIMER1  : origin = 0x000C08, length = 0x000008     /* CPU Timer0 registers (CPU Timer1 & Timer2 reserved TI use)*/
    CPU_TIMER2  : origin = 0x000C10, length = 0x000008     /* CPU Timer0 registers (CPU Timer1 & Timer2 reserved TI use)*/
    PIE_CTRL    : origin = 0x000CE0, length = 0x000020     /* PIE control registers */
    DMA         : origin = 0x001000, length = 0x000200     /* DMA registers */
    MCBSPA      : origin = 0x005000, length = 0x000040     /* McBSP-A registers */
    MCBSPB      : origin = 0x005040, length = 0x000040     /* McBSP-B registers */
    ECANA       : origin = 0x006000, length = 0x000040     /* eCAN-A control and status registers */
    ECANA_LAM   : origin = 0x006040, length = 0x000040     /* eCAN-A local acceptance masks */
    ECANA_MOTS  : origin = 0x006080, length = 0x000040     /* eCAN-A message object time stamps */
    ECANA_MOTO  : origin = 0x0060C0, length = 0x000040     /* eCAN-A object time-out registers */
    ECANA_MBOX  : origin = 0x006100, length = 0x000100     /* eCAN-A mailboxes */
    ECANB       : origin = 0x006200, length = 0x000040     /* eCAN-B control and status registers */
    ECANB_LAM   : origin = 0x006240, length = 0x000040     /* eCAN-B local acceptance masks */
    ECANB_MOTS  : origin = 0x006280, length = 0x000040     /* eCAN-B message object time stamps */
    ECANB_MOTO  : origin = 0x0062C0, length = 0x000040     /* eCAN-B object time-out registers */
    ECANB_MBOX  : origin = 0x006300, length = 0x000100     /* eCAN-B mailboxes */
    EPWM1       : origin = 0x006800, length = 0x000022     /* Enhanced PWM 1 registers */
    EPWM2       : origin = 0x006840, length = 0x000022     /* Enhanced PWM 2 registers */
    EPWM3       : origin = 0x006880, length = 0x000022     /* Enhanced PWM 3 registers */
    EPWM4       : origin = 0x0068C0, length = 0x000022     /* Enhanced PWM 4 registers */
    EPWM5       : origin = 0x006900, length = 0x000022     /* Enhanced PWM 5 registers */
    EPWM6       : origin = 0x006940, length = 0x000022     /* Enhanced PWM 6 registers */
    ECAP1       : origin = 0x006A00, length = 0x000020     /* Enhanced Capture 1 registers */
    ECAP2       : origin = 0x006A20, length = 0x000020     /* Enhanced Capture 2 registers */
    ECAP3       : origin = 0x006A40, length = 0x000020     /* Enhanced Capture 3 registers */
    ECAP4       : origin = 0x006A60, length = 0x000020     /* Enhanced Capture 4 registers */
    ECAP5       : origin = 0x006A80, length = 0x000020     /* Enhanced Capture 5 registers */
    ECAP6       : origin = 0x006AA0, length = 0x000020     /* Enhanced Capture 6 registers */
    EQEP1       : origin = 0x006B00, length = 0x000040     /* Enhanced QEP 1 registers */
    EQEP2       : origin = 0x006B40, length = 0x000040     /* Enhanced QEP 2 registers */
    GPIOCTRL    : origin = 0x006F80, length = 0x000040     /* GPIO control registers */
    GPIODAT     : origin = 0x006FC0, length = 0x000020     /* GPIO data registers */
    GPIOINT     : origin = 0x006FE0, length = 0x000020     /* GPIO interrupt/LPM registers */
    SYSTEM      : origin = 0x007010, length = 0x000020     /* System control registers */
    SPIA        : origin = 0x007040, length = 0x000010     /* SPI-A registers */
    SCIA        : origin = 0x007050, length = 0x000010     /* SCI-A registers */
    XINTRUPT    : origin = 0x007070, length = 0x000010     /* external interrupt registers */
    ADC         : origin = 0x007100, length = 0x000020     /* ADC registers */
    SCIB        : origin = 0x007750, length = 0x000010     /* SCI-B registers */
    SCIC        : origin = 0x007770, length = 0x000010     /* SCI-C registers */
    I2CA        : origin = 0x007900, length = 0x000040     /* I2C-A registers */
    CSM_PWL     : origin = 0x33FFF8, length = 0x000008     /* Part of FLASHA.  CSM password locations. */
    PARTID      : origin = 0x380090, length = 0x000001     /* Part ID register location */
}

/*
 *  Allocate sections to memory blocks.
 *  Note:
 *      codestart   user defined section in DSP28_CodeStartBranch.asm
 *                  used to redirect code execution when booting to flash
 *
 *      ramfuncs    user defined section to store functions that will be
 *                  copied from Flash into RAM
 */

SECTIONS
{
    /* Allocate program areas: */
    .cinit              : > FLASH       PAGE = 0
    .pinit              : > FLASH       PAGE = 0
    .text               : > FLASH       PAGE = 0
    codestart           : > BEGIN       PAGE = 0
    ramfuncs            : LOAD = FLASH      PAGE = 0,
                          RUN  = L07SARAM   PAGE = 1,
                          LOAD_START(_RamfuncsLoadStart),
                          LOAD_SIZE(_RamfuncsLoadSize),
                          LOAD_END(_RamfuncsLoadEnd),
                          RUN_START(_RamfuncsRunStart)

    csmpasswds          : > CSM_PWL     PAGE = 0
    csm_rsvd            : > CSM_RSVD    PAGE = 0

    /* Allocate uninitalized data sections: */
    .stack              : > M01SARAM | L07SARAM     PAGE = 1
    .ebss               : > M01SARAM | L07SARAM     PAGE = 1
    .data               : > M01SARAM | L07SARAM     PAGE = 1
    .esysmem            : > L07SARAM | M01SARAM     PAGE = 1
    .cio                : > L07SARAM | M01SARAM     PAGE = 1

    /* Initalized sections go in Flash */
    /* For SDFlash to program these, they must be allocated to page 0 */
    .econst             : > FLASH       PAGE = 0
    .switch             : > FLASH       PAGE = 0
    .args               : > FLASH       PAGE = 0

#ifdef __TI_COMPILER_VERSION__
#if __TI_COMPILER_VERSION__ >= 15009000
    .TI.ramfunc         : {} LOAD = FLASH    PAGE = 0,
                             RUN  = L07SARAM PAGE = 1,
                             table(BINIT)
#endif
#endif

    /* Allocate IQ math areas: */
    IQmath              : > FLASH       PAGE = 0        /* Math Code */
    IQmathTables        : > IQTABLES    PAGE = 0, TYPE = NOLOAD

    /*
     *  Uncomment the section below if calling the IQNexp() or IQexp()
     *  functions from the IQMath.lib library in order to utilize the
     *  relevant IQ Math table in Boot ROM (This saves space and Boot ROM
     *  is 1 wait-state). If this section is not uncommented, IQmathTables2
     *  will be loaded into other memory (SARAM, Flash, etc.) and will take
     *  up space, but 0 wait-state is possible.
     */
    /*
    IQmathTables2       : > IQTABLES2, PAGE = 0, TYPE = NOLOAD
    {
        IQmath.lib<IQNexpTable.obj> (IQmathTablesRam)
    }
    */

    FPUmathTables       : > FPUTABLES, PAGE = 0, TYPE = NOLOAD

    /* Allocate ADC_cal function (pre-programmed by factory into TI reserved memory) */
    .adc_cal            : load = ADC_CAL,   PAGE = 0, TYPE = NOLOAD

    /*** The PIE Vector table is called PIEVECT by DSP/BIOS ***/
    PieVectTableFile  : > PIEVECT,     PAGE = 1,  TYPE = DSECT
    /*** Peripheral Frame 0 Register Structures ***/
    DevEmuRegsFile    : > DEV_EMU,     PAGE = 1
    FlashRegsFile     : > FLASH_REGS,  PAGE = 1
    CsmRegsFile       : > CSM,         PAGE = 1
    AdcMirrorFile     : > ADC_MIRROR,  PAGE = 1
    XintfRegsFile     : > XINTF,       PAGE = 1
    CpuTimer0RegsFile : > CPU_TIMER0,  PAGE = 1
    CpuTimer1RegsFile : > CPU_TIMER1,  PAGE = 1
    CpuTimer2RegsFile : > CPU_TIMER2,  PAGE = 1
    PieCtrlRegsFile   : > PIE_CTRL,    PAGE = 1
    DmaRegsFile       : > DMA,         PAGE = 1
    /*** Peripheral Frame 3 Register Structures ***/
    McbspaRegsFile    : > MCBSPA,      PAGE = 1
    McbspbRegsFile    : > MCBSPB,      PAGE = 1
    /*** Peripheral Frame 1 Register Structures ***/
    ECanaRegsFile     : > ECANA,       PAGE = 1
    ECanaLAMRegsFile  : > ECANA_LAM    PAGE = 1
    ECanaMboxesFile   : > ECANA_MBOX   PAGE = 1
    ECanaMOTSRegsFile : > ECANA_MOTS   PAGE = 1
    ECanaMOTORegsFile : > ECANA_MOTO   PAGE = 1
    ECanbRegsFile     : > ECANB,       PAGE = 1
    ECanbLAMRegsFile  : > ECANB_LAM    PAGE = 1
    ECanbMboxesFile   : > ECANB_MBOX   PAGE = 1
    ECanbMOTSRegsFile : > ECANB_MOTS   PAGE = 1
    ECanbMOTORegsFile : > ECANB_MOTO   PAGE = 1
    EPwm1RegsFile     : > EPWM1        PAGE = 1
    EPwm2RegsFile     : > EPWM2        PAGE = 1
    EPwm3RegsFile     : > EPWM3        PAGE = 1
    EPwm4RegsFile     : > EPWM4        PAGE = 1
    EPwm5RegsFile     : > EPWM5        PAGE = 1
    EPwm6RegsFile     : > EPWM6        PAGE = 1
    ECap1RegsFile     : > ECAP1        PAGE = 1
    ECap2RegsFile     : > ECAP2        PAGE = 1
    ECap3RegsFile     : > ECAP3        PAGE = 1
    ECap4RegsFile     : > ECAP4        PAGE = 1
    ECap5RegsFile     : > ECAP5        PAGE = 1
    ECap6RegsFile     : > ECAP6        PAGE = 1
    EQep1RegsFile     : > EQEP1        PAGE = 1
    EQep2RegsFile     : > EQEP2        PAGE = 1
    GpioCtrlRegsFile  : > GPIOCTRL     PAGE = 1
    GpioDataRegsFile  : > GPIODAT      PAGE = 1
    GpioIntRegsFile   : > GPIOINT      PAGE = 1
    /*** Peripheral Frame 2 Register Structures ***/
    SysCtrlRegsFile   : > SYSTEM,      PAGE = 1
    SpiaRegsFile      : > SPIA,        PAGE = 1
    SciaRegsFile      : > SCIA,        PAGE = 1
    XIntruptRegsFile  : > XINTRUPT,    PAGE = 1
    AdcRegsFile       : > ADC,         PAGE = 1
    ScibRegsFile      : > SCIB,        PAGE = 1
    ScicRegsFile      : > SCIC,        PAGE = 1
    I2caRegsFile      : > I2CA,        PAGE = 1
    /*** Code Security Module Register Structures ***/
    CsmPwlFile        : > CSM_PWL,     PAGE = 1
    /*** Device Part ID Register Structures ***/
    PartIdRegsFile    : > PARTID,      PAGE = 1
}
