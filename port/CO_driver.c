/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "conf.h"
#include "301/CO_driver.h"

typedef struct {
    uint16_t bit_rate_kbps;
    uint16_t brp;
    uint16_t tseg1;
    uint16_t tseg2;
} BTC_CONFIG_ENTRY_T;

#if CONFIG_CORE_FREQ_150MHZ
static BTC_CONFIG_ENTRY_T const BTC_CONFIG_TABLE[] = {
    /* 20kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 20,
        .brp = 249,
        .tseg1 = 11,
        .tseg2 = 1
    },
    /* 50kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 50,
        .brp = 99,
        .tseg1 = 11,
        .tseg2 = 1
    },
    /* 125kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 125,
        .brp = 39,
        .tseg1 = 11,
        .tseg2 = 1
    },
    /* 250kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 250,
        .brp = 19,
        .tseg1 = 11,
        .tseg2 = 1
    },
    /* 500kbps, sampling at 86.7% */
    {
        .bit_rate_kbps = 500,
        .brp = 9,
        .tseg1 = 11,
        .tseg2 = 1
    },
    /* 1000kbps, sampling at 73.3% */
    {
        .bit_rate_kbps = 1000,
        .brp = 4,
        .tseg1 = 9,
        .tseg2 = 3
    },
};
#elif CONFIG_CORE_FREQ_100MHZ
static BTC_CONFIG_ENTRY_T const BTC_CONFIG_TABLE[] = {
    /* 250kbps */
    {
        .bit_rate_kbps = 250,
        .brp = 19,
        .tseg1 = 6,
        .tseg2 = 1
    },
    /* 500kbps */
    {
        .bit_rate_kbps = 500,
        .brp = 9,
        .tseg1 = 6,
        .tseg2 = 1
    },
    /* 1000kbps */
    {
        .bit_rate_kbps = 1000,
        .brp = 4,
        .tseg1 = 6,
        .tseg2 = 1
    },
};
#else
#error "Invalid Core Clock configuration"
#endif

#define N_BTC_CONFIG_ENTRY      (sizeof(BTC_CONFIG_TABLE) / sizeof(BTC_CONFIG_TABLE[0]))
#define CAN_MAILBOX_TX          (31)

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
    /* Put CAN module in configuration mode */
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANMC_REG shadow_canmc;
    union CANES_REG shadow_canes;

    if(CANptr == NULL) return;

    ECanRegPtr = (volatile struct ECAN_REGS *)CANptr;

    EALLOW;
    shadow_canmc.all = ECanRegPtr->CANMC.all;
    shadow_canmc.bit.CCR = 1;
    ECanRegPtr->CANMC.all = shadow_canmc.all;

    shadow_canes.all = ECanRegPtr->CANES.all;
    do {
        shadow_canes.all = ECanRegPtr->CANES.all;
    } while(shadow_canes.bit.CCE != 1);

    EDIS;
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANMC_REG shadow_canmc;
    union CANES_REG shadow_canes;

    if(CANmodule == NULL) return;

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);

    EALLOW;
    shadow_canmc.all = ECanRegPtr->CANMC.all;
    shadow_canmc.bit.CCR = 0;
    ECanRegPtr->CANMC.all = shadow_canmc.all;

    shadow_canes.all = ECanRegPtr->CANES.all;
    do {
        shadow_canes.all = ECanRegPtr->CANES.all;
    } while(shadow_canes.bit.CCE != 0);
    EDIS;

    CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANptr,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    /* Local variable used for CAN configuration */
    volatile struct ECAN_REGS * ECanRegPtr;
    volatile struct MBOX * MBoxPtr;
    volatile union CANLAM_REG * LamPtr;
    union CANMC_REG shadow_canmc;
    union CANES_REG shadow_canes;
    union CANBTC_REG shadow_canbtc;
    union CANMSGID_REG shadow_canmsgid;
    union CANLAM_REG shadow_canlam;
    uint16_t i;

    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL || CANptr==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    ECanRegPtr = (volatile struct ECAN_REGS *)CANptr;

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = (rxSize <= 31U) ? true : false;  // Receive uses Mailbox0-30, Transmit uses Mailbox31
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for(i = 0; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }

    for(i = 0; i < txSize; i++) {
        txArray[i].bufferFull = false;
    }

    /* Configure CAN module registers */
    if(ECanRegPtr == &ECanaRegs) {
        MBoxPtr = &(ECanaMboxes.MBOX0);
        LamPtr = &(ECanaLAMRegs.LAM0);
    } else if(ECanRegPtr == &ECanbRegs) {
        MBoxPtr = &(ECanbMboxes.MBOX0);
        LamPtr = &(ECanbLAMRegs.LAM0);
    } else {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    EALLOW;

    shadow_canmc.all = ECanRegPtr->CANMC.all;
    shadow_canmc.bit.DBO = 0;
    shadow_canmc.bit.SCB = 1;
    shadow_canmc.bit.CCR = 1;
    shadow_canmc.bit.ABO = 1;
    ECanRegPtr->CANMC.all = shadow_canmc.all;

    do {
        shadow_canes.all = ECanRegPtr->CANES.all;
    } while(shadow_canes.bit.CCE != 1);

    for(i = 0; i < 32; i++) {
        MBoxPtr[i].MSGCTRL.all = 0x00000000;
    }

    ECanRegPtr->CANGIM.all = 0x00000000;        // disable all interrupts
    ECanRegPtr->CANMIM.all = 0x00000000;        // disable all mailbox interrupts
    ECanRegPtr->CANMIL.all = 0x00000000;        // mailbox interrupts on interrupt line0
    ECanRegPtr->CANTA.all  = 0xFFFFFFFF;        // Clears CANTA by writing 1's
    ECanRegPtr->CANRMP.all = 0xFFFFFFFF;        // Clears RMP by writing 1's
    ECanRegPtr->CANGIF0.all = 0xFFFFFFFF;       // Clears Line0 Global interrupt flags by writing 1's
    ECanRegPtr->CANGIF1.all = 0xFFFFFFFF;       // Clears Line1 Global interrupt flags by writing 1's
    ECanRegPtr->CANME.all  = 0x00000000;        // disable all mailbox
    ECanRegPtr->CANMD.all  = 0x00000000;        // Default all mailbox as transmit.

    /* Configure CAN timing */
    for(i = 0; i < N_BTC_CONFIG_ENTRY; i++) {
        if(CANbitRate == BTC_CONFIG_TABLE[i].bit_rate_kbps) {
            shadow_canbtc.all = 0;
            shadow_canbtc.bit.BRPREG = BTC_CONFIG_TABLE[i].brp;
            shadow_canbtc.bit.TSEG1REG = BTC_CONFIG_TABLE[i].tseg1;
            shadow_canbtc.bit.TSEG2REG = BTC_CONFIG_TABLE[i].tseg2;
            shadow_canbtc.bit.SAM = 1;
            ECanRegPtr->CANBTC.all = shadow_canbtc.all;
            break;
        }
    }

    if(i == N_BTC_CONFIG_ENTRY) {
        /* Invalid Bit Rate */
        EDIS;
        return CO_ERROR_ILLEGAL_BAUDRATE;
    }

    /* Configure CAN module hardware filters */
    if(CANmodule->useCANrxFilters){
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        shadow_canmsgid.all = 0;
        shadow_canmsgid.bit.IDE = 0;    // Uses 11-bit Standard identifier
        shadow_canmsgid.bit.STDMSGID = 0;
        shadow_canmsgid.bit.AME = 1;    // Use acceptance mask
        for(i = 0; i < 32; i++) {
            MBoxPtr[i].MSGID.all = shadow_canmsgid.all;
        }

        /* Configure all masks so, that received message must match filter */
        shadow_canlam.all = 0;
        shadow_canlam.bit.LAMI = 1;         // Uses local acceptance mask
        shadow_canlam.bit.LAM_L = 0;
        shadow_canlam.bit.LAM_H = 0x0000;   // Received 11-bit Standard Identifier
                                            // must match the corresponding MSGID register
        for(i = 0; i < 32; i++) {
            LamPtr[i].all = shadow_canlam.all;
        }
    } else {
        /* CAN module filters are not used, all messages with standard 11-bit */
        /* identifier will be received */
        shadow_canmsgid.all = 0;
        shadow_canmsgid.bit.IDE = 0;    // Uses 11-bit Standard identifier
        shadow_canmsgid.bit.STDMSGID = 0;
        shadow_canmsgid.bit.AME = 1;    // Use acceptance mask
        for(i = 0; i < 32; i++) {
            MBoxPtr[i].MSGID.all = shadow_canmsgid.all;
        }

        /* Configure mask 0 so, that all messages with standard identifier are accepted */
        shadow_canlam.all = 0;
        shadow_canlam.bit.LAMI = 1;         // Use local acceptance mask
        shadow_canlam.bit.LAM_L = 0;
        shadow_canlam.bit.LAM_H = 0x1FFC;   // Don't care on 11-bit Standard Identifier, LAM[28:18]
                                            // Accepts all standard identifier
        for(i = 0; i < 32; i++) {
            LamPtr[i].all = shadow_canlam.all;
        }

    }

    EDIS;

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANMC_REG shadow_canmc;
    union CANES_REG shadow_canes;

    if (CANmodule != NULL) {
        /*
         * Set CAN module to power down mode
         */
        ECanRegPtr = (volatile struct ECAN_REGS *)CANmodule->CANptr;

        /*
         * Allow transmission of any packet in progess to complete
         * and enter power-down mode.
         */
        EALLOW;
        shadow_canmc.all = ECanRegPtr->CANMC.all;
        shadow_canmc.bit.PDR = 1;
        shadow_canmc.bit.WUBA = 0;
        ECanRegPtr->CANMC.all = shadow_canmc.all;

        /* Wait for Power-down mode acknowledge */
        do {
            shadow_canes.all = ECanRegPtr->CANES.all;
        } while (shadow_canes.bit.PDA != 1);

        /*
         * Turn off the Clock to CAN peripheral
         */
        /// TODO
        EDIS;
    }
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
    volatile struct ECAN_REGS * ECanRegPtr;
    volatile struct MBOX * MBoxPtr;
    volatile union CANLAM_REG * LamPtr;
    union CANME_REG shadow_canme;
    union CANMSGID_REG shadow_canmsgid;
    union CANMSGCTRL_REG shadow_canmsgctrl;
    union CANLAM_REG shadow_canlam;
    union CANMD_REG shadow_canmd;

    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (CANrx_callback!=NULL) && (index < CANmodule->rxSize)){
        /*
         * Note: In C2000 MCU, Higher Mailbox number takes higher priority
         * Change index, i.e. NMT (index0) has highest receive mailbox priority
         */
        index = CANmodule->rxSize - index - 1;

        ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);

        if(ECanRegPtr == &ECanaRegs) {
            MBoxPtr = &(ECanaMboxes.MBOX0);
            LamPtr = &(ECanaLAMRegs.LAM0);
        } else if(ECanRegPtr == &ECanbRegs) {
            MBoxPtr = &(ECanbMboxes.MBOX0);
            LamPtr = &(ECanbLAMRegs.LAM0);
        } else {
            return CO_ERROR_ILLEGAL_ARGUMENT;
        }

        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* Disable Mailbox to allow configuration */
        shadow_canme.all = ECanRegPtr->CANME.all;
        shadow_canme.all &= ~(1UL << index);
        ECanRegPtr->CANME.all = shadow_canme.all;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        shadow_canmsgid.all = 0;
        shadow_canmsgid.bit.AME = 1;
        shadow_canmsgid.bit.STDMSGID = buffer->ident;
        MBoxPtr[index].MSGID.all = shadow_canmsgid.all;

        shadow_canmsgctrl.all = 0;
        if(rtr){
            buffer->ident |= 0x0800U;
            shadow_canmsgctrl.bit.RTR = 1;
        }
        MBoxPtr[index].MSGCTRL.all = shadow_canmsgctrl.all;

        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){
            /* Set local acceptance mask */
            shadow_canlam.all = 0;
            shadow_canlam.bit.LAMI = 1;
            shadow_canlam.bit.LAM_L = 0;
            shadow_canlam.bit.LAM_H = 0;
            LamPtr[index].all = shadow_canlam.all;  // Received 11-bit Standard Identifier
                                                    // must match the corresponding MSGID register
        }

        /* Configure mailbox direction */
        shadow_canmd.all = ECanRegPtr->CANMD.all;
        shadow_canmd.all |= (1UL << index);
        ECanRegPtr->CANMD.all = shadow_canmd.all;

        /* Re-enable mailbox */
        shadow_canme.all = ECanRegPtr->CANME.all;
        shadow_canme.all |= (1UL << index);
        ECanRegPtr->CANME.all = shadow_canme.all;
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
         * Microcontroller specific. */
        buffer->ident = (uint32_t)ident & 0x07FFU;
        if(rtr) {
            buffer->ident |= 0x0800U;
        }
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
    volatile struct ECAN_REGS * ECanRegPtr;
    volatile struct MBOX * MBoxPtr;
    union CANTRS_REG shadow_cantrs;
    union CANME_REG shadow_canme;
    union CANMSGID_REG shadow_canmsgid;
    union CANMSGCTRL_REG shadow_canmsgctrl;
    union CANMDL_REG shadow_canmdl;
    union CANMDH_REG shadow_canmdh;

    CO_ReturnError_t err = CO_ERROR_NO;

    if(CANmodule == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);
    if(ECanRegPtr == &ECanaRegs) {
        MBoxPtr = &(ECanaMboxes.MBOX0);
    } else if(ECanRegPtr == &ECanbRegs) {
        MBoxPtr = &(ECanbMboxes.MBOX0);
    } else {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Verify overflow */
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    shadow_cantrs.all = ECanRegPtr->CANTRS.all;
    /* if CAN TX buffer is free, copy message to it */
    if(((shadow_cantrs.all & (1UL << CAN_MAILBOX_TX)) == 0) && (CANmodule->CANtxCount == 0)){
        CANmodule->bufferInhibitFlag = buffer->syncFlag;

        /*
         * copy message and txRequest
         */
        /* Disable mailbox to be able to modify MSGID */
        shadow_canme.all = ECanRegPtr->CANME.all;
        shadow_canme.all &= ~(1UL << CAN_MAILBOX_TX);
        ECanRegPtr->CANME.all = shadow_canme.all;
        /* Configure MSGID */
        shadow_canmsgid.all = 0;
        shadow_canmsgid.bit.STDMSGID = buffer->ident & 0x07FFU;
        MBoxPtr[CAN_MAILBOX_TX].MSGID.all = shadow_canmsgid.all;
        /* MSGCTRL */
        shadow_canmsgctrl.all = 0;
        shadow_canmsgctrl.bit.DLC = buffer->DLC;
        if(buffer->ident & 0x0800U) {
            shadow_canmsgctrl.bit.RTR = 1;
        }
        /* DATA */
        shadow_canmdl.byte.BYTE0 = buffer->data[0];
        shadow_canmdl.byte.BYTE1 = buffer->data[1];
        shadow_canmdl.byte.BYTE2 = buffer->data[2];
        shadow_canmdl.byte.BYTE3 = buffer->data[3];
        shadow_canmdh.byte.BYTE4 = buffer->data[4];
        shadow_canmdh.byte.BYTE5 = buffer->data[5];
        shadow_canmdh.byte.BYTE6 = buffer->data[6];
        shadow_canmdh.byte.BYTE7 = buffer->data[7];

        MBoxPtr[CAN_MAILBOX_TX].MSGCTRL.all = shadow_canmsgctrl.all;
        MBoxPtr[CAN_MAILBOX_TX].MDL.all = shadow_canmdl.all;
        MBoxPtr[CAN_MAILBOX_TX].MDH.all = shadow_canmdh.all;

        /* Re-enable mailbox */
        shadow_canme.all = ECanRegPtr->CANME.all;
        shadow_canme.all |= (1UL << CAN_MAILBOX_TX);
        ECanRegPtr->CANME.all = shadow_canme.all;

        shadow_cantrs.all = (1UL << CAN_MAILBOX_TX);
        ECanRegPtr->CANTRS.all = shadow_cantrs.all;
    }
    /* if no buffer is free, message will be sent by interrupt */
    else{
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANTRS_REG shadow_cantrs;
    union CANTRR_REG shadow_cantrr;

    uint32_t tpdoDeleted = 0U;

    if(CANmodule == NULL) {
        return;
    }

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    shadow_cantrs.all = ECanRegPtr->CANTRS.all;
    if((shadow_cantrs.all & (1UL << CAN_MAILBOX_TX)) && CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        shadow_cantrr.all = (1UL << CAN_MAILBOX_TX);
        ECanRegPtr->CANTRR.all = shadow_cantrr.all;
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);


    if(tpdoDeleted != 0U){
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}


/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
    * different way to determine errors. */

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
    volatile struct ECAN_REGS * ECanRegPtr;
    union CANES_REG shadow_canes;
    union CANTEC_REG shadow_cantec;
    union CANREC_REG shadow_canrec;

    uint32_t err;

    if(CANmodule == NULL) {
        return;
    }

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);
    shadow_canes.all = ECanRegPtr->CANES.all;
    shadow_cantec.all = ECanRegPtr->CANTEC.all;
    shadow_canrec.all = ECanRegPtr->CANREC.all;

    err = (shadow_canes.all & 0x01FF0000U) |
          ((shadow_cantec.bit.TEC << 8) & 0x0000FF00U) |
          (shadow_canrec.bit.REC & 0x000000FFU);

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        /* bus off */
        if(shadow_canes.bit.BO) {
            status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            uint16_t rxErrors = shadow_canrec.bit.REC;
            uint16_t txErrors = shadow_cantec.bit.TEC;

            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (txErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        // DONE in ISR -->
//        if (ECanRegPtr->CANGIF0.bit.RMLIF0) {
//            /* CAN RX bus overflow */
//            status |= CO_CAN_ERRRX_OVERFLOW;
//            shadow_canrmp.all = ECanRegPtr->CANRMP.all;
//            ECanRegPtr->CANRMP.all = shadow_canrmp.all; // Clears RMP by writing 1's
//        }
        // <-- DONE in ISR

        CANmodule->CANerrorStatus = status;
    }
}

void CO_CANpacket_process(CO_CANmodule_t *CANmodule) {
    volatile struct ECAN_REGS * ECanRegPtr;
    volatile struct MBOX * MBoxPtr;
    union CANGIF0_REG shadow_cangif0;
    union CANTA_REG shadow_canta;
    union CANTRS_REG shadow_cantrs;
    union CANRMP_REG shadow_canrmp;
    union CANME_REG shadow_canme;
    union CANMSGID_REG shadow_canmsgid;
    union CANMSGCTRL_REG shadow_canmsgctrl;
    union CANMDL_REG shadow_canmdl;
    union CANMDH_REG shadow_canmdh;

    CO_CANrxMsg_t canRxMsg;

    if(CANmodule == NULL) {
        return;
    }

    ECanRegPtr = (volatile struct ECAN_REGS *)(CANmodule->CANptr);
    if(ECanRegPtr == &ECanaRegs) {
        MBoxPtr = &(ECanaMboxes.MBOX0);
    } else if(ECanRegPtr == &ECanbRegs) {
        MBoxPtr = &(ECanbMboxes.MBOX0);
    } else {
        return;
    }

    /* receive */
    shadow_canrmp.all = ECanRegPtr->CANRMP.all;

    if(shadow_canrmp.all != 0) {
        CO_CANrxMsg_t *rcvMsg;      /* pointer to received message in CAN module */
        uint16_t index;             /* index of received message */
        uint32_t rcvMsgIdent;       /* identifier of the received message */
        CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        /* clear Rx pending flag */
        ECanRegPtr->CANRMP.all = shadow_canrmp.all;

        for(index = 0; index < CANmodule->rxSize; index++) {
            if(shadow_canrmp.all & (1UL << index)) {
                /* get message from module here */
                canRxMsg.ident = MBoxPtr[index].MSGID.bit.STDMSGID;
                canRxMsg.DLC = MBoxPtr[index].MSGCTRL.bit.DLC;
                canRxMsg.data[0] = MBoxPtr[index].MDL.byte.BYTE0;
                canRxMsg.data[1] = MBoxPtr[index].MDL.byte.BYTE1;
                canRxMsg.data[2] = MBoxPtr[index].MDL.byte.BYTE2;
                canRxMsg.data[3] = MBoxPtr[index].MDL.byte.BYTE3;
                canRxMsg.data[4] = MBoxPtr[index].MDH.byte.BYTE4;
                canRxMsg.data[5] = MBoxPtr[index].MDH.byte.BYTE5;
                canRxMsg.data[6] = MBoxPtr[index].MDH.byte.BYTE6;
                canRxMsg.data[7] = MBoxPtr[index].MDH.byte.BYTE7;

                rcvMsg = &canRxMsg;
                rcvMsgIdent = rcvMsg->ident;
                if(CANmodule->useCANrxFilters){
                    /* CAN module filters are used. Message with known 11-bit identifier has */
                    /* been received */
                    buffer = &CANmodule->rxArray[index];
                    /* verify also RTR */
                    if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                        msgMatched = true;
                    }
                }
                else{
                    /* CAN module filters are not used, message with any standard 11-bit identifier */
                    /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
                    buffer = &CANmodule->rxArray[0];
                    uint16_t i;
                    for(i = CANmodule->rxSize; i > 0U; i--){
                        if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                            msgMatched = true;
                            break;
                        }
                        buffer++;
                    }
                }

                /* Call specific function, which will process the message */
                if(msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)){
                    buffer->CANrx_callback(buffer->object, (void*) rcvMsg);
                }
            }
        }
    }


    /* transmit interrupt */
    shadow_cantrs.all = ECanRegPtr->CANTRS.all;
    shadow_canta.all = ECanRegPtr->CANTA.all;
    if(shadow_canta.all){
        /* Clear interrupt flag */
        ECanRegPtr->CANTA.all = shadow_canta.all;

        if(shadow_canta.all & (1UL << CAN_MAILBOX_TX)) {
            /* First CAN message (bootup) was sent successfully */
            CANmodule->firstCANtxMessage = false;
            /* clear flag from previous message */
            CANmodule->bufferInhibitFlag = false;
            /* Are there any new messages waiting to be send */
            if(CANmodule->CANtxCount > 0U){
                uint16_t i;             /* index of transmitting message */

                /* first buffer */
                CO_CANtx_t *buffer = &CANmodule->txArray[0];
                /* search through whole array of pointers to transmit message buffers. */
                for(i = CANmodule->txSize; i > 0U; i--){
                    /* if message buffer is full, send it. */
                    if(buffer->bufferFull){
                        buffer->bufferFull = false;
                        CANmodule->CANtxCount--;

                        /* Copy message to CAN buffer */
                        CANmodule->bufferInhibitFlag = buffer->syncFlag;

                        /* can Send */
                        /* Disable mailbox to be able to modify MSGID */
                        shadow_canme.all = ECanRegPtr->CANME.all;
                        shadow_canme.all &= ~(1UL << CAN_MAILBOX_TX);
                        ECanRegPtr->CANME.all = shadow_canme.all;
                        /* Configure MSGID */
                        shadow_canmsgid.all = 0;
                        shadow_canmsgid.bit.STDMSGID = buffer->ident & 0x07FFU;
                        MBoxPtr[CAN_MAILBOX_TX].MSGID.all = shadow_canmsgid.all;
                        /* MSGCTRL */
                        shadow_canmsgctrl.all = 0;
                        shadow_canmsgctrl.bit.DLC = buffer->DLC;
                        if(buffer->ident & 0x0800U) {
                            shadow_canmsgctrl.bit.RTR = 1;
                        }
                        /* DATA */
                        shadow_canmdl.byte.BYTE0 = buffer->data[0];
                        shadow_canmdl.byte.BYTE1 = buffer->data[1];
                        shadow_canmdl.byte.BYTE2 = buffer->data[2];
                        shadow_canmdl.byte.BYTE3 = buffer->data[3];
                        shadow_canmdh.byte.BYTE4 = buffer->data[4];
                        shadow_canmdh.byte.BYTE5 = buffer->data[5];
                        shadow_canmdh.byte.BYTE6 = buffer->data[6];
                        shadow_canmdh.byte.BYTE7 = buffer->data[7];

                        MBoxPtr[CAN_MAILBOX_TX].MSGCTRL.all = shadow_canmsgctrl.all;
                        MBoxPtr[CAN_MAILBOX_TX].MDL.all = shadow_canmdl.all;
                        MBoxPtr[CAN_MAILBOX_TX].MDH.all = shadow_canmdh.all;

                        /* Re-enable mailbox */
                        shadow_canme.all = ECanRegPtr->CANME.all;
                        shadow_canme.all |= (1UL << CAN_MAILBOX_TX);
                        ECanRegPtr->CANME.all = shadow_canme.all;

                        shadow_cantrs.all = (1UL << CAN_MAILBOX_TX);
                        ECanRegPtr->CANTRS.all = shadow_cantrs.all;

                        break;                      /* exit for loop */
                    }
                    buffer++;
                }/* end of for loop */

                /* Clear counter if no more messages */
                if(i == 0U){
                    CANmodule->CANtxCount = 0U;
                }
            }
        }
    }
}
