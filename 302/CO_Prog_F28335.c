#include "string.h"
#include "CO_Prog_F28335.h"
#include "OD.h"
#include "Flash2833x_API_Config.h"
#include "Flash2833x_API_Library.h"
#include "crc_tbl.h"

#if (CO_CONFIG_PROG & CO_CONFIG_PROG_ENABLE)

#define INTEL_HEX_COUNT_OFFSET      (0)
#define INTEL_HEX_ADDRESS_OFFSET    (1)
#define INTEL_HEX_TYPE_OFFSET       (3)
#define INTEL_HEX_DATA_OFFSET       (4)

#define INTEL_HEX_TYPE_DATA         (0)
#define INTEL_HEX_TYPE_EOL          (1)
#define INTEL_HEX_TYPE_EXT_ADDR     (4)

#define APP_FLASH_START_ADDRESS     (0x300000)
#define APP_FLASH_END_ADDRESS       (0x337FFF)
#define PROGRAM_PAGE_SIZE           (256)  // words

#define APP_ENTRY_ADDRESS           (0x337F7E)  // refer to linker memory definition of APP_BEGIN
#define APP_CRC_ADDRESS             (0x337F80)  // refer to linker memory definition of APP_CRC

static const MEMRANGE_CRC_TABLE * const pCrcTable = (MEMRANGE_CRC_TABLE *)APP_CRC_ADDRESS;
static CO_Program_t program;

static uint32_t extendedLinearAddress = 0;
static uint32_t programAddress = 0;
static uint16_t sector = 0;
static uint16_t sectorErasedFlag = 0;
static uint16_t progBuffer[PROGRAM_PAGE_SIZE];
static uint16_t progLen;

static uint16_t GetSector(uint32_t addr)
{
    uint16_t ret = 0;

    if((addr >= 0x300000) && (addr <= 0x307FFF)) {
        ret = SECTORH;
    } else if((addr >= 0x308000) && (addr <= 0x30FFFF)) {
        ret = SECTORG;
    } else if((addr >= 0x310000) && (addr <= 0x317FFF)) {
        ret = SECTORF;
    } else if((addr >= 0x318000) && (addr <= 0x31FFFF)) {
        ret = SECTORE;
    } else if((addr >= 0x320000) && (addr <= 0x327FFF)) {
        ret = SECTORD;
    } else if((addr >= 0x328000) && (addr <= 0x32FFFF)) {
        ret = SECTORC;
    } else if((addr >= 0x330000) && (addr <= 0x337FFF)) {
        ret = SECTORB;
    }

    return ret;
}


static void ConvertBufToWord(uint8_t * buf, uint8_t len)
{
    progLen = len / 2;
    if(progLen > PROGRAM_PAGE_SIZE) {
        progLen = 0;
        return;
    }

    for(uint16_t i = 0; i < progLen; i++) {
        progBuffer[i] = ((((uint16_t)buf[2*i]) << 8) & 0xFF00) +
                        ((uint16_t)buf[(2*i) + 1] & 0x00FF);
    }
}


bool CheckAppCrc()
{
    /* Paranoid test: Record size */
    if(pCrcTable->rec_size != sizeof(MEMRANGE_CRC_RECORD)) {
        return false;
    }

    /* Note: application linker computes a single CRC over several ranges */
    size_t total_len = 0;
    for(int i = 0; i < pCrcTable->num_recs; i++) {
        total_len += pCrcTable->recs[i].size;
    }

    /* Calculate CRC */
    extern unsigned long gen_crc(int id, const unsigned char *data, size_t len);
    uint32_t crcVal = gen_crc((int)(pCrcTable->crc_alg_ID),
                              (const unsigned char *)(pCrcTable->recs[0].addr),
                              total_len);
    if(crcVal != pCrcTable->crc_value) {
        return false;
    }

    return true;
}


bool CheckAppAndJump()
{
    void (*appEntry)(void);

    if(!CheckAppCrc()) {
        return false;
    }

    /* De-init peripheral before jumping to Application */
    volatile struct ECAN_REGS * ECanRegPtr = (volatile struct ECAN_REGS *)program.CANmodule->CANptr;
    ECanRegPtr->CANME.all  = 0x00000000;        // disable all mailbox
    ECanRegPtr->CANMD.all  = 0x00000000;        // Default all mailbox as transmit.
    ECanRegPtr->CANGIF0.all = 0xFFFFFFFF;       // Clears Line0 Global interrupt flags by writing 1's
    ECanRegPtr->CANGIF1.all = 0xFFFFFFFF;       // Clears Line1 Global interrupt flags by writing 1's
    ECanRegPtr->CANTA.all  = 0xFFFFFFFF;        // Clears CANTA by writing 1's
    ECanRegPtr->CANRMP.all = 0xFFFFFFFF;        // Clears RMP by writing 1's
    ECanRegPtr->CANMIM.all = 0x00000000;        // disable all mailbox interrupts
    ECanRegPtr->CANGIM.all = 0x00000000;        // disable all interrupts

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.ECANAENCLK = 0;
    SysCtrlRegs.PCLKCR0.bit.ECANBENCLK = 0;
    EDIS;

    I2caRegs.I2CMDR.bit.IRS = 0;        // Disable I2C peripheral
    I2caRegs.I2CFFTX.bit.I2CFFEN = 0;   // Disable FIFO
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 0;
    EDIS;

    appEntry = (void (*)(void))APP_ENTRY_ADDRESS;
    appEntry();

    /* Does not reach here */
    return true;
}


static ODR_t OD_write_1F50_callback(OD_stream_t * stream,
        const void * buf,
        OD_size_t count,
        OD_size_t * countWritten)
{
    /*
     * Verify arguments
     */
    if((stream == NULL) || (stream->subIndex == 0) || (buf == NULL) ||
       (countWritten == NULL)) {
        return ODR_DEV_INCOMPAT;
    }

    uint8_t * pBuf8 = (uint8_t *)buf;

    /*
     * Calculate Checksum
     */
    uint32_t checksum = 0;
    for(uint16_t i = 0; i < count; i++) {
        checksum += pBuf8[i];
    }
    if((checksum & 0x00FF) != 0) {
        return ODR_HW;
    }
    /*
     * Parse Intel Hex Record
     */
    uint8_t recType = pBuf8[INTEL_HEX_TYPE_OFFSET] & 0x00FF;
    switch(recType) {
        case INTEL_HEX_TYPE_DATA: {
            uint16_t retFlash = STATUS_SUCCESS;
            FLASH_ST flashSt;
            uint32_t addressWordLow = ((((uint16_t)pBuf8[INTEL_HEX_ADDRESS_OFFSET]) << 8) & 0xFF00) +
                                       (((uint16_t)pBuf8[INTEL_HEX_ADDRESS_OFFSET + 1]) & 0x00FF);
            programAddress = extendedLinearAddress | addressWordLow;
            /* Is programAddress valid? */
            if((programAddress >= APP_FLASH_START_ADDRESS) && (programAddress <= APP_FLASH_END_ADDRESS)) {
                sector = GetSector(programAddress);
                if((sector & sectorErasedFlag) == 0) {
                    /* Erase Sector */
                    retFlash = Flash_Erase(sector, &flashSt);
                    if(retFlash == STATUS_SUCCESS) {
                        sectorErasedFlag |= sector;
                    } else {
                        /* Failed Flash Erase */
                        sector = 0;
                        sectorErasedFlag = 0;
                        return ODR_HW;
                    }
                }
                /*
                 * Convert from byte to word
                 */
                ConvertBufToWord(&(pBuf8[INTEL_HEX_DATA_OFFSET]), pBuf8[INTEL_HEX_COUNT_OFFSET]);
                if(progLen != 0) {
                    retFlash = Flash_Program((uint16_t *)programAddress, progBuffer, progLen, &flashSt);
                    if(retFlash != STATUS_SUCCESS) {
                        /* Failed Flash Program */
                        sector = 0;
                        sectorErasedFlag = 0;
                        return ODR_HW;
                    }
                }
            }
            break;
        }
        case INTEL_HEX_TYPE_EOL: {
            /* Clear flags */
            sector = 0;
            sectorErasedFlag = 0;
            break;
        }
        case INTEL_HEX_TYPE_EXT_ADDR: {
            uint16_t addressWordHigh = ((((uint16_t)pBuf8[INTEL_HEX_DATA_OFFSET]) << 8) & 0xFF00) +
                                        (((uint16_t)pBuf8[INTEL_HEX_DATA_OFFSET + 1]) & 0x00FF);
            extendedLinearAddress = (((uint32_t)addressWordHigh) << 16) & 0xFFFF0000;
            break;
        }
        default: {
            return ODR_GENERAL;
        }
    }
    return ODR_OK;
}


static ODR_t OD_write_1F51_callback(OD_stream_t * stream,
        const void * buf,
        OD_size_t count,
        OD_size_t * countWritten)
{
    /*
     * Verify arguments
     */
    if((stream == NULL) || (stream->subIndex == 0) || (buf == NULL) ||
       (countWritten == NULL)) {
        return ODR_DEV_INCOMPAT;
    }

    switch(stream->subIndex) {
        case 1: {
            uint32_t val = CO_getUint32(buf);
            if(val == 1) {
                if(!CheckAppAndJump()) {
                    return ODR_NO_DATA;
                }
            }
            break;
        }
        default: {
            return ODR_DEV_INCOMPAT;
        }
    }

    return ODR_OK;
}


static ODR_t OD_write_2000_callback(OD_stream_t * stream,
        const void * buf,
        OD_size_t count,
        OD_size_t * countWritten)
{
    /*
     * Verify arguments
     */
    if((stream == NULL) || (stream->subIndex == 0) || (buf == NULL) ||
       (countWritten == NULL)) {
        return ODR_DEV_INCOMPAT;
    }

    switch(stream->subIndex) {
        case 1: {
            /* Command */
            uint16_t command = CO_getUint16(buf);
            switch(command) {
                case 0x01: {
                    /* Calculate CRC */
                    if(!CheckAppCrc()) {
                        return ODR_HW;
                    }
                    break;
                }
                case 0x02: {
                    /* Jump to application */
                    if(!CheckAppAndJump()) {
                        return ODR_HW;
                    }
                    break;
                }
                case 0x03: {
                    /* Erase sector */
                    FLASH_ST flashSt = {0};
                    uint16_t retFlash = STATUS_SUCCESS;
                    uint16_t sectorMask;
                    OD_size_t bytesRd;
                    OD_IO_t io_bootloaderEraseSectorMask;
                    if(ODR_OK != OD_getSub(OD_ENTRY_H2000, 0x02, &io_bootloaderEraseSectorMask, true)) {
                        return ODR_HW;
                    }
                    if(ODR_OK != io_bootloaderEraseSectorMask.read(&io_bootloaderEraseSectorMask.stream, &sectorMask, 2, &bytesRd)) {
                        return ODR_HW;
                    }
                    sectorMask &= 0x00FE;
                    retFlash = Flash_Erase(sectorMask, &flashSt);
                    if(retFlash == STATUS_SUCCESS) {
                        sectorErasedFlag |= sectorMask;
                    } else {
                        /* Failed Flash Erase */
                        sector = 0;
                        sectorErasedFlag = 0;
                        return ODR_HW;
                    }
                    break;
                }
                default: {
                    return ODR_HW;
                }
            }
            break;
        }
        case 2: {
            if(count != 2) {
                /* Expected uint16_t */
                return ODR_DEV_INCOMPAT;
            }
            uint16_t sectorMask = CO_getUint16(buf);
            /* Paranoid check: SectorA must not be overwritten */
            sectorMask &= 0x00FE;
            /* Set Erase Sector Mask */
            if(ODR_OK != OD_writeOriginal(stream, &sectorMask, count, countWritten)) {
                return ODR_HW;
            }
            break;
        }
        default: {
            return ODR_DEV_INCOMPAT;
        }
    }
    return ODR_OK;
}


CO_ReturnError_t CO_Prog_F28335_init(CO_CANmodule_t * CANmodule)
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    memset(&program, 0, sizeof(program));

    /* verify arguments */
    if (CANmodule == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Initialize Flash API */
    Flash_CPUScaleFactor = SCALE_FACTOR;
    Flash_CallbackPtr = NULL;

    /* COnfigure object variables */
    program.CANmodule = CANmodule;

    ret = CO_Prog_init(&program,
                       OD_ENTRY_H1F50_downloadProgramData,
                       OD_ENTRY_H1F51_programControl,
                       OD_ENTRY_H2000_bootloader,
                       OD_write_1F50_callback,
                       OD_write_1F51_callback,
                       OD_write_2000_callback);

    return ret;
}

#endif /* (CO_CONFIG_PROG & CO_CONFIG_PROG_ENABLE) */
