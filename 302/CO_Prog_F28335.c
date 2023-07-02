#include "string.h"
#include "CO_Prog_F28335.h"
#include "OD.h"
#include "Flash2833x_API_Config.h"
#include "Flash2833x_API_Library.h"

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
        return ODR_GENERAL;
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
                        /// TODO: Handle error
                    }
                }
                /*
                 * Convert from byte to word
                 */
                ConvertBufToWord(&(pBuf8[INTEL_HEX_DATA_OFFSET]), pBuf8[INTEL_HEX_COUNT_OFFSET]);
                if(progLen != 0) {
                    retFlash = Flash_Program((uint16_t *)programAddress, progBuffer, progLen, &flashSt);
                    if(retFlash != STATUS_SUCCESS) {
                        /// TODO
                    }
                }
            }
            break;
        }
        case INTEL_HEX_TYPE_EOL: {
            /* Clear flags */
            sectorErasedFlag = 0;
            /* Calculate CRC */
            /// TODO
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
                void (*appEntry)(void);
                appEntry = (void (*)(void))0x337C00;
                appEntry();
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
                       OD_write_1F50_callback,
                       OD_write_1F51_callback);

    return ret;
}

#endif /* (CO_CONFIG_PROG & CO_CONFIG_PROG_ENABLE) */
