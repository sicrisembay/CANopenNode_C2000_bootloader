/*!
 * \file eeprom.c
 */
#include "autoconf.h"

#if CONFIG_HAS_EEPROM
#include "DSP2833x_Device.h"
#include "eeprom.h"

void EEPROM_init(void)
{
    /*
     * Enable peripheral clock and Initialize I2C pins
     */
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // Enable pull-up for GPIO32 (SDAA)
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;    // Enable pull-up for GPIO33 (SCLA)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SDAA)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;  // Asynch input GPIO33 (SCLA)
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;   // Configure GPIO32 to SDAA
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;   // Configure GPIO33 to SCLA
    EDIS;

    /*
     * Initialize I2C peripheral
     */
    I2caRegs.I2CSAR = CONFIG_EEPROM_DEV_ADDR;
#if CONFIG_CORE_FREQ_150MHZ
    I2caRegs.I2CPSC.all = 14;   // Prescaler - need 7-12 Mhz on module clk (150/15 = 10MHz)
#elif CONFIG_CORE_FREQ_100MHZ
    I2caRegs.I2CPSC.all = 9;    // Prescaler - need 7-12 Mhz on module clk (100/10 = 10MHz)
#else
#error "Invalid CORE Frequency in I2C"
#endif
    I2caRegs.I2CCLKL = 10;          // NOTE: must be non zero
    I2caRegs.I2CCLKH = 5;           // NOTE: must be non zero
    I2caRegs.I2CIER.all = 0;        // Disable ALL interrupts
    I2caRegs.I2CMDR.all = 0x0020;   // Take out of Reset
    I2caRegs.I2CFFTX.all = 0x4040;  // Enable FIFO and clear TXFFINT.  TXFIFO is held in reset
    I2caRegs.I2CFFRX.all = 0x0040;  // Clear RXFFINT. RXFIFO is held in reset
}


EEPROM_RET_T EEPROM_read(const uint16_t address, uint8_t * pBuf, const uint16_t len)
{
    if((pBuf == NULL) || (len == 0) || (len > CONFIG_I2C_FIFO_BUFFER_SIZE)) {
        return EEPROM_INVALID_ARG;
    }

    if((address + len) >= CONFIG_EEPROM_SIZE) {
        return EEPROM_INVALID_ADDR;
    }

    while(I2caRegs.I2CMDR.bit.STP == 1);

    /*
     * Enable FIFO
     */
    I2caRegs.I2CFFTX.bit.TXFFRST = 1;
    I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1;
    I2caRegs.I2CFFRX.bit.RXFFRST = 1;
    /*
     * Setup slave address
     */
    I2caRegs.I2CSAR = CONFIG_EEPROM_DEV_ADDR;
    /*
     * Send EEPROM Address
     */
    while(I2caRegs.I2CSTR.bit.BB == 1);
    I2caRegs.I2CCNT = 2;
    I2caRegs.I2CDXR = (address >> 8) & 0x00FF;
    I2caRegs.I2CDXR = address & 0x00FF;
    I2caRegs.I2CMDR.all = 0x2620;
    while(I2caRegs.I2CSTR.bit.ARDY != 1);
    /*
     * Read EEPROM Data
     */
    while(I2caRegs.I2CFFRX.bit.RXFFST != 0) {
        // Empty Rx FIFO by dummy read
        pBuf[0] = I2caRegs.I2CDRR;
    }
    I2caRegs.I2CCNT = len;
    I2caRegs.I2CMDR.all = 0x2C20;
    while(I2caRegs.I2CMDR.bit.STP == 1);
    for(uint16_t i = 0; i < len; i++) {
        pBuf[i] = I2caRegs.I2CDRR & 0x00FF;
    }

    /*
     * Disable FIFO
     */
    I2caRegs.I2CFFTX.bit.TXFFRST = 0;
    I2caRegs.I2CFFRX.bit.RXFFRST = 0;

    return EEPROM_OK;
}

EEPROM_RET_T EEPROM_write(const uint16_t address, uint16_t * pBuf, const uint16_t len)
{
    if((pBuf == NULL) || (len == 0) || (len > (CONFIG_I2C_FIFO_BUFFER_SIZE - 2))) {
        return EEPROM_INVALID_ARG;
    }

    if((address + len) >= CONFIG_EEPROM_SIZE) {
        return EEPROM_INVALID_ADDR;
    }

    while(I2caRegs.I2CMDR.bit.STP == 1);
    /*
     * Enable FIFO
     */
    I2caRegs.I2CFFTX.bit.TXFFRST = 1;
    I2caRegs.I2CFFTX.bit.TXFFINTCLR = 1;

    /*
     * Setup slave device address
     */
    I2caRegs.I2CSAR = CONFIG_EEPROM_DEV_ADDR;
    while(I2caRegs.I2CSTR.bit.BB == 1);
    /*
     * Setup EEPROM address and data to write
     */
    I2caRegs.I2CCNT = len + 2;
    I2caRegs.I2CDXR = (address >> 8) & 0x00FF;
    I2caRegs.I2CDXR = address & 0x00FF;
    for(uint16_t i = 0; i < len; i++) {
        I2caRegs.I2CDXR = pBuf[i] & 0x00FF;
    }
    I2caRegs.I2CMDR.all = 0x6E20;

    while(I2caRegs.I2CMDR.bit.STP == 1);

    return EEPROM_OK;
}

#endif /* CONFIG_HAS_EEPROM */

