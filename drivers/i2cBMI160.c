#include "i2cBMI160.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"

/*
 * Sets slave address to ui8Addr
 * Puts ui8Reg followed by two data bytes in *data and transfers
 * over i2c
 */
bool writeI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C2_BASE)) { }

    // Send Data
    UARTprintf("CMD being sent: 0x%x\n", *data);
    I2CMasterDataPut(I2C2_BASE, *data);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C2_BASE)) { }

    // I2CMasterDataPut(I2C2_BASE, data[1]);
    // I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    // Delay until transmission completes
    // while(I2CMasterBusBusy(I2C2_BASE)) { }

    return true;
}


/*
 * Sets slave address to ui8Addr
 * Writes ui8Reg over i2c to specify register being read from
 * Reads three bytes from i2c slave. The third is redundant but
 * helps to flush the i2c register
 * Stores first two received bytes into *data
 */
bool readI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    uint16_t delay = 1000;
    uint8_t byteA, byteB;
    

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C2_BASE)) { }

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);

    // Read two bytes from I2C
    // I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C2_BASE)) { }
    byteA = I2CMasterDataGet(I2C2_BASE);
    // UARTprintf("BytaA: %d", byteA);

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    SysCtlDelay(delay);
    byteB = I2CMasterDataGet(I2C2_BASE);
    // UARTprintf("BytaB: %d", byteB);


    // *data = byteA;
    data[0] = byteA;
    data[1] = byteB;

    return true;
}

bool readI2CBMIData(uint8_t ui8Addr, uint8_t ui8Reg, int8_t *data)
{
    uint16_t delay = 1000;
    int8_t byteA, byteB;
    

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    // Place the character to be sent in the data register
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C2_BASE)) { }

    // Load device slave address
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);

    // Read two bytes from I2C
    // I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C2_BASE)) { }
    byteA = I2CMasterDataGet(I2C2_BASE);
    // UARTprintf("BytaA: %d", byteA);

    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    SysCtlDelay(delay);
    byteB = I2CMasterDataGet(I2C2_BASE);
    // UARTprintf("BytaB: %d", byteB);


    // *data = byteA;
    data[0] = byteA;
    data[1] = byteB;

    return true;
}