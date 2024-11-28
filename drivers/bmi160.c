#include "bmi160.h"
#include "i2cBMI160.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"

extern uint32_t g_ui32SysClock;

bool sensorBMI160Init(void){
    uint8_t ID;
    uint8_t PMU_MODE = BMI160_ACCEL_NORM_PMU;
    uint8_t PMU_STATUS;
    uint8_t ACCEL_RANGE_16G = BMI160_ACCEL_RANGE_16g;
    // uint8_t FOC_CONF_SETTING = BMI160_FOC_SETTINGS;
    // uint8_t OFFSET_EN = BMI160_OFFSET_ACCEL_EN;
    // uint8_t FOC_EN = BMI160_FOC_EN;

    /* Reading Sensor Chip ID*/
    sensorBMI160ReadChipID(&ID);
    if(ID != BMI160_CHIP_ID_VAL){
        UARTprintf("Read BMI160 Chip ID is incorrect value of 0x%x, when it should be 0x%x\n", ID, BMI160_CHIP_ID_VAL);
        return false;
    }
    UARTprintf("Read BMI160 Chip ID is correct value of: 0x%X\n", ID);

    /* Calibration of Accelerometer */
    writeI2CBMI(BMI160_I2C_ADD, BMI160_ACCEL_RANGE_REG, (uint8_t *)&ACCEL_RANGE_16G); // Set range to 16G
    writeI2CBMI(BMI160_I2C_ADD, BMI160_CMD_REG, (uint8_t *)&PMU_MODE); // Set to normal operation
    SysCtlDelay(g_ui32SysClock);
    readI2CBMI(BMI160_I2C_ADD, BMI160_PMU_STATUS_REG, (uint8_t *)&PMU_STATUS); // Check Status of Accelerometer
    if(PMU_STATUS == 0x10){
        UARTprintf("The Accel is in normal mode!\n");
    }else{
        UARTprintf("Something went wrong, Accel is not in normal mode... PMU_STATUS: %x\n", PMU_STATUS);
        sensorBMI160ReadErr();

        return false;
    }

    return true;   
}

void sensorBMI160ReadChipID(uint8_t *ID){

    uint8_t val = 0;
    readI2CBMI(BMI160_I2C_ADD, BMI160_CHIP_ID_ADD, (uint8_t *)&val);

    *ID = val;
}


void sensorBMI160ReadAccel(struct accelData *accelPacket){
    
    struct accelData temp;
    // uint8_t MSB;
    // uint8_t LSB;
    // uint16_t data;
    int16_t xAccelData;
    int16_t yAccelData;
    int16_t zAccelData;

    // readI2CBMI(BMI160_I2C_ADD, BMI160_ACC_X_MSB, (uint8_t *)&MSB);
    readI2CBMIData(BMI160_I2C_ADD, BMI160_ACC_X_LSB, (int8_t *)&xAccelData);
    // temp.xAccel = xAccelData/2048;
    temp.xAccel = xAccelData;

    readI2CBMIData(BMI160_I2C_ADD, BMI160_ACC_Y_LSB, (int8_t *)&yAccelData);
    temp.yAccel = yAccelData;

    readI2CBMIData(BMI160_I2C_ADD, BMI160_ACC_Z_LSB, (int8_t *)&zAccelData);
    temp.zAccel = zAccelData;

    *accelPacket = temp;
}

void sensorBMI160ReadErr(void){
    uint8_t errVal;
    readI2CBMI(BMI160_I2C_ADD, BMI160_ERR_REG,(uint8_t *)&errVal);

    UARTprintf("Error Code: 0x%x\n", errVal);
}