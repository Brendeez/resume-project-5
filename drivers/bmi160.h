#ifndef BMI160_H
#define BMI160_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Slave Address */
#define BMI160_I2C_ADD          0x69 // P.93 of BMI160 Datasheet

/* Register Addresses */             // p.48 of BMI160 Datasheet
#define BMI160_CHIP_ID_ADD      0x00
#define BMI160_CHIP_ID_VAL      0xD1
#define BMI160_ERR_REG          0x02

#define BMI160_ACC_X_LSB        0x12 
#define BMI160_ACC_X_MSB        0x13
#define BMI160_ACC_Y_LSB        0x14 
#define BMI160_ACC_Y_MSB        0x15
#define BMI160_ACC_Z_LSB        0x16 
#define BMI160_ACC_Z_MSB        0x17

#define BMI160_CMD_REG          0x7E
#define BMI160_ACCEL_NORM_PMU   0x11
#define BMI160_PMU_STATUS_REG   0x03
#define BMI160_ACCEL_RANGE_REG  0x41
#define BMI160_ACCEL_RANGE_16g  0x0C
#define BMI160_FOC_CONF_REG     0x69
#define BMI160_FOC_SETTINGS     0x3D
#define BMI160_OFFSET_REG       0x77
#define BMI160_OFFSET_ACCEL_EN  0x40
#define BMI160_FOC_EN           0x03

struct accelData
{
    int16_t xAccel;
    int16_t yAccel;
    int16_t zAccel;
};

extern bool sensorBMI160Init(void);
extern void sensorBMI160ReadAccel(struct accelData *accelPacket);
extern void sensorBMI160ReadChipID(uint8_t *ID);
void sensorBMI160ReadErr(void);


#ifdef __cplusplus
}
#endif

#endif /* BMI160_H*/