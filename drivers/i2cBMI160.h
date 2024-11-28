#ifndef _I2CBMI160_H_
#define _I2CBMI160_H_

#include <stdbool.h>
#include <stdint.h>

extern bool writeI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readI2CBMI(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool readI2CBMIData(uint8_t ui8Addr, uint8_t ui8Reg, int8_t *Data);


#endif /* _I2CBMI160_H_ */