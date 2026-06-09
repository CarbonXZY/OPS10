#ifndef LSM6DSV16X_REGISTERS_H_
#define LSM6DSV16X_REGISTERS_H_

#include <stdint.h>

#define LSM6DSV16X_WHO_AM_I_REG        0x0F
#define LSM6DSV16X_WHO_AM_I_VAL        0x70

#define LSM6DSV16X_CTRL1               0x10
#define LSM6DSV16X_CTRL2               0x11
#define LSM6DSV16X_CTRL3               0x12
#define LSM6DSV16X_CTRL6               0x15
#define LSM6DSV16X_CTRL8               0x17
#define LSM6DSV16X_CTRL9               0x18
#define LSM6DSV16X_CTRL10              0x19

#define LSM6DSV16X_STATUS              0x1E

#define LSM6DSV16X_OUT_TEMP_L          0x20
#define LSM6DSV16X_OUT_TEMP_H          0x21
#define LSM6DSV16X_OUTX_L_G            0x22
#define LSM6DSV16X_OUTX_H_G            0x23
#define LSM6DSV16X_OUTY_L_G            0x24
#define LSM6DSV16X_OUTY_H_G            0x25
#define LSM6DSV16X_OUTZ_L_G            0x26
#define LSM6DSV16X_OUTZ_H_G            0x27
#define LSM6DSV16X_OUTX_L_A            0x28
#define LSM6DSV16X_OUTX_H_A            0x29
#define LSM6DSV16X_OUTY_L_A            0x2A
#define LSM6DSV16X_OUTY_H_A            0x2B
#define LSM6DSV16X_OUTZ_L_A            0x2C
#define LSM6DSV16X_OUTZ_H_A            0x2D

#endif
