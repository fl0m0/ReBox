/* 
 * File:   i2c.h
 * Author: Florian
 *
 * Created on December 12, 2021, 1:52 PM
 */

#ifndef I2C_H
#define	I2C_H

//#include "mcc_generated_files/i2c1_master.h"

#ifdef	__cplusplus
extern "C" {
#endif

    
// I2C PCA9955B LED driver
    
typedef struct
{
    size_t len;
    uint8_t *data;
}i2c1_buffer_t;
    
typedef struct
{
    uint8_t MODE1;
    uint8_t LEDOUT0;
    uint8_t LEDOUT1;
    uint8_t LEDOUT2;
    uint8_t LEDOUT3;
    uint8_t GRPPWM;
    uint8_t GRPFREQ;
    uint8_t PWM0;
    uint8_t PWM1;
    uint8_t PWM2;
    uint8_t PWM3;
    uint8_t PWM4;
    uint8_t PWM5;
    uint8_t PWM6;
    uint8_t PWM7;
    uint8_t PWM8;
    uint8_t PWM9;
    uint8_t PWM10;
    uint8_t PWM11;
    uint8_t PWM12;
    uint8_t PWM13;
    uint8_t PWM14;
    uint8_t PWM15;
    uint8_t IREF0;
    uint8_t RAMP_RATE_GRP0;
    uint8_t STEP_TIME_GRP0;
    uint8_t HOLD_CNTL_GRP0;
    uint8_t IREF_GRP0;
    uint8_t RAMP_RATE_GRP1;
    uint8_t STEP_TIME_GRP1;
    uint8_t HOLD_CNTL_GRP1;
    uint8_t IREF_GRP1;
    uint8_t RAMP_RATE_GRP2;
    uint8_t STEP_TIME_GRP2;
    uint8_t HOLD_CNTL_GRP2;
    uint8_t IREF_GRP2;
    uint8_t RAMP_RATE_GRP3;
    uint8_t STEP_TIME_GRP3;
    uint8_t HOLD_CNTL_GRP3;
    uint8_t IREF_GRP3;
    uint8_t GRAD_MODE_SEL0;
    uint8_t GRAD_MODE_SEL1;
    uint8_t GRAD_GRP_SEL0;
    uint8_t GRAD_GRP_SEL1;
    uint8_t GRAD_GRP_SEL2;
    uint8_t GRAD_GRP_SEL3;
    uint8_t GRAD_CNTL;
    uint8_t OFFSET;
    uint8_t PWMALL;
    uint8_t IREFALL;
}pca9955b_addr_map_t;

const pca9955b_addr_map_t PCA9955B_ADDR_MAP = 
{
    .MODE1 = 0x00,
    .LEDOUT0 = 0x02,
    .LEDOUT1 = 0x03,
    .LEDOUT2 = 0x04,
    .LEDOUT3 = 0x05,
    .GRPPWM = 0x06,
    .GRPFREQ = 0x07,
    .PWM0 = 0x08,
    .PWM1 = 0x09,
    .PWM2 = 0x0A,
    .PWM3 = 0x0B,
    .PWM4 = 0x0C,
    .PWM5 = 0x0D,
    .PWM6 = 0x0E,
    .PWM7 = 0x0F,
    .PWM8 = 0x10,
    .PWM9 = 0x11,
    .PWM10 = 0x12,
    .PWM11 = 0x13,
    .PWM12 = 0x14,
    .PWM13 = 0x15,
    .PWM14 = 0x16,
    .PWM15 = 0x17,
    .IREF0 = 0x18,
    .RAMP_RATE_GRP0 = 0x28,
    .STEP_TIME_GRP0 = 0x29,
    .HOLD_CNTL_GRP0 = 0x2A,
    .IREF_GRP0 = 0x2B,
    .RAMP_RATE_GRP1 = 0x2C,
    .STEP_TIME_GRP1 = 0x2D,
    .HOLD_CNTL_GRP1 = 0x2E,
    .IREF_GRP1 = 0x2F,
    .RAMP_RATE_GRP2 = 0x30,
    .STEP_TIME_GRP2 = 0x31,
    .HOLD_CNTL_GRP2 = 0x32,
    .IREF_GRP2 = 0x33,
    .RAMP_RATE_GRP3 = 0x34,
    .STEP_TIME_GRP3 = 0x35,
    .HOLD_CNTL_GRP3 = 0x36,
    .IREF_GRP3 = 0x37,
    .GRAD_MODE_SEL0 = 0x38,
    .GRAD_MODE_SEL1 = 0x39,
    .GRAD_GRP_SEL0 = 0x3A,
    .GRAD_GRP_SEL1 = 0x3B,
    .GRAD_GRP_SEL2 = 0x3C,
    .GRAD_GRP_SEL3 = 0x3D,
    .GRAD_CNTL = 0x3E,
    .OFFSET = 0x3F,
    .PWMALL = 0x44,
    .IREFALL = 0x45
};

typedef struct
{
    uint8_t IN_PORT0;
    uint8_t IN_PORT1;
    uint8_t OUT_PORT0;
    uint8_t OUT_PORT1;
    uint8_t INV_PORT0;
    uint8_t INV_PORT1;
    uint8_t CFG_PORT0;
    uint8_t CFG_PORT1;
}pca9535_cmd_t;

const pca9535_cmd_t PCA9535_CMD =
{
    .IN_PORT0 = 0x00,
    .IN_PORT1 = 0x01,
    .OUT_PORT0 = 0x02,
    .OUT_PORT1 = 0x03,
    .INV_PORT0 = 0x04,
    .INV_PORT1 = 0x05,
    .CFG_PORT0 = 0x06,
    .CFG_PORT1 = 0x07
};

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

