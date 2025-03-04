/* 
 * File:   rebox.h
 * Author: Florian
 *
 * Created on 12. Februar 2022, 12:53
 */

#ifndef REBOX_H
#define	REBOX_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "mcc_generated_files/i2c1_master.h"
// Pin definitions
#define STATUS_LED  LATAbits.LATA4
#define LED_OE_N    LATAbits.LATA5
#define IO_INT_N    PORTAbits.RA5
    
// Hardware addresses
// LED controller
#define LED_CTRL0_ADDR  0x15
#define LED_CTRL1_ADDR  0x16
#define LED_CTRL2_ADDR  0x17
#define LED_CTRL3_ADDR  0x18
// IO expander
#define IO_CTRL_ADDR    0x20

// Constants
#define IO_CTRL_CFG_REGS 0x007F // IOs 0-6,15 as inputs, IOs 7-15 as outputs
#define IO_CTRL_OUT_REGS 0xC0FF // Default states for outputs (relays are off)
#define LED_CTRL_IREF    0x3F   // LED controller current setting = 20 mA / 4 = 5 mA
#define PWR_LED_PWM_ON   0x3F   // Relay closed indicator LEDs, PWM value
#define PWR_LED_PWM_OFF  0x00
#define TMR0_INIT_VAL    0x79   // 2 s
#define RELAY_MASK       0xC0   // Mask for relais on IO expander IOs 0 to 5
#define PB_GROUPS        0x06   // Push button groups, only used for PBs 1 to 6, not 7

// Global variables and typedefs
typedef enum {STARTUP=0, NORMAL, PROGRAMMING} operating_mode_t;
    
// Push button stuff
typedef struct
{
    uint8_t PB1; // Channels 1 - 7
    uint8_t PB2;
    uint8_t PB3;
    uint8_t PB4;
    uint8_t PB5;
    uint8_t PB6;
    uint8_t PB7; // Mode select button
    uint8_t PB_1TO6; // Channels 1-6
}pb_mask_t;
const pb_mask_t PB_MASK =
{
    .PB1 = 0x01,
    .PB2 = 0x02,
    .PB3 = 0x04,
    .PB4 = 0x08,
    .PB5 = 0x20, // 5 and 6 are swapped, PCB mistake
    .PB6 = 0x10,
    .PB7 = 0x40,
    .PB_1TO6 = 0x3F
};

// Relay controll stuff
//typedef struct
//{
//    uint8_t RLY1;
//    uint8_t RLY2;
//    uint8_t RLY3;
//    uint8_t RLY4;
//    uint8_t RLY5;
//    uint8_t RLY6;
//}relay_mask_t;

//const relay_mask_t RELAY_MASK =
//{
//    .RLY1 = 0x01,
//    .RLY2 = 0x02,
//    .RLY3 = 0x04,
//    .RLY4 = 0x08,
//    .RLY5 = 0x20,
//    .RLY6 = 0x10
//};

// LED stuff
typedef struct
{
    i2c1_address_t I2C_ADDR;
    uint8_t PB_LED_R0;
    uint8_t PB_LED_G0;
    uint8_t PB_LED_B0;
    uint8_t PWR_LED0;
}led_addr_t;
// LEDs of LED controller 0
const led_addr_t CH1_LED =
{
    .I2C_ADDR  = LED_CTRL0_ADDR,
    .PB_LED_R0 = 0x00,
    .PB_LED_G0 = 0x01,
    .PB_LED_B0 = 0x02,
    .PWR_LED0  = 0x0C
};
const led_addr_t CH2_LED =
{
    .I2C_ADDR  = LED_CTRL0_ADDR,
    .PB_LED_R0 = 0x06,
    .PB_LED_G0 = 0x07,
    .PB_LED_B0 = 0x08,
    .PWR_LED0  = 0x0E
};
// LEDs of LED controller 1
const led_addr_t CH3_LED =
{
    .I2C_ADDR  = LED_CTRL1_ADDR,
    .PB_LED_R0 = 0x00,
    .PB_LED_G0 = 0x01,
    .PB_LED_B0 = 0x02,
    .PWR_LED0  = 0x0C
};
const led_addr_t CH4_LED =
{
    .I2C_ADDR  = LED_CTRL1_ADDR,
    .PB_LED_R0 = 0x06,
    .PB_LED_G0 = 0x07,
    .PB_LED_B0 = 0x08,
    .PWR_LED0  = 0x0E
};
// LEDs of LED controller 3
// 5 and 6 are swapped, PCB mistake
const led_addr_t CH5_LED =
{
    .I2C_ADDR  = LED_CTRL2_ADDR,
    .PB_LED_R0 = 0x06,
    .PB_LED_G0 = 0x07,
    .PB_LED_B0 = 0x08,
    .PWR_LED0  = 0x0E
};
const led_addr_t CH6_LED =
{
    .I2C_ADDR  = LED_CTRL2_ADDR,
    .PB_LED_R0 = 0x00,
    .PB_LED_G0 = 0x01,
    .PB_LED_B0 = 0x02,
    .PWR_LED0  = 0x0C
    
};
// LEDs of LED controller 7
const led_addr_t CH7_LED =
{
    .I2C_ADDR  = LED_CTRL3_ADDR,
    .PB_LED_R0 = 0x00,
    .PB_LED_G0 = 0x01,
    .PB_LED_B0 = 0x02,
    .PWR_LED0  = 0x1F // not available
};



// Function definitions
void boot_msg(void);
void PCA9955B_init(i2c1_address_t addr);
void PCA9955B_gradation_init(i2c1_address_t addr);
void PCA9955B_gradation_enable(i2c1_address_t addr);
void PCA9955B_gradation_disable(i2c1_address_t addr);
void PCA9955B_read(i2c1_address_t device_addr, uint8_t reg_addr);
void set_color1(i2c1_address_t addr, uint8_t *p_color);
void set_color2(i2c1_address_t addr, uint8_t *p_color);
void set_pb_color(led_addr_t led_addr, uint8_t *p_color);
void set_pb_color_by_group(uint8_t *p_pb_groups);
void run_startup_animation(uint8_t n_cycles);
uint8_t io_ctrl_cfg(i2c1_address_t addr, uint16_t cfg_reg_pair, uint16_t out_reg_pair);
uint8_t get_pb_states(i2c1_address_t addr);
void set_relays(i2c1_address_t addr, uint8_t relay_states);
uint8_t pb_edge_detect(uint8_t current_state);
uint8_t pb7_long_press_detect(uint8_t pb_states, uint8_t rising_edges);
void pb7_led_flash(void);
void set_pb_groups(uint8_t *p_pb_groups, uint8_t pb_rising_egdges);
void set_relay_closed_led(uint8_t relay_states);
void _set_relay_closed_led_helper(led_addr_t led_addr, uint8_t pwm_val);
void calculate_pb_group_masks(uint8_t *p_pb_groups, uint8_t *p_pb_group_mask);
uint8_t set_relays_by_group(i2c1_address_t addr, uint8_t relay_states,
        uint8_t pb_rising_edges, uint8_t *p_pb_groups, uint8_t *p_pb_group_masks);

#ifdef	__cplusplus
}
#endif

#endif	/* REBOX_H */


