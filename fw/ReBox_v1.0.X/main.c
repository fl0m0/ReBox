/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC16F18326
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include "rebox.h"
#include "i2c.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"

// Global variables
// Color definitions
uint8_t COLOR_RED[3]    = {0xC0,0x00,0x00};
uint8_t COLOR_GREEN[3]  = {0x00,0x80,0x00};
uint8_t COLOR_BLUE[3]   = {0x00,0x00,0xFF};
uint8_t COLOR_YELLOW[3] = {0xD0,0x80,0x00};
uint8_t COLOR_CYAN[3]   = {0x00,0xC0,0xFF};
uint8_t COLOR_VIOLET[3] = {0xC0,0x00,0xFF};
uint8_t COLOR_WHITE[3]  = {0xFF,0x60,0x40};
uint8_t COLOR_OFF[3]    = {0x00,0x00,0x00};
uint8_t COLORS[21]      = {0xC0,0x00,0x00,  // red
                           0x00,0x80,0x00,  // green
                           0x00,0x00,0xFF,  // blue
                           0xD0,0x80,0x00,  // yellow
                           0x00,0xC0,0xFF,  // cyan
                           0xC0,0x00,0xFF,  // violet
                           0xFF,0x60,0x40}; // white

// Timer0: Used for detection of long (2 s) push button presses and some other
//         time delays.
// Timer2: Used for Super Loop, sets tmr0_trigger, 64 ms


/*
                         Main application
 */
void main(void)
{
    operating_mode_t operating_mode = STARTUP;
    // I2C dadresses
    i2c1_address_t led_ctrl0_addr = LED_CTRL0_ADDR;
    i2c1_address_t led_ctrl1_addr = LED_CTRL1_ADDR;
    i2c1_address_t led_ctrl2_addr = LED_CTRL2_ADDR;
    i2c1_address_t led_ctrl3_addr = LED_CTRL3_ADDR;
    i2c1_address_t io_ctrl_addr = IO_CTRL_ADDR;
    uint8_t status_led_state = 0; // Green status LED on PCB, flashes in super loop, 0 = off
    uint8_t pb_states = 0x00; // 0 for pressed, bitwise
    uint8_t pb_rising_edges = 0x00; // 1 for rising edge, bitwise
    uint8_t pb7_long_press = 0;
    uint8_t relay_states = 0x00; // 0 = relay open
    uint8_t relay_states_old = 0x00;
    
    // Group assignment for PBs 1 to PB_GROUPS
    uint8_t pb_groups[PB_GROUPS] = {0, 1, 2, 3, 4, 5};
    uint8_t pb_group_masks[PB_GROUPS] = {0x01,0x02,0x04,0x08,0x10,0x20};

    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    // INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    // INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    // Configure IO expander in/out pins, initial states
    io_ctrl_cfg(io_ctrl_addr, IO_CTRL_CFG_REGS, IO_CTRL_OUT_REGS);
     // Init LED controllers (current value, pwm off)
    PCA9955B_init(led_ctrl0_addr);
    PCA9955B_init(led_ctrl1_addr);
    PCA9955B_init(led_ctrl2_addr);
    PCA9955B_init(led_ctrl3_addr);
    PCA9955B_gradation_init(led_ctrl3_addr);
    // Initial PB LED colors
    set_pb_color_by_group(pb_groups); // For PBs 1 to 6
    set_pb_color(CH7_LED, COLOR_WHITE); // For PB 7
    LED_OE_N = 0; // LED controller global enable signal
    // Startup animation
    run_startup_animation(0x0D); // 13 cycles
    boot_msg();
    TMR0_StopTimer();
    TMR0_Reload(TMR0_INIT_VAL);
    PIR0bits.TMR0IF = 0;
    // Super loop
    while (1)
    {
        // Periodic trigger from TRM2
        if (TMR2_HasOverflowOccured())
        {
            // Status LED blinki
            STATUS_LED = status_led_state;
            status_led_state = ~status_led_state;
            
            // Get push button states (1 means pressed, bits 0 to 6 = PB 1 to 7)
            pb_states = get_pb_states(io_ctrl_addr);
            pb_rising_edges = pb_edge_detect(pb_states);
            pb7_long_press = pb7_long_press_detect(pb_states, pb_rising_edges);
            
            switch (operating_mode)
            {
                case(STARTUP):
                    operating_mode = NORMAL;
                    break;

                case(NORMAL):
                    // PB7 always opens all realays
                    if (pb7_long_press > 0)
                    {
                        printf("PB7 pressed\r\n");
                        relay_states = 0x00;
                        set_relays(io_ctrl_addr, relay_states);
                    }
                    
                    // PB7 long press switches to programming mode
                    if (pb7_long_press == 2)
                    {
                        printf("Entering programming mode\r\n");
                        operating_mode = PROGRAMMING;
                    }
                    
                    // PBs 1 to 6 are switching the relays
                    if (pb_rising_edges > 0)
                    {
                        // XORing, example:
                        // 1 0 0 1 1 1 relay_states
                        // 0 1 0 1 0 0 rising_edges
                        // ----------- XOR
                        // 1 1 0 0 1 1
                        relay_states = set_relays_by_group(io_ctrl_addr, relay_states, pb_rising_edges, pb_groups, pb_group_masks);
                    }
                    
                    if ((relay_states > 0x00) && (relay_states_old == 0x00))
                    {
                        PCA9955B_gradation_enable(led_ctrl3_addr);
                    }
                    else if ((relay_states == 0x00) && (relay_states_old > 0x00))
                    {
                        PCA9955B_gradation_disable(led_ctrl3_addr);
                    }
                    relay_states_old = relay_states;
                    break;

                case(PROGRAMMING):
                    pb7_led_flash();
                    // PB7 press returns to normal programming mode
                    if (pb7_long_press > 0)
                    {
                        printf("Entering normal mode\r\n");
                        operating_mode = NORMAL;
                        calculate_pb_group_masks(pb_groups, pb_group_masks); // Needed for relay control
                        set_pb_color(CH7_LED, COLOR_WHITE); // Reset after flashing
                    }
                    if (pb_rising_edges > 0)
                    {
                        set_pb_groups(pb_groups, pb_rising_edges);
                        set_pb_color_by_group(pb_groups);
                    }
                    break;
            }
        }

    }
}

void boot_msg()
{
    printf("\r\n---------------------------------------------\r\n");
    printf("                ReBox Rev. A1                \r\n");
    printf("---------------------------------------------\r\n");
    printf("FW version: 0.1, build date 2022-20-02\r\n");
}

void PCA9955B_init(i2c1_address_t addr)
{
    // Global current & PWM, overwrites group settings
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWMALL, 0x00);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.IREFALL, LED_CTRL_IREF);
}

void PCA9955B_gradation_init(i2c1_address_t addr)
{
    // Gradation control:
    // Ramp up enables, ramp down enabled, ramp rate value 0
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.RAMP_RATE_GRP0, 0xC0);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.RAMP_RATE_GRP1, 0xC0);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.RAMP_RATE_GRP2, 0xC0);
    // 2*8ms step time
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.STEP_TIME_GRP0, 0x41);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.STEP_TIME_GRP1, 0x41);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.STEP_TIME_GRP2, 0x41);
    // Hold ON enable, Hold OFF enable, different Hold OFF times
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.HOLD_CNTL_GRP0, 0x80);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.HOLD_CNTL_GRP1, 0x88);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.HOLD_CNTL_GRP2, 0x90);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.IREF_GRP0, LED_CTRL_IREF); // independant from IREFALL
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.IREF_GRP1, LED_CTRL_IREF);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.IREF_GRP2, LED_CTRL_IREF);
    // Enable gradiation mode for LEDs 0 to 5
//    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_MODE_SEL0, 0x3F);
//    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_MODE_SEL1, 0x00);
    // Assign all LEDs to gradiation groups
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_GRP_SEL0, 0x09);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_GRP_SEL1, 0x12);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_GRP_SEL2, 0x24);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_GRP_SEL3, 0x00);
}

void PCA9955B_gradation_enable(i2c1_address_t addr)
{
    // Enable gradiation mode for LEDs 0 to 5
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_MODE_SEL0, 0x3F);
    // Start group 0
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_CNTL, 0x3F);
}

void PCA9955B_gradation_disable(i2c1_address_t addr)
{
    // Disable gradiation mode for LEDs 0 to 5
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_MODE_SEL0, 0x00);
    // Stop group 0
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.GRAD_CNTL, 0x00);
}


/**
 * Reads one register 
 * @param i2c1_address_t device_addr
 * @param uint8_t reg_addr
 */
void PCA9955B_read(i2c1_address_t device_addr, uint8_t reg_addr)
{
    uint8_t val = 0x00;
    val = I2C1_Read1ByteRegister(device_addr, reg_addr);
    printf("0x%x -> 0x%x\r\n", reg_addr, val);
}

/**
 * Sets the led colors (RGB) of one push button
 * @param led_addr_t led_addr
 * @param uint8_t *p_color, pointer to array with 3 elements
 */
void set_pb_color(led_addr_t led_addr, uint8_t *p_color)
{
    uint8_t idx = 0;
    // Three colors (RGB) per push button.
    // Two LEDs per push button. Same color on addresses x and x+3.
    for (idx = 0; idx < 3; idx++)
    {
        I2C1_Write1ByteRegister(led_addr.I2C_ADDR, PCA9955B_ADDR_MAP.PWM0+led_addr.PB_LED_R0 + idx, p_color[idx]);
        I2C1_Write1ByteRegister(led_addr.I2C_ADDR, PCA9955B_ADDR_MAP.PWM0+led_addr.PB_LED_R0 + idx + 3, p_color[idx]);
    }
}

/**
 * Sets the indicator LEDs for each channel (relay closed)
 * Accounts for the swapped LEDs: CH1 <=> CH2, CH3 <=> CH4
 * @param relay_states
 */
void set_relay_closed_led(uint8_t relay_states)
{
    // For both LEDs
    for (uint8_t ii = 0; ii < PB_GROUPS; ii++)
    {
        // Bit masks due to another PCB layout mistake :-(
        _set_relay_closed_led_helper(CH1_LED, relay_states & 0x02);
        _set_relay_closed_led_helper(CH2_LED, relay_states & 0x01);
        _set_relay_closed_led_helper(CH3_LED, relay_states & 0x08);
        _set_relay_closed_led_helper(CH4_LED, relay_states & 0x04);
        _set_relay_closed_led_helper(CH5_LED, relay_states & 0x10);
        _set_relay_closed_led_helper(CH6_LED, relay_states & 0x20);
    }
    
}

/**
 * Helper function for set_relay_closed_led
 * @param led_addr
 * @param pwm_val
 */
void _set_relay_closed_led_helper(led_addr_t led_addr, uint8_t pwm_val)
{
    
    uint8_t pwm_local = 0x00;
    if (pwm_val > 0)
    {
        pwm_local = PWR_LED_PWM_ON;
    }
    // For both LEDs
    for (uint8_t ii = 0; ii < 2; ii++)
    {
        I2C1_Write1ByteRegister(led_addr.I2C_ADDR, PCA9955B_ADDR_MAP.PWM0+led_addr.PWR_LED0+ii, pwm_local);
    }
}

/**
 * Sets the PB color according to it's group number
 * @param p_pb_groups
 */
void set_pb_color_by_group(uint8_t *p_pb_groups)
{
    // p_pb_groups points to the first value in the COLOR array
    set_pb_color(CH1_LED, COLORS + p_pb_groups[0]*3);
    set_pb_color(CH2_LED, COLORS + p_pb_groups[1]*3);
    set_pb_color(CH3_LED, COLORS + p_pb_groups[2]*3);
    set_pb_color(CH4_LED, COLORS + p_pb_groups[3]*3);
    set_pb_color(CH5_LED, COLORS + p_pb_groups[4]*3);
    set_pb_color(CH6_LED, COLORS + p_pb_groups[5]*3);
}

/**
 * Cycles push button colors
 * Uses global variables CHx_LED and COLORS, uses TMR0
 * @param uint8_t n_cycles, number of cycles
 */
void run_startup_animation(uint8_t n_cycles)
{
    uint8_t cycle_counter = 0;
    uint8_t color_indices[PB_GROUPS] = {0,3,6,9,12,15};
    uint8_t ii = 0;
    uint8_t tmp = 0;
    TMR0_Reload(0x20);
    TMR0_StartTimer();
    // Cycle loop, one cycle = one color shift
    for (cycle_counter = 0; cycle_counter < n_cycles; cycle_counter++)
    {
        while (~TMR0_HasOverflowOccured())
        {
            NOP();
        }
        PIR0bits.TMR0IF = 0;
        set_pb_color(CH1_LED, COLORS+color_indices[0]);
        set_pb_color(CH2_LED, COLORS+color_indices[1]);
        set_pb_color(CH3_LED, COLORS+color_indices[2]);
        set_pb_color(CH4_LED, COLORS+color_indices[3]);
        set_pb_color(CH5_LED, COLORS+color_indices[4]);
        set_pb_color(CH6_LED, COLORS+color_indices[5]);
        // Cyclic shift of color_indeces
        tmp = color_indices[0];
        for (ii = 0; ii< 5; ii++)
        {
            color_indices[ii] = color_indices[ii+1];
        }
        color_indices[5] = tmp;
    }
    TMR0_StopTimer();
    TMR0_Reload(TMR0_INIT_VAL);
    PIR0bits.TMR0IF = 0;
}

void set_color1(i2c1_address_t addr, uint8_t *p_color)
{
    /*
     red = 0,3
     green = 1,4
     blue = 2,5
     */
    uint8_t color_index = 0;
    for (color_index = 0; color_index < 3; color_index++)
    {
        I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM0+color_index, p_color[color_index]);
        I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM0+color_index+3, p_color[color_index]);
    }

}

void set_color2(i2c1_address_t addr, uint8_t *p_color)
{
    /*
     red = 0,3
     green = 1,4
     blue = 2,5
     */
    uint8_t color_index = 0;
    for (color_index = 0; color_index < 3; color_index++)
    {
        I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM0+color_index, p_color[color_index]);
        I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM0+color_index+3, p_color[color_index]);
        I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM6+color_index, p_color[color_index]);
        I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM6+color_index+3, p_color[color_index]);
    }
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM12, 0x0F);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM13, 0x0F);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM14, 0x0F);
    I2C1_Write1ByteRegister(addr, PCA9955B_ADDR_MAP.PWM15, 0x0F);
}

uint8_t get_pb_states(i2c1_address_t addr)
/*
 * Some extra bit manipulations are required, because of swapped PB numbers
 * on the PCB ;-(
 */
{
    uint8_t pb_states_messed_up; // PBs 5 & 6 are swapped (1 based indexing!)
    uint8_t pb_states;
    pb_states_messed_up = I2C1_Read1ByteRegister(addr, PCA9535_CMD.IN_PORT0);
    pb_states = pb_states_messed_up & 0xCF; // Copy states, but without PBs 6 & 7
    // Swapping...
    // Bit:  7 6 5 4 3 2 1 0
    // PB:   - 7 5 6 4 3 2 1
    // Mask: 0 0 1 0 0 0 0 0 (0x20)
    // Mask: 0 0 0 1 0 0 0 0 (0x10)
    // PB 6 on bit pos. 5: left shift
    pb_states |= (pb_states_messed_up & 0x20) >> 1;
    // PB 5 on bit pos. 6: right shift
    pb_states |= (pb_states_messed_up & 0x10) << 1;

    //printf("SW: 0x%x -> 0x%x\r\n", PCA9535_CMD.IN_PORT0, pb_states);
    // relay_status = I2C1_Read1ByteRegister(addr, PCA9535_CMD.IN_PORT1);
    //printf("RLY: 0x%x -> 0x%x\r\n", PCA9535_CMD.IN_PORT1, pb_states);
    return pb_states;

}

uint8_t io_ctrl_cfg(i2c1_address_t addr, uint16_t cfg_reg_pair, uint16_t out_reg_pair)
{
    uint16_t result;
    // Set relay IOs to 0 before configuring them as output
    I2C1_Write2ByteRegister(addr, PCA9535_CMD.OUT_PORT0, out_reg_pair);
    I2C1_Write2ByteRegister(addr, PCA9535_CMD.CFG_PORT0, cfg_reg_pair);
    /* I2C1_Read2ByteRegister returns high byte first!
     * By reading 2 bytes starting at CFG_PORT1 result contains PORT1
     * in the upper byte, PORT0 at the lower byte.
    */
    result = I2C1_Read2ByteRegister(addr, PCA9535_CMD.OUT_PORT1);
    // printf("OUT PORT reg: 0x%x\r\n", result);
    result = I2C1_Read2ByteRegister(addr, PCA9535_CMD.CFG_PORT1);
    // printf("CFG reg: 0x%x\r\n", result);
    
    
    if (result != cfg_reg_pair)
    {
        printf("IO_CTRL_CFG failure!\r\n");
        printf("IO-CFG: Wrote 0x%x, got 0x%x\r\n", cfg_reg_pair, result);
        return 1;
    }
    return 0;
}

void set_relays(i2c1_address_t addr, uint8_t relay_states)
/* 
 * Direct mapping from PB input to relay control:
 * PB pressed = 1 = relay closed
 */
{
    printf("SET RLY: 0x%x\r\n", relay_states | RELAY_MASK);
    // RELAY_MASK sets unused channels to high
    I2C1_Write1ByteRegister(addr, PCA9535_CMD.OUT_PORT1, relay_states | RELAY_MASK);
    set_relay_closed_led(relay_states);
}

uint8_t set_relays_by_group(i2c1_address_t addr, uint8_t relay_states,
        uint8_t pb_rising_edges, uint8_t *p_pb_groups, uint8_t *p_pb_group_masks)
{
    uint8_t grouped_relay_states = 0x00;
    // Going through PB groups and set new relay state by XORing
    for (uint8_t ii = 0; ii < PB_GROUPS; ii++)
    {
        if ((pb_rising_edges & 0x01) == 1)
        {
            // Ugly look-up
            grouped_relay_states |= p_pb_group_masks[p_pb_groups[ii]] ^ relay_states;
        }
        pb_rising_edges >>= 1;
    }
    // printf("FINAL: 0x%x\r\n", grouped_relay_states);
    set_relays(addr, grouped_relay_states);
    return grouped_relay_states;
}

uint8_t pb_edge_detect(uint8_t current_state)
/*
 Push button (PB) rising edge detection.
 uint8_t argument contains 8 PB states (every 1 means PB was pressed).
 uint8_t return value contains the rising edgeds, compared to the former PB states.
 The former PB states are stored in local static variable with the inital value 0.
*/
{
    static uint8_t last_state = 0x00;
    uint8_t rising_edges = 0x00;
    rising_edges = current_state & ~last_state; // Get changes from last state
    last_state = current_state; // Store current state
//    if (rising_edges > 0)
//    {
//        printf("PB: 0x%x\r\n",rising_edges);
//    }
    return rising_edges;
    
}

uint8_t pb7_long_press_detect(uint8_t pb_states, uint8_t rising_edges)
/*
 Returns uint8_t value: 0 - nothing happend
                        1 - Short button press detected
                        2 - Long button press detected
 */
{
    static uint8_t button_pressed = 0;
    //printf("TMR0: 0x%x\r\n", TMR0_ReadTimer());
    // Start counter only at rising edges
    if ((rising_edges & PB_MASK.PB7) == PB_MASK.PB7)
    {
        TMR0_StartTimer();
        button_pressed = 1;
        return(1);
    }
    // Time limit for long press reached
    if (TMR0_HasOverflowOccured())
    {
        TMR0_StopTimer();
        TMR0_WriteTimer(0x00);
        PIR0bits.TMR0IF = 0;
        button_pressed = 0; // To avoid short button press at next function call
        return(2);
    }
    // PB released prematurely
    if ((button_pressed == 1) && (pb_states & PB_MASK.PB7) == 0x00)
    {
        TMR0_StopTimer();
        TMR0_WriteTimer(0x00);
        PIR0bits.TMR0IF = 0;
        button_pressed = 0;
        return(1);
    }
    
    return(0);
}

void pb7_led_flash()
{
    static uint8_t led_flash = 0;
    if (led_flash < 5)
    {
        set_pb_color(CH7_LED, COLOR_WHITE);
    }
    else
    {
        set_pb_color(CH7_LED, COLOR_OFF);
    }
    if (led_flash > 9)
    {
        led_flash = 0;
    }
    led_flash++;
}

void set_pb_groups(uint8_t *p_pb_groups, uint8_t pb_rising_egdges)
{
    // For each PB press, increment the respective push button group by one.
    for (uint8_t ii = 0; ii < PB_GROUPS; ii++)
    {
        if (pb_rising_egdges & 0x01)
        {
            // Increase group, reset to 0 after last group number
            p_pb_groups[ii] = (p_pb_groups[ii] + 1) % PB_GROUPS;
            printf("PB 0x%x: GROUP 0x%x\r\n", ii, p_pb_groups[ii]);
        }
        pb_rising_egdges >>= 1; // Right shift for next PB
    }
}

void calculate_pb_group_masks(uint8_t *p_pb_groups, uint8_t *p_pb_group_mask)
{
    // Loop over pb_group_mask
    for (uint8_t mask_idx = 0; mask_idx < PB_GROUPS; mask_idx++)
    {
        p_pb_group_mask[mask_idx] = 0x00; // Reset before update
        // Loop over pb_groups
        for (uint8_t group_idx = 0; group_idx < PB_GROUPS; group_idx++)
        {
            if (p_pb_groups[group_idx] == mask_idx)
            {
                p_pb_group_mask[mask_idx] |= 1 << PB_GROUPS; // Set MSB
            }
            p_pb_group_mask[mask_idx] >>= 1;
        }
        // printf("GROUP_MASK %x: 0x%x\r\n", mask_idx, p_pb_group_mask[mask_idx]);
    }
}
/**
 End of File
*/