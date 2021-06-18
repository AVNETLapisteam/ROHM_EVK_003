/*
The MIT License (MIT)
Copyright (c) 2020 Rohm Semiconductor

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <stdio.h> 
#include <stddef.h>
#include <stdint.h>

#include "sensors.h"
#include "BH1749NUC_drv.h"
#include "BH1749NUC_registers.h"


/* Driver data struct */
struct BH1749NUC_drv {
    uint8_t sad;
};

static struct BH1749NUC_drv drv_data;

/* Platform functions passed to driver */
static struct platform_functions *platform_funcs;


/* Set plafrom functions */
uint8_t BH1749NUC_set_platform_functions(struct platform_functions *functions)
{
    if (functions == NULL ||
          functions->twi_read == NULL ||
            functions->twi_write == NULL ) {
        return RC_FAIL;
    }
    platform_funcs = functions;
    return RC_OK;
}


/* Wrapper to platform functions */
uint8_t BH1749NUC_plat_func_i2c_read(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_read is mandatory */

    if (platform_funcs && platform_funcs->twi_read) {
        rc = platform_funcs->twi_read(sad, reg, data, size);
    }
    return rc;
}


uint8_t BH1749NUC_plat_func_i2c_write(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_write is mandatory */

    if (platform_funcs && platform_funcs->twi_write) {
        rc = platform_funcs->twi_write(sad, reg, data, size);
    }

    return rc;
}


uint8_t BH1749NUC_plat_func_delay_ms(uint16_t ms)
{
    uint8_t rc = RC_OK; /* delay_ms is optional */

    if (platform_funcs && platform_funcs->delay_ms) {
        rc = platform_funcs->delay_ms(ms);
    }

    return rc;
}


uint8_t BH1749NUC_plat_func_debug_print_line(char *str)
{
    uint8_t rc = RC_OK; /* debug_println is optional */

    if (platform_funcs && platform_funcs->debug_println && str != NULL) {
        rc = platform_funcs->debug_println(str);
    }

    return rc;
}


/* BH1749NUC reg read/write and reg bits handling */
uint8_t BH1749NUC_reg_read(uint8_t reg, uint8_t *data, uint8_t size)
{
    return BH1749NUC_plat_func_i2c_read(drv_data.sad, reg, data, size);
}


uint8_t BH1749NUC_reg_write(uint8_t reg, uint8_t *data, uint8_t size)
{
    return BH1749NUC_plat_func_i2c_write(drv_data.sad, reg, data, size);
}


uint8_t BH1749NUC_reg_set_bit_pattern(uint8_t reg, uint8_t bit_pattern, uint8_t mask)
{
    uint8_t rc;
    uint8_t reg_val;
    uint8_t confirm_val;

    rc = BH1749NUC_reg_read(reg, &reg_val, sizeof(reg_val));
    if (rc != RC_OK) {
        return rc;
    }

    reg_val &= ~mask;
    reg_val |= bit_pattern;
    confirm_val = reg_val;

    rc = BH1749NUC_reg_write(reg, &reg_val, sizeof(reg_val));
    if (rc != RC_OK) {
        return rc;
    }

    rc = BH1749NUC_reg_read(reg, &reg_val, sizeof(reg_val));
    if (rc != RC_OK) {
        return rc;
    }
    if( confirm_val != reg_val )
    {
        return RC_FAIL;
    }

    return RC_OK;
}


uint8_t BH1749NUC_reg_set_bit(uint8_t reg, uint8_t bits)
{
    return BH1749NUC_reg_set_bit_pattern(reg, bits, 0x00);
}


uint8_t BH1749NUC_reg_reset_bit(uint8_t reg, uint8_t bits)
{
    return BH1749NUC_reg_set_bit_pattern(reg, 0x00, bits);
}


/* Debug prints */
uint8_t BH1749NUC_debug_print_line(char *str)
{
    return BH1749NUC_plat_func_debug_print_line(str);
}


void BH1749NUC_debug_print_reg_and_val(uint8_t reg, uint8_t val)
{
    char buffer[40];
    sprintf(buffer, "reg 0x%x val 0x%x",reg,val);
    BH1749NUC_debug_print_line(buffer);
}


void BH1749NUC_debug_dump_reg(uint8_t reg)
{
    uint8_t rc;
    uint8_t reg_val;

    rc = BH1749NUC_reg_read(reg, &reg_val, sizeof(reg_val));
    if (rc != RC_OK) {
        return;
    }

    BH1749NUC_debug_print_reg_and_val(reg, reg_val);
}


/* BH1749NUC delay */
void BH1749NUC_delay_ms(uint16_t ms)
{
    BH1749NUC_plat_func_delay_ms(ms);
}

/* BH1749NUC drv_data code  */
uint8_t BH1749NUC_RGB_enable()
{
    uint8_t rc;
    uint8_t reg;

    BH1749NUC_delay_ms(100);

    reg = BH1749NUC_MODE_CTL2_REG_RGB_EN;
    rc = BH1749NUC_reg_write(BH1749NUC_MODE_CONTROL2_REG, &reg, sizeof(reg));
    if (rc != RC_OK) {
        return rc;
    }
    
/*    BH1749NUC_delay_ms(2);

    reg = BH1749NUC_SYSTEM_REG_SW_RESET|BH1749NUC_SYSTEM_REG_INT_RESET;
    rc = BH1749NUC_reg_write(BH1749NUC_SYSTEM_CONTROL_REG, &reg, sizeof(reg));
    if (rc != RC_OK) {
        return rc;
    }*/

    return RC_OK;
}

uint8_t BH1749NUC_mode_set()
{
    uint8_t rc;
    
    rc = BH1749NUC_reg_set_bit_pattern(BH1749NUC_MODE_CONTROL1_REG,
                                        BH1749NUC_MODE_CTL1_REG_IR_GAIN_1|BH1749NUC_MODE_CTL1_REG_RGB_GAIN_1|BH1749NUC_MODE_CTL1_REG_MEASURE_240,
                                        BH1749NUC_MODE_CONTROL1_MASK);
    if (rc != RC_OK) {
        return rc;
    }
        
    return RC_OK;
}


uint8_t BH1749NUC_start_continuous_measurement()
{
    uint8_t rc;
    
    rc = BH1749NUC_reg_set_bit_pattern(BH1749NUC_MODE_CONTROL2_REG,
                                        BH1749NUC_MODE_CTL2_REG_RGB_EN,
                                        BH1749NUC_MODE_CONTROL2_MASK);
    if (rc != RC_OK) {
        return rc;
    }

    return RC_OK;
}

/*Only one interrupt pin supported with active low polarity.*/
uint8_t BH1749NUC_int1_configure(void) {

    uint8_t rc;

    rc = BH1749NUC_reg_set_bit_pattern(BH1749NUC_INTERRUPT_REG,
                                        BH1749NUC_INTERRUPT_INTERVAL_CHANNEL_RED|BH1749NUC_INTERRUPT_INT_PIN_ENABLE,
                                        BH1749NUC_INTERRUPT_MASK);
    if (rc != RC_OK) {
        return rc;
    }

    rc = BH1749NUC_reg_set_bit_pattern(BH1749NUC_PERSISTENCE_REG,
                                        BH1749NUC_INT_PERSISTENCE_MEASURE_END_ACTIVE,
                                        BH1749NUC_PERSISTENCE_MASK);
    if (rc != RC_OK) {
        return rc;
    }

    return RC_OK;
}

uint8_t BH1749NUC_threshold_configure( ) {

    uint8_t rc;

    rc = BH1749NUC_reg_set_bit_pattern(BH1749NUC_TH_HIGH_LSB,
                                        BH1749NUC_THRESHOLD_HIGH_LSB_DEFAULT,
                                        BH1749NUC_TH_HIGH_MASK);
    if (rc != RC_OK) {
        return rc;
    }

    rc = BH1749NUC_reg_set_bit_pattern(BH1749NUC_TH_HIGH_MSB,
                                        BH1749NUC_THRESHOLD_HIGH_MSB_DEFAULT,
                                        BH1749NUC_TH_HIGH_MASK);
    if (rc != RC_OK) {
        return rc;
    }

    rc = BH1749NUC_reg_set_bit_pattern(BH1749NUC_TH_LOW_LSB,
                                        BH1749NUC_THRESHOLD_LOW_LSB_DEFAULT,
                                        BH1749NUC_TH_LOW_MASK);
    if (rc != RC_OK) {
        return rc;
    }

    rc = BH1749NUC_reg_set_bit_pattern(BH1749NUC_TH_LOW_MSB,
                                        BH1749NUC_THRESHOLD_LOW_MSB_DEFAULT,
                                        BH1749NUC_TH_LOW_MASK);
    if (rc != RC_OK) {
        return rc;
    }

    return RC_OK;
}

uint8_t BH1749NUC_start_set( uint8_t int_pin ) 
{
    uint8_t rc;

    /* set sensor odr.*/
    rc = BH1749NUC_mode_set();
    if (rc != RC_OK) {
        return rc;
    }

    /*enable interrupt 1.*/
    /*if(int_pin == BH1749NUC_INT1) {
        rc = BH1749NUC_threshold_configure();
        if (rc != RC_OK) {
          return rc;
        }
        
        rc = BH1749NUC_int1_configure();
        if (rc != RC_OK) {
          return rc;
        }
    }*/

    rc = BH1749NUC_RGB_enable();
    if (rc != RC_OK) {
        return rc;
    }

    /*start measurement*/
    rc = BH1749NUC_start_continuous_measurement();
    if (rc != RC_OK) {
          return rc;
    }

    return RC_OK;
}


uint8_t BH1749NUC_init(uint8_t sad)
{
    uint8_t rc;
    uint8_t reg;

    /*platform_funcs needs to be set, or overwrite local read/write/print functions.*/
    if (platform_funcs == NULL) {
        return RC_FAIL;
    }

    drv_data.sad = sad;
    rc = BH1749NUC_reg_read(BH1749NUC_SYSTEM_CONTROL_REG, &reg, sizeof(reg));
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("Read BH1749NUC_ID_REG fail");
        return rc;
    }
  
    if((reg & BH1749NUC_SYSTEM_REG_PART_ID_MASK) != BH1749NUC_SYSTEM_REG_PART_ID) {
        BH1749NUC_debug_print_line("BH1749NUC PART ID fail");
        return rc;
    }

    rc = BH1749NUC_reg_read(BH1749NUC_MANUFACTURER_ID_REG, &reg, sizeof(reg));
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_MANUFACTURER_ID_REG fail");
        return rc;
    }
  
    if(reg != BH1749NUC_MANUFACTURER_ID) {
        BH1749NUC_debug_print_line("BH1749NUC MANUFACTURER ID fail");
        return rc;
    }
    
    return RC_OK;
}


uint8_t BH1749NUC_get_raw(uint8_t *data, uint8_t *size)
{
    uint8_t rc;

    rc = BH1749NUC_reg_read(BH1749NUC_RED_DATA_OUT_LSB, &data[0], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }
    rc = BH1749NUC_reg_read(BH1749NUC_RED_DATA_OUT_MSB, &data[1], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }
    rc = BH1749NUC_reg_read(BH1749NUC_GREEN_DATA_OUT_LSB, &data[2], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }
    rc = BH1749NUC_reg_read(BH1749NUC_GREEN_DATA_OUT_MSB, &data[3], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }
    rc = BH1749NUC_reg_read(BH1749NUC_BLUE_DATA_OUT_LSB, &data[4], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }
    rc = BH1749NUC_reg_read(BH1749NUC_BLUE_DATA_OUT_MSB, &data[5], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }


    rc = BH1749NUC_reg_read(BH1749NUC_IR_DATA_OUT_LSB, &data[6], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }
    rc = BH1749NUC_reg_read(BH1749NUC_IR_DATA_OUT_MSB, &data[7], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }
    rc = BH1749NUC_reg_read(BH1749NUC_GREEN2_DATA_OUT_LSB, &data[8], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }
    rc = BH1749NUC_reg_read(BH1749NUC_GREEN2_DATA_OUT_MSB, &data[9], 1);
    if (rc != RC_OK) {
        BH1749NUC_debug_print_line("BH1749NUC_get_rawval fail");
        return rc;
    }

    *size = 10;
    return RC_OK;
}


uint8_t BH1749NUC_debug_dump_regs()
{
/*
    uint8_t dump_regs[] = {
        BH1749NUC_STATUS_REG,
    };

    uint8_t i = 0;
    uint8_t n = sizeof(dump_regs)/sizeof(dump_regs[0]);

    BH1749NUC_debug_print_line("BH1749NUC_debug_dump_regs :");
    for(i=0; i < n ; i++) {
      BH1749NUC_debug_dump_reg(dump_regs[i]);
    }
*/
    return RC_OK;
}
