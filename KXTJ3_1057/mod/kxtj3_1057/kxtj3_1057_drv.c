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
#include "kxtj3_1057_drv.h"
#include "kxtj3_1057_registers.h"

/* Driver data struct */
struct KXTJ3_1057_drv {
    uint8_t sad;   
};

static struct KXTJ3_1057_drv drv_data;
uint32_t GDivide;

/* Platform functions passed to driver */
static struct platform_functions *platform_funcs;


/* Set platform functions */
uint8_t KXTJ3_1057_set_platform_functions(struct platform_functions *functions)
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
uint8_t KXTJ3_1057_plat_func_i2c_read(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_read is mandatory */

    if (platform_funcs && platform_funcs->twi_read) {
        rc = platform_funcs->twi_read(sad, reg, data, size);
    }
    return rc;
}


uint8_t KXTJ3_1057_plat_func_i2c_write(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_write is mandatory */

    if (platform_funcs && platform_funcs->twi_write) {
        rc = platform_funcs->twi_write(sad, reg, data, size);
    }

    return rc;
}


uint8_t KXTJ3_1057_plat_func_delay_ms(uint16_t ms)
{
    uint8_t rc = RC_OK; /* delay_ms is optional */

    if (platform_funcs && platform_funcs->delay_ms) {
        rc = platform_funcs->delay_ms(ms);
    }

    return rc;
}


uint8_t KXTJ3_1057_plat_func_debug_print_line(char *str)
{
    uint8_t rc = RC_OK; /* debug_println is optional */

    if (platform_funcs && platform_funcs->debug_println && str != NULL) {
        rc = platform_funcs->debug_println(str);
    }

    return rc;
}


/* KXTJ3_1057 reg read/write and reg bits handling */
uint8_t KXTJ3_1057_reg_read(uint8_t reg, uint8_t *data, uint8_t size)
{
    return KXTJ3_1057_plat_func_i2c_read(drv_data.sad, reg, data, size);
}


uint8_t KXTJ3_1057_reg_write(uint8_t reg, uint8_t *data, uint8_t size)
{
    return KXTJ3_1057_plat_func_i2c_write(drv_data.sad, reg, data, size);
}


uint8_t KXTJ3_1057_reg_set_bit_pattern(uint8_t reg, uint8_t bit_pattern, uint8_t mask)
{
    uint8_t rc;
    uint8_t reg_val;

    rc = KXTJ3_1057_reg_read(reg, &reg_val, sizeof(reg_val));
    if (rc != RC_OK) {
        return rc;
    }

    reg_val &= ~mask;
    reg_val |= bit_pattern;

    rc = KXTJ3_1057_reg_write(reg, &reg_val, sizeof(reg_val));
    if (rc != RC_OK) {
        return rc;
    }
    return RC_OK;
}


uint8_t KXTJ3_1057_reg_set_bit(uint8_t reg, uint8_t bits)
{
    return KXTJ3_1057_reg_set_bit_pattern(reg, bits, 0x00);
}


uint8_t KXTJ3_1057_reg_reset_bit(uint8_t reg, uint8_t bits)
{
    return KXTJ3_1057_reg_set_bit_pattern(reg, 0x00, bits);
}


/* Debug prints */
uint8_t KXTJ3_1057_debug_print_line(char *str)
{
    return KXTJ3_1057_plat_func_debug_print_line(str);
}


void KXTJ3_1057_debug_print_reg_and_val(uint8_t reg, uint8_t val)
{
    char buffer[40];
    sprintf(buffer, "reg 0x%x val 0x%x",reg,val);
    KXTJ3_1057_debug_print_line(buffer);
}


/* KXTJ3_1057 delay */
void KXTJ3_1057_delay_ms(uint16_t ms)
{
    KXTJ3_1057_plat_func_delay_ms(ms);
}


/* KXTJ3_1057 start */
uint8_t KXTJ3_1057_start_func( void )
{
    uint8_t reg[1];
    uint8_t dump;
    uint8_t rc;

    rc = KXTJ3_1057_reg_read(KXTJ3_1057_CTRL_REG1, reg, sizeof(reg));
    if (rc != RC_OK) {
        KXTJ3_1057_debug_print_line("start CTRL REG1 read fail");
        return rc;
    }

    dump = *reg;

    *reg = dump | KXTJ3_1057_CNTL1_PC1;
    rc = KXTJ3_1057_reg_write(KXTJ3_1057_CTRL_REG1, reg, sizeof(reg));
    if (rc != RC_OK) {
        KXTJ3_1057_debug_print_line("start KXTJ3_1057_CTRL_REG1 setting fail");
        return rc;
    }

    return RC_OK;
}

/* KXTJ3_1057 stop */
uint8_t KXTJ3_1057_stop_func( void )
{
    uint8_t reg[1];
    uint8_t dump;
    uint8_t rc;

    rc = KXTJ3_1057_reg_read(KXTJ3_1057_CTRL_REG1, reg, sizeof(reg));
    if (rc != RC_OK) {
        KXTJ3_1057_debug_print_line("stop CTRL REG1 read fail");
        return rc;
    }

    dump = *reg;

    *reg = dump & ~KXTJ3_1057_CNTL1_PC1;
    rc = KXTJ3_1057_reg_write(KXTJ3_1057_CTRL_REG1, reg, sizeof(reg));
    if (rc != RC_OK) {
        KXTJ3_1057_debug_print_line("stop KXTJ3_1057_CTRL_REG1 setting fail");
        return rc;
    }

    return RC_OK;
}


uint8_t KXTJ3_1057_int1_configure(gpio_pin_polarity_t polarity) {

    uint8_t rc;
    rc = KXTJ3_1057_reg_set_bit(KXTJ3_1057_CTRL_REG1, KXTJ3_1057_CNTL1_DRDYE);
    if (rc != RC_OK) {
        return rc;
    }

    rc = KXTJ3_1057_reg_reset_bit(KXTJ3_1057_INT_CTRL_REG1, KXTJ3_1057_INC1_IEL1);
    if (rc != RC_OK) {
        return rc;
    }
  
    if(polarity == GPIO_POLARITY_HITOLO)
        rc = KXTJ3_1057_reg_reset_bit(KXTJ3_1057_INT_CTRL_REG1, KXTJ3_1057_INC1_IEA1);
    else 
        rc = KXTJ3_1057_reg_set_bit(KXTJ3_1057_INT_CTRL_REG1, KXTJ3_1057_INC1_IEA1);

    if (rc != RC_OK) {
        return rc;
    }

    rc = KXTJ3_1057_reg_set_bit(KXTJ3_1057_INT_CTRL_REG1, KXTJ3_1057_INC1_IEN1);
    if (rc != RC_OK) {
        return rc;
    }
    
    return RC_OK;
}


uint8_t KXTJ3_1057_init(uint8_t sad)
{
    uint8_t rc;
    uint8_t reg[1];

    /*platform_funcs needs to be set, or overwrite local read/write/print functions.*/
    if (platform_funcs == NULL) {
        return RC_FAIL;
    }

    drv_data.sad = sad;

    rc = KXTJ3_1057_reg_read(KXTJ3_1057_WHO_AM_I, reg, sizeof(reg));
    if (rc != RC_OK) {
        KXTJ3_1057_debug_print_line("KXTJ3_1057_reg_read WHO_AM_I fail");
        return rc;
    }

    /*verify that we got correct wai value.*/
    if (*reg != KXTJ3_1057_WAI_VAL) {
        KXTJ3_1057_debug_print_line("Incorrect WAI");
        return RC_FAIL;
    }

    *reg = KXTJ3_1057_CNTL1_VAL;
    rc = KXTJ3_1057_reg_write(KXTJ3_1057_CTRL_REG1, reg, sizeof(reg));
    if (rc != RC_OK) {
        KXTJ3_1057_debug_print_line("KXTJ3_1057_CTRL_REG1 setting fail");
        return rc;
    }

    *reg = KXTJ3_1057_DATA_CNTL_VAL;
    rc = KXTJ3_1057_reg_write(KXTJ3_1057_DATA_CTRL_REG, reg, sizeof(reg));
    if (rc != RC_OK) {
        KXTJ3_1057_debug_print_line("KXTJ3_1057_DATA_CTRL_REG setting fail");
        return rc;
    }
// setting gval
    rc = KXTJ3_1057_reg_read(KXTJ3_1057_DATA_CTRL_REG, reg, sizeof(reg));
    if (rc != RC_OK) {
        KXTJ3_1057_debug_print_line("KXTJ3_1057_reg_read WHO_AM_I fail");
        return rc;
    }

    uint8_t res, gsel, en16g;
    int32_t gsense;

    res = (*reg) & KXTJ3_1057_CNTL1_RES;
    gsel = (*reg) & KXTJ3_1057_CNTL1_GSELMASK;
    en16g = (*reg) & KXTJ3_1057_CNTL1_EN16GMASK;

    if ((res == KXTJ3_1057_CNTL1_LOWPOWER) && (gsel == KXTJ3_1057_CNTL1_GSEL_14BIT)) {
        KXTJ3_1057_debug_print_line("parameter error");
        return rc;
    }
    else {
      if (en16g == KXTJ3_1057_CNTL1_GSEL_16G) {
          gsense = KXTJ3_1057_GSENS_16G;
      } else {
          switch (gsel) {
              case KXTJ3_1057_CNTL1_GSEL_2G:
                  gsense = KXTJ3_1057_GSENS_2G;
                  break;
              case KXTJ3_1057_CNTL1_GSEL_4G:
                  gsense = KXTJ3_1057_GSENS_4G;
                  break;
              default:
                  gsense = KXTJ3_1057_GSENS_8G;
                  break;
          }
      }
      GDivide = ((int32_t)1 << KXTJ3_1057_DIVIDE_SHIFT) / gsense;
    }

    /*enable interrupt 1.*/

    rc = KXTJ3_1057_int1_configure(GPIO_POLARITY_HITOLO);
    if (rc != RC_OK) {
      return rc;
    }

    return RC_OK;
}


uint8_t KXTJ3_1057_get_raw_xyz(uint8_t *xyz, uint8_t *size)
{
    uint8_t rc;

    rc = KXTJ3_1057_reg_read(KXTJ3_1057_XOUT_L, xyz, 6);
    if (rc != RC_OK) {
        KXTJ3_1057_debug_print_line("KXTJ3_1057_get_rawval fail");
        return rc;
    }
    *size = 6;
    return RC_OK;
}
