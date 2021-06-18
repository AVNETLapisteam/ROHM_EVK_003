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
#include "kx224_drv.h"
#include "kx224_registers.h"

/* Driver data struct */
struct KX224_drv {
    uint8_t sad;   
};

static struct KX224_drv drv_data;

/* Platform functions passed to driver */
static struct platform_functions *platform_funcs;


/* Set platform functions */
uint8_t KX224_set_platform_functions(struct platform_functions *functions)
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
uint8_t KX224_plat_func_i2c_read(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_read is mandatory */

    if (platform_funcs && platform_funcs->twi_read) {
        rc = platform_funcs->twi_read(sad, reg, data, size);
    }
    return rc;
}


uint8_t KX224_plat_func_i2c_write(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_write is mandatory */

    if (platform_funcs && platform_funcs->twi_write) {
        rc = platform_funcs->twi_write(sad, reg, data, size);
    }

    return rc;
}


uint8_t KX224_plat_func_delay_ms(uint16_t ms)
{
    uint8_t rc = RC_OK; /* delay_ms is optional */

    if (platform_funcs && platform_funcs->delay_ms) {
        rc = platform_funcs->delay_ms(ms);
    }

    return rc;
}


uint8_t KX224_plat_func_debug_print_line(char *str)
{
    uint8_t rc = RC_OK; /* debug_println is optional */

    if (platform_funcs && platform_funcs->debug_println && str != NULL) {
        rc = platform_funcs->debug_println(str);
    }

    return rc;
}


/* KX224 reg read/write and reg bits handling */
uint8_t KX224_reg_read(uint8_t reg, uint8_t *data, uint8_t size)
{
    return KX224_plat_func_i2c_read(drv_data.sad, reg, data, size);
}


uint8_t KX224_reg_write(uint8_t reg, uint8_t *data, uint8_t size)
{
    return KX224_plat_func_i2c_write(drv_data.sad, reg, data, size);
}


uint8_t KX224_reg_set_bit_pattern(uint8_t reg, uint8_t bit_pattern, uint8_t mask)
{
    uint8_t rc;
    uint8_t reg_val;

    rc = KX224_reg_read(reg, &reg_val, sizeof(reg_val));
    if (rc != RC_OK) {
        return rc;
    }

    reg_val &= ~mask;
    reg_val |= bit_pattern;

    rc = KX224_reg_write(reg, &reg_val, sizeof(reg_val));
    if (rc != RC_OK) {
        return rc;
    }
    return RC_OK;
}


uint8_t KX224_reg_set_bit(uint8_t reg, uint8_t bits)
{
    return KX224_reg_set_bit_pattern(reg, bits, 0x00);
}


uint8_t KX224_reg_reset_bit(uint8_t reg, uint8_t bits)
{
    return KX224_reg_set_bit_pattern(reg, 0x00, bits);
}


/* Debug prints */
uint8_t KX224_debug_print_line(char *str)
{
    return KX224_plat_func_debug_print_line(str);
}


void KX224_debug_print_reg_and_val(uint8_t reg, uint8_t val)
{
    char buffer[40];
    sprintf(buffer, "reg 0x%x val 0x%x",reg,val);
    KX224_debug_print_line(buffer);
}


void KX224_debug_dump_reg(uint8_t reg)
{
    uint8_t rc;
    uint8_t reg_val;

    rc = KX224_reg_read(reg, &reg_val, sizeof(reg_val));
    if (rc != RC_OK) {
        return;
    }

    KX224_debug_print_reg_and_val(reg, reg_val);
}


/* KX224 delay */
void KX224_delay_ms(uint16_t ms)
{
    KX224_plat_func_delay_ms(ms);
}


/* KX224 drv_data code  */
uint8_t KX224_set_CNTL1_PC1_on()
{
    uint8_t rc;
    rc = KX224_reg_set_bit(KX224_CNTL1, KX224_CNTL1_PC1);
    return rc;
}


uint8_t KX224_set_CNTL1_PC1_off()
{
    uint8_t rc;
    rc = KX224_reg_reset_bit(KX224_CNTL1, KX224_CNTL1_PC1);
    // wait at least 0.1 seconds
    KX224_delay_ms(100);
    return rc;
}


uint8_t KX224_set_odr(uint8_t ord)
{
    uint8_t rc;
    rc = KX224_reg_set_bit_pattern(KX224_ODCNTL, ord, KX224_ODCNTL_OSA_MASK);
    return rc;
}


uint8_t KX224_set_grange(uint8_t grange)
{
    uint8_t rc;

    rc = KX224_reg_set_bit_pattern(KX224_CNTL1, grange, KX224_CNTL1_GSEL_MASK);
    return rc;
}


uint8_t KX224_set_high_resolution_mode()
{    
    uint8_t rc;
    /*enable high resolution mode.*/
    rc = KX224_reg_set_bit(KX224_CNTL1, KX224_CNTL1_RES);
    return rc;
}


uint8_t KX224_sensor_soft_reset()
{
    uint8_t rc;

    rc = KX224_reg_set_bit(KX224_CNTL2, KX224_CNTL2_SRST);
    if (rc != RC_OK) {
      return rc;
    }

    KX224_delay_ms(500);

    return RC_OK;
}


uint8_t KX224_int1_configure(gpio_pin_polarity_t polarity) {

    uint8_t rc;
    rc = KX224_reg_set_bit(KX224_CNTL1, KX224_CNTL1_DRDYE);
    if (rc != RC_OK) {
        return rc;
    }

    rc = KX224_reg_set_bit(KX224_INC4, KX224_INC4_DRDYI1);
    if (rc != RC_OK) {
        return rc;
    }

    rc = KX224_reg_reset_bit(KX224_CNTL1, KX224_INC1_IEL1);
    if (rc != RC_OK) {
        return rc;
    }
  
    if(polarity == GPIO_POLARITY_HITOLO)
        rc = KX224_reg_reset_bit(KX224_INC1, KX224_INC1_IEA1);
    else 
        rc = KX224_reg_set_bit(KX224_INC1, KX224_INC1_IEA1);

    if (rc != RC_OK) {
        return rc;
    }

    rc = KX224_reg_set_bit(KX224_INC1, KX224_INC1_IEN1);
    if (rc != RC_OK) {
        return rc;
    }
    
    return RC_OK;
}


uint8_t KX224_start(uint8_t int_pin, gpio_pin_polarity_t polarity) {

    uint8_t rc;
    uint8_t grange = KX224_CNTL1_GSEL_16G;
    uint8_t odr = KX224_ODCNTL_OSA_50;

    /* Sensor must be in standby mode before configs are changed.*/
    rc = KX224_set_CNTL1_PC1_off();
    if (rc != RC_OK) {
        return rc;
    }

    /* set sensor odr.*/
    rc = KX224_set_odr(odr);
    if (rc != RC_OK) {
        return rc;
    }

    /* set sensor grange.*/
    rc = KX224_set_grange(grange);
    if (rc != RC_OK) {
        return rc;
    }

    /* set high resolution mode.*/
    rc = KX224_set_high_resolution_mode();
    if (rc != RC_OK) {
        return rc;
    }

    /*enable interrupt 1.*/
    if(int_pin == KX224_INT1) {
        rc = KX224_int1_configure(polarity);
        if (rc != RC_OK) {
          return rc;
        }
    }

    /*enable high resolution power mode.*/
    rc = KX224_set_CNTL1_PC1_on();
    if (rc != RC_OK) {
        return rc;
    }

    return RC_OK;
}


uint8_t KX224_init(uint8_t sad)
{
    uint8_t rc;
    uint8_t reg[1];

    /*platform_funcs needs to be set, or overwrite local read/write/print functions.*/
    if (platform_funcs == NULL) {
        return RC_FAIL;
    }

    drv_data.sad = sad;

    rc = KX224_reg_read(KX224_WHO_AM_I, reg, sizeof(reg));
    if (rc != RC_OK) {
        KX224_debug_print_line("KX224_reg_read WHO_AM_I fail");
        return rc;
    }

    /*verify that we got correct wai value.*/
    if (*reg != KX224_WHO_AM_I_WAI_ID) {
        KX224_debug_print_line("Incorrect WAI");
        return RC_FAIL;
    }

    /*do sensor soft reset.*/
    rc = KX224_sensor_soft_reset();
    if (rc != RC_OK) {
        KX224_debug_print_line("KX224_sensor_soft_reset fail");
        return rc;
    }

    return RC_OK;
}


uint8_t KX224_get_raw_xyz(uint8_t *xyz, uint8_t *size)
{
    uint8_t rc;

    rc = KX224_reg_read(KX224_XOUT_L, xyz, 6);
    if (rc != RC_OK) {
        KX224_debug_print_line("KX224_get_rawval fail");
        return rc;
    }
    *size = 6;
    return RC_OK;
}


uint8_t KX224_debug_dump_regs()
{

    uint8_t dump_regs[] = {
        KX224_STATUS_REG,
        KX224_CNTL2,
        KX224_ODCNTL,
    };

      uint8_t i = 0;
      uint8_t n = sizeof(dump_regs)/sizeof(dump_regs[0]);

      KX224_debug_print_line("KX224_debug_dump_regs :");
      for(i=0; i < n ; i++) {
          KX224_debug_dump_reg(dump_regs[i]);
      }

    return RC_OK;
}
