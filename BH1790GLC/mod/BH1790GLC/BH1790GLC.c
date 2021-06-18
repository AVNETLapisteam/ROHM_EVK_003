/*****************************************************************************
  BH1790GLC.cpp

 Copyright (c) 2016 ROHM Co.,Ltd.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/

#include <stdio.h> 
#include <stddef.h>
#include <stdint.h>

#include "sensors.h"
#include "BH1790GLC.h"

/* Driver data struct */
struct BH1790GLC_drv {
    uint8_t sad;
};

static struct BH1790GLC_drv drv_data;

/* Platform functions passed to driver */
static struct platform_functions *platform_funcs;

uint8_t BH1790GLC_plat_func_delay_ms(uint16_t ms)
{
    uint8_t rc = RC_OK; /* delay_ms is optional */

    if (platform_funcs && platform_funcs->delay_ms) {
        rc = platform_funcs->delay_ms(ms);
    }

    return rc;
}

/* BH1790GLC delay */
void BH1790GLC_delay_ms(uint16_t ms)
{
    BH1790GLC_plat_func_delay_ms(ms);
}

/* Set platfrom functions */
uint8_t BH1790GLC_set_platform_functions(struct platform_functions *functions)
{
    if (functions == NULL ||
            functions->twi_read == NULL ||
            functions->twi_write == NULL) {
        return RC_FAIL;
    }
    platform_funcs = functions;
    return RC_OK;
}

uint8_t BH1790GLC_set_plat_func_debug_print_line(char *str)
{
    uint8_t rc = RC_OK; /* debug_println is optional */

    if (platform_funcs && platform_funcs->debug_println && str != NULL) {
        rc = platform_funcs->debug_println(str);
    }

    return rc;
}

uint8_t BH1790GLC_plat_func_debug_print_line(char *str)
{
    uint8_t rc = RC_OK; /* debug_println is optional */

    if (platform_funcs && platform_funcs->debug_println && str != NULL) {
        rc = platform_funcs->debug_println(str);
    }

    return rc;
}

/* Debug prints */
uint8_t BH1790GLC_set_debug_print_line(char *str)
{
    return BH1790GLC_set_plat_func_debug_print_line(str);
}

/* Debug prints */
uint8_t BH1790GLC_debug_print_line(char *str)
{
    return BH1790GLC_plat_func_debug_print_line(str);
}

/* Wrapper to platform functions */
uint8_t BH1790GLC_plat_func_i2c_read(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_read is mandatory */

    if (platform_funcs && platform_funcs->twi_read) {
        rc = platform_funcs->twi_read(sad, reg, data, size);
    }
    return rc;
}

/* BH1790GLC reg read/write and reg bits handling */
uint8_t BH1790GLC_reg_read(uint8_t reg, uint8_t *data, uint8_t size)
{
    return BH1790GLC_plat_func_i2c_read(drv_data.sad, reg, data, size);
}

uint8_t BH1790GLC_get_raw(uint8_t *data, uint8_t *size)
{
    uint8_t rc;

    rc = BH1790GLC_reg_read(BH1790GLC_DATAOUT_LEDOFF_L, &data[0], 1);
    if (rc != RC_OK) {
        BH1790GLC_debug_print_line("BH1790GLC_get_rawval fail");
        return rc;
    }
    rc = BH1790GLC_reg_read(BH1790GLC_DATAOUT_LEDOFF_U, &data[1], 1);
    if (rc != RC_OK) {
        BH1790GLC_debug_print_line("BH1790GLC_get_rawval fail");
        return rc;
    }
    rc = BH1790GLC_reg_read(BH1790GLC_DATAOUT_LEDON_L, &data[2], 1);
    if (rc != RC_OK) {
        BH1790GLC_debug_print_line("BH1790GLC_get_rawval fail");
        return rc;
    }
    rc = BH1790GLC_reg_read(BH1790GLC_DATAOUT_LEDON_U, &data[3], 1);
    if (rc != RC_OK) {
        BH1790GLC_debug_print_line("BH1790GLC_get_rawval fail");
        return rc;
    }
    *size = 4;
    return RC_OK;
}

uint8_t BH1790GLC_plat_func_i2c_write(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_write is mandatory */

    if (platform_funcs && platform_funcs->twi_write) {
        rc = platform_funcs->twi_write(sad, reg, data, size);
    }

    return rc;
}


uint8_t BH1790GLC_reg_write(uint8_t reg, uint8_t *data, uint8_t size)
{
    return BH1790GLC_plat_func_i2c_write(drv_data.sad, reg, data, size);
}

uint8_t BH1790GLC_init(uint8_t sad)
{
    uint8_t rc;
    uint8_t reg[1];

    /*platform_funcs needs to be set, or overwrite local read/write/print functions.*/
    if (platform_funcs == NULL) {
        return RC_FAIL;
    }

    drv_data.sad = sad;

    BH1790GLC_delay_ms(2);

    rc = BH1790GLC_reg_read(BH1790GLC_PART_ID, reg, sizeof(reg));
    if (rc != RC_OK )
    {
        if(reg[0] != BH1790GLC_PID_VAL)
        {
            BH1790GLC_debug_print_line("BH1790GLC operation fail");
            return RC_FAIL;
        }
        else
        {
            BH1790GLC_debug_print_line("BH1790GLC PART ID fail");
            return RC_FAIL;
        }
    }
    
    rc = BH1790GLC_reg_read(BH1790GLC_MANUFACTURER_ID, reg, sizeof(reg));
    if (rc == RC_OK )
    {
        if(reg[0] != BH1790GLC_MID_VAL)
        {
            BH1790GLC_debug_print_line("BH1790GLC MANUFACTURER ID fail");
            return RC_FAIL;
        }
    }
    else
    {
        return rc;
    }

    return RC_OK;
}

uint8_t BH1790GLC_control1_setting( void )
{
    uint8_t rc = RC_FAIL;
    uint8_t reg = BH1790GLC_MEAS_CONTROL1_VAL;
    rc = BH1790GLC_reg_write(BH1790GLC_MEAS_CONTROL1,&reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
      return rc;
    }

    return RC_OK;
}

uint8_t BH1790GLC_control2_setting( void )
{
    uint8_t rc = RC_FAIL;
    uint8_t reg = BH1790GLC_MEAS_CONTROL2_VAL;
    rc = BH1790GLC_reg_write(BH1790GLC_MEAS_CONTROL2,&reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
      return rc;
    }

    return RC_OK;
}

uint8_t BH1790GLC_measurement_start( void )
{
    uint8_t rc = RC_FAIL;
    uint8_t reg = BH1790GLC_MEAS_START_VAL;
    rc = BH1790GLC_reg_write(BH1790GLC_MEAS_START,&reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
      return rc;
    }

    return RC_OK;
}













