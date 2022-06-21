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
#include "BH1792GLC.h"
#include "platform_functions.h"

/* Driver data struct */
struct BH1792GLC_drv {
    uint8_t sad;
};

static struct BH1792GLC_drv drv_data;
bh1792_t m_bh1792;

/* Platform functions passed to driver */
static struct platform_functions *platform_funcs;

static uint8_t bh1792_getDataOut(bh1792_data_t *dat);
static uint8_t BH1792_getFifoData(bh1792_data_t *dat);

extern uint8_t mode_select;

uint8_t BH1792GLC_plat_func_delay_ms(uint16_t ms)
{
    uint8_t rc = RC_OK; /* delay_ms is optional */

    if (platform_funcs && platform_funcs->delay_ms) {
        rc = platform_funcs->delay_ms(ms);
    }

    return rc;
}

/* BH1790GLC delay */
void BH1792GLC_delay_ms(uint16_t ms)
{
    BH1792GLC_plat_func_delay_ms(ms);
}

/* Set platfrom functions */
uint8_t BH1792GLC_set_platform_functions(struct platform_functions *functions)
{
    if (functions == NULL ||
            functions->twi_read == NULL ||
            functions->twi_write == NULL) {
        return RC_FAIL;
    }
    platform_funcs = functions;
    return RC_OK;
}

uint8_t BH1792GLC_set_plat_func_debug_print_line(char *str)
{
    uint8_t rc = RC_OK; /* debug_println is optional */

    if (platform_funcs && platform_funcs->debug_println && str != NULL) {
        rc = platform_funcs->debug_println(str);
    }

    return rc;
}

uint8_t BH1792GLC_plat_func_debug_print_line(char *str)
{
    uint8_t rc = RC_OK; /* debug_println is optional */

    if (platform_funcs && platform_funcs->debug_println && str != NULL) {
        rc = platform_funcs->debug_println(str);
    }

    return rc;
}

/* Debug prints */
uint8_t BH1792GLC_set_debug_print_line(char *str)
{
    return BH1792GLC_set_plat_func_debug_print_line(str);
}

/* Debug prints */
uint8_t BH1792GLC_debug_print_line(char *str)
{
    return BH1792GLC_plat_func_debug_print_line(str);
}

/* Wrapper to platform functions */
uint8_t BH1792GLC_plat_func_i2c_read(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_read is mandatory */

    if (platform_funcs && platform_funcs->twi_read) {
        rc = platform_funcs->twi_read(sad, reg, data, size);
    }
    return rc;
}

/* BH1790GLC reg read/write and reg bits handling */
uint8_t BH1792GLC_reg_read(uint8_t reg, uint8_t *data, uint8_t size)
{
    return BH1792GLC_plat_func_i2c_read(drv_data.sad, reg, data, size);
}

static uint8_t BH1792_getFifoData(bh1792_data_t *dat)
{
    uint8_t rc = 0;
    uint8_t i = 0U;
    uint8_t reg[1];
    bh1792_maPrm_t *ma_prm;

    ma_prm = &m_bh1792.ma_prm;

    for(i = 0U; i < dat->fifo_lev; i++)
    {
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_DATA0_LSBS,reg,1u);
        dat->fifo[i].off = (uint16_t)reg[0];
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_DATA0_MSBS,reg,1u);
        dat->fifo[i].off |= ((uint16_t)reg[0] << 8);
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_DATA1_LSBS,reg,1u);
        dat->fifo[i].on  = (uint16_t)reg[0];
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_DATA1_MSBS,reg,1u);
        dat->fifo[i].on  |= ((uint16_t)reg[0] << 8);
        if(rc != 0)
        {
            break;
        }
        

        // Moving Average
        if(ma_prm->num == ma_prm->len)
        {
            ma_prm->sum.off -= ma_prm->buf[ma_prm->pos].off;
            ma_prm->sum.on  -= ma_prm->buf[ma_prm->pos].on;
        }
        ma_prm->sum.off += dat->fifo[i].off;
        ma_prm->sum.on  += dat->fifo[i].on;
        ma_prm->buf[ma_prm->pos].off = dat->fifo[i].off;
        ma_prm->buf[ma_prm->pos].on  = dat->fifo[i].on;

        ma_prm->pos++;
        if(ma_prm->pos >= ma_prm->len)
        {
            ma_prm->pos = 0;
        }
        if(ma_prm->num < ma_prm->len)
        {
            ma_prm->num++;
        }

        if(ma_prm->num > 0)
        {
            dat->fifo_lpf[i].off = ma_prm->sum.off / ma_prm->num;
            dat->fifo_lpf[i].on  = ma_prm->sum.on  / ma_prm->num;
        }
    }

    return rc;
}


uint8_t bh1792_GetMeasData(bh1792_data_t *dat)
{
    uint8_t rc         = BH1792_SUCCESS;
    uint8_t int_clear  = 0U;
    uint8_t fifo_level = 0U;

    if(m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) 
    {
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_LEV,&dat->fifo_lev,1u);
        if(rc == 0)
        {
            if(dat->fifo_lev == BH1792_PRM_FIFO_LEV_FULL)
            {
                rc = BH1792_FIFO_FULL;
            }

            if(m_bh1792.prm.int_sel == BH1792_PRM_INT_SEL_WTM)
            {
                dat->fifo_lev = BH1792_PRM_FIFO_LEV_WTM;
            }

            rc = BH1792_getFifoData(dat);
            if(rc == 0) 
            {
                rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_LEV,&fifo_level,1U);
            }
        }
    }
    else 
    {
        dat->fifo_lev = 0U;
        rc = bh1792_getDataOut(dat);

        if(rc == 0)
        {
            if(m_bh1792.prm.int_sel >= BH1792_PRM_INT_SEL_IR)
            {
                rc = BH1792GLC_reg_read(BH1792_ADDR_INT_CLEAR,&int_clear,1U);
            }
        }
    }

    if(rc != 0) {
        m_bh1792.i2c_err = rc;
        rc = BH1792_I2C_ERR;
    }

    return rc;
}

uint8_t bh1792_SetSync(void)
{
    uint8_t rc     = BH1792_SUCCESS;
    uint8_t reg[1];

    reg[0] = BH1792_PRM_MEAS_SYNC;
    rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_SYNC,reg,sizeof(reg));
    if(rc != 0)
    {
        m_bh1792.i2c_err = rc;
        rc = BH1792_I2C_ERR;
    }
    else
    {
        if(m_bh1792.sync_seq < 3)
        {
            m_bh1792.sync_seq++;
        }
    }

    return rc;
}

uint8_t bh1792_ClearFifoData(void)
{
    uint8_t rc        = BH1792_SUCCESS;
    uint8_t i          = 0U;
    uint8_t r_cnt      = 35U;
    uint8_t fifo_level = 0U;
    uint8_t reg[1];

    if(m_bh1792.prm.msr == BH1792_PRM_MSR_32HZ)
    {
        r_cnt = 32U;
    }

    for(i = 0; i < r_cnt; i++) 
    {
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_DATA0_LSBS,reg,1u);
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_DATA0_MSBS,reg,1u);
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_DATA1_LSBS,reg,1u);
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_DATA1_MSBS,reg,1u);
        if(rc != 0) 
        {
            break;
        }
    }

    if(rc == 0)
    {
        rc = BH1792GLC_reg_read(BH1792_ADDR_FIFO_LEV,&fifo_level,1u);
    }

    if(rc != 0) {
        m_bh1792.i2c_err = rc;
        rc = BH1792_I2C_ERR;
    }

    return rc;
}

static uint8_t bh1792_getDataOut(bh1792_data_t *dat)
{
    uint8_t rc = 0;
    uint8_t reg[1]  = {0};

    bh1792_maPrm_t *ma_prm;
    ma_prm = &m_bh1792.ma_prm;
    
    if(m_bh1792.prm.sel_adc == BH1792_PRM_SEL_ADC_GREEN)
    {
        rc = BH1792GLC_reg_read(BH1792_ADDR_GDATA_LEDOFF_LSBS,reg,1u);
        dat->green.off = (uint16_t)reg[0];
        rc = BH1792GLC_reg_read(BH1792_ADDR_GDATA_LEDOFF_MSBS,reg,1u);
        dat->green.off |= ((uint16_t)reg[0] << 8);
        rc = BH1792GLC_reg_read(BH1792_ADDR_GDATA_LEDON_LSBS,reg,1u);
        dat->green.on  = (uint16_t)reg[0];
        rc = BH1792GLC_reg_read(BH1792_ADDR_GDATA_LEDON_MSBS,reg,1u);
        dat->green.on  |= ((uint16_t)reg[0] << 8);

        // Moving Average
        if(ma_prm->num == ma_prm->len)
        {
            ma_prm->sum.off -= ma_prm->buf[ma_prm->pos].off;
            ma_prm->sum.on  -= ma_prm->buf[ma_prm->pos].on;
        }
        ma_prm->sum.off += dat->green.off;
        ma_prm->sum.on  += dat->green.on;
        ma_prm->buf[ma_prm->pos].off = dat->green.off;
        ma_prm->buf[ma_prm->pos].on  = dat->green.on;

        ma_prm->pos++;
        if(ma_prm->pos >= ma_prm->len)
        {
            ma_prm->pos = 0;
        }
        if(ma_prm->num < ma_prm->len)
        {
            ma_prm->num++;
        }

        if(ma_prm->num > 0)
        {
            dat->fifo_lpf[0].off = ma_prm->sum.off / ma_prm->num;
            dat->fifo_lpf[0].on  = ma_prm->sum.on  / ma_prm->num;
        }
    }
    else
    {
        rc = BH1792GLC_reg_read(BH1792_ADDR_IRDATA_LEDOFF_LSBS,reg,1u);
        dat->ir.off    = (uint16_t)reg[0];
        rc = BH1792GLC_reg_read(BH1792_ADDR_IRDATA_LEDOFF_MSBS,reg,1u);
        dat->ir.off    |= ((uint16_t)reg[0] << 8);
        rc = BH1792GLC_reg_read(BH1792_ADDR_IRDATA_LEDON_LSBS,reg,1u);
        dat->ir.on     = (uint16_t)reg[0];
        rc = BH1792GLC_reg_read(BH1792_ADDR_IRDATA_LEDON_MSBS,reg,1u);
        dat->ir.on     |= ((uint16_t)reg[0] << 8);
    }

    return rc;
}


uint8_t BH1792GLC_plat_func_i2c_write(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_write is mandatory */

    if (platform_funcs && platform_funcs->twi_write) {
        rc = platform_funcs->twi_write(sad, reg, data, size);
    }

    return rc;
}


uint8_t BH1792GLC_reg_write(uint8_t reg, uint8_t *data, uint8_t size)
{
    return BH1792GLC_plat_func_i2c_write(drv_data.sad, reg, data, size);
}

uint8_t BH1792GLC_init(uint8_t sad)
{
    uint8_t rc;
    uint8_t reg[1];

    /*platform_funcs needs to be set, or overwrite local read/write/print functions.*/
    if (platform_funcs == NULL) {
        return RC_FAIL;
    }

    drv_data.sad = sad;

    BH1792GLC_delay_ms(2);

    rc = BH1792GLC_reg_read(BH1792_ADDR_PARTID, reg, sizeof(reg));
    if (rc != RC_OK )
    {
        if(reg[0] != BH1792_PRM_PARTID)
        {
            BH1792GLC_debug_print_line("BH1792GLC operation fail");
            return RC_FAIL;
        }
        else
        {
            BH1792GLC_debug_print_line("BH1792GLC PART ID fail");
            return RC_FAIL;
        }
    }
    
    rc = BH1792GLC_reg_read(BH1792_ADDR_MANUFACTURERID, reg, sizeof(reg));
    if (rc == RC_OK )
    {
        if(reg[0] != BH1792_PRM_MANUFACTURERID)
        {
            BH1792GLC_debug_print_line("BH1790GLC MANUFACTURER ID fail");
            return RC_FAIL;
        }
    }
    else
    {
        return rc;
    }

    reg[0] = BH1792_PRM_SWRESET<<7;
    rc = BH1792GLC_reg_write(BH1792_ADDR_RESET, reg, sizeof(reg));
    BH1792GLC_delay_ms(2);

    if (rc == RC_FAIL )
    {
        BH1792GLC_debug_print_line("BH1790GLC reset fail");
        return RC_FAIL;
    }
    
    //BH1792_PRM_MSR_1024HZ; failed
    switch(mode_select)
    {
        case 0 : 
          m_bh1792.prm.msr      = BH1792_PRM_MSR_32HZ;
          m_bh1792.prm.int_sel  = BH1792_PRM_INT_SEL_WTM;
          break;
        case 1 : 
          m_bh1792.prm.msr      = BH1792_PRM_MSR_128HZ;
          m_bh1792.prm.int_sel  = BH1792_PRM_INT_SEL_WTM;
          break;
        case 2 : 
          m_bh1792.prm.msr      = BH1792_PRM_MSR_256HZ;
          m_bh1792.prm.int_sel  = BH1792_PRM_INT_SEL_WTM;
          break;
        case 3 : 
          m_bh1792.prm.msr      = BH1792_PRM_MSR_SINGLE;
          m_bh1792.prm.int_sel  = BH1792_PRM_INT_SEL_SGL;
        default : break;
    }

    m_bh1792.prm.sel_adc  = BH1792_PRM_SEL_ADC_GREEN;
    m_bh1792.prm.led_en   = (BH1792_PRM_LED_EN1_0<<1)|(BH1792_PRM_LED_EN2_0<<0);
    m_bh1792.prm.led_cur1 = BH1792_PRM_LED_CUR1_MA(1);
    m_bh1792.prm.led_cur2 = BH1792_PRM_LED_CUR2_MA(0);
    m_bh1792.prm.ir_th    = 0xFFFC;

    m_bh1792.i2c_err = 0;
    m_bh1792.is_measuring = 0;

    return RC_OK;
}

uint8_t BH1792GLC_Setting_params( void )
{
    uint8_t rc = RC_FAIL;
    uint8_t reg[7];

    BH1792GLC_delay_ms(2);
    
    reg[0] = (BH1792_PRM_RDY<<7)|(BH1792_PRM_SEL_LED<<6)|(m_bh1792.prm.sel_adc<<4)|(m_bh1792.prm.msr)<<0;
    reg[1] = ((m_bh1792.prm.led_en>>1)<<6)|(m_bh1792.prm.led_cur1<<0);
    reg[2] = ((m_bh1792.prm.led_en&0x01U)<<7)|(m_bh1792.prm.led_cur2<<0);
    reg[3] = (uint8_t)m_bh1792.prm.ir_th;
    reg[4] = (uint8_t)(m_bh1792.prm.ir_th>>8);
    reg[5] = m_bh1792.prm.int_sel;

    rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_CTRL1,&reg[0],1u);
    rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_CTRL2,&reg[1],1u);
    rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_CTRL3,&reg[2],1u);
    rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_CTRL4_LSBS,&reg[3],1u);
    rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_CTRL4_MSBS,&reg[4],1u);
    rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_CTRL5,&reg[5],1u);

    if(m_bh1792.is_measuring > 0)
    {
        reg[6] = BH1792_PRM_MEAS_ST;
        rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_START,&reg[6],1u);
    }

    if( rc == RC_FAIL )
    {
        m_bh1792.i2c_err = rc;
        return rc;
    }

    return RC_OK;
}


uint8_t BH1792GLC_measurement_start( void )
{
    uint8_t rc = RC_FAIL;
    uint8_t reg[1];
    uint8_t int_clear;
    reg[0] = BH1792_PRM_MEAS_ST;

    BH1792GLC_delay_ms(2);
    
    rc = BH1792GLC_reg_read(BH1792_ADDR_INT_CLEAR,&int_clear,1U);
    if( rc == RC_FAIL )
    {
        m_bh1792.i2c_err = rc;
        return rc;
    }


    if(m_bh1792.prm.msr < BH1792_PRM_MSR_SINGLE)
    {
        m_bh1792.is_measuring = 1;

        // Initialize moving average parameters
        m_bh1792.ma_prm.sum.off = 0U;
        m_bh1792.ma_prm.sum.on = 0U;
        m_bh1792.ma_prm.pos = 0;
        m_bh1792.ma_prm.num = 0;

        switch( m_bh1792.prm.msr )
        {
            case BH1792_PRM_MSR_32HZ:
            case BH1792_PRM_MSR_64HZ:
            case BH1792_PRM_MSR_128HZ:
                m_bh1792.ma_prm.len = 1;
                break;
            case BH1792_PRM_MSR_256HZ:
                m_bh1792.ma_prm.len = 2;
                break;
            case BH1792_PRM_MSR_1024HZ:
                m_bh1792.ma_prm.len = 8;
                break;
            default:
                m_bh1792.ma_prm.len = 0;
                break;
        }
    }
    else if(m_bh1792.prm.msr == BH1792_PRM_MSR_SINGLE)
    {
        m_bh1792.is_measuring = 1;

        // Initialize moving average parameters
        m_bh1792.ma_prm.sum.off = 0U;
        m_bh1792.ma_prm.sum.on = 0U;
        m_bh1792.ma_prm.pos = 0;
        m_bh1792.ma_prm.num = 0;
        m_bh1792.ma_prm.len = 1;
    }


    rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_START,reg,1U);
    if( rc == RC_FAIL )
    {
        m_bh1792.i2c_err = rc;
        return rc;
    }

    return RC_OK;
}

uint8_t BH1792GLC_measurement_stop( void )
{
    uint8_t rc = RC_OK;
    uint8_t reg[1];
    reg[0] = BH1792_PRM_SWRESET<<7;

    m_bh1792.is_measuring = 0;
    m_bh1792.sync_seq = 0;

    rc = BH1792GLC_reg_write(BH1792_ADDR_MEAS_START,reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
        m_bh1792.i2c_err = rc;
        return rc;
    }

    return RC_OK;
}

