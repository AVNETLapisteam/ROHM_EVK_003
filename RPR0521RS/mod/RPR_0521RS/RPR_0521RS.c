#include <stdio.h> 
#include <stddef.h>
#include <stdint.h>

#include "sensors.h"
#include "RPR_0521RS.h"

/* Driver data struct */
struct RPR_0521RS_drv {
    uint8_t sad;
};

static struct RPR_0521RS_drv drv_data;

/* Platform functions passed to driver */
static struct platform_functions *platform_funcs;

uint8_t RPR_0521RS_plat_func_delay_ms(uint16_t ms)
{
    uint8_t rc = RC_OK; /* delay_ms is optional */

    if (platform_funcs && platform_funcs->delay_ms) {
        rc = platform_funcs->delay_ms(ms);
    }

    return rc;
}

/* RPR_0521RS delay */
void RPR_0521RS_delay_ms(uint16_t ms)
{
    RPR_0521RS_plat_func_delay_ms(ms);
}

/* Set platfrom functions */
uint8_t RPR_0521RS_set_platform_functions(struct platform_functions *functions)
{
    if (functions == NULL ||
            functions->twi_read == NULL ||
            functions->twi_write == NULL) {
        return RC_FAIL;
    }
    platform_funcs = functions;
    return RC_OK;
}

uint8_t RPR_0521RS_set_plat_func_debug_print_line(char *str)
{
    uint8_t rc = RC_OK; /* debug_println is optional */

    if (platform_funcs && platform_funcs->debug_println && str != NULL) {
        rc = platform_funcs->debug_println(str);
    }

    return rc;
}

uint8_t RPR_0521RS_plat_func_debug_print_line(char *str)
{
    uint8_t rc = RC_OK; /* debug_println is optional */

    if (platform_funcs && platform_funcs->debug_println && str != NULL) {
        rc = platform_funcs->debug_println(str);
    }

    return rc;
}

/* Debug prints */
uint8_t RPR_0521RS_set_debug_print_line(char *str)
{
    return RPR_0521RS_set_plat_func_debug_print_line(str);
}

/* Debug prints */
uint8_t RPR_0521RS_debug_print_line(char *str)
{
    return RPR_0521RS_plat_func_debug_print_line(str);
}

/* Wrapper to platform functions */
uint8_t RPR_0521RS_plat_func_i2c_read(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_read is mandatory */

    if (platform_funcs && platform_funcs->twi_read) {
        rc = platform_funcs->twi_read(sad, reg, data, size);
    }
    return rc;
}

/* BH1790GLC reg read/write and reg bits handling */
uint8_t RPR_0521RS_reg_read(uint8_t reg, uint8_t *data, uint8_t size)
{
    return RPR_0521RS_plat_func_i2c_read(drv_data.sad, reg, data, size);
}

uint8_t RPR_0521RS_get_raw(uint8_t *data, uint8_t *size)
{
    uint8_t rc;

    rc = RPR_0521RS_reg_read(RPR0521RS_PS_DATA_LSB, &data[0], 2);
    if (rc != RC_OK) {
        RPR_0521RS_debug_print_line("RPR_0521RS_get_rawval fail");
        return rc;
    }

    rc = RPR_0521RS_reg_read(RPR0521RS_ALS_DATA0_LSB, &data[2], 2);
    if (rc != RC_OK) {
        RPR_0521RS_debug_print_line("RPR_0521RS_get_rawval fail");
        return rc;
    }

    rc = RPR_0521RS_reg_read(RPR0521RS_ALS_DATA1_LSB, &data[4], 2);
    if (rc != RC_OK) {
        RPR_0521RS_debug_print_line("RPR_0521RS_get_rawval fail");
        return rc;
    }

    *size = 6;
    return RC_OK;
}

uint8_t RPR_0521RS_plat_func_i2c_write(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t rc = RC_FAIL; /* i2c_write is mandatory */

    if (platform_funcs && platform_funcs->twi_write) {
        rc = platform_funcs->twi_write(sad, reg, data, size);
    }

    return rc;
}

uint8_t RPR_0521RS_reg_write(uint8_t reg, uint8_t *data, uint8_t size)
{
    return RPR_0521RS_plat_func_i2c_write(drv_data.sad, reg, data, size);
}

uint8_t RPR_0521RS_init(uint8_t sad)
{
    uint8_t rc;
    uint8_t reg[1];

    /*platform_funcs needs to be set, or overwrite local read/write/print functions.*/
    if (platform_funcs == NULL) {
        return RC_FAIL;
    }

    drv_data.sad = sad;

    RPR_0521RS_delay_ms(2);

    rc = RPR_0521RS_reg_read(RPR0521RS_SYSTEM_CONTROL, reg, sizeof(reg));
    if (rc != RC_OK )
    {
        if((reg[0]&0x3F) != RPR0521RS_PART_ID_VAL)
        {
            RPR_0521RS_debug_print_line("RPR_0521RS operation fail");
            return RC_FAIL;
        }
        else
        {
            RPR_0521RS_debug_print_line("RPR_0521RS PART ID fail");
            return RC_FAIL;
        }
    }
    else
    {
        RPR_0521RS_debug_print_line("RPR_0521RS PART ID seccess");
    }
    
    rc = RPR_0521RS_reg_read(RPR0521RS_MANUFACT_ID, reg, sizeof(reg));
    if (rc == RC_OK )
    {
        if(reg[0] != RPR0521RS_MANUFACT_ID_VAL)
        {
            RPR_0521RS_debug_print_line("RPR_0521RS MANUFACTURER ID fail");
            return RC_FAIL;
        }
    }
    else
    {
        return rc;
    }

    return RC_OK;
}

uint8_t RPR_0521RS_mode_control_setting( void )
{
    uint8_t rc = RC_FAIL;
    uint8_t reg = RPR0521RS_MODE_CONTROL_VAL;
    rc = RPR_0521RS_reg_write(RPR0521RS_MODE_CONTROL,&reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
      return rc;
    }

    return RC_OK;
}

uint8_t RPR_0521RS_ALS_PS_control_setting( void )
{
    uint8_t rc = RC_FAIL;
    uint8_t reg = RPR0521RS_ALS_PS_CONTROL_VAL;
    rc = RPR_0521RS_reg_write(RPR0521RS_ALS_PS_CONTROL,&reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
      return rc;
    }

    return RC_OK;
}

uint8_t RPR_0521RS_PS_control_setting( void )
{
    uint8_t rc = RC_FAIL;
    uint8_t reg;

    rc = RPR_0521RS_reg_read(RPR0521RS_PS_CONTROL, &reg, sizeof(reg));
    if (rc == RC_FAIL )
    {
        RPR_0521RS_debug_print_line("RPR_0521RS PART ID fail");
        return rc;
    }

    reg |= RPR0521RS_PS_CONTROL_VAL;

    rc = RPR_0521RS_reg_write(RPR0521RS_PS_CONTROL,&reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
      return rc;
    }

    return RC_OK;
}

uint8_t RPR_0521RS_interrupt_setting( void )
{
    uint8_t rc = RC_FAIL;
    uint8_t reg = RPR0521RS_INTERRUPT_VAL;
    rc = RPR_0521RS_reg_write(RPR0521RS_INTERRUPT,&reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
      return rc;
    }

    reg = RPR0521RS_TH_MSB_VAL;
    rc = RPR_0521RS_reg_write(RPR0521RS_TH_MSB,&reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
      return rc;
    }

    reg = RPR0521RS_TH_LSB_VAL;
    rc = RPR_0521RS_reg_write(RPR0521RS_TH_LSB,&reg,sizeof(reg));
    if( rc == RC_FAIL )
    {
      return rc;
    }

    return RC_OK;
}
