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


#include <stdint.h>
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "sensors.h"
#include "RPR_0521RS.h"

uint16_t _als_data0_gain;
uint16_t _als_data1_gain;
uint16_t _als_measure_time;

void RPR_0521RS_driver_init(struct platform_functions * p_functions) {
    /* assign platform dedicated functions to driver. */
    RPR_0521RS_set_platform_functions(p_functions);
    ret_code_t ret = RC_FAIL;
    /* init RPR_0521RS sensor. */
    
    ret = RPR_0521RS_init(RPR0521RS_DEVICE_ADDRESS);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("RPR_0521RS_init fail=%d", ret);
        return;
    }

    ret = RPR_0521RS_ALS_PS_control_setting();
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("RPR_0521RS_ALS_PS_control_setting fail=%d", ret);
        return;
    }
    
    ret = RPR_0521RS_PS_control_setting();
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("RPR_0521RS_PS_control_setting fail=%d", ret);
        return;
    }

    ret = RPR_0521RS_mode_control_setting();
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("RPR_0521RS_mode_control_setting fail=%d", ret);
        return;
    }

    ret = RPR_0521RS_interrupt_setting();
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("RPR_0521RS_interrupt_setting fail=%d", ret);
        return;
    }

    /* configure platform interrupt */
    gpio_pin_polarity_t interrupt_polarity = GPIO_POLARITY_HITOLO;
    nrf52_enable_interrupt(RPR_0521RS_INT1_PIN, interrupt_polarity, GPIO_PULLUP);

    uint8_t reg;
    uint8_t index;
    uint8_t als_gain_table[]={1,2,64,128};
    uint16_t als_meas_time_table[]={0,0,0,0,0,100,100,100,100,100,400,400,50,0,0,0};
    
    reg = RPR0521RS_ALS_PS_CONTROL_VAL;
    index = (reg>>4)&0x03;
    _als_data0_gain = als_gain_table[index];
    
    reg = RPR0521RS_ALS_PS_CONTROL_VAL;
    index = (reg>>2)&0x03;
    _als_data1_gain = als_gain_table[index];
    
    index = RPR0521RS_MODE_CONTROL_VAL&0x0f;
    _als_measure_time = als_meas_time_table[index];

}

/**@brief Init sensor drivers.*/
void sensors_init(struct platform_functions * p_functions) {
    
    /* Add new sensor init here. */
    if (RPR_0521RS_ENABLED) {
        RPR_0521RS_driver_init(p_functions);
    }
}
