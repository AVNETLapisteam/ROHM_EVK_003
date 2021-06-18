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
#include "BH1790GLC.h"

void BH1790GLC_driver_init(struct platform_functions * p_functions) {
    /* assign platform dedicated functions to driver. */
    BH1790GLC_set_platform_functions(p_functions);
    ret_code_t ret = RC_FAIL;
    /* init BH1790GLC sensor. */
    
    ret = BH1790GLC_init(BH1790GLC_DEVICE_ADDRESS);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("BH1790GLC_init fail=%d", ret);
        return;
    }

    ret = BH1790GLC_control1_setting();
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("BH1790GLC_control1_setting fail=%d", ret);
        return;
    }
    
    ret = BH1790GLC_control2_setting();
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("BH1790GLC_control2_setting fail=%d", ret);
        return;
    }

    ret = BH1790GLC_measurement_start();
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("BH1790GLC_measurement_start fail=%d", ret);
        return;
    }

    /* configure platform interrupt */
    gpio_pin_polarity_t interrupt_polarity = GPIO_POLARITY_HITOLO;
    nrf52_enable_interrupt(BH1790GLC_INT1_PIN, interrupt_polarity, GPIO_PULLUP);
}

/**@brief Init sensor drivers.*/
void sensors_init(struct platform_functions * p_functions) {
    
    /* Add new sensor init here. */
    if (BH1790GLC_ENABLED)
    {
        BH1790GLC_driver_init(p_functions);
    }
}
