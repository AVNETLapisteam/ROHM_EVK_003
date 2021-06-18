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
#include "bm1422gmv_drv.h"

extern uint8_t offset_measure_flag;

void BM1422GMV_driver_init(struct platform_functions * p_functions) {
    /* assign platform dedicated functions to driver. */
    BM1422GMV_set_platform_functions(p_functions);

    /* init BM1422GMV sensor. */
    ret_code_t ret = BM1422GMV_init(BM1422GMV_DEVICE_ADDRESS_0E);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("BM1422GMV_init fail=%d", ret);
        return;
    }

    /* configure platform interrupt */
    gpio_pin_polarity_t interrupt_polarity = GPIO_POLARITY_HITOLO;
    nrf52_enable_interrupt(BM1422GMV_INT1_PIN, interrupt_polarity, GPIO_PULLUP);

    ret = BM1422GMV_offset_start(BM1422GMV_INT1, interrupt_polarity);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("BM1422GMV_offset_start fail=%d", ret);
        return;
    }

}


/**@brief Init sensor drivers.*/
void sensors_init(struct platform_functions * p_functions) {
    
    /* Add new sensor init here. */
    if (BM1422GMV_ENABLED)
    {
        BM1422GMV_driver_init(p_functions);
    }
}
