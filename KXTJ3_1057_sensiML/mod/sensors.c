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
#include "kxtj3_1057_drv.h"

void KXTJ3_1057_driver_init(struct platform_functions * p_functions) {

    ret_code_t ret;
    
    /* assign platform dedicated functions to driver. */
    KXTJ3_1057_set_platform_functions(p_functions);
    
    /* init KXTJ3_1057 sensor. */
    ret = KXTJ3_1057_init(KXTJ3_1057_DEVICE_ADDRESS_0F);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("KXTJ3_1057_init fail=%d", ret);
        return;
    }

    ret = KXTJ3_1057_start_func();
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("KXTJ3_1057 start fail=%d", ret);
        return;
    }

    /* configure platform interrupt */
    nrf52_enable_interrupt(KXTJ3_1057_INT1_PIN, GPIO_POLARITY_HITOLO, GPIO_NOPULL);
}

/**@brief Init sensor drivers.*/
void sensors_init(struct platform_functions * p_functions) {
    
    /* Add new sensor init here. */
    if (KXTJ3_1057_ENABLED)
    {
        KXTJ3_1057_driver_init(p_functions);
    }
}
