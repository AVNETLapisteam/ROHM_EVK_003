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
#include "kx224_drv.h"

void KX224_driver_init(struct platform_functions * p_functions) {

    ret_code_t ret;
    
    /* assign platform dedicated functions to driver. */
    KX224_set_platform_functions(p_functions);
    
    /* init KX224 sensor. */
    ret = KX224_init(KX224_DEVICE_ADDRESS_1F);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("KX224_init fail=%d", ret);
        return;
    }

    /* enable sensor interrupt */
    ret = KX224_start(KX224_INT1, GPIO_POLARITY_HITOLO);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("KX224_enable_interrupt fail=%d", ret);
        return;
    }

    /* configure platform interrupt */
    nrf52_enable_interrupt(KX224_INT1_PIN, GPIO_POLARITY_HITOLO, GPIO_NOPULL);
}

/**@brief Init sensor drivers.*/
void sensors_init(struct platform_functions * p_functions) {
    
    /* Add new sensor init here. */
    if (KX224_ENABLED)
    {
        KX224_driver_init(p_functions);
    }
}
