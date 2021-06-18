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
#include "bh1749nuc_drv.h"

void BH1749NUC_driver_init(struct platform_functions * p_functions) {
    /* assign platform dedicated functions to driver. */
    BH1749NUC_set_platform_functions(p_functions);

    /* init BH1749NUC sensor. */
    ret_code_t ret = BH1749NUC_init(BH1749NUC_DEVICE_ADDRESS_39);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("BH1749NUC_init fail=%d", ret);
        return;
    }

    gpio_pin_polarity_t interrupt_polarity = GPIO_POLARITY_HITOLO;
    ret = BH1749NUC_start_set(BH1749NUC_INT1);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("BH1749NUC_start fail=%d", ret);
        return;
    }
}


/**@brief Init sensor drivers.*/
void sensors_init(struct platform_functions * p_functions) {
    
    /* Add new sensor init here. */
    if (BH1749NUC_ENABLED) {
        BH1749NUC_driver_init(p_functions);
    }
}
