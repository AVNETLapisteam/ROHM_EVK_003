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


#ifndef _SENSORS_H
#define _SENSORS_H

#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "platform_functions.h"

#define MAX_SENSOR_DATA_SIZE 256

/* Only one sensor should be enabled at a time. */

#define RPR_0521RS_ENABLED        1
#define RPR_0521RS_INT1           1
#define RPR_0521RS_INT1_PIN       NRF_GPIO_PIN_MAP(0, 14)

/**@brief Init sensor drivers.*/
void sensors_init(struct platform_functions * p_functions);

extern uint16_t _als_data0_gain;
extern uint16_t _als_data1_gain;
extern uint16_t _als_measure_time;

#endif
