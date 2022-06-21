/*****************************************************************************
  BH1790GLC.h

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
#ifndef _BH1790GLC_H_
#define _BH1790GLC_H_

#include "platform_functions.h"

#define BH1790GLC_DEVICE_ADDRESS      (0x5B)    // 7bit Addrss
#define BH1790GLC_MID_VAL             (0xE0)
#define BH1790GLC_PID_VAL             (0x0D)

#define BH1790GLC_MANUFACTURER_ID     (0x0F)
#define BH1790GLC_PART_ID             (0x10)
#define BH1790GLC_RESET               (0x40)
#define BH1790GLC_MEAS_CONTROL1       (0x41)
#define BH1790GLC_MEAS_CONTROL2       (0x42)
#define BH1790GLC_MEAS_START          (0x43)
#define BH1790GLC_DATAOUT_LEDOFF_L    (0x54)
#define BH1790GLC_DATAOUT_LEDOFF_U    (0x55)
#define BH1790GLC_DATAOUT_LEDON_L     (0x56)
#define BH1790GLC_DATAOUT_LEDON_U     (0x57)

#define BH1790GLC_RESET_ON                              0x40
#define BH1790GLC_MEAS_CONTROL1_RDY                     (1 << 7)
#define BH1790GLC_MEAS_CONTROL1_LED_LIGHTING_FREQ_128HZ (0 << 2)
#define BH1790GLC_MEAS_CONTROL1_RCYCLE_32HZ             (2 << 0)

#define BH1790GLC_MEAS_CONTROL2_LED_EN_00               (0 << 6)
#define BH1790GLC_MEAS_CONTROL2_LED_ON_TIME_0_3MS       (0 << 5)
#define BH1790GLC_MEAS_CONTROL2_LED_CURRENT_20MA        (0xD << 0)

#define BH1790GLC_MEAS_START_MEAS_ST                    (1 << 0)

#define BH1790GLC_MEAS_CONTROL1_VAL   ((BH1790GLC_MEAS_CONTROL1_RDY | BH1790GLC_MEAS_CONTROL1_LED_LIGHTING_FREQ_128HZ | BH1790GLC_MEAS_CONTROL1_RCYCLE_32HZ)&0xFF)
#define BH1790GLC_MEAS_CONTROL2_VAL   ((BH1790GLC_MEAS_CONTROL2_LED_EN_00 | BH1790GLC_MEAS_CONTROL2_LED_ON_TIME_0_3MS | BH1790GLC_MEAS_CONTROL2_LED_CURRENT_20MA)&0xFF)
#define BH1790GLC_MEAS_START_VAL      (BH1790GLC_MEAS_START_MEAS_ST&0xFF)

/* function return codes */
#define RC_OK 0
#define RC_FAIL 1

uint8_t BH1790GLC_init(uint8_t sad);
uint8_t BH1790GLC_control1_setting( void );
uint8_t BH1790GLC_control2_setting( void );
uint8_t BH1790GLC_measurement_start( void );
uint8_t BH1790GLC_set_platform_functions(struct platform_functions *functions);
uint8_t BH1790GLC_get_raw(uint8_t *data, uint8_t *size);
uint8_t BH1790GLC_reg_write(uint8_t reg, uint8_t *data, uint8_t size);

#endif // _BH1790GLC_H_
