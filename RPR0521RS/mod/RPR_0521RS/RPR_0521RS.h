/*****************************************************************************
  RPR-0521RS.h

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
#ifndef _RPR0521RS_H_
#define _RPR0521RS_H_

/* function return codes */
#define RC_OK 0
#define RC_FAIL 1

#define RPR0521RS_DEVICE_ADDRESS                   (0x38)    // 7bit Addrss
#define RPR0521RS_PART_ID_VAL                      (0x0A)
#define RPR0521RS_MANUFACT_ID_VAL                  (0xE0)

#define RPR0521RS_SYSTEM_CONTROL                   (0x40)
#define RPR0521RS_MODE_CONTROL                     (0x41)
#define RPR0521RS_ALS_PS_CONTROL                   (0x42)
#define RPR0521RS_PS_CONTROL                       (0x43)
#define RPR0521RS_PS_DATA_LSB                      (0x44)
#define RPR0521RS_ALS_DATA0_LSB                    (0x46)
#define RPR0521RS_ALS_DATA1_LSB                    (0x48)
#define RPR0521RS_INTERRUPT                        (0x4A)
#define RPR0521RS_TH_LSB                           (0x4B)
#define RPR0521RS_TH_MSB                           (0x4C)
#define RPR0521RS_MANUFACT_ID                      (0x92)

#define RPR0521RS_MODE_CONTROL_MEASTIME_100_100MS  (6 << 0)
#define RPR0521RS_MODE_CONTROL_PS_EN               (1 << 6)
#define RPR0521RS_MODE_CONTROL_ALS_EN              (1 << 7)

#define RPR0521RS_ALS_PS_CONTROL_LED_CURRENT_25MA  (0 << 0)
#define RPR0521RS_ALS_PS_CONTROL_DATA1_GAIN_X1     (0 << 2)
#define RPR0521RS_ALS_PS_CONTROL_DATA0_GAIN_X1     (0 << 4)

#define RPR0521RS_PS_CONTROL_PS_GAINX1             (0 << 4)
#define RPR0521RS_PS_CONTROL_PS_PERSISTENCE        (1 << 0)

#define RPR0521RS_INTERRUPT_INT_MODE               (0 << 4)
#define RPR0521RS_INTERRUPT_INT_ASSERT             (0 << 3)
#define RPR0521RS_INTERRUPT_INT_LATCH              (1 << 2)
#define RPR0521RS_INTERRUPT_INT_TRIG               (1 << 0)

#define RPR0521RS_MODE_CONTROL_VAL                 (RPR0521RS_MODE_CONTROL_MEASTIME_100_100MS | RPR0521RS_MODE_CONTROL_PS_EN | RPR0521RS_MODE_CONTROL_ALS_EN)
#define RPR0521RS_ALS_PS_CONTROL_VAL               (RPR0521RS_ALS_PS_CONTROL_DATA0_GAIN_X1 | RPR0521RS_ALS_PS_CONTROL_DATA1_GAIN_X1 | RPR0521RS_ALS_PS_CONTROL_LED_CURRENT_25MA)
#define RPR0521RS_PS_CONTROL_VAL                   (RPR0521RS_PS_CONTROL_PS_GAINX1 | RPR0521RS_PS_CONTROL_PS_PERSISTENCE)
#define RPR0521RS_INTERRUPT_VAL                    (RPR0521RS_INTERRUPT_INT_MODE | RPR0521RS_INTERRUPT_INT_ASSERT | RPR0521RS_INTERRUPT_INT_LATCH | RPR0521RS_INTERRUPT_INT_TRIG)
#define RPR0521RS_TH_LSB_VAL                       (0xFF)
#define RPR0521RS_TH_MSB_VAL                       (0x0F)

#define RPR0521RS_NEAR_THRESH                      (1000) // example value
#define RPR0521RS_FAR_VAL                          (0)
#define RPR0521RS_NEAR_VAL                         (1)

#define RPR0521RS_ERROR                            (-1)

uint8_t RPR_0521RS_get_raw(uint8_t *data, uint8_t *size);
uint8_t RPR_0521RS_set_platform_functions(struct platform_functions *functions);
uint8_t RPR_0521RS_init(uint8_t sad);
uint8_t RPR_0521RS_mode_control_setting( void );
uint8_t RPR_0521RS_ALS_PS_control_setting( void );
uint8_t RPR_0521RS_PS_control_setting( void );
uint8_t RPR_0521RS_interrupt_setting( void );

#endif // _RPR0521RS_H_
