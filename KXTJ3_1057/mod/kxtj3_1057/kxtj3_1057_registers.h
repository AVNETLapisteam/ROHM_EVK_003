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

#ifndef __KXTJ3_1057_REGISTERS_H__
#define __KXTJ3_1057_REGISTERS_H__
/* registers */
// output register x
#define KXTJ3_1057_XOUT_L 0x06
#define KXTJ3_1057_XOUT_H 0x07
// output register y
#define KXTJ3_1057_YOUT_L 0x08
#define KXTJ3_1057_YOUT_H 0x09
// output register z
#define KXTJ3_1057_ZOUT_L 0x0A
#define KXTJ3_1057_ZOUT_H 0x0B
// communication selftest
#define KXTJ3_1057_DCST_RESP 0x0C
// WHO_AM_I
#define KXTJ3_1057_WHO_AM_I 0x0F
// This register reports which function caused an interrupt.
#define KXTJ3_1057_INT_SOURCE1 0x16
// This register reports the axis and direction of detected motion.
#define KXTJ3_1057_INT_SOURCE2 0x17
// This register reports the status of the interrupt.
#define KXTJ3_1057_STATUS 0x18
// Latched interrupt source information (INT_SOURCE1 and INT_SOURCE2) is cleared and physical interrupt latched pin (INT) is changed to its inactive state when this register is read 
#define KXTJ3_1057_INT_REL 0x1A
// Read/write control register that controls the main feature set. 
#define KXTJ3_1057_CTRL_REG1 0x1B
// Read/write control register that provides more feature set control. Note that to properly change the value of this register, the PC1 bit in CTRL_REG1 must first be set to Åg0Åh.
#define KXTJ3_1057_CTRL_REG2 0x1D
// This register controls the settings for the physical interrupt pin (INT). Note that to properly change the value of this register, the PC1 bit in CTRL_REG1 must first be set to Åg0Åh.
#define KXTJ3_1057_INT_CTRL_REG1 0x1E
// This register controls which axis and direction of detected motion can cause an interrupt. Note that to properly change the value of this register, the PC1 bit in CTRL_REG1 must first be set to Åg0Åh.
#define KXTJ3_1057_INT_CTRL_REG2 0x1F
// Read/write control register that configures the acceleration outputs. Note that to properly change the value of this register, the PC1 bit in CTRL_REG1 must first be set to Åg0Åh.
#define KXTJ3_1057_DATA_CTRL_REG 0x21
// This register sets the time motion must be present before a wake-up interrupt is set.
#define KXTJ3_1057_WAKEUP_COUNTER 0x29
// This register sets the non-activity time required before another wake-up interrupt can be set. 
#define KXTJ3_1057_NA_COUNTER 0x2A
// When 0xCA is written to this register, the MEMS self-test function is enabled.
#define KXTJ3_1057_SELF_TEST 0x3A
// This register sets the threshold for wake-up (motion detect) interrupt is set.
#define KXTJ3_1057_WAKEUP_THRESHOLD_H 0x6A
#define KXTJ3_1057_WAKEUP_THRESHOLD_L 0x6B

/* registers bits */
#define KXTJ3_1057_WAI_VAL            (0x35)
#define KXTJ3_1057_CNTL1_WUFE         (1 << 1)
#define KXTJ3_1057_CNTL1_TDTE         (1 << 2)
#define KXTJ3_1057_CNTL1_EN16GMASK    (0x04)
#define KXTJ3_1057_CNTL1_GSEL_16G     (0x04)
#define KXTJ3_1057_CNTL1_GSELMASK     (0x18)
#define KXTJ3_1057_CNTL1_GSEL_14BIT   (0x18)
#define KXTJ3_1057_CNTL1_GSEL_2G      (0x00)
#define KXTJ3_1057_CNTL1_GSEL_4G      (0x08)
#define KXTJ3_1057_CNTL1_GSEL_8G      (0x10)
#define KXTJ3_1057_CNTL1_DRDYE        (1 << 5)
#define KXTJ3_1057_CNTL1_RES          (1 << 6)
#define KXTJ3_1057_CNTL1_LOWPOWER     (0x00)
#define KXTJ3_1057_CNTL1_HIGHRESO     (0x40)
#define KXTJ3_1057_CNTL1_PC1          (1 << 7)
#define KXTJ3_1057_DATA_CNTL_OSA_50HZ (2)

#define KXTJ3_1057_CNTL1_VAL          (KXTJ3_1057_CNTL1_LOWPOWER | KXTJ3_1057_CNTL1_GSEL_2G)
#define KXTJ3_1057_DATA_CNTL_VAL      (KXTJ3_1057_DATA_CNTL_OSA_50HZ)

#define KXTJ3_1057_GSENS_2G           (2)
#define KXTJ3_1057_GSENS_4G           (4)
#define KXTJ3_1057_GSENS_8G           (8)
#define KXTJ3_1057_GSENS_16G          (16)
#define KXTJ3_1057_GSENS_ERROR        (0xFF)

#define KXTJ3_1057_READ_DATA_SIZE     (6)
#define KXTJ3_1057_AXIS_SIZE          (KXTJ3_READ_DATA_SIZE / 2)
#define KXTJ3_1057_DIVIDE_SHIFT       (15)

// sets the response of the physical interrupt pin
#define KXTJ3_1057_INC1_IEL1 (0x01 << 3)
// enables/disables the physical interrupt
#define KXTJ3_1057_INC1_IEN1 (0x01 << 5)
// sets the polarity of the physical interrupt pin
#define KXTJ3_1057_INC1_IEA1 (0x01 << 4)

 /*registers bit masks */
#define KXTJ3_1057_WHO_AM_I_WIA_MASK 0xFF

#endif

