#ifndef __BH1749NUC_REGISTERS_H__
#define __BH1749NUC_REGISTERS_H__
/* registers */

#define BH1749NUC_SYSTEM_CONTROL_REG                  0x40
#define BH1749NUC_MODE_CONTROL1_REG                   0x41
#define BH1749NUC_MODE_CONTROL2_REG                   0x42
                                                      
#define BH1749NUC_RED_DATA_OUT_LSB                    0x50
#define BH1749NUC_RED_DATA_OUT_MSB                    0x51
#define BH1749NUC_GREEN_DATA_OUT_LSB                  0x52
#define BH1749NUC_GREEN_DATA_OUT_MSB                  0x53
#define BH1749NUC_BLUE_DATA_OUT_LSB                   0x54
#define BH1749NUC_BLUE_DATA_OUT_MSB                   0x55
                                                      
#define BH1749NUC_IR_DATA_OUT_LSB                     0x58
#define BH1749NUC_IR_DATA_OUT_MSB                     0x59
#define BH1749NUC_GREEN2_DATA_OUT_LSB                 0x5A
#define BH1749NUC_GREEN2_DATA_OUT_MSB                 0x5B
                                                      
#define BH1749NUC_INTERRUPT_REG                       0x60
#define BH1749NUC_PERSISTENCE_REG                     0x61
                                                      
#define BH1749NUC_TH_HIGH_LSB                         0x62
#define BH1749NUC_TH_HIGH_MSB                         0x63
                                                      
#define BH1749NUC_TH_LOW_LSB                          0x64
#define BH1749NUC_TH_LOW_MSB                          0x65
                                                      
#define BH1749NUC_MANUFACTURER_ID_REG                 0x92
                                                      
#define BH1749NUC_REGISTER_DUMP_END                   0x1E
                                                      
/* registers bits */                                  
#define BH1749NUC_SYSTEM_REG_PART_ID                  (0x0D << 0)
#define BH1749NUC_SYSTEM_REG_SW_RESET                 (0x01 << 7)
#define BH1749NUC_SYSTEM_REG_INT_RESET                (0x01 << 6)
                                                      
#define BH1749NUC_MODE_CTL1_REG_IR_GAIN_1             (0x01 << 5)
#define BH1749NUC_MODE_CTL1_REG_IR_GAIN_32            (0x03 << 5)
                                                      
#define BH1749NUC_MODE_CTL1_REG_RGB_GAIN_1            (0x01 << 3)
#define BH1749NUC_MODE_CTL1_REG_RGB_GAIN_32           (0x03 << 3)
                                                      
#define BH1749NUC_MODE_CTL1_REG_MEASURE_120           (0x02 << 0)
#define BH1749NUC_MODE_CTL1_REG_MEASURE_240           (0x03 << 0)
#define BH1749NUC_MODE_CTL1_REG_MEASURE_35            (0x05 << 0)
                                                      
#define BH1749NUC_MODE_CTL2_REG_UPDATED_FLAG          (0x01 << 7)
#define BH1749NUC_MODE_CTL2_REG_NOT_UPDATE_FLAG       (0x00 << 7)
#define BH1749NUC_MODE_CTL2_REG_POWERDOWN             (0x00 << 4)
#define BH1749NUC_MODE_CTL2_REG_RGB_EN                (0x01 << 4)
                                                      
#define BH1749NUC_INTERRUPT_STATUS_INACTIVE           (0x00 << 7)
#define BH1749NUC_INTERRUPT_STATUS_ACTIVE             (0x01 << 7)
#define BH1749NUC_INTERRUPT_INTERVAL_CHANNEL_RED      (0x00 << 2)
#define BH1749NUC_INTERRUPT_INTERVAL_CHANNEL_GREEN    (0x01 << 2)
#define BH1749NUC_INTERRUPT_INTERVAL_CHANNEL_BLUE     (0x02 << 2)
#define BH1749NUC_INTERRUPT_INT_PIN_DISABLE           (0x00 << 0)
#define BH1749NUC_INTERRUPT_INT_PIN_ENABLE            (0x01 << 0)

#define BH1749NUC_INT_PERSISTENCE_MEASURE_END_ACTIVE  (0x00)
#define BH1749NUC_INT_PERSISTENCE_MEASURED_UPDATE     (0x01)
#define BH1749NUC_INT_PERSISTENCE_4TIME_EQUAL_UPDATE  (0x02)
#define BH1749NUC_INT_PERSISTENCE_8TIME_EQUAL_UPDATE  (0x03)

#define BH1749NUC_MANUFACTURER_ID                     (0xE0)

#define BH1749NUC_THRESHOLD_HIGH_LSB_DEFAULT          (0xFF)
#define BH1749NUC_THRESHOLD_HIGH_MSB_DEFAULT          (0xFF)

#define BH1749NUC_THRESHOLD_LOW_LSB_DEFAULT           (0x00)
#define BH1749NUC_THRESHOLD_LOW_MSB_DEFAULT           (0x00)

 /*registers bit masks */
#define BH1749NUC_SYSTEM_REG_PART_ID_MASK             0x3F
#define BH1749NUC_MODE_CONTROL1_MASK                  0x7F
#define BH1749NUC_MODE_CONTROL2_MASK                  0x90
                                                      
#define BH1749NUC_INTERRUPT_MASK                      0x85
#define BH1749NUC_PERSISTENCE_MASK                    0x03
                                                      
#define BH1749NUC_TH_HIGH_MASK                        0xFF
#define BH1749NUC_TH_LOW_MASK                         0xFF

#endif

