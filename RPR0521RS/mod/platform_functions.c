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
#include "boards.h"
#include "app_scheduler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrfx_twim.h"

#include "sensors.h"
#include "platform_functions.h"
#include "RPR_0521RS.h"

uint8_t blocking = false;

/* TWI module instance in use */
static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(0);

/**
 * @brief Enable GPIO interrupt.*/
uint8_t nrf52_enable_interrupt(uint8_t gpio, gpio_pin_polarity_t polarity, gpio_pin_pullup_t pull) 
{

    nrf_drv_gpiote_in_config_t *config;
    ret_code_t ret;

    /* initialize default configs. */
    nrf_drv_gpiote_in_config_t config_hitolo = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    nrf_drv_gpiote_in_config_t config_lotohi = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    
    /* set pin polarity. */
    if(polarity == GPIO_POLARITY_HITOLO) {
        config = &config_hitolo;
    } else if(polarity == GPIO_POLARITY_LOTOHI) {
        config = &config_lotohi;
    } else {        
        return NRF_ERROR_INVALID_PARAM;
    }
    
    /* set pull-up resistor. */
    if(pull == GPIO_NOPULL) {
        config->pull = NRF_GPIO_PIN_NOPULL;
    } else if (pull == GPIO_PULLDOWN) {
        config->pull = NRF_GPIO_PIN_PULLDOWN;
    } else if (pull == GPIO_PULLUP) {
        config->pull = NRF_GPIO_PIN_PULLUP;
    } else {
        return NRF_ERROR_INVALID_PARAM;
    }

    return NRF_SUCCESS;
}

/**
 * @brief GPIOTE initialization.*/
void nrf52_gpiote_init(void) {
  
    ret_code_t err_code;
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        NRF_LOG_INFO("gpiote_init %d\r\n", err_code);
    }
}

/**
 * @brief TWI initialization.*/
void nrf52_twi_init (void)
{
    uint32_t err_code;
    nrfx_twim_config_t twi_config = NRFX_TWIM_DEFAULT_CONFIG;
    
    twi_config.frequency  = NRF_TWIM_FREQ_400K;
    twi_config.scl        = NRF_GPIO_PIN_MAP(0, 27);
    twi_config.sda        = NRF_GPIO_PIN_MAP(0, 26);
    
    /** Blocking mode enabled */
    err_code = nrfx_twim_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrfx_twim_enable(&m_twi);
}

/**
 * @brief TWI write.*/
uint8_t nrf52_twi_write(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    ret_code_t ret;
    uint8_t buf[size+1];
    
    buf[0] = reg;    
    memcpy(&buf[1], data, size);

    ret = nrfx_twim_tx(&m_twi, sad, buf, size+1, false);
    if(ret != 0) {
        return ret;
    }

    return NRF_SUCCESS;
}


/**
 * @brief TWI read.*/
uint8_t nrf52_twi_read(uint8_t sad, uint8_t reg, uint8_t *data, uint8_t size)
{
    ret_code_t ret;
    ret = nrfx_twim_tx(&m_twi, sad, &reg, 1, false);
    if(ret == NRF_SUCCESS) {
        ret = nrfx_twim_rx(&m_twi, sad, data, size);
    }
    if(ret != 0) {
        return ret;
    }
    return NRF_SUCCESS;
}


/**
 * @brief nrf52 delay.*/
uint8_t nrf52_delay_ms(uint16_t ms)
{
    nrf_delay_ms(ms);
    return NRF_SUCCESS;
}


/**
 * @brief nrf52 debug.*/
uint8_t nrf52_debug_println(char *str)
{
    NRF_LOG_INFO("%s", str);
    return NRF_SUCCESS;
}
