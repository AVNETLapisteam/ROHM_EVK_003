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
#include "app_uart.h"

#include "sensors.h"
#include "platform_functions.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256  

uint8_t blocking = false;

/* TWI module instance in use */
static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(0);

/* application event handler implemented in the main module */
extern void application_event_handler(void * p_event_data, uint16_t event_size);


/* Interrupt handler for the sensor DRDY */
static void nrf52_gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{            
    ret_code_t ret;
    static app_internal_evt_t app_event =
    {
        .type = EVENT_READ_TWI,        
    };

    /* Send event to main thread */
    ret = app_sched_event_put(&app_event,
                                   sizeof(app_internal_evt_t),
                                   application_event_handler);
    if (ret != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Application event not scheduled!");
    }
}

/** @brief Handling application events from the scheduler. */
void application_event_handler(void * p_event_data, uint16_t event_size)
{
    uint8_t size;
    ret_code_t ret;
    uint8_t data[MAX_SENSOR_DATA_SIZE];

    app_internal_evt_t *event = (app_internal_evt_t*)p_event_data;
    if(event->type == EVENT_READ_TWI) {
        ret = event->app_events.sensor_evt.sensor_read(data, &size);
        if (ret == NRF_SUCCESS) {

        }
    } else if(event->type == EVENT_BATTERY_MEASUREMENT) {
        //NRF_LOG_INFO( "ADC raw=%d", event->app_events.adc_evt.value);
        //NRF_LOG_INFO( "ADC=%d mV", ADC_RESULT_IN_MILLI_VOLTS(event->app_events.adc_evt.value));
        /*Add code here to send battery measurement result over USB or BLE*/
    }
}


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

    /* set ISR handler. */
    ret = nrf_drv_gpiote_in_init(gpio,config,nrf52_gpiote_event_handler);
    if(ret != NRF_SUCCESS) {
        NRF_LOG_INFO("%s err=%d", __func__, ret);
        return ret;
    }

    /* enable interrupt event. */
    nrf_drv_gpiote_in_event_enable(gpio, true);

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
#if 0
void nrf_twim_interrupt(nrfx_twim_evt_t const *p_event,void *p_context)
{
    if(p_event->type == NRFX_TWIM_EVT_DONE)
    {
        blocking = true;
    }
}
#endif

/**
 * @brief TWI initialization.*/
void nrf52_twi_init (void)
{
    uint32_t err_code;
    nrfx_twim_config_t twi_config = NRFX_TWIM_DEFAULT_CONFIG;
    
    twi_config.frequency  = NRF_TWIM_FREQ_400K;
#if defined(K2_BOARD_CUSTOM)
    twi_config.scl        = K2_TWIM0_SCL_0_27_PIN;
    twi_config.sda        = K2_TWIM0_SDA_0_11_PIN;
#elif defined(BOARD_PCA10040) || defined(BOARD_PCA10056)
    twi_config.scl        = NRF_GPIO_PIN_MAP(0, 27);
    twi_config.sda        = NRF_GPIO_PIN_MAP(0, 26);
//    twi_config.interrupt_priority = APP_IRQ_PRIORITY_HIGH;
//    twi_config.hold_bus_uninit = true;
#else
#error "unsupported board"
#endif
    
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

/**@brief   Function for handling app_uart events.
/**@snippet [Handling the data received over UART] */
extern uint8_t input_string[15];
extern bool config_receive;
void uart_event_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            {
                static uint8_t index = 0;
                UNUSED_VARIABLE(app_uart_get(&input_string[index]));
                if(input_string[index] == '\r' || input_string[index] == '\n')
                {

                }
                else
                {
                    index++;
                }

                if(!strcmp((void *)input_string,"connect"))
                {
                    config_receive = true;
                    index = 0;
                    memset(input_string,0x00,sizeof(input_string));
                }
                if(!strcmp((void *)input_string,"disconnect"))
                {
                    config_receive = false;
                    index = 0;
                    memset(input_string,0x00,sizeof(input_string));
                }

                if(index > 15)
                {
                    index = 0;
                    memset(input_string,0x00,sizeof(input_string));
                }
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */