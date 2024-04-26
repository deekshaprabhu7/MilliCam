#ifndef GPIO_H
#define GPIO_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <nrfx_gpiote.h>
#include "hm01b0_lvld_timer.h"
#include "hm01b0_capture.h"
#include "hm01b0_ble_defines.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100 //LED Blink interval

#define HIGH 1
#define LOW 0

#define FRAME_VALID_PIN 27 //gpio0
//#define FRAME_VALID_PIN NRF_GPIO_PIN_MAP(1, 4)
#define LINE_VALID_PIN 25 //gpio0
//#define MCLK_PIN NRF_GPIO_PIN_MAP(0, 27) //gpio0
#define MCLK_PIN 26 //gpio0
#define CAM_SPI_CS_PIN 11 //gpio1
#define DUMMY_CS_PIN 12
//Reserved P1.12 for SPI CS


extern const struct device *gpio0;
extern const struct device *gpio1;

extern uint32_t line_count;
extern uint16_t m_length_rx_done;
extern uint8_t image_rd_done;

int gpio_init();
void gpio_setting_init(void);
//static void in_pin_handler_frame_vld(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger);

#endif