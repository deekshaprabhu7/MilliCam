#ifndef GPIO_H
#define GPIO_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <nrfx_gpiote.h>
#include "hm01b0_lvld_timer.h"
#include "hm01b0_capture.h"
#include "hm01b0_ble_defines.h"
#include "nrf.h"
#include "hm01b0_spi.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100 //LED Blink interval

#define HIGH 1
#define LOW 0

#define FRAME_VALID_PIN 27 //gpio0
#define LINE_VALID_PIN 25 //gpio0
#define MCLK_PIN 26 //gpio0
#define CAM_SPI_GPIO_CS_PIN 11 //gpio1

#define TEST_PIN 4 //gpio0

#define CAM_SPI_PIN_MASK(pin) (1 << (pin))

extern const struct device *gpio0;
extern const struct device *gpio1;

extern uint32_t line_count;
extern volatile uint8_t image_rd_done;
extern uint8_t image_frame_done;
extern bool m_stream_mode_active;
extern bool stream_first_image_done;
extern bool acc_int_cmd_sweep;
extern bool cmd_acc_init_flag;
extern uint8_t m_new_command_received;
extern uint8_t single_capture_flag;
extern bool img_info_sent;
extern uint16_t total_image_size;

extern bool acc_rec_flag;
extern bool acc_int_cmd_flag;

int gpio_init();
void gpio_setting_init(void);

#endif