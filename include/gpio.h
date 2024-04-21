#ifndef GPIO_H
#define GPIO_H

#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100 //LED Blink interval

#define HIGH 1
#define LOW 0

#define FRAME_VALID_PIN 4 //gpio1
#define LINE_VALID_PIN 25 //gpio0
#define MCLK_PIN 26 //gpio0
#define CAM_SPI_CS_PIN 11 //gpio1
//Reserved P1.12 for SPI CS


extern const struct device *gpio0;
extern const struct device *gpio1;

int gpio_init();

#endif