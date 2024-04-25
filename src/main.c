/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_dppi.h>

#include "hm01b0_clk.h"
#include "gpio.h"
#include "hm01b0_capture.h"

//#include <zephyr/drivers/video.h>
//#include <zephyr/drivers/video/arducam_mega.h>
#include "app_bluetooth.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(millicam, CONFIG_LOG_DEFAULT_LEVEL);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100 //LED Blink interval

#define RESET_CAMERA 0XFF
#define SET_PICTURE_RESOLUTION 0X01
#define SET_VIDEO_RESOLUTION 0X02
#define SET_BRIGHTNESS 0X03
#define SET_CONTRAST 0X04
#define SET_SATURATION 0X05
#define SET_EV 0X06
#define SET_WHITEBALANCE 0X07
#define SET_SPECIAL_EFFECTS 0X08
#define SET_FOCUS_ENABLE 0X09
#define SET_EXPOSURE_GAIN_ENABLE 0X0A
#define SET_WHITE_BALANCE_ENABLE 0X0C
#define SET_MANUAL_GAIN 0X0D
#define SET_MANUAL_EXPOSURE 0X0E
#define GET_CAMERA_INFO 0X0F
#define TAKE_PICTURE 0X10
#define SET_SHARPNESS 0X11
#define DEBUG_WRITE_REGISTER 0X12
#define STOP_STREAM 0X21
#define GET_FRM_VER_INFO 0X30
#define GET_SDK_VER_INFO 0X40
#define SET_IMAGE_QUALITY 0X50
#define SET_LOWPOWER_MODE 0X60

uint32_t CAM_LINE_VLD;

/*
** Arducam mega communication protocols
** https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/HostCommunicationProtocol.html
*/
#define COMMAND_MAX_SIZE 6

void app_bt_connected_callback(void)
{	
	LOG_INF("Bluetooth connection established. Entering BLE app mode");

}

void app_bt_disconnected_callback(void)
{
	LOG_INF("Bluetooth disconnected. Entering UDP app mode");
	
}

void app_bt_take_picture_callback(void)
{
	LOG_INF("TAKE PICTURE");
}

void app_bt_enable_stream_callback(bool enable)
{
	LOG_INF("Starting stream!");

}

void app_bt_change_resolution_callback(uint8_t resolution)
{
	LOG_INF("Change resolution to");

}


const struct app_bt_cb app_bt_callbacks = {
	.connected = app_bt_connected_callback,
	.disconnected = app_bt_disconnected_callback,
    .take_picture = app_bt_take_picture_callback,
	.enable_stream = app_bt_enable_stream_callback,
	.change_resolution = app_bt_change_resolution_callback,
};



int main(void)
{

	int ret;

	gpio_init();
	clk_init();
	gpio_setting_init();
	i2c_init();
	hm01b0_init();
	lvld_timer_init();

	ret = app_bt_init(&app_bt_callbacks);
	if (ret < 0) {
		LOG_INF("Error initializing Bluetooth");
		return -1;
	}

	while (1) {
		k_msleep(SLEEP_TIME_MS);
	}

	return 0;

}
