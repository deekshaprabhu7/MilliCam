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
#include "hm01b0_spi.h"

//#include <zephyr/drivers/video.h>
//#include <zephyr/drivers/video/arducam_mega.h>
#include "app_bluetooth.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(millicam, CONFIG_LOG_DEFAULT_LEVEL);

/* 1000 msec = 1 sec */
//#define SLEEP_TIME_MS   500 //LED Blink interval

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
uint8_t LINE_NUM = ROW_INIT;
uint8_t m_new_command_received = 0;
uint8_t jpeg_active = 0;
uint32_t img_data_length = 0;

/*
uint8_t m_new_acc = 0;
bool acc_int_flag = false;
uint8_t acc_pic_num = 0;
bool acc_int_cmd_flag = false;//flag for for accelerometer is giving interrupt or not
bool acc_int_cmd_sweep = false;//flag for if the sweeping is active for accelerometer or not
bool acc_rec_flag = false;//flag for checking if we want to record ACC data or not
bool cmd_acc_init_flag = false;//flag to show if the camera is already initialized or not
uint16_t packet_sent_acc = 0; */


enum {APP_CMD_NOCOMMAND = 0, APP_CMD_SINGLE_CAPTURE, APP_CMD_START_STREAM, APP_CMD_STOP_STREAM, 
      APP_CMD_CHANGE_RESOLUTION, APP_CMD_CHANGE_PHY, APP_CMD_SEND_BLE_PARAMS, APP_CMD_CHANGE_ANGLE, 
      APP_CMD_JPEG, APP_CMD_ACC, APP_CMD_WR_REG, APP_CMD_SWEEP_CAM, APP_CMD_CAM_ACC_REC};


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
	//spi_init();

	ret = app_bt_init(&app_bt_callbacks);
	if (ret < 0) {
		LOG_INF("Error initializing Bluetooth");
		return -1;
	}

	#if (JPEG_COMPRESS == 0)
	for(;;)
	{
        LOG_INF("Entering for");
		k_msleep(SLEEP_TIME_MS);
		if(m_new_command_received != APP_CMD_NOCOMMAND)
		{
			uint32_t new_command = m_new_command_received;
			LOG_INF("new_command=%d", new_command);
            m_new_command_received = APP_CMD_NOCOMMAND;
			switch(new_command)
			{
				case APP_CMD_SINGLE_CAPTURE:
					if(ble_bytes_sent_counter >= m_length_rx_done){
						LOG_INF("Starting capture...");
						if(jpeg_active == 0){
                            hm_peripheral_connected_init();
                            hm_single_capture_spi_832(); //DEEKSHA: Uncomment while loops in this function late
                            LOG_INF("Capture complete: size %i bytes", (uint32_t)(m_length_rx_done));
							single_capture_flag = 1;
					}
				/*	else {
						LOG_INF("Starting capture...");

                            hm_single_capture_spi_832_compressed();
                            LOG_INF("Capture complete: size %i bytes", (uint32_t)(compressed_size));
                            #if (RELEASE_CODE ==0)
                            LOG_INF("Capture complete: size %i bytes", (uint32_t)(compressed_size));
                            #endif

                            single_capture_flag = 1;
					} */  //DEEKSHA: Enable this later
				}
					break;

                case APP_CMD_START_STREAM:
                    LOG_INF("Stream mode enabled");

                    #if (RELEASE_CODE ==0)
                        LOG_INF("Stream mode enabled");
                    #endif
                    break;

                case APP_CMD_STOP_STREAM:
                    LOG_INF("Stream mode disabled");
                    #if (RELEASE_CODE ==0)
                        LOG_INF("Stream mode disabled");
                    #endif
                    break;

                case APP_CMD_SEND_BLE_PARAMS:
                    app_bt_send_picture_header(m_length_rx_done);
                    break;

				default:
					break;
			}
		}

		if(m_stream_mode_active)
        {
			LOG_INF("Entering m_stream_mode_active");
            #if (FRAME_VLD_INT == 1)
            if(!stream_first_image_done){
                if(!acc_int_cmd_sweep){
                    stream_first_image_done = true;
                    hm_peripheral_connected_init();
                    if(cmd_acc_init_flag){
                        cmd_acc_init_flag = false;
                        //cam_power_up(); //DEEKSHA Enable later
                        hm01b0_init_fixed_rom_qvga_fixed();
                        hm_i2c_write(REG_FRAME_LENGTH_LINES_H, 0x02);
                        hm_i2c_write(REG_FRAME_LENGTH_LINES_L, 0x12);
                    }
                    hm_single_capture_spi_832_stream();
                } else {
                    stream_first_image_done = true;
                    hm_peripheral_connected_init();
                    hm_i2c_write(REG_FRAME_LENGTH_LINES_H,CAM_SWEEP_REG_FRAME_LENGTH_LINES_H);
                    hm_i2c_write(REG_FRAME_LENGTH_LINES_L,CAM_SWEEP_REG_FRAME_LENGTH_LINES_L);
                   // pwm_boost_step_rise_fall(DUTY_VALUES_PWM_BOOST[7], PWM_STEPS_NUM, PWM_STEPS_NUM_DOWN , PWM_STEP_DELAY, false); //DEEKSHA : Enable later
                    hm_single_capture_spi_832_stream();
                }
            }
			#else
            if(ble_bytes_sent_counter >= m_length_rx_done) //I don't think this if is needed
//            if(ble_bytes_sent_counter >= compressed_size)
            {

                hm_single_capture_spi_832();

                ble_its_img_info_t image_info;
//                image_info.file_size_bytes = hm_pixel_counter;
                image_info.file_size_bytes = m_length_rx_done;

                ble_its_img_info_send(&m_its, &image_info);

            }

            #endif
        }

		if(ble_bytes_sent_counter < m_length_rx_done )
        {
                #if (FRAME_VLD_INT == 1)
                if(!img_info_sent){
                    // ble_its_img_info_t image_info; //DEEKSHA Check if this is required
                    if(acc_rec_flag == true){
                        m_length_rx_done = m_length_rx_done + m_length_rx;
                    }
                    //image_info.file_size_bytes = m_length_rx_done; //DEEKSHA delete this
                    //ble_its_img_info_send(&m_its, &image_info); //DEEKSHA delete this
                    app_bt_send_picture_header(m_length_rx_done);
                    img_info_sent = true;
                    LOG_INF("BLE1");
                }

                #endif

                #if (BLE_TEST_PIN_USED == 1)
                nrf_gpio_pin_clear(BLE_START_PIN);
                #endif

                int ret_code;
                do
                {
                    if(img_data_length == 0)
                    {
                        /*Send the max number of bytes that the m_ble_its_max_data_len lets you*/
                        img_data_length = ((m_length_rx_done - ble_bytes_sent_counter) > le_tx_data_length ? le_tx_data_length : (m_length_rx_done - ble_bytes_sent_counter));
                    }

                    //ret_code = ble_its_send_file_fragment(&m_its, m_rx_buf+ble_bytes_sent_counter , img_data_length); //DEEKSHA Delete later
                    ret_code = app_bt_send_picture_data(m_rx_buf+ble_bytes_sent_counter, img_data_length);

                    //LOG_INF("m_rx_buf = %d", *m_rx_buf);

                   // LOG_INF("BLE2");
                  /*  int app_bt_send_picture_data(uint8_t *buf, uint16_t len)
                    {
	                    return bt_its_send_img_data(current_conn, buf, len, le_tx_data_length);
                    } */ //DEEKSHA Delete later
                    if(ret_code == 0)
                    {
                        ble_bytes_sent_counter += img_data_length;
                        img_data_length = 0;

                        if (ble_bytes_sent_counter >= m_length_rx_done)
                        {
                            break;
                        }
                        else
                        {
                            //DEEKSHA ToDo: Logic to be added to handle this condition. Check what happens in streaming as well
                        }
                      /* if(acc_rec_flag == true){
                            packet_sent_acc++;
                            //Read the Acc and Mag data and save it at the end of the image
                            if(packet_sent_acc == 20){
                                packet_sent_acc = 0;
                                mc6470_acc_read();
                            }
                        } */ //DEEKSHA: Enable later
                    }
                }while(1);

                LOG_INF("BLE9");
              #if (FRAME_VLD_INT == 1)
              if((ble_bytes_sent_counter >= m_length_rx_done) && m_stream_mode_active){
                LOG_INF("BLE3");

                  img_info_sent = false;
                  if(!acc_int_cmd_flag){
                    //APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length_tx, m_rx_buf, m_length_rx));
                          int error = spi_slave_write_msg();
	                        if(error != 0){
		                        printk("SPI slave transceive error: %i\n", error);
		                        //return error;
	                            }

                      nrfx_gpiote_trigger_enable(FRAME_VALID_PIN, true);
                  } 
                  else 
                  { /*
                      memset(m_rx_buf+total_image_size, 0, m_length_rx);
                      acc_rec_counter = 0;
                      if(!acc_int_cmd_sweep){
                          if(acc_pic_num > 0){
                             // APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length_tx, m_rx_buf, m_length_rx));
                              if(acc_rec_flag == true){
                                  acc_rec_counter = 0;
                                  packet_sent_acc = 0;
                              }
                              nrf_drv_gpiote_in_event_enable(FRAME_VLD, true);
                              acc_pic_num--;
                          } else{
                              hm_i2c_write( REG_MODE_SELECT, 0x00);
                              m_stream_mode_active = false;
                              stream_first_image_done = false;
                              nrf_delay_ms(POR_DELAY);
                              nrf_drv_gpiote_in_event_enable(ACC_INT_PIN, true);
                              accel_sr = hm_i2c_read_8b(SR, SLAV_ADDR_ACC);
                              #if(CAM_CLK_GATING == 1)
                              nrf_drv_timer_disable(&CAM_TIMER);
                              #endif
                              hm_peripheral_uninit();
                          }
                      } else {
                          if(acc_pic_num > 0){
                              APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length_tx, m_rx_buf, m_length_rx));
                              switch(acc_pic_num){
                                  case 5: 
                                      break;

                                  case 4: 
                                      pwm_boost_step_rise_fall(DUTY_VALUES_PWM_BOOST[2], PWM_STEPS_NUM, PWM_STEPS_NUM_DOWN , PWM_STEP_DELAY, false);
                                      break;

                                  case 3: 
                                      pwm_boost_zero();
                                      break;

                                  case 2: 
                                      pwm_boost_step_rise_fall(DUTY_VALUES_PWM_BOOST[2], PWM_STEPS_NUM, PWM_STEPS_NUM_DOWN , PWM_STEP_DELAY, true);
                                      break;

                                  case 1: 
                                      pwm_boost_step_rise_fall(DUTY_VALUES_PWM_BOOST[7], PWM_STEPS_NUM, PWM_STEPS_NUM_DOWN , PWM_STEP_DELAY, true);
                                      break;
                              }
                              acc_pic_num--;
                              nrf_drv_gpiote_in_event_enable(FRAME_VLD, true);
                          } else {
                              hm_i2c_write( REG_MODE_SELECT, 0x00);
                              m_stream_mode_active = false;
                              stream_first_image_done = false;
                              nrf_delay_ms(POR_DELAY);
                              nrf_drv_gpiote_in_event_enable(ACC_INT_PIN, true);
                              accel_sr = hm_i2c_read_8b(SR, SLAV_ADDR_ACC);
                              #if(CAM_CLK_GATING == 1)
                              nrf_drv_timer_disable(&CAM_TIMER);
                              #endif

                              pwm_boost_zero();

                              hm_peripheral_uninit();

                            //  Make the boost clock pin 0 to turn off the switching transistor
                              nrf_gpio_cfg(
                              BOOST_PIN,
                              NRF_GPIO_PIN_DIR_OUTPUT,
                              NRF_GPIO_PIN_INPUT_DISCONNECT,
                              NRF_GPIO_PIN_PULLDOWN,
                              NRF_GPIO_PIN_S0S1,
                              NRF_GPIO_PIN_NOSENSE);
                              nrf_gpio_pin_clear(BOOST_PIN);
                          }
                      }
                */  }   //DEEKSHA: Enable this section later 
             } else if(!m_stream_mode_active && (ble_bytes_sent_counter < m_length_rx_done)){
                   // LOG_INF("BLE4");
                  hm_i2c_write( REG_MODE_SELECT, 0x00);
              } else if(!m_stream_mode_active && (ble_bytes_sent_counter >= m_length_rx_done)){
                LOG_INF("BLE5");
                  #if(CAM_CLK_GATING == 1)
                   nrfx_timer_disable(&CAM_TIMER);
                  #endif

                //  hm_peripheral_uninit(); //DEEKSHA Enable later

                  #if(CAM_SINGLE_CAPTURE_POWER_FIX == 1)
                  nrf_delay_ms(CAM_TURN_OFF_DELAY);
                  nrf_gpio_pin_clear(CAM_POWER);
                  #endif
              }
              #endif
        }

        if(m_new_command_received == APP_CMD_NOCOMMAND)
        {
          // idle_state_handle(); //DEEKSHA: Check if this is needed
          LOG_INF("BLE6");
        }
	}

	#endif
/*
    #if (JPEG_COMPRESS == 0)
    for (;;)
    {
        uint32_t image_size;
       // ble_gap_phys_t gap_phys_settings;

        if(m_new_command_received != APP_CMD_NOCOMMAND)
        {
            uint32_t new_command = m_new_command_received;
            m_new_command_received = APP_CMD_NOCOMMAND;
            switch(new_command)
            {
				case APP_CMD_SINGLE_CAPTURE:
					if(ble_bytes_sent_counter >= m_length_rx_done){
						LOG_INF("Starting capture...");
						if(jpeg_active == 0){
                            hm_peripheral_connected_init();
                            hm_single_capture_spi_832();
					}
				}

				default:
                    break;
			}

        if(m_stream_mode_active)
        {
            #if (FRAME_VLD_INT == 1)
            if(!stream_first_image_done){
                if(!acc_int_cmd_sweep){
                    stream_first_image_done = true;
                    hm_peripheral_connected_init();
                    if(cmd_acc_init_flag){
                        cmd_acc_init_flag = false;
                        //cam_power_up(); //DEEKSHA Enable later
                        hm01b0_init_fixed_rom_qvga_fixed();
                        hm_i2c_write(REG_FRAME_LENGTH_LINES_H, 0x02);
                        hm_i2c_write(REG_FRAME_LENGTH_LINES_L, 0x12);
                    }
                    hm_single_capture_spi_832_stream();
                } else {
                    stream_first_image_done = true;
                    hm_peripheral_connected_init();
                    hm_i2c_write(REG_FRAME_LENGTH_LINES_H,CAM_SWEEP_REG_FRAME_LENGTH_LINES_H);
                    hm_i2c_write(REG_FRAME_LENGTH_LINES_L,CAM_SWEEP_REG_FRAME_LENGTH_LINES_L);
                   // pwm_boost_step_rise_fall(DUTY_VALUES_PWM_BOOST[7], PWM_STEPS_NUM, PWM_STEPS_NUM_DOWN , PWM_STEP_DELAY, false); //DEEKSHA : Enable later
                    hm_single_capture_spi_832_stream();
                }
            }
			#else
            if(ble_bytes_sent_counter >= m_length_rx_done) //I don't think this if is needed
//            if(ble_bytes_sent_counter >= compressed_size)
            {

                hm_single_capture_spi_832();

                ble_its_img_info_t image_info;
//                image_info.file_size_bytes = hm_pixel_counter;
                image_info.file_size_bytes = m_length_rx_done;

                ble_its_img_info_send(&m_its, &image_info);

            }

            #endif
        }

		}
	}
	#endif


*/
/*
	while (1) {
		k_msleep(SLEEP_TIME_MS);
	} */

	return 0;

}