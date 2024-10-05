#include "hm01b0_capture.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hm01b0capture, CONFIG_LOG_DEFAULT_LEVEL);

uint32_t ble_bytes_sent_counter = 0;

void hm01b0_init(void)
{
    /*Test if camera is functional*/
    hm_i2c_write(REG_MODE_SELECT,0x00);
    uint8_t version = hm_i2c_read(REG_MODEL_ID_L);
    if(version != 0xB0){
        LOG_INF("REG_MODEL_ID_L: 0x%x", version);
        LOG_ERR("Camera version problem");
    }
    else
    {
        LOG_INF("Camera version: 0x%02x", version);
    }

    hm_i2c_write( REG_MODE_SELECT, 0x00); //go to stand by mode

    gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, 1);

    /*Camera settings initialization*/
    hm01b0_init_fixed_rom_qvga_fixed();
}


void hm_peripheral_connected_init(void){
    spi_init();
}


void hm_single_capture_spi_832(void){
    nrfx_err_t status;
    (void)status;

    #if(CAM_CLK_GATING == 1)
        nrfx_timer_enable(&CAM_TIMER);
    #endif

    m_length_rx_done = 0;
    ble_bytes_sent_counter = 0;
    line_count = 0;

    /*SPI registers initilization*/
    spis_xfer_done = false;
    #if (MEM_INIT == 1)
      memset(m_rx_buf, MEM_INIT_VALUE, total_spi_buffer_size);
    #endif

    int error = spi_slave_write_msg();
	if(error != 0){
		//LOG_ERR("SPI slave transceive error: %i", error); //DEEKSHA: Uncomment this after SPI bug Fix
	}

    /*Camera values initialized*/
    image_rd_done = 0;
    image_frame_done = 0;
    ble_bytes_sent_counter = 0;

    /*Enable the FRAME VALID interrupt*/
    nrfx_gpiote_trigger_enable(FRAME_VALID_PIN, true);

    /*Count the number of frames sent out*/
    #if (defined(CAMERA_DEBUG) && (CAMERA_DEBUG == 1) && (FINAL_CODE == 0))
        uint8_t frame_cnt = hm_i2c_read(REG_FRAME_COUNT);
        printk("Initial number of frames: %d \n", frame_cnt);
    #endif
    //printf("Initial number of frames: %d \n", frame_cnt);

    nrfx_timer_enable(&TIMER_LVLD);
    gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, 0);


    #if (BLE_TEST_PIN_USED == 1)
    nrf_gpio_pin_set(BLE_START_PIN);
    #endif
    hm_i2c_write( REG_MODE_SELECT, capture_mode);//If we use the 0x03 mode for single capture, the power of camera stays high after capturing one frame
    while (image_rd_done != 1);
    gpio_pin_set(gpio0, TEST_PIN,0);

    //while (!spis_xfer_done); //DEEKSHA Enable this.currently getting stuck if this is enabled
    // ToDo: The above while should actually be enabled. But since SPI handshake isn't happening as expected, the below line is written as workaround
    for (int x=0; x<50000; x++);
    //gpio_pin_set(gpio0, TEST_PIN, 1);
    LOG_DBG("spis_xfer_done");

    spis_xfer_done = false;
    m_length_rx_done = total_image_size;
}


void hm_single_capture_spi_832_stream(void){
    nrfx_err_t status;
    (void)status;

    m_length_rx_done = 0;

    /*SPI registers initilization*/
    spis_xfer_done = false;
    #if (MEM_INIT == 1)
      memset(m_rx_buf, MEM_INIT_VALUE, total_spi_buffer_size);
    #endif

    int error = spi_slave_write_msg();
	if(error != 0){
		//LOG_ERR("SPI slave transceive error: %i\n", error); //DEEKSHA: Uncomment after SPI bug Fix
	}

    /*Camera values initialized*/

    image_rd_done = 0;

    nrfx_gpiote_trigger_enable(FRAME_VALID_PIN, true);

    /*Count the number of frames sent out*/
    #if (defined(CAMERA_DEBUG) && (CAMERA_DEBUG == 1) && (FINAL_CODE == 0))
        uint8_t frame_cnt = hm_i2c_read(REG_FRAME_COUNT);
        printk("Initial number of frames: %d \n", frame_cnt);
    #endif

    #if (BLE_TEST_PIN_USED == 1)
    nrf_gpio_pin_set(BLE_START_PIN);
    #endif
    hm_i2c_write( REG_MODE_SELECT, 0x01);
    m_length_rx_done = total_image_size;
}