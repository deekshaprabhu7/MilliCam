#include "hm01b0_capture.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hm01b0capture, CONFIG_LOG_DEFAULT_LEVEL);

uint32_t ble_bytes_sent_counter = 0; //DEEKSHA define it in some other file?

void hm01b0_init(void)
{
    /*Test if camera is functional*/
    hm_i2c_write(REG_MODE_SELECT,0x00);
    uint8_t version = hm_i2c_read(REG_MODEL_ID_L);
    if(version != 0xB0){
        LOG_INF("REG_MODEL_ID_L: 0x%x", version);
        LOG_INF("Camera version problem");
    }
    else
    {
        LOG_INF("Camera version: 0x%02x", version);
    }

    hm_i2c_write( REG_MODE_SELECT, 0x00); //go to stand by mode

    gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, HIGH);

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

    status = nrfx_spis_buffers_set(&spis_inst, m_tx_buf, m_length_tx, m_rx_buf, m_length_rx);
    LOG_INF("SPI buffers set");
    /*Camera values initialized*/
    image_rd_done = 0;
    image_frame_done = 0;
    ble_bytes_sent_counter = 0;

    /*Enable the FRAME VALID interrupt*/
    nrfx_gpiote_trigger_enable(FRAME_VALID_PIN, true);

    /*Count the number of frames sent out*/
    #if (defined(CAMERA_DEBUG) && (CAMERA_DEBUG == 1) && (FINAL_CODE == 0))
        uint8_t frame_cnt = hm_i2c_read(REG_FRAME_COUNT);
        printf("Initial number of frames: %d \n", frame_cnt);
    #endif
    //printf("Initial number of frames: %d \n", frame_cnt);

    nrfx_timer_enable(&TIMER_LVLD);
    gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, 0);


    #if (BLE_TEST_PIN_USED == 1)
    nrf_gpio_pin_set(BLE_START_PIN);
    #endif
    hm_i2c_write( REG_MODE_SELECT, capture_mode);//If we use the 0x03 mode for single capture, the power of camera stays high after capturing one frame
    while (image_rd_done != 1);

    // while (!spis_xfer_done); //DEEKSHA Enable this.currently getting stuck if this is enabled

    spis_xfer_done = false;

    #if defined(BOARD_PCA10056)
    m_length_rx_done = m_length_rx;
    #endif

    #if defined(BOARD_PCA10040)
    m_length_rx_done = total_image_size;
    #endif

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

    status = nrfx_spis_buffers_set(&spis_inst, m_tx_buf, m_length_tx, m_rx_buf, m_length_rx);

    /*Camera values initialized*/

    image_rd_done = 0;

    nrfx_gpiote_trigger_enable(FRAME_VALID_PIN, true);

    /*Count the number of frames sent out*/
    #if (defined(CAMERA_DEBUG) && (CAMERA_DEBUG == 1) && (FINAL_CODE == 0))
        uint8_t frame_cnt = hm_i2c_read(REG_FRAME_COUNT);
        printf("Initial number of frames: %d \n", frame_cnt);
    #endif

    #if (BLE_TEST_PIN_USED == 1)
    nrf_gpio_pin_set(BLE_START_PIN);
    #endif
    hm_i2c_write( REG_MODE_SELECT, 0x01);

    #if defined(BOARD_PCA10056)
    m_length_rx_done = m_length_rx;
    #endif

    #if defined(BOARD_PCA10040)
    m_length_rx_done = total_image_size;
    #endif
}