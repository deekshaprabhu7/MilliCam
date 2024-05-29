#include "hm01b0_lvld_timer.h"
#include "gpio.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lvldTimer, CONFIG_LOG_DEFAULT_LEVEL);

uint32_t lvld_timer_val = LVLD_TIMER_VALUE;

nrfx_timer_t TIMER_LVLD = NRFX_TIMER_INSTANCE(TIMER_INST_LVLD_IDX);

void timer_lvld_event_handler(nrf_timer_event_t event_type, void* p_context)
{
  nrfx_err_t status;
    // DEEKSHA: ToDo: This is getting executed approx every 4.2 mins (visual verif using led toggling). Is this correct? (The same old lvld_timer_val value is copied here)
    // Check the behaviour in SES with NRF52.

        if (line_count<LINE_NUM){
        // here we need to activate SPI CS; enable the lvld_timer; and activate the line_vld interrupt; increase the counter of lines
        nrfx_timer_disable(&TIMER_LVLD);
        //gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, 1);
        //z_impl_gpio_port_set_bits_raw(gpio1, CAM_SPI_PIN_MASK(CAM_SPI_GPIO_CS_PIN));
       //  gpio_p_reg->OUTSET = CAM_SPI_PIN_MASK; //DEEKSHA enable this
        // nrfx_spis_buffers_set_back(&spis, m_tx_buf, m_length_tx, m_rx_buf + line_count*m_length_rx, m_length_rx); DEEKSHA: Enable after adding/ configuring SPI
      NRF_P1->OUTSET = (1 << 11);
    /*  status = nrfx_spis_buffers_set(&spis_inst, m_tx_buf, m_length_tx, m_rx_buf + line_count*m_length_rx, m_length_rx);
        //gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, 1);
        NRFX_ASSERT(status == NRFX_SUCCESS);
        if (status != NRFX_SUCCESS) {
          LOG_ERR("nrfx_spis_buffer failed: 0x%02X", status);
          return;
    }*/
        int error = spi_slave_write_msg();
	      if(error != 0){
		      printk("SPI slave transceive error: %i\n", error);
		    //return error;
	  }

       // LOG_INF("m_rx_buf = %d", *m_rx_buf);
        nrfx_gpiote_trigger_enable(LINE_VALID_PIN, true);
        }
    else{
        nrfx_gpiote_trigger_disable(LINE_VALID_PIN);
        nrfx_gpiote_trigger_disable(FRAME_VALID_PIN);
        nrfx_timer_disable(&TIMER_LVLD);
        //gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, 1);
        NRF_P1->OUTSET = (1 << 11);
       // z_impl_gpio_port_set_bits_raw(gpio1, CAM_SPI_PIN_MASK(CAM_SPI_GPIO_CS_PIN));
        //gpio_p_reg->OUTSET = CAM_SPI_PIN_MASK;  //DEEKSHA enable this

        #if (FRAME_VLD_INT == 1)

          #if defined(BOARD_PCA10056)
          m_length_rx_done = m_length_rx;
          #endif

          #if defined(BOARD_PCA10040)
          m_length_rx_done = total_image_size;
          #endif
          ble_bytes_sent_counter = 0;

        /*SPI registers initilization*/
          #if (MEM_INIT == 1)
            memset(m_rx_buf, MEM_INIT_VALUE, total_image_size);
          #endif
          #if (defined(CAMERA_DEBUG) && (CAMERA_DEBUG == 1))
              nrf_drv_gpiote_in_event_enable(FRAME_VLD, true);
              APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length_tx, m_rx_buf, m_length_rx));
          #endif
        #endif
        image_rd_done = 1;

        if(acc_rec_flag == true){
        //Read the Acc and Mag data and save it at the end of the image
            // mc6470_acc_read(); //DEEKSHA Add this function later
        }

        #if (FINAL_CODE ==0)
        printf("Received all \n");
        #endif

    }

   // LOG_INF("LVLD Timer");
}

void lvld_timer_init(void)
{

    #if defined(__ZEPHYR__)
        IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_LVLD_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_TIMER_INST_HANDLER_GET(TIMER_INST_LVLD_IDX), 0);
    #endif

    nrfx_err_t status;
    (void)status;

    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
    nrfx_timer_config_t lvld_timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    lvld_timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    lvld_timer_config.p_context = "Some context";

    status = nrfx_timer_init(&TIMER_LVLD, &lvld_timer_config, timer_lvld_event_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_timer_clear(&TIMER_LVLD);

    /*
     * Setting the timer channel NRF_TIMER_CC_CHANNEL4 in the extended compare mode to clear
     * the timer.
     */
    nrfx_timer_extended_compare(&TIMER_LVLD, NRF_TIMER_CC_CHANNEL1, lvld_timer_val,
                                NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);

    // nrfx_timer_enable(&TIMER_LVLD);

    LOG_INF("LVLD Timer Initialized");

}