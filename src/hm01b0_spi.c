#include "hm01b0_spi.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(SPIModule, CONFIG_LOG_DEFAULT_LEVEL);

uint8_t m_tx_buf[1] = {0} ;                           /**< TX buffer. */
uint8_t m_rx_buf[total_spi_buffer_size_max+200];       /**< RX buffer. 200 added for the ACC and Mag data */
uint16_t m_length_rx;        /**< Transfer length. */
uint16_t m_length_rx_done = 0;        /**< Transfer length. */
uint8_t m_length_tx = 0;        /**< Transfer length. */
bool spis_xfer_done;

/*

int spi_init(){

    struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(hm01b0spi), SPIOP, 0);

    int err;

    err = spi_is_ready_dt(&spispec);
        if (!err) {
	        LOG_INF("Error: SPI device is not ready, err: %d", err);
        }
        else{
            LOG_INF("SPI device is ready");
        }

    return 0;

} */


nrfx_spis_t spis_inst = NRFX_SPIS_INSTANCE(SPIS_INST_IDX);

static void spis_handler(nrfx_spis_evt_t const * p_event, void * p_context)
{
    if (p_event->evt_type == NRFX_SPIS_XFER_DONE)
    {
        char * p_msg = p_context;
        LOG_INF("SPIS finished. Context passed to the handler: >%s<", p_msg);
        LOG_INF("SPIS rx buffer: %s", m_rx_buf);
    }
}


void spi_init(){

    nrfx_err_t status;
    (void)status;
    void * p_context = "Some context";


    nrfx_spis_config_t spis_config = NRFX_SPIS_DEFAULT_CONFIG(CAM_SPI_SCK_PIN,
                                                              CAM_SPI_MOSI_PIN,
                                                              CAM_SPI_MISO_PIN,
                                                              CAM_SPI_CS_PIN);

    status = nrfx_spis_init(&spis_inst, &spis_config, spis_handler, p_context);
    NRFX_ASSERT(status == NRFX_SUCCESS);

#if defined(__ZEPHYR__)
    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIS_INST_GET(SPIS_INST_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_SPIS_INST_HANDLER_GET(SPIS_INST_IDX), 0);
#endif

  //  status = nrfx_spis_buffers_set(&spis_inst,
   //                                m_tx_buffer_slave, sizeof(m_tx_buffer_slave),
     //                              m_rx_buffer_slave, sizeof(m_rx_buffer_slave));  //DEEKSHA: This is the default function

    status = nrfx_spis_buffers_set(&spis_inst,
                                   m_tx_buf, m_length_tx,
                                   m_rx_buf, m_length_rx);
    NRFX_ASSERT(status == NRFX_SUCCESS);
}