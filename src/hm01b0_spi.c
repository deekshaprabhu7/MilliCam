#include "hm01b0_spi.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(SPIModule, CONFIG_LOG_DEFAULT_LEVEL);

uint8_t m_tx_buf[1] = {0};                           /**< TX buffer. */
uint8_t m_rx_buf[total_spi_buffer_size_max+200] = {0xAA};       /**< RX buffer. 200 added for the ACC and Mag data */
uint16_t m_length_rx = spi_buffer_size;        /**< Transfer length. */
uint16_t m_length_rx_done = 0;        /**< Transfer length. */
uint8_t m_length_tx = 0;        /**< Transfer length. */
volatile bool spis_xfer_done;


#define MY_SPI_SLAVE  DT_NODELABEL(my_spi_slave)

// SPI slave functionality
const struct device *spi_dev;
const struct device *spi_slave_dev;
static struct k_poll_signal spi_slave_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_slave_done_sig);

const struct spi_config spi_slave_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE | SPI_CS_ACTIVE_HIGH,
	.frequency = 8000000,
	.slave = 0,
};
/*	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_OP_MODE_SLAVE,*/

 int spi_slave_write_msg(void)
{
    const struct spi_buf s_tx_buf = {
		.buf = m_tx_buf,
		.len = m_length_tx,
	};
	const struct spi_buf_set s_tx = {
		.buffers = &s_tx_buf,
		.count = 1
	};

	// Adjust the receive buffer address for each call
    uint8_t* adjusted_rx_buf = m_rx_buf + line_count * m_length_rx;

	struct spi_buf s_rx_buf = {
		.buf = adjusted_rx_buf,
		.len = m_length_rx,
	};
	const struct spi_buf_set s_rx = {
		.buffers = &s_rx_buf,
		.count = 1
	};
    // Reset signal
    k_poll_signal_reset(&spi_slave_done_sig);

LOG_INF("Before inside spi_slave_write_msg function");
	// Start transaction
//LOG_INF("Value of rxbuffer: %i", m_rx_buf);

	int error = spi_transceive_signal(spi_slave_dev, &spi_slave_cfg, &s_tx, &s_rx, &spi_slave_done_sig);
	if(error != 0){
		LOG_INF("SPI slave transceive error: %i", error);
		return error;
	}
else {
        LOG_INF("NO SPI slave transceive error: %i", error);
        error = spi_slave_check_for_message();
        if (error != 0) {
            LOG_INF("SPI slave check for message error: %i", error);
        }
    }
}


void spi_init(void)
{
	spi_slave_dev = DEVICE_DT_GET(MY_SPI_SLAVE);
	if(!device_is_ready(spi_slave_dev)) {
		LOG_INF("SPI slave device not ready!");
	}
    else{
        LOG_INF("SPI slave device is ready!");
    }
}



int spi_slave_check_for_message(void)
{
	int signaled, result;
	k_poll_signal_check(&spi_slave_done_sig, &signaled, &result);
    LOG_INF("spi_slave_check_for_message: signaled=%d, result=%d", signaled, result);
	if(signaled != 0){
        spis_xfer_done = true;
        LOG_INF("spis_xfer_done = TRUE!!!!!");
		return 0;
	}
	else {
        LOG_INF("SPI transfer not completed yet.");
        return -1;
    }
}
