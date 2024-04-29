#ifndef HM01B0_SPI_H
#define HM01B0_SPI_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <nrfx_spis.h>
#include "hm01b0_ble_defines.h"
#include "gpio.h"

#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define SPIS_INST_IDX 2

extern nrfx_spis_t spis_inst;

extern uint8_t m_tx_buf[1];                        /**< TX buffer. */
extern uint8_t m_rx_buf[total_spi_buffer_size_max+200];       /**< RX buffer. 200 added for the ACC and Mag data */
extern uint16_t m_length_rx;        /**< Transfer length. */
extern uint16_t m_length_rx_done;       /**< Transfer length. */
extern uint8_t m_length_tx;
extern bool spis_xfer_done;

void spi_init();

#endif