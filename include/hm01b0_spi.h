#ifndef HM01B0_SPI_H
#define HM01B0_SPI_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

//#include <nrfx_spis.h>
#include <nrfx_spim.h>
#include "hm01b0_ble_defines.h"
#include "gpio.h"

#define SPIOP      SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define SPIS_INST_IDX 2

extern uint16_t m_length_rx_done;

void spi_init();

#endif