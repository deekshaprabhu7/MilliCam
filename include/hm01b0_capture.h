#ifndef HM01B0_CAPTURE_H
#define HM01B0_CAPTURE_H

#include "i2c.h"
#include "hm01b0Regs.h"
#include "gpio.h"
#include "hm01b0_func.h"
#include "hm01b0_clk.h"
#include "hm01b0_spi.h"

extern uint32_t ble_bytes_sent_counter;

void hm01b0_init(void);
void hm_peripheral_connected_init(void);
void hm_single_capture_spi_832(void);
void hm_single_capture_spi_832_stream(void);

#endif