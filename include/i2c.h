#ifndef I2C_H
#define I2C_H

#include <zephyr/drivers/i2c.h>

#define HM01B0_I2C_ADDRESS 0x24
#define I2C1_NODE DT_NODELABEL(hm01b0) //i2c1

extern const struct i2c_dt_spec dev_i2c;

int i2c_init();
int hm_i2c_write(uint16_t reg_addr, uint8_t data);
uint8_t hm_i2c_read(uint16_t reg_addr);

#endif