#ifndef I2C_H
#define I2C_H

#include <zephyr/drivers/i2c.h>

#define I2C1_NODE DT_NODELABEL(hm01b0) //i2c1

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);

#endif