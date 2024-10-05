#include "i2c.h"
#include <zephyr/logging/log.h>
#include "hm01b0Regs.h"

LOG_MODULE_REGISTER(I2CModule, CONFIG_LOG_DEFAULT_LEVEL);

const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);

int i2c_init()
{
    if (!device_is_ready(dev_i2c.bus)) {
		LOG_ERR("I2C bus %s is not ready!",dev_i2c.bus->name);
	    return -1;
	}
	else{
		LOG_INF("I2C bus 0x%x is ready!",dev_i2c.addr);
		return 0;
	}
}

// Function to read from 16-bit Register address

uint8_t hm_i2c_read(uint16_t reg_addr){
	int ret;
	uint8_t data_out;
	uint8_t addr[2];

	addr[0] = (uint8_t)((reg_addr >> 8) & 0xFF);
	addr[1] = (uint8_t)(reg_addr & 0xFF);

	ret = i2c_write_read_dt(&dev_i2c, addr, sizeof(addr), &data_out, sizeof(data_out));

	if (ret != 0 ){
		LOG_ERR("Failed to write/read I2C address 0x%2x at Reg. 0x%04x", dev_i2c.addr, (addr[0] << 8) | addr[1]);
	}
	return data_out;

	//DEEKSHA: ToDo: I2C reset Timer Implementation
}


// Function to Write to 16-bit register address

int hm_i2c_write(uint16_t reg_addr, uint8_t data) {
	int ret;
    uint8_t addr[3];

    addr[0] = (uint8_t)((reg_addr >> 8) & 0xFF);   // MSB of the register address
    addr[1] = (uint8_t)(reg_addr & 0xFF); // LSB of the register address
	addr[2] = data;

    // Write the buffer to the I2C device
    ret = i2c_write_dt(&dev_i2c, addr, sizeof(addr));
	if(ret != 0){
		LOG_ERR("Failed to write to I2C address 0x%2x at Reg. 0x%04x", dev_i2c.addr, (addr[0] << 8) | addr[1]);
	}
	return ret;
}

