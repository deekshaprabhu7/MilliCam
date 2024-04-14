#include "i2c.h"


int i2c_init()
{
    if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
	    return;
	}
	printk("I2C bus 0x%x is ready!\n\r",dev_i2c.addr);
}

