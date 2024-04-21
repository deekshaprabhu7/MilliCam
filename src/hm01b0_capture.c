#include "hm01b0_capture.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hm01b0capture, CONFIG_LOG_DEFAULT_LEVEL);

void hm01b0_init(void)
{
    /*Test if camera is functional*/
    hm_i2c_write(REG_MODE_SELECT,0x00);
    uint8_t version = hm_i2c_read(REG_MODEL_ID_L);
    if(version != 0xB0){
        LOG_INF("REG_MODEL_ID_L: 0x%x", version);
        LOG_INF("Camera version problem");
    }
    else
    {
        LOG_INF("Camera version: 0x%02x", version);
    }

    hm_i2c_write( REG_MODE_SELECT, 0x00); //go to stand by mode

    gpio_pin_set(gpio1, CAM_SPI_CS_PIN, HIGH);

    /*Camera settings initialization*/
    hm01b0_init_fixed_rom_qvga_fixed();
}