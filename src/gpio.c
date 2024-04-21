#include "gpio.h"

const struct device *gpio0;
const struct device *gpio1;

int m_clk_gpio_err = 0;
int frame_valid_gpio_err = 0;
int line_valid_gpio_err = 0;
int cam_spi_gpio_err = 0;


int gpio_init()
{
    gpio0 = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(gpio0)));
    gpio1 = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(gpio1)));

	m_clk_gpio_err =  gpio_pin_configure(gpio0, MCLK_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    frame_valid_gpio_err = gpio_pin_configure(gpio1, FRAME_VALID_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    line_valid_gpio_err = gpio_pin_configure(gpio0, LINE_VALID_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    cam_spi_gpio_err = gpio_pin_configure(gpio1, CAM_SPI_CS_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
}