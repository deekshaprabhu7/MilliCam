#include "gpio.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio, CONFIG_LOG_DEFAULT_LEVEL);

const struct device *gpio0;
const struct device *gpio1;

int m_clk_gpio_err = 0;
int frame_valid_gpio_err = 0;
int line_valid_gpio_err = 0;
int cam_spi_gpio_err = 0;
int dummy_spi_gpio_err = 0;

uint32_t line_count;
uint16_t m_length_rx_done = 0;

int gpio_init()
{
    gpio0 = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(gpio0)));
    gpio1 = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(gpio1)));

	m_clk_gpio_err =  gpio_pin_configure(gpio0, MCLK_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    frame_valid_gpio_err = gpio_pin_configure(gpio0, FRAME_VALID_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    line_valid_gpio_err = gpio_pin_configure(gpio0, LINE_VALID_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    cam_spi_gpio_err = gpio_pin_configure(gpio1, CAM_SPI_CS_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    dummy_spi_gpio_err = gpio_pin_configure(gpio1, DUMMY_CS_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
}

static void in_pin_handler_frame_vld(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *p_context)
{
    //nrf_gpiote_event_disable(FRAME_VALID_PIN); //DEEKSHA Check this
    nrfx_gpiote_trigger_disable(FRAME_VALID_PIN);

    line_count = 0;
    m_length_rx_done = 0;
    nrfx_timer_enable(&TIMER_LVLD);
    //lvld_timer_run(&TIMER_LVLD); DEEKSHA UNCOMMENT
    //gpio_pin_set(gpio1, CAM_SPI_CS_PIN, 0);
    LOG_INF("test");
}

static void in_pin_handler_line_vld(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *p_context)
{
    //nrf_gpiote_event_disable(FRAME_VALID_PIN); //DEEKSHA Check this

    //line_count = 0;
    //m_length_rx_done = 0;
    //lvld_timer_run(&TIMER_LVLD); DEEKSHA UNCOMMENT
    //gpio_pin_set(gpio1, CAM_SPI_CS_PIN, 0);
    LOG_INF("test2");

    gpio_pin_set(gpio1, CAM_SPI_CS_PIN, HIGH);
}


void gpio_setting_init(void)
{
    nrfx_err_t err_code;
    uint8_t fvld_channel;
/*
#if defined(__ZEPHYR__)
    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_GPIOTE), IRQ_PRIO_LOWEST, nrfx_gpiote_irq_handler,
                       0);
#endif

    err_code = nrfx_gpiote_init(NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY); */

    err_code = nrfx_gpiote_channel_alloc(&fvld_channel);
    NRFX_ASSERT(status == NRFX_SUCCESS);

   static const nrfx_gpiote_input_config_t in_config_frmvld =
    {
        .pull = NRF_GPIO_PIN_NOPULL,
    };

   /*Finds rising edge instead of just toggling*/
    static const nrfx_gpiote_trigger_config_t trigger_config_frmvalid = {
		.trigger = NRFX_GPIOTE_TRIGGER_LOTOHI,
       // .p_in_channel = &fvld_channel,
	};

	static const nrfx_gpiote_handler_config_t handler_config_frmvalid = {
		.handler = in_pin_handler_frame_vld,
	};

    /*gpiote initialization has been done in clock and timer initialization, no need to do it again*/
    err_code = nrfx_gpiote_input_configure(FRAME_VALID_PIN, &in_config_frmvld, &trigger_config_frmvalid, &handler_config_frmvalid);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_gpiote_trigger_enable(FRAME_VALID_PIN, true);

    static const nrfx_gpiote_input_config_t in_config_linevld =
    {
        .pull = NRF_GPIO_PIN_NOPULL,
    };

   /*Finds rising edge instead of just toggling*/
    static const nrfx_gpiote_trigger_config_t trigger_config_linevalid = {
		.trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
       // .p_in_channel = &fvld_channel,
	};

	static const nrfx_gpiote_handler_config_t handler_config_linevalid = {
		.handler = in_pin_handler_line_vld,
	};

    /*gpiote initialization has been done in clock and timer initialization, no need to do it again*/
    err_code = nrfx_gpiote_input_configure(LINE_VALID_PIN, &in_config_linevld, &trigger_config_linevalid, &handler_config_linevalid);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_gpiote_trigger_enable(LINE_VALID_PIN, true);

    LOG_INF("nrfx_gpiote initialized");

}