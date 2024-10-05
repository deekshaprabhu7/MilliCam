#include "gpio.h"
#include <zephyr/logging/log.h>
#include "hm01b0_spi.h"
#include <nrf.h>

LOG_MODULE_REGISTER(gpio, CONFIG_LOG_DEFAULT_LEVEL);

const struct device *gpio0;
const struct device *gpio1;

int m_clk_gpio_err = 0;
int frame_valid_gpio_err = 0;
int line_valid_gpio_err = 0;
int cam_spi_gpio_err = 0;
int test_pin_err = 0;

uint32_t line_count;
uint8_t volatile image_rd_done = 0;
uint8_t image_frame_done = 0;
bool m_stream_mode_active = false;
bool stream_first_image_done = false;
bool acc_int_cmd_sweep;
bool cmd_acc_init_flag = false;
uint8_t single_capture_flag = 0;
bool img_info_sent = false;
uint16_t total_image_size = total_spi_buffer_size;

bool acc_rec_flag = false;
bool acc_int_cmd_flag = false;



int gpio_init()
{
    gpio0 = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(gpio0)));
    gpio1 = device_get_binding(DEVICE_DT_NAME(DT_NODELABEL(gpio1)));

	m_clk_gpio_err =  gpio_pin_configure(gpio0, MCLK_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    frame_valid_gpio_err = gpio_pin_configure(gpio0, FRAME_VALID_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    line_valid_gpio_err = gpio_pin_configure(gpio0, LINE_VALID_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    cam_spi_gpio_err = gpio_pin_configure(gpio1, CAM_SPI_GPIO_CS_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    test_pin_err = gpio_pin_configure(gpio0, TEST_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);

    if (m_clk_gpio_err || frame_valid_gpio_err || line_valid_gpio_err || cam_spi_gpio_err || test_pin_err)
        LOG_ERR("GPIO Initialization Error");
    else
        LOG_INF("GPIOs Initialized");
}

static void in_pin_handler_frame_vld(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *p_context)
{
    nrfx_gpiote_trigger_disable(FRAME_VALID_PIN);

    line_count = 0;
    m_length_rx_done = 0;
    nrfx_timer_enable(&TIMER_LVLD);
    gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, 0);
}

static void in_pin_handler_line_vld(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *p_context)
{
    if (line_count<LINE_NUM){
    // here we need to activate SPI CS; enable the lvld_timer; and activate the line_vld interrupt; increase the counter of lines
        nrfx_timer_enable(&TIMER_LVLD);
       // gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, 0);
        NRF_P1->OUTCLR = (1 << CAM_SPI_GPIO_CS_PIN);
        line_count++;
        }
    else{
        nrfx_gpiote_trigger_disable(LINE_VALID_PIN);
        nrfx_gpiote_trigger_disable(FRAME_VALID_PIN);
        nrfx_timer_disable(&TIMER_LVLD);
        //gpio_pin_set(gpio1, CAM_SPI_GPIO_CS_PIN, 1);
        NRF_P1->OUTSET = (1 << CAM_SPI_GPIO_CS_PIN);

        #if (FRAME_VLD_INT == 1)
          m_length_rx_done = total_image_size;
          ble_bytes_sent_counter = 0;

        /*SPI registers initilization*/
          #if (MEM_INIT == 1)
            memset(m_rx_buf, MEM_INIT_VALUE, total_image_size);
          #endif
          #if (defined(CAMERA_DEBUG) && (CAMERA_DEBUG == 1))
              nrfx_gpiote_trigger_enable(FRAME_VALID_PIN, true);
              //APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length_tx, m_rx_buf, m_length_rx)); //DEEKSHA rewrite this for new SPI driver
          #endif
        #endif
        image_rd_done = 1;
    }
}

void gpio_setting_init(void)
{
    nrfx_err_t err_code;
    static uint8_t fvld_channel;
    static uint8_t lvld_channel;

    if (!nrfx_gpiote_is_init())
    {
        err_code = nrfx_gpiote_init(NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    }

    err_code = nrfx_gpiote_channel_alloc(&fvld_channel);
    NRFX_ASSERT(status == NRFX_SUCCESS);
    err_code = nrfx_gpiote_channel_alloc(&lvld_channel);
    NRFX_ASSERT(status == NRFX_SUCCESS);

   static const nrfx_gpiote_input_config_t in_config_frmvld =
    {
        .pull = NRF_GPIO_PIN_NOPULL,
    };

   // Finds rising edge instead of just toggling
    static const nrfx_gpiote_trigger_config_t trigger_config_frmvalid = {
		.trigger = NRFX_GPIOTE_TRIGGER_LOTOHI,
        .p_in_channel = &fvld_channel,
	};

	static const nrfx_gpiote_handler_config_t handler_config_frmvalid = {
		.handler = in_pin_handler_frame_vld,
	};

    // gpiote initialization has been done in clock and timer initialization, no need to do it again
    err_code = nrfx_gpiote_input_configure(FRAME_VALID_PIN, &in_config_frmvld, &trigger_config_frmvalid, &handler_config_frmvalid);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    static const nrfx_gpiote_input_config_t in_config_linevld = {
        .pull = NRF_GPIO_PIN_NOPULL,
    };

  // Finds rising edge instead of just toggling
    static const nrfx_gpiote_trigger_config_t trigger_config_linevalid = {
		.trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
        .p_in_channel = &lvld_channel,
	};

	static const nrfx_gpiote_handler_config_t handler_config_linevalid = {
		.handler = in_pin_handler_line_vld,
	};

    // gpiote initialization has been done in clock and timer initialization, no need to do it again
    err_code = nrfx_gpiote_input_configure(LINE_VALID_PIN, &in_config_linevld, &trigger_config_linevalid, &handler_config_linevalid);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    LOG_INF("nrfx_gpiote initialized");

}