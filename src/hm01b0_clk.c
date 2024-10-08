#include "hm01b0_clk.h"

/**
 * @brief Function for handling TIMER driver events.
 *
 * @param[in] event_type Timer event.
 * @param[in] p_context  General purpose parameter set during initialization of the timer.
 *                       This parameter can be used to pass additional information to the handler
 *                       function for example the timer ID.
 */

// Dummy Timer Handler
static void timer_handler(nrf_timer_event_t event_type, void * p_context){}

nrfx_timer_t CAM_TIMER = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);

int clk_init()
{

    nrfx_err_t status;
    (void)status;

    uint8_t out_channel;
    uint8_t gppi_channel;

#if defined(__ZEPHYR__)
    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_TIMER_INST_HANDLER_GET(TIMER_INST_IDX), 0);
    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_GPIOTE), IRQ_PRIO_LOWEST, nrfx_gpiote_irq_handler,
                       0);
#endif

    status = nrfx_gpiote_init(NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    NRFX_ASSERT(status == NRFX_SUCCESS);
    //NRFX_LOG_INFO("GPIOTE status: %s", nrfx_gpiote_is_init() ? "initialized" : "not initialized");

    status = nrfx_gpiote_channel_alloc(&out_channel);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    /*
     * Initialize output pin. The SET task will turn the LED on,
     * CLR will turn it off and OUT will toggle it.
     */
    static const nrfx_gpiote_output_config_t output_config =
    {
        .drive = NRF_GPIO_PIN_S0S1,
        .input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
        .pull = NRF_GPIO_PIN_NOPULL,
    };

    const nrfx_gpiote_task_config_t task_config =
    {
        .task_ch = out_channel,
        .polarity = NRF_GPIOTE_POLARITY_TOGGLE,
        .init_val = NRF_GPIOTE_INITIAL_VALUE_HIGH,
    };

    status = nrfx_gpiote_output_configure(MCLK_PIN, &output_config, &task_config);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_gpiote_out_task_enable(MCLK_PIN);

    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_config.p_context = "Some context";

    status = nrfx_timer_init(&CAM_TIMER, &timer_config, timer_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_timer_clear(&CAM_TIMER);

    /* Creating variable desired_ticks to store the output of nrfx_timer_ms_to_ticks function. */
    // uint32_t desired_ticks = nrfx_timer_ms_to_ticks(&CAM_TIMER, TIME_TO_WAIT_MS);
    //NRFX_LOG_INFO("Time to wait: %lu ms", TIME_TO_WAIT_MS);
    uint32_t desired_ticks = 1UL;

    /*
     * Setting the timer channel NRF_TIMER_CC_CHANNEL0 in the extended compare mode to clear
     * the timer.
     */
    nrfx_timer_extended_compare(&CAM_TIMER, NRF_TIMER_CC_CHANNEL0, desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    status = nrfx_gppi_channel_alloc(&gppi_channel);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    /*
     * Configure endpoints of the channel so that the input timer event is connected with the output
     * pin OUT task. This means that each time the timer interrupt occurs, the LED pin will be toggled.
     */
    nrfx_gppi_channel_endpoints_setup(gppi_channel,
        nrfx_timer_compare_event_address_get(&CAM_TIMER, NRF_TIMER_CC_CHANNEL0),
        nrfx_gpiote_out_task_address_get(MCLK_PIN));

    nrfx_gppi_channels_enable(BIT(gppi_channel));

    nrfx_timer_enable(&CAM_TIMER);
   // NRFX_LOG_INFO("Timer status: %s", nrfx_timer_is_enabled(&timer_inst) ? "enabled" : "disabled");

    return 0;
}