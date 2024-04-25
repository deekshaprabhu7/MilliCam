#include "hm01b0_lvld_timer.h"
#include "gpio.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lvldTimer, CONFIG_LOG_DEFAULT_LEVEL);

uint32_t lvld_timer_val = LVLD_TIMER_VALUE;

nrfx_timer_t TIMER_LVLD = NRFX_TIMER_INSTANCE(TIMER_INST_LVLD_IDX);

void timer_lvld_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    // DEEKSHA: ToDo: This is getting executed approx every 4.2 mins (visual verif using led toggling). Is this correct? (The same old lvld_timer_val value is copied here)
    // Check the behaviour in SES with NRF52.

    LOG_INF("LVLD Timer");
    //gpio_pin_toggle(gpio1, CAM_SPI_CS_PIN);
    //k_msleep(SLEEP_TIME_MS);
}

void lvld_timer_init(void)
{

    #if defined(__ZEPHYR__)
        IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(TIMER_INST_LVLD_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_TIMER_INST_HANDLER_GET(TIMER_INST_LVLD_IDX), 0);
    #endif

    nrfx_err_t status;
    (void)status;

    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
    nrfx_timer_config_t lvld_timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    lvld_timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    lvld_timer_config.p_context = "Some context";

    status = nrfx_timer_init(&TIMER_LVLD, &lvld_timer_config, timer_lvld_event_handler);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_timer_clear(&TIMER_LVLD);

    /*
     * Setting the timer channel NRF_TIMER_CC_CHANNEL4 in the extended compare mode to clear
     * the timer.
     */
    nrfx_timer_extended_compare(&TIMER_LVLD, NRF_TIMER_CC_CHANNEL4, lvld_timer_val,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    // nrfx_timer_enable(&TIMER_LVLD);

}