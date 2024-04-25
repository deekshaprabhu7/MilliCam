#ifndef HM01B0_LVLD_TIMER_H
#define HM01B0_LVLD_TIMER_H

#include <nrfx_timer.h>
#include "hm01b0_ble_defines.h"

#define TIMER_INST_LVLD_IDX 1

extern uint32_t lvld_timer_val;
extern nrfx_timer_t TIMER_LVLD;

void timer_lvld_event_handler(nrf_timer_event_t event_type, void* p_context);
void lvld_timer_init(void);

#endif