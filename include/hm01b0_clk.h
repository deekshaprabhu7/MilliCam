
#include <helpers/nrfx_gppi.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include "gpio.h"



//#define NRFX_LOG_MODULE                 EXAMPLE
//#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
//#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL   3
//#include <nrfx_log.h>


/**
 * @defgroup nrfx_gppi_one_to_one_example One-to-one GPPI example
 * @{
 * @ingroup nrfx_gppi_examples
 *
 * @brief Example showing basic functionality of a nrfx_gppi helper.
 *
 * @details Application initializes nrfx_gpiote, nrfx_timer drivers and nrfx_gppi helper in a way that
 *          TIMER compare event is set up to be forwarded via PPI/DPPI to GPIOTE and toggle a pin.
 */

/** @brief Symbol specifying timer instance to be used. */
#define TIMER_INST_IDX 0

/** @brief Symbol specifying time in milliseconds to wait for handler execution. */
#define TIME_TO_WAIT_MS 100UL


int clk_init();