/* IBM PC Keyboard Converter (Pico) — ChibiOS HAL configuration overrides
 *
 * PAL_USE_CALLBACKS must be TRUE so that palEnableLineEvent() /
 * palSetLineCallback() are available for the clock-line falling-edge ISR
 * used by ibmpc_host.c.  All other settings inherit from the board-common
 * halconf.h via the normal include chain.
 */
#pragma once

/* Enable PAL event callbacks (required for PS/2 / XT clock interrupt) */
#define PAL_USE_CALLBACKS TRUE

/* Pull in the rest of the common HAL configuration */
#include_next "halconf.h"
