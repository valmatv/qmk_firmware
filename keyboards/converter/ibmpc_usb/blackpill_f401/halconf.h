/* IBM PC Keyboard Converter (BlackPill F401) — ChibiOS HAL configuration
 *
 * PAL_USE_CALLBACKS must be TRUE so that palEnableLineEvent() /
 * palSetLineCallback() are available for the clock-line falling-edge ISR
 * used by ibmpc_host.c on the STM32F401.
 *
 * HAL_USE_PAL must be TRUE to enable the PAL driver (port abstraction layer).
 *
 * All other settings inherit from the board-common halconf.h.
 */
#pragma once

/* Enable PAL driver (required for GPIO + events) */
#define HAL_USE_PAL TRUE

/* Enable PAL event callbacks (required for PS/2 / XT clock line interrupt) */
#define PAL_USE_CALLBACKS TRUE

/* Pull in the rest of the common HAL configuration */
#include_next <halconf.h>
