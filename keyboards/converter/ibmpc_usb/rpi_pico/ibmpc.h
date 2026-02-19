/*
Copyright 2024

This software is licensed with a Modified BSD License.
All of this is supposed to be Free Software, Open Source, DFSG-free,
GPL-compatible, and OK to use in both free and proprietary applications.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.

* Neither the name of the copyright holders nor the names of
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"

/*
 * IBM PC Keyboard Protocol Types
 */
enum ibmpc_protocol {
    IBMPC_PROTOCOL_UNKNOWN = 0,
    IBMPC_PROTOCOL_XT,      // Scan Code Set 1, 10-bit frame (start0, start1, 8 data)
    IBMPC_PROTOCOL_AT,      // Scan Code Set 2, 11-bit frame (start, 8 data, parity, stop)
};

/*
 * Error codes
 */
#define IBMPC_ERR_NONE      0
#define IBMPC_ERR_PARITY    0x10
#define IBMPC_ERR_NODATA    0x20
#define IBMPC_ERR_TIMEOUT   0x30

/*
 * PS/2 protocol constants (used in AT mode)
 */
#define IBMPC_ACK           0xFA
#define IBMPC_RESEND        0xFE
#define IBMPC_BAT_OK        0xAA
#define IBMPC_BAT_FAIL      0xFC
#define IBMPC_CMD_RESET     0xFF
#define IBMPC_CMD_SET_LED   0xED

#define IBMPC_LED_SCROLL_LOCK 0
#define IBMPC_LED_NUM_LOCK    1
#define IBMPC_LED_CAPS_LOCK   2

/*
 * GPIO macros for clock and data lines.
 *
 * NOTE: Uses gpio_set_pin_input_high() for correct pull-up behavior on
 * ChibiOS/RP2040. The AVR idiom (gpio_set_pin_input + gpio_write_pin_high)
 * does NOT enable pull-ups on ChibiOS.
 */
#define IBMPC_DATA_IN()     gpio_set_pin_input_high(IBMPC_DATA_PIN)
#define IBMPC_DATA_READ()   gpio_read_pin(IBMPC_DATA_PIN)
#define IBMPC_DATA_LO()              \
    do {                              \
        gpio_write_pin_low(IBMPC_DATA_PIN);  \
        gpio_set_pin_output(IBMPC_DATA_PIN); \
    } while (0)
#define IBMPC_DATA_HI()     gpio_set_pin_input_high(IBMPC_DATA_PIN)

#define IBMPC_CLOCK_IN()    gpio_set_pin_input_high(IBMPC_CLOCK_PIN)
#define IBMPC_CLOCK_READ()  gpio_read_pin(IBMPC_CLOCK_PIN)
#define IBMPC_CLOCK_LO()             \
    do {                              \
        gpio_write_pin_low(IBMPC_CLOCK_PIN);  \
        gpio_set_pin_output(IBMPC_CLOCK_PIN); \
    } while (0)
#define IBMPC_CLOCK_HI()    gpio_set_pin_input_high(IBMPC_CLOCK_PIN)

/* idle state: both lines released (pulled high by external/internal pull-ups) */
#define IBMPC_IDLE()    do { IBMPC_CLOCK_IN(); IBMPC_DATA_IN(); } while (0)

/* inhibit: hold clock low to prevent keyboard from sending */
#define IBMPC_INHIBIT() do { IBMPC_CLOCK_LO(); IBMPC_DATA_IN(); } while (0)

/*
 * Public API
 */

/* Initialize the IBMPC host: sets up GPIO, performs keyboard reset, starts
 * auto-detection, and enables the clock interrupt. */
void ibmpc_host_init(void);

/* Receive a byte from the keyboard. Returns 0 if no data available. */
uint8_t ibmpc_host_recv(void);

/* Send a byte to the keyboard (PS/2 mode only). Returns the keyboard's
 * response byte, or 0 on failure. No-op in XT mode. */
uint8_t ibmpc_host_send(uint8_t data);

/* Get the detected protocol type. */
enum ibmpc_protocol ibmpc_host_protocol(void);

/* Set keyboard LEDs (PS/2 mode only). No-op in XT mode. */
void ibmpc_host_set_led(uint8_t led);

/* Last error code */
extern uint8_t ibmpc_error;
