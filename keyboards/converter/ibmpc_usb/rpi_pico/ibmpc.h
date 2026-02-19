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
 *
 * Numeric values are kept compatible with TMK ibmpc.hpp IBMPC_PROTOCOL_* defines
 * so that bit-tests (AT vs XT family) still work.
 */
enum ibmpc_protocol {
    IBMPC_PROTOCOL_UNKNOWN  = 0x00,
    IBMPC_PROTOCOL_AT       = 0x10,  // Scan Code Set 2, 11-bit frame
    IBMPC_PROTOCOL_AT_Z150  = 0x11,  // Zenith Z-150 AT (no ID response)
    IBMPC_PROTOCOL_XT       = 0x20,  // Scan Code Set 1, 10-bit frame
    IBMPC_PROTOCOL_XT_IBM   = 0x21,  // IBM XT (detected via ID)
    IBMPC_PROTOCOL_XT_CLONE = 0x22,  // Clone XT
};

/* Convenience bit-masks matching TMK protocol grouping */
#define IBMPC_PROTOCOL_IS_AT(p)  (((p) & 0xF0) == 0x10)
#define IBMPC_PROTOCOL_IS_XT(p)  (((p) & 0xF0) == 0x20)

/*
 * Keyboard kind — identifies the detected keyboard family.
 * Maps directly from the 2-byte ID returned after the F2 command.
 */
typedef enum {
    PC_NONE     = 0,
    PC_XT,          // XT (Scan Code Set 1) keyboard
    PC_AT,          // AT/PS2 (Scan Code Set 2) keyboard
    PC_TERMINAL,    // Terminal (Scan Code Set 3) keyboard
    PC_MOUSE,       // PS/2 Mouse (not supported as a keyboard)
} keyboard_kind_t;

#define KEYBOARD_KIND_STR(k) \
    ((k) == PC_XT       ? "XT"       : \
     (k) == PC_AT       ? "AT"       : \
     (k) == PC_TERMINAL ? "TERMINAL" : \
     (k) == PC_MOUSE    ? "MOUSE"    : "NONE")

/*
 * Error codes (match TMK ibmpc.hpp IBMPC_ERR_* values)
 */
#define IBMPC_ERR_NONE        0x00
#define IBMPC_ERR_PARITY      0x01  // PS/2 parity error
#define IBMPC_ERR_PARITY_AA   0x02  // Parity error on 0xAA (AT/XT auto-switch signal)
#define IBMPC_ERR_SEND        0x10  // Host-to-device send error
#define IBMPC_ERR_TIMEOUT     0x20  // Receive timeout
#define IBMPC_ERR_FULL        0x40  // Ring buffer full — data dropped
#define IBMPC_ERR_ILLEGAL     0x80  // Illegal/unexpected byte in ISR

/*
 * PS/2 protocol constants (used in AT mode)
 */
#define IBMPC_ACK           0xFA
#define IBMPC_RESEND        0xFE
#define IBMPC_BAT_OK        0xAA
#define IBMPC_BAT_FAIL      0xFC
#define IBMPC_CMD_RESET     0xFF
#define IBMPC_CMD_ECHO      0xEE
#define IBMPC_CMD_SET_LED   0xED
#define IBMPC_CMD_SCANCODE  0xF0  // Select/query scan code set
#define IBMPC_CMD_ID        0xF2  // Read keyboard ID
#define IBMPC_CMD_ENABLE    0xF4  // Enable scanning
#define IBMPC_CMD_DISABLE   0xF5  // Disable scanning
#define IBMPC_CMD_DEFAULTS  0xF6  // Set default parameters
#define IBMPC_CMD_ALLM_TMB  0xF8  // Set all keys make/break (CS3)
#define IBMPC_CMD_RESEND    0xFE

/*
 * LED bit positions (match TMK ibmpc.hpp IBMPC_LED_* values)
 */
#define IBMPC_LED_SCROLL_LOCK 0
#define IBMPC_LED_NUM_LOCK    1
#define IBMPC_LED_CAPS_LOCK   2

/*
 * Optional feature guards — define these in config.h to enable optional scan-code
 * decoding paths.  Mirrors the TMK config.h defines.
 */
// #define CS2_80CODE_SUPPORT   // Tandberg TDV 5020 0x80-prefixed codes
// #define G80_2551_SUPPORT     // Cherry G80-2551 0x80-prefixed CS3 codes
// #define SIEMENS_PCD_SUPPORT  // Siemens PCD 2 keyboard

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

/* Initialise GPIO and the interrupt pin; does NOT start the ISR or run
 * protocol detection.  The matrix.c state machine drives initialisation. */
void ibmpc_host_init(void);

/* Non-blocking receive.  Returns data byte (0x00–0xFF) or -1 if empty. */
int16_t ibmpc_host_recv(void);

/* Blocking receive with timeout_ms deadline.
 * Returns data byte or -1 on timeout.  Mirrors TMK read_wait(). */
int16_t ibmpc_host_recv_wait(uint16_t timeout_ms);

/* Send a byte (PS/2 only). Returns keyboard response byte or 0 on failure.
 * No-op in XT mode. */
uint8_t ibmpc_host_send(uint8_t data);

/* Send F2, read 2-byte keyboard ID.  Sentinels: 0xFFFF=no kb/XT,
 * 0xFFFE=broken PS/2, 0x0000=IBM 84-key, 0x00FF=mouse. */
uint16_t ibmpc_host_keyboard_id(void);

/* Release the clock line and enable the clock interrupt (start receiving). */
void ibmpc_host_enable(void);

/* Inhibit the keyboard: hold clock low, disable interrupt (stop receiving).
 * Also used for XT soft-reset when data is then pulled low. */
void ibmpc_host_disable(void);

/* Flush the receive ring buffer and clear isr_debug. */
void ibmpc_host_isr_clear(void);

/* Read and atomically clear the error register.
 * Returns one of IBMPC_ERR_*. */
uint8_t ibmpc_host_error(void);

/* Override the detected protocol and switch the ISR accordingly.
 * Used by the state machine after keyboard-kind identification. */
void ibmpc_host_set_protocol(enum ibmpc_protocol proto);

/* Get the currently active protocol. */
enum ibmpc_protocol ibmpc_host_protocol(void);

/* Set keyboard LEDs (PS/2 / terminal only; no-op for XT and mouse). */
void ibmpc_host_set_led(uint8_t led);

/* Error register (also accessible via ibmpc_host_error()). */
extern uint8_t  ibmpc_error;
/* ISR debug capture (last ISR state at point of error). */
extern uint16_t ibmpc_isr_debug;
