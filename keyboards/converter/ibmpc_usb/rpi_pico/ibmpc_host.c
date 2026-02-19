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

/*
 * IBM PC Keyboard Protocol Host Driver for RP2040 (ChibiOS)
 *
 * Supports both XT (Scan Code Set 1) and AT/PS2 (Scan Code Set 2) protocols
 * with auto-detection on startup.
 *
 * Detection strategy:
 *   1. Inhibit the bus (pull clock low) for 100ms, then release.
 *   2. Try to send PS/2 Reset command (0xFF).
 *   3. If we get ACK (0xFA) -> AT/PS2 protocol detected.
 *   4. If timeout (no ACK) -> XT protocol assumed.
 *   5. As a fallback, count clock pulses per frame during initial reception:
 *      - 10 clocks = XT frame
 *      - 11 clocks = PS/2 frame
 *
 * Frame formats:
 *   XT:   [Start(0)] Start(1) D0 D1 D2 D3 D4 D5 D6 D7
 *         10 clock pulses, data sampled on falling edge, no parity.
 *         Note: Start(0) may be absent on some clones.
 *
 *   PS/2: Start(0) D0 D1 D2 D3 D4 D5 D6 D7 Parity Stop(1)
 *         11 clock pulses, data sampled on falling edge, odd parity.
 */

#include <stdbool.h>
#include "ch.h"
#include "hal.h"
#include "gpio.h"
#include "ibmpc.h"
#include "wait.h"
#include "print.h"
#include "debug.h"
#include "timer.h"

/*--------------------------------------------------------------------
 * Ring buffer to store scan codes from keyboard
 *------------------------------------------------------------------*/
#define PBUF_SIZE 32
static uint8_t pbuf[PBUF_SIZE];
static uint8_t pbuf_head = 0;
static uint8_t pbuf_tail = 0;

static inline void pbuf_enqueue(uint8_t data) {
    chSysLockFromISR();
    uint8_t next = (pbuf_head + 1) % PBUF_SIZE;
    if (next != pbuf_tail) {
        pbuf[pbuf_head] = data;
        pbuf_head       = next;
    } else {
        dprintf("pbuf: full\n");
    }
    chSysUnlockFromISR();
}

static inline uint8_t pbuf_dequeue(void) {
    uint8_t val = 0;
    chSysLock();
    if (pbuf_head != pbuf_tail) {
        val       = pbuf[pbuf_tail];
        pbuf_tail = (pbuf_tail + 1) % PBUF_SIZE;
    }
    chSysUnlock();
    return val;
}

static inline bool pbuf_has_data(void) {
    chSysLock();
    bool has_data = (pbuf_head != pbuf_tail);
    chSysUnlock();
    return has_data;
}

static inline void pbuf_clear(void) {
    chSysLock();
    pbuf_head = pbuf_tail = 0;
    chSysUnlock();
}

/*--------------------------------------------------------------------
 * Protocol state
 *------------------------------------------------------------------*/
static volatile enum ibmpc_protocol detected_protocol = IBMPC_PROTOCOL_UNKNOWN;
uint8_t ibmpc_error = IBMPC_ERR_NONE;
/* isr_debug mirrors TMK ibmpc.isr_debug — stores last ISR state on error */
uint16_t ibmpc_isr_debug = 0;

/*--------------------------------------------------------------------
 * XT protocol ISR state machine
 *
 * XT frame: start(0) start(1) D0 D1 D2 D3 D4 D5 D6 D7
 * Data sampled on falling clock edge.
 * start(0) is low on data line, start(1) is high.
 * Some clones skip start(0).
 *------------------------------------------------------------------*/
static void xt_isr(void) {
    static enum { XT_START, XT_BIT0, XT_BIT1, XT_BIT2, XT_BIT3, XT_BIT4, XT_BIT5, XT_BIT6, XT_BIT7 } state = XT_START;
    static uint8_t data = 0;

    uint8_t dbit = IBMPC_DATA_READ();

    switch (state) {
        case XT_START:
            // Ignore start(0) bit (data=low), wait for start(1) bit (data=high)
            if (!dbit) return;
            break;
        case XT_BIT0 ... XT_BIT7:
            data >>= 1;
            if (dbit) data |= 0x80;
            break;
    }
    if (state++ == XT_BIT7) {
        pbuf_enqueue(data);
        state = XT_START;
        data  = 0;
    }
}

/*--------------------------------------------------------------------
 * PS/2 (AT) protocol ISR state machine
 *
 * PS/2 frame: Start(0) D0 D1 D2 D3 D4 D5 D6 D7 Parity Stop(1)
 * Data sampled on falling clock edge. Odd parity.
 *------------------------------------------------------------------*/
static void ps2_isr(void) {
    static enum {
        PS2_INIT,
        PS2_START,
        PS2_BIT0, PS2_BIT1, PS2_BIT2, PS2_BIT3,
        PS2_BIT4, PS2_BIT5, PS2_BIT6, PS2_BIT7,
        PS2_PARITY,
        PS2_STOP,
    } state             = PS2_INIT;
    static uint8_t data   = 0;
    static uint8_t parity = 1;

    // Return unless falling edge (safety check if using PCINT-like interrupts)
    if (IBMPC_CLOCK_READ()) {
        return;
    }

    state++;
    switch (state) {
        case PS2_START:
            if (IBMPC_DATA_READ()) goto ERROR;
            break;
        case PS2_BIT0:
        case PS2_BIT1:
        case PS2_BIT2:
        case PS2_BIT3:
        case PS2_BIT4:
        case PS2_BIT5:
        case PS2_BIT6:
        case PS2_BIT7:
            data >>= 1;
            if (IBMPC_DATA_READ()) {
                data |= 0x80;
                parity++;
            }
            break;
        case PS2_PARITY:
            if (IBMPC_DATA_READ()) {
                if (!(parity & 0x01)) goto ERROR;
            } else {
                if (parity & 0x01) goto ERROR;
            }
            break;
        case PS2_STOP:
            if (!IBMPC_DATA_READ()) goto ERROR;
            pbuf_enqueue(data);
            goto DONE;
        default:
            goto ERROR;
    }
    return;
ERROR:
    ibmpc_isr_debug = state;
    /* AT/XT Auto-Switching: a parity error on 0xAA means an XT keyboard sent
     * its power-on code using XT framing, which looks like a bad PS/2 byte. */
    ibmpc_error = (data == 0xAA) ? IBMPC_ERR_PARITY_AA : IBMPC_ERR_PARITY;
DONE:
    state  = PS2_INIT;
    data   = 0;
    parity = 1;
}

/* detect_isr, detect_reset_isr_state, and their state variables are only
 * used by detect_protocol() which is disabled.  Guard them to avoid
 * -Werror=unused-function build failures.
 */
#if 0
/*--------------------------------------------------------------------
 * Detection-phase ISR
 *
 * During detection, we count clock pulses per frame using a timeout
 * to detect frame boundaries. If we see 10 clocks per frame -> XT.
 * If we see 11 clocks per frame -> PS/2.
 *------------------------------------------------------------------*/
static volatile uint8_t detect_clock_count = 0;
static volatile uint8_t detect_data_byte   = 0;
static volatile bool    detect_frame_ready = false;

static void detect_isr(void) {
    static uint8_t bit_count = 0;
    static uint8_t data      = 0;

    uint8_t dbit = IBMPC_DATA_READ();

    bit_count++;

    // Accumulate data bits (skip start bits, collect up to 8 data bits)
    if (bit_count >= 2 && bit_count <= 9) {
        data >>= 1;
        if (dbit) data |= 0x80;
    }

    // We'll let the main loop check for frame completion via timeout
    detect_clock_count = bit_count;
    detect_data_byte   = data;

    // Reset on long idle (will be handled by main loop calling detect_reset)
    (void)data;
}

static void detect_reset_isr_state(void) {
    detect_clock_count = 0;
    detect_data_byte   = 0;
    detect_frame_ready = false;
}
#endif /* 0 — detection-phase helpers */

/*--------------------------------------------------------------------
 * ChibiOS PAL interrupt callback
 *------------------------------------------------------------------*/
typedef void (*isr_func_t)(void);
static volatile isr_func_t active_isr = NULL;

static void ibmpc_pal_callback(void *arg) {
    (void)arg;
    if (active_isr) {
        active_isr();
    }
}

/*--------------------------------------------------------------------
 * Interrupt control
 *------------------------------------------------------------------*/
static void ibmpc_int_init(void) {
    palSetLineMode(IBMPC_CLOCK_PIN, PAL_MODE_INPUT_PULLUP);
}

static void ibmpc_int_on(void) {
    palEnableLineEvent(IBMPC_CLOCK_PIN, PAL_EVENT_MODE_FALLING_EDGE);
    palSetLineCallback(IBMPC_CLOCK_PIN, ibmpc_pal_callback, NULL);
}

static void ibmpc_int_off(void) {
    palDisableLineEvent(IBMPC_CLOCK_PIN);
}

/*--------------------------------------------------------------------
 * Wait helpers for PS/2 bit-bang send
 *------------------------------------------------------------------*/
static inline uint16_t wait_clock_lo(uint16_t us) {
    while (IBMPC_CLOCK_READ() && us) {
        wait_us(1);
        us--;
    }
    return us;
}

static inline uint16_t wait_clock_hi(uint16_t us) {
    while (!IBMPC_CLOCK_READ() && us) {
        wait_us(1);
        us--;
    }
    return us;
}

static inline uint16_t wait_data_lo(uint16_t us) {
    while (IBMPC_DATA_READ() && us) {
        wait_us(1);
        us--;
    }
    return us;
}

static inline uint16_t wait_data_hi(uint16_t us) {
    while (!IBMPC_DATA_READ() && us) {
        wait_us(1);
        us--;
    }
    return us;
}

/*--------------------------------------------------------------------
 * PS/2 host-to-device send (bit-bang)
 *
 * Used in AT/PS2 mode for reset, LED commands, etc.
 * Follows the PS/2 protocol spec for host-to-device communication.
 *------------------------------------------------------------------*/
static uint8_t ibmpc_ps2_send(uint8_t data) {
    bool parity_bit = true;
    ibmpc_error = IBMPC_ERR_NONE;

    ibmpc_int_off();

    // Inhibit: pull clock low for at least 100us
    IBMPC_INHIBIT();
    wait_us(100);

    // Request to Send: pull data low, then release clock
    IBMPC_DATA_LO();
    IBMPC_CLOCK_HI();
    if (!wait_clock_lo(10000)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }

    // Clock out 8 data bits (LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            parity_bit = !parity_bit;
            IBMPC_DATA_HI();
        } else {
            IBMPC_DATA_LO();
        }
        if (!wait_clock_hi(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
        if (!wait_clock_lo(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
    }

    // Parity bit
    wait_us(15);
    if (parity_bit) {
        IBMPC_DATA_HI();
    } else {
        IBMPC_DATA_LO();
    }
    if (!wait_clock_hi(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
    if (!wait_clock_lo(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }

    // Stop bit
    wait_us(15);
    IBMPC_DATA_HI();

    // Wait for ACK from device
    if (!wait_data_lo(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
    if (!wait_clock_lo(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }

    // Wait for idle
    if (!wait_clock_hi(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
    if (!wait_data_hi(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }

    IBMPC_IDLE();
    ibmpc_int_on();

    // Wait for response byte
    {
        uint8_t retry = 25;
        while (retry-- && !pbuf_has_data()) {
            wait_ms(1);
        }
        return pbuf_dequeue();
    }

ERROR:
    IBMPC_IDLE();
    ibmpc_int_on();
    return 0;
}

/* detect_protocol() removed: protocol detection is now driven by the
 * matrix.c state machine (ibmpc_host_enable / set_protocol / keyboard_id).
 * This stub keeps the compiler happy if any reference remains.
 * TODO: delete this comment block once confirmed unused.
 */
#if 0
static enum ibmpc_protocol detect_protocol(void) {
    uint8_t response;

    dprintf("IBMPC: detecting protocol...\n");

    // Inhibit bus to reset keyboard communication
    IBMPC_INHIBIT();
    wait_ms(100);
    IBMPC_IDLE();

    // Brief settling time
    wait_ms(50);

    // Enable interrupt with PS/2 ISR to catch ACK
    active_isr = ps2_isr;
    ibmpc_int_on();

    // Try sending PS/2 Reset command
    response = ibmpc_ps2_send(IBMPC_CMD_RESET);
    dprintf("IBMPC: reset response=0x%02X\n", response);

    if (response == IBMPC_ACK) {
        // Got ACK - this is a PS/2 keyboard
        dprintf("IBMPC: AT/PS2 detected (got ACK)\n");

        // Wait for BAT result (0xAA = OK, 0xFC = error)
        // BAT can take up to 2.5 seconds
        uint16_t retry = 2500;
        while (retry-- && !pbuf_has_data()) {
            wait_ms(1);
        }
        uint8_t bat = pbuf_dequeue();
        dprintf("IBMPC: BAT=0x%02X\n", bat);
        // Zenith Z-150 does not respond to ID command — keep as plain AT
        return IBMPC_PROTOCOL_AT;
    }

    // No ACK - likely XT keyboard. Try the XT reset approach.
    dprintf("IBMPC: no ACK, trying XT reset...\n");
    ibmpc_int_off();
    pbuf_clear();

    // XT soft reset: pull data and clock low for 20ms
    IBMPC_DATA_LO();
    IBMPC_CLOCK_LO();
    wait_ms(20);

    // Release and listen
    IBMPC_IDLE();
    wait_ms(50);

    // Switch to detection ISR to count bits per frame
    detect_reset_isr_state();
    active_isr = detect_isr;
    ibmpc_int_on();

    // Wait for first frame (XT keyboards send BAT/power-on scan codes)
    uint16_t timeout = 3000; // 3 seconds
    uint8_t last_count = 0;
    uint16_t idle_ms = 0;

    while (timeout--) {
        wait_ms(1);
        uint8_t count = detect_clock_count;

        if (count > 0 && count == last_count) {
            idle_ms++;
            // If no new clock for 2ms after receiving some clocks, frame is complete
            if (idle_ms >= 2) {
                dprintf("IBMPC: detect frame clocks=%d\n", count);
                if (count >= 10 && count <= 11) {
                    ibmpc_int_off();
                    if (count == 11) {
                        dprintf("IBMPC: AT/PS2 detected (11-bit frame)\n");
                        return IBMPC_PROTOCOL_AT;
                    } else {
                        dprintf("IBMPC: XT detected (10-bit frame)\n");
                        return IBMPC_PROTOCOL_XT_CLONE;
                    }
                } else if (count >= 8 && count <= 9) {
                    // Some XT clones send 9-bit frames (skip start(0))
                    ibmpc_int_off();
                    dprintf("IBMPC: XT detected (%d-bit frame)\n", count);
                    return IBMPC_PROTOCOL_XT_CLONE;
                }
                // Unexpected count, reset and keep listening
                detect_reset_isr_state();
                idle_ms = 0;
            }
        } else {
            idle_ms = 0;
        }
        last_count = count;
    }

    ibmpc_int_off();

    // Timeout with no response - default to XT (keyboard may need manual key press)
    dprintf("IBMPC: detection timeout, defaulting to XT\n");
    return IBMPC_PROTOCOL_XT_CLONE;
}
#endif /* 0 — detect_protocol() */

/*--------------------------------------------------------------------
 * Public API
 *------------------------------------------------------------------*/

void ibmpc_host_init(void) {
    /* Minimal hardware setup.  Protocol detection and ISR start are delegated
     * to the matrix.c state machine via ibmpc_host_enable() / host_disable(). */
    ibmpc_int_init();

    pbuf_clear();
    ibmpc_error     = IBMPC_ERR_NONE;
    ibmpc_isr_debug = 0;
    detected_protocol = IBMPC_PROTOCOL_UNKNOWN;

    /* Pre-load PS/2 ISR so ibmpc_host_enable() starts in AT mode by default.
     * ibmpc_host_set_protocol() switches to xt_isr if READ_ID reveals XT. */
    active_isr = ps2_isr;

    /* Inhibit the keyboard until the state machine is ready */
    IBMPC_INHIBIT();

    dprintf("IBMPC: host init (state machine will drive detection)\n");
}

/*
 * ibmpc_host_recv() — non-blocking receive.
 * Returns 0 if no data available (caller must check pbuf state separately).
 */
int16_t ibmpc_host_recv(void) {
    if (pbuf_has_data()) {
        return (int16_t)(uint8_t)pbuf_dequeue();
    }
    return -1;
}

/*
 * ibmpc_host_recv_wait() — blocking receive with timeout.
 * Returns byte value (0x00–0xFF) or -1 on timeout.
 * Mirrors TMK IBMPCConverter::read_wait().
 */
int16_t ibmpc_host_recv_wait(uint16_t timeout_ms) {
    uint16_t start = timer_read();
    int16_t code;
    while ((code = ibmpc_host_recv()) == -1 && timer_elapsed(start) < timeout_ms);
    return code;
}

/*
 * ibmpc_host_keyboard_id() — send 0xF2 and read the 2-byte keyboard ID.
 * Returns 16-bit ID or a sentinel:
 *   0xFFFF  No keyboard / XT (no ACK)
 *   0xFFFE  Broken PS/2 (ACK but no first byte)
 *   0x0000  IBM 84-key AT (ACK, no bytes follow)
 *   0x00FF  PS/2 Mouse
 * Mirrors TMK read_keyboard_id().
 */
uint16_t ibmpc_host_keyboard_id(void) {
    uint16_t id   = 0;
    int16_t  code = 0;

    /* protocol-based shortcuts */
    if (detected_protocol == IBMPC_PROTOCOL_AT_Z150)  return 0xFFFD;
    if (detected_protocol == IBMPC_PROTOCOL_XT_IBM)   return 0xFFFC;
    if (detected_protocol == IBMPC_PROTOCOL_XT_CLONE) return 0xFFFB;

    code = ibmpc_host_send(IBMPC_CMD_ID);
    if (code == 0)    { id = 0xFFFF; goto DONE; }  // no ACK → XT or no keyboard
    if (code != IBMPC_ACK) { id = 0xFFFE; goto DONE; }  // broken?

    /* IBM AT 84-key: ACK with no bytes following */
    code = ibmpc_host_recv_wait(500);
    if (code == -1)   { id = 0x0000; goto DONE; }  // IBM 84-key
    id = (uint16_t)((code & 0xFF) << 8);

    /* Mouse sends only one byte (0x00); that gives id=0x00FF below */
    code = ibmpc_host_recv_wait(500);
    id |= (uint16_t)(code & 0xFF);

DONE:
    return id;
}

/*
 * ibmpc_host_enable() / ibmpc_host_disable()
 * Control clock inhibit — used during XT reset and AT/XT auto-switching.
 */
void ibmpc_host_enable(void) {
    IBMPC_IDLE();
    ibmpc_int_on();
}

void ibmpc_host_disable(void) {
    ibmpc_int_off();
    /* Clock Lo / Data Hi = PS/2 inhibit; also XT soft-reset when Data later goes Lo */
    IBMPC_CLOCK_LO();
    IBMPC_DATA_HI();
}

/*
 * ibmpc_host_isr_clear() — flush the receive ring buffer.
 * Called before sending commands during initialisation.
 */
void ibmpc_host_isr_clear(void) {
    pbuf_clear();
    ibmpc_isr_debug = 0;
}

/*
 * ibmpc_host_error() — read and atomically clear the error register.
 */
uint8_t ibmpc_host_error(void) {
    uint8_t err;
    chSysLock();
    err        = ibmpc_error;
    ibmpc_error = IBMPC_ERR_NONE;
    chSysUnlock();
    return err;
}

uint8_t ibmpc_host_send(uint8_t data) {
    if (!IBMPC_PROTOCOL_IS_AT(detected_protocol)) {
        // XT is receive-only
        return 0;
    }
    return ibmpc_ps2_send(data);
}

enum ibmpc_protocol ibmpc_host_protocol(void) {
    return detected_protocol;
}

void ibmpc_host_set_protocol(enum ibmpc_protocol proto) {
    detected_protocol = proto;
    /* Switch ISR to match */
    ibmpc_int_off();
    pbuf_clear();
    if (IBMPC_PROTOCOL_IS_AT(proto)) {
        active_isr = ps2_isr;
    } else {
        active_isr = xt_isr;
    }
    ibmpc_int_on();
}

void ibmpc_host_set_led(uint8_t led) {
    if (!IBMPC_PROTOCOL_IS_AT(detected_protocol)) {
        return; // XT keyboards don't support LED commands
    }
    ibmpc_host_send(IBMPC_CMD_SET_LED);
    ibmpc_host_send(led);
}
