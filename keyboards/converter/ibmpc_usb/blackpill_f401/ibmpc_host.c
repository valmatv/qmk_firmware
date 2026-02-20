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
 * IBM PC Keyboard Protocol Host Driver — STM32F401 (ChibiOS/PAL)
 *
 * Ported from the RP2040 version.  The host logic (ring buffer, XT/PS2 ISR
 * state machines, send/receive protocol) is identical.  The only platform
 * differences are:
 *
 *   1. Interrupt registration order: on STM32/ChibiOS the PAL callback
 *      MUST be registered with palSetLineCallback() BEFORE the event is
 *      enabled with palEnableLineEvent().  The RP2040 port tolerates either
 *      order, but STM32 does not.
 *
 *   2. ibmpc_int_init() calls palSetLineMode() directly (same as RP2040)
 *      to configure the clock pin as input with pull-up.
 *
 * Supports:
 *   - XT  (Scan Code Set 1, 10-bit frame, 83/84/86-key)
 *   - AT/PS2 (Scan Code Set 2, 11-bit frame)
 *   - Terminal keyboards (Scan Code Set 3) via matrix.c state machine
 *
 * Protocol detection is driven by the matrix.c state machine
 * (ibmpc_host_enable / ibmpc_host_set_protocol / ibmpc_host_keyboard_id).
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
            /* Ignore start(0) bit (data=low), wait for start(1) bit (data=high) */
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

    /* Safety check: only process on falling edge (clock should be low) */
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
     * its power-on code using XT framing, which looks like a PS/2 byte. */
    ibmpc_error = (data == 0xAA) ? IBMPC_ERR_PARITY_AA : IBMPC_ERR_PARITY;
DONE:
    state  = PS2_INIT;
    data   = 0;
    parity = 1;
}

/*--------------------------------------------------------------------
 * ChibiOS PAL interrupt callback (called from EXTI IRQ handler)
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
 *
 * STM32/ChibiOS IMPORTANT: palSetLineCallback() MUST be called before
 * palEnableLineEvent().  The RP2040 port tolerates the reverse order,
 * but STM32 EXTI will miss the first edge if the callback is not yet
 * registered when the event is armed.
 *------------------------------------------------------------------*/
static void ibmpc_int_init(void) {
    palSetLineMode(IBMPC_CLOCK_PIN, PAL_MODE_INPUT_PULLUP);
}

static void ibmpc_int_on(void) {
    /* Register callback FIRST, then arm the falling-edge event */
    palSetLineCallback(IBMPC_CLOCK_PIN, ibmpc_pal_callback, NULL);
    palEnableLineEvent(IBMPC_CLOCK_PIN, PAL_EVENT_MODE_FALLING_EDGE);
}

static void ibmpc_int_off(void) {
    palDisableLineEvent(IBMPC_CLOCK_PIN);
}

/*--------------------------------------------------------------------
 * Wait helpers for PS/2 bit-bang send
 *------------------------------------------------------------------*/
static inline uint16_t wait_clock_lo(uint16_t us) {
    while (IBMPC_CLOCK_READ() && us) { wait_us(1); us--; }
    return us;
}

static inline uint16_t wait_clock_hi(uint16_t us) {
    while (!IBMPC_CLOCK_READ() && us) { wait_us(1); us--; }
    return us;
}

static inline uint16_t wait_data_lo(uint16_t us) {
    while (IBMPC_DATA_READ() && us) { wait_us(1); us--; }
    return us;
}

static inline uint16_t wait_data_hi(uint16_t us) {
    while (!IBMPC_DATA_READ() && us) { wait_us(1); us--; }
    return us;
}

/*--------------------------------------------------------------------
 * PS/2 host-to-device send (bit-bang)
 *
 * Used in AT/PS2 mode for reset, LED commands, etc.
 * Follows the PS/2 protocol spec for host-to-device communication.
 *------------------------------------------------------------------*/
static uint8_t ibmpc_ps2_send(uint8_t data) {
    bool    parity_bit = true;
    uint8_t send_step  = 0;
    ibmpc_error = IBMPC_ERR_NONE;

    dprintf("PS2 send 0x%02X\n", data);

    ibmpc_int_off();

    /* Inhibit: pull clock low for at least 100 µs */
    IBMPC_INHIBIT();
    wait_us(100);

    /* Request to Send: pull data low, then release clock */
    IBMPC_DATA_LO();
    IBMPC_CLOCK_HI();
    send_step = 1;
    if (!wait_clock_lo(10000)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }

    /* Clock out 8 data bits (LSB first) */
    for (uint8_t i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            parity_bit = !parity_bit;
            IBMPC_DATA_HI();
        } else {
            IBMPC_DATA_LO();
        }
        send_step = (uint8_t)(2 + i * 2);
        if (!wait_clock_hi(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
        send_step = (uint8_t)(3 + i * 2);
        if (!wait_clock_lo(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
    }

    /* Parity bit */
    wait_us(15);
    if (parity_bit) {
        IBMPC_DATA_HI();
    } else {
        IBMPC_DATA_LO();
    }
    send_step = 18;
    if (!wait_clock_hi(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
    send_step = 19;
    if (!wait_clock_lo(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }

    /* Stop bit */
    wait_us(15);
    IBMPC_DATA_HI();

    /* Wait for ACK from device */
    send_step = 20;
    if (!wait_data_lo(50))  { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
    send_step = 21;
    if (!wait_clock_lo(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }

    /* Wait for idle */
    send_step = 22;
    if (!wait_clock_hi(50)) { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }
    send_step = 23;
    if (!wait_data_hi(50))  { ibmpc_error = IBMPC_ERR_TIMEOUT; goto ERROR; }

    IBMPC_IDLE();
    ibmpc_int_on();

    /* Wait for keyboard response byte */
    {
        uint8_t retry = 25;
        while (retry-- && !pbuf_has_data()) { wait_ms(1); }
        uint8_t resp = pbuf_dequeue();
        dprintf("PS2 resp 0x%02X\n", resp);
        return resp;
    }

ERROR:
    dprintf("PS2 send timeout at step=%d\n", send_step);
    IBMPC_IDLE();
    ibmpc_int_on();
    return 0;
}

/*--------------------------------------------------------------------
 * Public API
 *------------------------------------------------------------------*/

void ibmpc_host_init(void) {
    /* Minimal hardware setup.  Protocol detection and ISR start are
     * delegated to the matrix.c state machine via ibmpc_host_enable()
     * and ibmpc_host_disable(). */
    ibmpc_int_init();

    pbuf_clear();
    ibmpc_error       = IBMPC_ERR_NONE;
    ibmpc_isr_debug   = 0;
    detected_protocol = IBMPC_PROTOCOL_UNKNOWN;

    /* Pre-load PS/2 ISR so ibmpc_host_enable() starts in AT mode by default.
     * ibmpc_host_set_protocol() switches to xt_isr if READ_ID reveals XT. */
    active_isr = ps2_isr;

    /* Inhibit the keyboard until the state machine is ready */
    IBMPC_INHIBIT();

    dprintf("IBMPC: host init (STM32F401, state machine drives detection)\n");
}

/*
 * ibmpc_host_recv() — non-blocking receive.
 * Returns scan code (0x00–0xFF) or -1 if buffer is empty.
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
 */
int16_t ibmpc_host_recv_wait(uint16_t timeout_ms) {
    uint16_t start = timer_read();
    int16_t code;
    while ((code = ibmpc_host_recv()) == -1 && timer_elapsed(start) < timeout_ms);
    if (code == -1) dprintf("IBMPC: recv_wait timeout (%ums)\n", timeout_ms);
    return code;
}

/*
 * ibmpc_host_keyboard_id() — send 0xF2 and read the 2-byte keyboard ID.
 * Returns 16-bit ID or sentinel values:
 *   0xFFFF  No keyboard / XT (no ACK)
 *   0xFFFE  Broken PS/2 (ACK but no first byte)
 *   0x0000  IBM 84-key AT (ACK, no bytes follow)
 *   0x00FF  PS/2 Mouse
 */
uint16_t ibmpc_host_keyboard_id(void) {
    uint16_t id   = 0;
    int16_t  code = 0;

    /* Protocol-based shortcuts — skip the ID command for known XT variants */
    if (detected_protocol == IBMPC_PROTOCOL_AT_Z150)  { dprintf("IBMPC: kbd_id shortcut Z150->0xFFFD\n");   return 0xFFFD; }
    if (detected_protocol == IBMPC_PROTOCOL_XT_IBM)   { dprintf("IBMPC: kbd_id shortcut XT_IBM->0xFFFC\n"); return 0xFFFC; }
    if (detected_protocol == IBMPC_PROTOCOL_XT_CLONE) { dprintf("IBMPC: kbd_id shortcut XT_CLONE->0xFFFB\n"); return 0xFFFB; }

    dprintf("IBMPC: kbd_id sending CMD_ID\n");
    code = ibmpc_host_send(IBMPC_CMD_ID);
    dprintf("IBMPC: kbd_id CMD_ID resp=0x%02X\n", (uint8_t)code);
    if (code == 0)         { id = 0xFFFF; dprintf("IBMPC: kbd_id no ACK (XT/absent)\n");      goto DONE; }
    if (code != IBMPC_ACK) { id = 0xFFFE; dprintf("IBMPC: kbd_id unexpected resp (broken)\n"); goto DONE; }

    /* IBM AT 84-key: ACK with no bytes following */
    code = ibmpc_host_recv_wait(500);
    if (code == -1) { id = 0x0000; dprintf("IBMPC: kbd_id no bytes after ACK (AT 84-key)\n"); goto DONE; }
    dprintf("IBMPC: kbd_id byte1=0x%02X\n", (uint8_t)code);
    id = (uint16_t)((code & 0xFF) << 8);

    /* Mouse sends only one byte (0x00); gives id=0x00FF */
    code = ibmpc_host_recv_wait(500);
    dprintf("IBMPC: kbd_id byte2=0x%02X\n", (uint8_t)code);
    id |= (uint16_t)(code & 0xFF);

DONE:
    dprintf("IBMPC: kbd_id -> 0x%04X\n", id);
    return id;
}

/*
 * ibmpc_host_enable() / ibmpc_host_disable()
 * Control clock inhibit — used during XT reset and AT/XT auto-switching.
 */
void ibmpc_host_enable(void) {
    dprintf("IBMPC: enable (IDLE+int_on)\n");
    IBMPC_IDLE();
    ibmpc_int_on();
}

void ibmpc_host_disable(void) {
    dprintf("IBMPC: disable (int_off+inhibit)\n");
    ibmpc_int_off();
    /* Clock Lo / Data Hi = PS/2 inhibit; also XT soft-reset when Data later goes Lo */
    IBMPC_CLOCK_LO();
    IBMPC_DATA_HI();
}

/*
 * ibmpc_host_isr_clear() — flush the receive ring buffer.
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
    err         = ibmpc_error;
    ibmpc_error = IBMPC_ERR_NONE;
    chSysUnlock();
    return err;
}

uint8_t ibmpc_host_send(uint8_t data) {
    if (!IBMPC_PROTOCOL_IS_AT(detected_protocol)) {
        dprintf("IBMPC: send 0x%02X skipped (XT receive-only)\n", data);
        return 0;
    }
    return ibmpc_ps2_send(data);
}

enum ibmpc_protocol ibmpc_host_protocol(void) {
    return detected_protocol;
}

void ibmpc_host_set_protocol(enum ibmpc_protocol proto) {
    dprintf("IBMPC: set_protocol %d (%s)\n", proto,
            IBMPC_PROTOCOL_IS_AT(proto) ? "PS/2" : "XT");
    detected_protocol = proto;
    /* Switch ISR to match the new protocol */
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
        return; /* XT keyboards do not support LED commands */
    }
    ibmpc_host_send(IBMPC_CMD_SET_LED);
    ibmpc_host_send(led);
}
