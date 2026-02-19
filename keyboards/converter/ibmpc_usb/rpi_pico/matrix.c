/*
Copyright 2024

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * IBM PC USB Converter — QMK matrix implementation for RP2040
 *
 * Ported from TMK IBMPCConverter (ibmpc_usb.cpp / ibmpc_usb.hpp).
 * Implements:
 *   - 14-state keyboard initialisation state machine (Step 1.3)
 *   - Scan Code Set 1 (XT) full processing with E0 fixes         (Step 1.4)
 *   - Scan Code Set 2 (AT/PS2) full processing + 5576 support    (Step 1.5)
 *   - Scan Code Set 3 (Terminal) full processing                  (Step 1.6)
 *   - clear_stuck_keys() on Error/Overrun codes                   (Step 1.7)
 *
 * Design: CS1 and CS2 both store Set-1-equivalent internal codes in the 8×16
 * matrix (ROW = code >> 3, COL = code & 7) so both keyboard families share
 * the same QMK keymap.  CS3 raw codes are stored directly (0x01–0x7F).
 */

#include <stdint.h>
#include <stdbool.h>
#include "action.h"
#include "led.h"
#include "host.h"
#include "print.h"
#include "util.h"
#include "debug.h"
#include "timer.h"
#include "ibmpc.h"
#include "matrix.h"

/* Host API — forward declarations (implemented in ibmpc_host.c) */
extern uint16_t ibmpc_isr_debug;

/*--------------------------------------------------------------------
 * Matrix state
 *------------------------------------------------------------------*/
static uint8_t matrix[MATRIX_ROWS];

#define ROW(code) ((code) >> 3)
#define COL(code) ((code) & 0x07)

/*--------------------------------------------------------------------
 * Keyboard kind and ID — determined during initialisation
 *------------------------------------------------------------------*/
static uint16_t        keyboard_id      = 0x0000;
static keyboard_kind_t keyboard_kind    = PC_NONE;
static uint8_t         current_protocol = 0;

/*--------------------------------------------------------------------
 * Converter state machine (Step 1.3)
 *------------------------------------------------------------------*/
typedef enum {
    STATE_INIT,
    STATE_WAIT_SETTLE,
    STATE_AT_RESET,
    STATE_XT_RESET,
    STATE_XT_RESET_WAIT,
    STATE_XT_RESET_DONE,
    STATE_WAIT_AA,
    STATE_WAIT_AABF,
    STATE_WAIT_AABFBF,
    STATE_READ_ID,
    STATE_SETUP,
    STATE_LOOP,
    STATE_ERROR,
    STATE_ERROR_PARITY_AA,
} converter_state_t;

static converter_state_t converter_state = STATE_INIT;
static uint16_t          init_time       = 0;

/*--------------------------------------------------------------------
 * QMK weak callbacks
 *------------------------------------------------------------------*/
__attribute__((weak)) void matrix_init_kb(void)   { matrix_init_user(); }
__attribute__((weak)) void matrix_scan_kb(void)   { matrix_scan_user(); }
__attribute__((weak)) void matrix_init_user(void) {}
__attribute__((weak)) void matrix_scan_user(void) {}

/*--------------------------------------------------------------------
 * Matrix helpers (Step 1.3)
 *------------------------------------------------------------------*/
bool matrix_is_on(uint8_t row, uint8_t col) {
    return (matrix[row] >> col) & 1;
}

static inline void matrix_make(uint8_t code) {
    if (!matrix_is_on(ROW(code), COL(code))) {
        matrix[ROW(code)] |= 1 << COL(code);
        dprintf("MK %02X (r%d,c%d)\n", code, ROW(code), COL(code));
    }
}

static inline void matrix_break(uint8_t code) {
    if (matrix_is_on(ROW(code), COL(code))) {
        matrix[ROW(code)] &= ~(1 << COL(code));
        dprintf("BK %02X (r%d,c%d)\n", code, ROW(code), COL(code));
    }
}

void matrix_clear(void) {
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) matrix[i] = 0x00;
}

/*--------------------------------------------------------------------
 * Step 1.7 — Clear stuck keys on Error/Overrun
 *------------------------------------------------------------------*/
static void clear_stuck_keys(void) {
    matrix_clear();
    clear_keyboard();
    xprintf("\n[CLR] stuck keys cleared\n");
}

/******************************************************************************
 * Step 1.4 — Scan Code Set 1 (XT) processing
 *
 * Internal code map (0x01–0x7F):
 *   0x01–0x53  Original IBM XT 83-key codes
 *   0x54–0x7F  E0-prefixed codes remapped into the unused area
 *
 *    54 PrintScr  55 Pause   5A LGui  5B RGui  5C App
 *    5D Mute      5E VolDn   5F VolUp
 *    60 Up        61 Left    62 Down  63 Right
 *    64–76 F13–F24  6F KpEnter  70 KANA  71 Ins  72 Del
 *    73 RO  74 Home  75 End  76 F24  77 PgUp  78 PgDn
 *    79 HENKAN  7A RCtrl  7B MUHENKAN  7C RAlt  7D JYEN
 *    7E PCMM   7F KP/
 *  Power/Sleep/Wake share KANA/HENKAN/MUHENKAN cells.
 ******************************************************************************/
static uint8_t cs1_e0code(uint8_t code) {
    switch (code) {
        case 0x37: return 0x54; /* Print Screen     */
        case 0x46: return 0x55; /* Ctrl + Pause     */
        case 0x5B: return 0x5A; /* Left GUI         */
        case 0x5C: return 0x5B; /* Right GUI        */
        case 0x5D: return 0x5C; /* Application      */
        case 0x20: return 0x5D; /* Mute             */
        case 0x2E: return 0x5E; /* Volume Down      */
        case 0x30: return 0x5F; /* Volume Up        */
        case 0x48: return 0x60; /* Up               */
        case 0x4B: return 0x61; /* Left             */
        case 0x50: return 0x62; /* Down             */
        case 0x4D: return 0x63; /* Right            */
        case 0x1C: return 0x6F; /* Keypad Enter     */
        case 0x52: return 0x71; /* Insert           */
        case 0x53: return 0x72; /* Delete           */
        case 0x47: return 0x74; /* Home             */
        case 0x4F: return 0x75; /* End              */
        case 0x49: return 0x77; /* Page Up          */
        case 0x51: return 0x78; /* Page Down        */
        case 0x1D: return 0x7A; /* Right Ctrl       */
        case 0x38: return 0x7C; /* Right Alt        */
        case 0x35: return 0x7F; /* Keypad /         */
        /* Shared cells with Power/Sleep/Wake */
        case 0x5E: return 0x70; /* Power  → KANA    */
        case 0x5F: return 0x79; /* Sleep  → HENKAN  */
        case 0x63: return 0x7B; /* Wake   → MUHENKAN */
        default:
            xprintf("!CS1_E0_%02X!\n", code);
            return 0x00;
    }
}

static int8_t process_cs1(uint8_t code) {
    static enum {
        CS1_INIT, CS1_E0, CS1_E1, CS1_E1_1D, CS1_E1_9D,
    } state = CS1_INIT;

    switch (state) {
        case CS1_INIT:
            switch (code) {
                case 0xFF:  /* Error/Overrun */
                    clear_stuck_keys();
                    break;
                case 0xE0: state = CS1_E0; break;
                case 0xE1: state = CS1_E1; break;
                default:
                    if (code < 0x80) matrix_make(code);
                    else             matrix_break(code & 0x7F);
                    break;
            }
            break;
        case CS1_E0:
            switch (code) {
                case 0x2A: case 0xAA: case 0x36: case 0xB6: /* fake shift */
                    break;
                default:
                    if (code < 0x80) matrix_make(cs1_e0code(code));
                    else             matrix_break(cs1_e0code(code & 0x7F));
                    break;
            }
            state = CS1_INIT;
            break;
        case CS1_E1:
            switch (code) {
                case 0x1D: state = CS1_E1_1D; break;
                case 0x9D: state = CS1_E1_9D; break;
                default:   state = CS1_INIT;  break;
            }
            break;
        case CS1_E1_1D:
            if (code == 0x45) matrix_make(0x55);   /* Pause make  */
            state = CS1_INIT;
            break;
        case CS1_E1_9D:
            if (code == 0xC5) matrix_break(0x55);  /* Pause break */
            state = CS1_INIT;
            break;
        default:
            state = CS1_INIT;
    }
    return 0;
}

/******************************************************************************
 * Step 1.5 — Scan Code Set 2 (AT/PS2) processing
 *
 * Raw CS2 make codes are translated to CS1-equivalent internal codes via
 * set2_to_internal[] so both keyboard families share one QMK keymap.
 * E0-prefixed codes are handled by cs2_e0code() which also returns CS1 codes.
 ******************************************************************************/

/* CS2 raw (index) → CS1 internal (value).  0x00 = unused/invalid. */
static const uint8_t set2_to_internal[128] = {
    /* 0x00 */ 0x00, /* 0x01 */ 0x43, /* 0x02 */ 0x00, /* 0x03 */ 0x3F,
    /* 0x04 */ 0x3D, /* 0x05 */ 0x3B, /* 0x06 */ 0x3C, /* 0x07 */ 0x58,
    /* 0x08 */ 0x00, /* 0x09 */ 0x44, /* 0x0A */ 0x42, /* 0x0B */ 0x40,
    /* 0x0C */ 0x3E, /* 0x0D */ 0x0F, /* 0x0E */ 0x29, /* 0x0F */ 0x00,
    /* 0x10 */ 0x00, /* 0x11 */ 0x38, /* 0x12 */ 0x2A, /* 0x13 */ 0x00,
    /* 0x14 */ 0x1D, /* 0x15 */ 0x10, /* 0x16 */ 0x02, /* 0x17 */ 0x00,
    /* 0x18 */ 0x00, /* 0x19 */ 0x00, /* 0x1A */ 0x2C, /* 0x1B */ 0x1F,
    /* 0x1C */ 0x1E, /* 0x1D */ 0x11, /* 0x1E */ 0x03, /* 0x1F */ 0x00,
    /* 0x20 */ 0x00, /* 0x21 */ 0x2E, /* 0x22 */ 0x2D, /* 0x23 */ 0x20,
    /* 0x24 */ 0x12, /* 0x25 */ 0x05, /* 0x26 */ 0x04, /* 0x27 */ 0x00,
    /* 0x28 */ 0x00, /* 0x29 */ 0x39, /* 0x2A */ 0x2F, /* 0x2B */ 0x21,
    /* 0x2C */ 0x14, /* 0x2D */ 0x13, /* 0x2E */ 0x06, /* 0x2F */ 0x00,
    /* 0x30 */ 0x00, /* 0x31 */ 0x31, /* 0x32 */ 0x30, /* 0x33 */ 0x23,
    /* 0x34 */ 0x22, /* 0x35 */ 0x15, /* 0x36 */ 0x07, /* 0x37 */ 0x00,
    /* 0x38 */ 0x00, /* 0x39 */ 0x00, /* 0x3A */ 0x32, /* 0x3B */ 0x24,
    /* 0x3C */ 0x16, /* 0x3D */ 0x08, /* 0x3E */ 0x09, /* 0x3F */ 0x00,
    /* 0x40 */ 0x00, /* 0x41 */ 0x33, /* 0x42 */ 0x25, /* 0x43 */ 0x17,
    /* 0x44 */ 0x18, /* 0x45 */ 0x0B, /* 0x46 */ 0x0A, /* 0x47 */ 0x00,
    /* 0x48 */ 0x00, /* 0x49 */ 0x34, /* 0x4A */ 0x35, /* 0x4B */ 0x26,
    /* 0x4C */ 0x27, /* 0x4D */ 0x19, /* 0x4E */ 0x0C, /* 0x4F */ 0x00,
    /* 0x50 */ 0x00, /* 0x51 */ 0x00, /* 0x52 */ 0x28, /* 0x53 */ 0x00,
    /* 0x54 */ 0x1A, /* 0x55 */ 0x0D, /* 0x56 */ 0x00, /* 0x57 */ 0x00,
    /* 0x58 */ 0x3A, /* 0x59 */ 0x36, /* 0x5A */ 0x1C, /* 0x5B */ 0x1B,
    /* 0x5C */ 0x00, /* 0x5D */ 0x2B, /* 0x5E */ 0x00, /* 0x5F */ 0x00,
    /* 0x60 */ 0x00, /* 0x61 */ 0x56, /* 0x62 */ 0x00, /* 0x63 */ 0x00,
    /* 0x64 */ 0x00, /* 0x65 */ 0x00, /* 0x66 */ 0x0E, /* 0x67 */ 0x00,
    /* 0x68 */ 0x00, /* 0x69 */ 0x4F, /* 0x6A */ 0x00, /* 0x6B */ 0x4B,
    /* 0x6C */ 0x47, /* 0x6D */ 0x00, /* 0x6E */ 0x00, /* 0x6F */ 0x00,
    /* 0x70 */ 0x52, /* 0x71 */ 0x53, /* 0x72 */ 0x50, /* 0x73 */ 0x4C,
    /* 0x74 */ 0x4D, /* 0x75 */ 0x48, /* 0x76 */ 0x01, /* 0x77 */ 0x45,
    /* 0x78 */ 0x57, /* 0x79 */ 0x4E, /* 0x7A */ 0x51, /* 0x7B */ 0x4A,
    /* 0x7C */ 0x37, /* 0x7D */ 0x49, /* 0x7E */ 0x46, /* 0x7F */ 0x00,
};

/* IBM 5576-002/003 base-code remap; applied before set2_to_internal lookup. */
static uint8_t translate_5576_cs2(uint8_t code) {
    switch (code) {
        case 0x11: return 0x0F; /* Zenmen  → (RALT path via e0 table) */
        case 0x13: return 0x11; /* Kanji   → LALT  */
        case 0x0E: return 0x54; /* @       → [     */
        case 0x54: return 0x5B; /* [       → ]     */
        case 0x5B: return 0x5D; /* ]       → \     */
        case 0x5C: return 0x6A; /* ¥(5576) → JPY   */
        case 0x5D: return 0x6A; /* ￥      → JPY   */
        case 0x62: return 0x0E; /* Han/Zen → Grave */
        case 0x7C: return 0x77; /* Kp *   → NumLk  */
    }
    return code;
}

/* E0-prefixed CS2 codes → CS1-equivalent internal codes. */
static uint8_t cs2_e0code(uint8_t code) {
    /* IBM 5576-002/003 specific E0 overrides */
    if (keyboard_id == 0xAB90 || keyboard_id == 0xAB91) {
        switch (code) {
            case 0x11: return 0x70; /* Hiragana (5576) → KANA     */
            case 0x41: return 0x37; /* Keypad , (5576) → Keypad * */
        }
    }
    switch (code) {
        /* Standard extended keys */
        case 0x11: return 0x7C; /* Right Alt      */
        case 0x14: return 0x7A; /* Right Ctrl     */
        case 0x1F: return 0x5A; /* Left GUI       */
        case 0x27: return 0x5B; /* Right GUI      */
        case 0x2F: return 0x5C; /* Application    */
        case 0x4A: return 0x7F; /* Keypad /       */
        case 0x5A: return 0x6F; /* Keypad Enter   */
        case 0x69: return 0x75; /* End            */
        case 0x6B: return 0x61; /* Left           */
        case 0x6C: return 0x74; /* Home           */
        case 0x70: return 0x71; /* Insert         */
        case 0x71: return 0x72; /* Delete         */
        case 0x72: return 0x62; /* Down           */
        case 0x74: return 0x63; /* Right          */
        case 0x75: return 0x60; /* Up             */
        case 0x7A: return 0x78; /* Page Down      */
        case 0x7D: return 0x77; /* Page Up        */
        case 0x7C: return 0x54; /* Print Screen   */
        case 0x77: return 0x00; /* Unicomp Pause/Break fix → discard */
        case 0x7E: return 0x00; /* Ctrl'd Pause            → discard */
        /* Volume / mute */
        case 0x21: return 0x5E; /* Volume Down    */
        case 0x32: return 0x5F; /* Volume Up      */
        case 0x23: return 0x5D; /* Mute           */
        /* Media / browser keys → F13–F24 internal range */
        case 0x10: return 0x64; /* WWW Search       → F13 */
        case 0x18: return 0x65; /* WWW Favourites   → F14 */
        case 0x20: return 0x66; /* WWW Refresh      → F15 */
        case 0x28: return 0x67; /* WWW Stop         → F16 */
        case 0x30: return 0x68; /* WWW Forward      → F17 */
        case 0x38: return 0x69; /* WWW Back         → F18 */
        case 0x3A: return 0x6A; /* WWW Home         → F19 */
        case 0x40: return 0x6B; /* My Computer      → F20 */
        case 0x48: return 0x6C; /* Email            → F21 */
        case 0x2B: return 0x6D; /* Calculator       → F22 */
        case 0x34: return 0x64; /* Play/Pause       → F13 */
        case 0x3B: return 0x65; /* Stop             → F14 */
        case 0x15: return 0x66; /* Previous Track   → F15 */
        case 0x4D: return 0x67; /* Next Track       → F16 */
        case 0x50: return 0x68; /* Media Select     → F17 */
        case 0x5E: return 0x6D; /* ACPI Wake        → F22 */
        case 0x3F: return 0x6E; /* ACPI Sleep       → F23 */
        case 0x37: return 0x76; /* ACPI Power       → F24 */
        /* DEC LK411 */
        case 0x03: return 0x66; /* Help             → F15 */
        case 0x04: return 0x64; /* F13              → F13 */
        case 0x0B: return 0x67; /* Do               → F16 */
        case 0x0C: return 0x65; /* F14              → F14 */
        case 0x0D: return 0x7A; /* LCompose         → RCtrl */
        case 0x79: return 0x7E; /* KP-              → PCMM  */
        case 0x83: return 0x68; /* F17              → F17   */
        /* Siemens F500 E0 00 */
        case 0x00: return 0x5E; /* TERM FUNC        → Vol Down */
        /* Silitek SK-7100P */
        case 0x43: return 0x6B; /* Close            → F20 */
        case 0x42: return 0x6C; /* CD               → F21 */
        case 0x44: return 0x6D; /* Video            → F22 */
        case 0x1C: return 0x69; /* U/P              → F18 */
        case 0x24: return 0x68; /* Pause            → F17 */
        case 0x4B: return 0x6E; /* Display          → F23 */
        default:
            xprintf("!CS2_E0_%02X!\n", code);
            return 0x00;
    }
}

#ifdef CS2_80CODE_SUPPORT
/* 0x80-prefixed CS2 codes — Tandberg TDV 5020. */
static uint8_t cs2_80code(uint8_t code) {
    switch (code) {
        case 0x2B: return 0x64; /* MERK  → F13      */
        case 0x34: return 0x65; /* ANGRE → F14      */
        case 0x33: return 0x66; /* SKRIV → F15      */
        case 0x42: return 0x67; /* SLUTT → F16      */
        case 0x2C: return 0x68; /* STRYK → F17      */
        case 0x3C: return 0x69; /* KOPI  → F18      */
        case 0x43: return 0x6A; /* FLYTT → F19      */
        case 0x4B: return 0x6B; /* FELT  → F20      */
        case 0x2A: return 0x6C; /* AVSN  → F21      */
        case 0x32: return 0x6D; /* SETN  → F22      */
        case 0x3A: return 0x6E; /* ORD   → F23      */
        case 0x61: return 0x7D; /* ⮎     → JYEN     */
        case 0x1D: return 0x76; /* HJELP → F24      */
        case 0x24: return 0x5A; /* ^^^   → LGui     */
        case 0x44: return 0x5E; /* >>/<<  → Vol Dn  */
        case 0x4D: return 0x5F; /* JUST  → Vol Up   */
        case 0x1C: return 0x5D; /* >< <> → Mute     */
        case 0x2D: return 0x73; /* ⇟     → RO       */
        case 0x1B: return 0x5B; /* ⇤     → RGui     */
        case 0x23: return 0x5C; /* ⇥     → App      */
        default:   return 0x00;
    }
}
#endif /* CS2_80CODE_SUPPORT */

static int8_t process_cs2(uint8_t code) {
    static enum {
        CS2_INIT, CS2_F0, CS2_E0, CS2_E0_F0,
        CS2_E1, CS2_E1_14, CS2_E1_F0, CS2_E1_F0_14, CS2_E1_F0_14_F0,
#ifdef CS2_80CODE_SUPPORT
        CS2_80, CS2_80_F0,
#endif
    } state = CS2_INIT;

    switch (state) {
        case CS2_INIT:
            if (keyboard_id == 0xAB90 || keyboard_id == 0xAB91)
                code = translate_5576_cs2(code);
            switch (code) {
                case 0x00: clear_stuck_keys(); break;  /* Error/Overrun */
                case 0xE0: state = CS2_E0;  break;
                case 0xF0: state = CS2_F0;  break;
                case 0xE1: state = CS2_E1;  break;
#ifdef CS2_80CODE_SUPPORT
                case 0x80: state = CS2_80;  break;
#endif
                case 0x83: matrix_make(0x02); break;  /* F7            */
                case 0x84: matrix_make(0x7F); break;  /* Alt'd PrtScr  */
                case 0xAA:                            /* Self-test OK  */
                case 0xFC:                            /* Self-test fail */
                    xprintf("!CS2_REINIT!\n");
                    return -1;
                default:
                    if (code < 0x80) {
                        uint8_t c = set2_to_internal[code];
                        if (c) matrix_make(c);
                    } else {
                        matrix_clear();
                        xprintf("!CS2_INIT_%02X!\n", code);
                        return -1;
                    }
                    break;
            }
            break;

        case CS2_F0:
            if (keyboard_id == 0xAB90 || keyboard_id == 0xAB91)
                code = translate_5576_cs2(code);
            state = CS2_INIT;
            switch (code) {
                case 0x83: matrix_break(0x02); break;
                case 0x84: matrix_break(0x7F); break;
                default:
                    if (code < 0x80) {
                        uint8_t c = set2_to_internal[code];
                        if (c) matrix_break(c);
                    } else {
                        matrix_clear();
                        xprintf("!CS2_F0_%02X!\n", code);
                        return -1;
                    }
                    break;
            }
            break;

        case CS2_E0:
            switch (code) {
                case 0x12: case 0x59: state = CS2_INIT; break;  /* fake shift */
                case 0xF0: state = CS2_E0_F0; break;
                default: {
                    uint8_t c = cs2_e0code(code);
                    if (c) matrix_make(c);
                    state = CS2_INIT;
                    break;
                }
            }
            break;

        case CS2_E0_F0:
            switch (code) {
                case 0x12: case 0x59: break;  /* fake shift */
                default: {
                    uint8_t c = cs2_e0code(code);
                    if (c) matrix_break(c);
                    break;
                }
            }
            state = CS2_INIT;
            break;

        /* Pause make: E1 14 77 */
        case CS2_E1:
            state = (code == 0x14) ? CS2_E1_14
                  : (code == 0xF0) ? CS2_E1_F0
                  :                  CS2_INIT;
            break;
        case CS2_E1_14:
            if (code == 0x77) matrix_make(0x55);
            state = CS2_INIT;
            break;

        /* Pause break: E1 F0 14 F0 77 */
        case CS2_E1_F0:
            state = (code == 0x14) ? CS2_E1_F0_14 : CS2_INIT;
            break;
        case CS2_E1_F0_14:
            state = (code == 0xF0) ? CS2_E1_F0_14_F0 : CS2_INIT;
            break;
        case CS2_E1_F0_14_F0:
            if (code == 0x77) matrix_break(0x55);
            state = CS2_INIT;
            break;

#ifdef CS2_80CODE_SUPPORT
        case CS2_80:
            if (code == 0xF0) { state = CS2_80_F0; }
            else { uint8_t c = cs2_80code(code); if (c) matrix_make(c); state = CS2_INIT; }
            break;
        case CS2_80_F0: {
            uint8_t c = cs2_80code(code); if (c) matrix_break(c);
            state = CS2_INIT;
            break;
        }
#endif

        default: state = CS2_INIT;
    }
    return 0;
}

/******************************************************************************
 * Step 1.6 — Scan Code Set 3 (Terminal) processing
 *
 * CS3 raw codes 0x01–0x7F stored directly in matrix.
 * Codes 0x83–0x8D are exceptional and mapped to otherwise-unused positions.
 ******************************************************************************/

/* IBM 5576-001 CS3 positional fix */
static uint8_t translate_5576_cs3(uint8_t code) {
    switch (code) {
        case 0x13: return 0x5D; /* JYEN       */
        case 0x5C: return 0x51; /* RO         */
        case 0x76: return 0x7E; /* Keypad '   */
        case 0x7E: return 0x76; /* Keypad Dup */
    }
    return code;
}

/* Televideo DEC keyboard CS3 remap */
static uint8_t translate_televideo_dec_cs3(uint8_t code) {
    switch (code) {
        case 0x08: return 0x76; /* Esc    */ case 0x8D: return 0x77; /* NumLk  */
        case 0x8E: return 0x67; /* KP /   */ case 0x8F: return 0x7F; /* KP *   */
        case 0x90: return 0x7B; /* KP -   */ case 0x6E: return 0x65; /* Insert */
        case 0x65: return 0x6D; /* Delete */ case 0x67: return 0x62; /* Home   */
        case 0x6D: return 0x64; /* End    */ case 0x64: return 0x6E; /* PgUp   */
        case 0x84: return 0x7C; /* KP +   */ case 0x87: return 0x02; /* PrtScr */
        case 0x88: return 0x7E; /* ScrLk  */ case 0x89: return 0x0C; /* Pause  */
        case 0x8A: return 0x03; /* VolDn  */ case 0x8B: return 0x04; /* VolUp  */
        case 0x8C: return 0x05; /* Mute   */ case 0x85: return 0x08; /* F13    */
        case 0x86: return 0x10; /* F14    */ case 0x91: return 0x01; /* LGui   */
        case 0x92: return 0x09; /* RGui   */ case 0x77: return 0x58; /* RCtrl  */
        case 0x57: return 0x5C; /* Bslash */ case 0x5C: return 0x53; /* NUHS   */
        case 0x7C: return 0x68; /* KP,    */
    }
    return code;
}

static int8_t process_cs3(uint8_t code) {
    static enum {
        CS3_READY, CS3_F0,
#ifdef G80_2551_SUPPORT
        CS3_G80, CS3_G80_F0,
#endif
    } state = CS3_READY;

    /* Re-init codes — handle regardless of state */
    switch (code) {
        case 0xAA: case 0xFC: case 0xBF: case 0xAB:
            state = CS3_READY;
            xprintf("!CS3_RESET!\n");
            return -1;
    }

    switch (state) {
        case CS3_READY:
            if (keyboard_id == 0xAB92) code = translate_5576_cs3(code);
            if (keyboard_id == 0xAB91) code = translate_televideo_dec_cs3(code);
            switch (code) {
                case 0x00:  clear_stuck_keys(); break;
                case 0xF0:  state = CS3_F0;     break;
                case 0x83:  matrix_make(0x02);  break; /* PrintScreen */
                case 0x84:  matrix_make(0x7F);  break; /* Keypad *   */
                case 0x85:  matrix_make(0x68);  break; /* Muhenkan   */
                case 0x86:  matrix_make(0x78);  break; /* Henkan     */
                case 0x87:  matrix_make(0x00);  break; /* Hiragana   */
                case 0x8B:  matrix_make(0x01);  break; /* Left GUI   */
                case 0x8C:  matrix_make(0x09);  break; /* Right GUI  */
                case 0x8D:  matrix_make(0x0A);  break; /* Application */
#ifdef G80_2551_SUPPORT
                case 0x80:  state = CS3_G80;    break;
#endif
                default:
                    if (code < 0x80) matrix_make(code);
                    else xprintf("!CS3_READY_%02X!\n", code);
                    break;
            }
            break;

        case CS3_F0:
            state = CS3_READY;
            if (keyboard_id == 0xAB92) code = translate_5576_cs3(code);
            if (keyboard_id == 0xAB91) code = translate_televideo_dec_cs3(code);
            switch (code) {
                case 0x83:  matrix_break(0x02); break;
                case 0x84:  matrix_break(0x7F); break;
                case 0x85:  matrix_break(0x68); break;
                case 0x86:  matrix_break(0x78); break;
                case 0x87:  matrix_break(0x00); break;
                case 0x8B:  matrix_break(0x01); break;
                case 0x8C:  matrix_break(0x09); break;
                case 0x8D:  matrix_break(0x0A); break;
                default:
                    if (code < 0x80) matrix_break(code);
                    else xprintf("!CS3_F0_%02X!\n", code);
                    break;
            }
            break;

#ifdef G80_2551_SUPPORT
        case CS3_G80:
            switch (code) {
                case 0x26: matrix_make(0x5D); break; /* → JYEN */
                case 0x25: matrix_make(0x53); break; /* → NUHS */
                case 0x16: matrix_make(0x51); break; /* → RO   */
                case 0x1E: matrix_make(0x00); break; /* → KANA */
                case 0xF0: state = CS3_G80_F0; return 0;
                default:   matrix_clear(); break;
            }
            state = CS3_READY;
            break;

        case CS3_G80_F0:
            switch (code) {
                case 0x26: matrix_break(0x5D); break;
                case 0x25: matrix_break(0x53); break;
                case 0x16: matrix_break(0x51); break;
                case 0x1E: matrix_break(0x00); break;
                default:   matrix_clear(); break;
            }
            state = CS3_READY;
            break;
#endif /* G80_2551_SUPPORT */

        default:
            state = CS3_READY;
    }
    return 0;
}

/******************************************************************************
 * Step 1.3 — Full keyboard initialisation state machine
 *
 * Port of TMK IBMPCConverter::process_interface() to plain C.
 * Called from matrix_scan() once per QMK scan loop tick.
 ******************************************************************************/
static void process_interface(void) {
    /* ------------------------------------------------------------------ */
    /* Error detection                                                      */
    /* ------------------------------------------------------------------ */
    uint8_t err = ibmpc_host_error();
    if (err) {
        xprintf("\nERR:%02X ISR:%04X ", err, ibmpc_isr_debug);
        ibmpc_isr_debug = 0;
        /* SEND / FULL are transient — ignore, keep current state */
        if (!(err & (IBMPC_ERR_SEND | IBMPC_ERR_FULL))) {
            converter_state = (err == IBMPC_ERR_PARITY_AA)
                              ? STATE_ERROR_PARITY_AA
                              : STATE_ERROR;
        }
    }

    /* Protocol change detection (AT ↔ XT hotswap) */
    uint8_t proto = (uint8_t)ibmpc_host_protocol();
    if (proto && proto != current_protocol) {
        xprintf("\nPRT:%02X ", proto);
        if (current_protocol &&
            ((IBMPC_PROTOCOL_IS_AT(current_protocol) && IBMPC_PROTOCOL_IS_XT(proto)) ||
             (IBMPC_PROTOCOL_IS_XT(current_protocol) && IBMPC_PROTOCOL_IS_AT(proto)))) {
            if (converter_state == STATE_LOOP) {
                xprintf("[CHG] ");
                converter_state = STATE_ERROR;
            }
        }
        current_protocol = proto;
    }

    /* ------------------------------------------------------------------ */
    /* State machine tick                                                   */
    /* ------------------------------------------------------------------ */
    switch (converter_state) {

        case STATE_INIT:
            xprintf("I ");
            init_time = timer_read();
            ibmpc_host_isr_clear();
            ibmpc_host_enable();
            converter_state = STATE_WAIT_SETTLE;
            break;

        case STATE_WAIT_SETTLE:
            while (ibmpc_host_recv() != -1) {}  /* drain noise */
            if (timer_elapsed(init_time) > 3000)
                converter_state = STATE_AT_RESET;
            break;

        case STATE_AT_RESET:
            xprintf("A ");
            if (IBMPC_ACK == ibmpc_host_send(IBMPC_CMD_RESET))
                converter_state = STATE_WAIT_AA;
            else
                converter_state = STATE_XT_RESET;
            break;

        case STATE_XT_RESET:
            /* XT soft reset: inhibit (clock lo) for ≥20 ms */
            ibmpc_host_disable();
            init_time = timer_read();
            converter_state = STATE_XT_RESET_WAIT;
            break;

        case STATE_XT_RESET_WAIT:
            if (timer_elapsed(init_time) > 20)
                converter_state = STATE_XT_RESET_DONE;
            break;

        case STATE_XT_RESET_DONE:
            ibmpc_host_isr_clear();
            ibmpc_host_enable();
            xprintf("X ");
            init_time = timer_read();
            converter_state = STATE_WAIT_AA;
            break;

        case STATE_WAIT_AA:
            if (ibmpc_host_recv() != -1) {
                xprintf("W ");
                init_time = timer_read();
                converter_state = STATE_WAIT_AABF;
            }
            break;

        case STATE_WAIT_AABF:
            if (timer_elapsed(init_time) > 500) { converter_state = STATE_READ_ID; break; }
            if (ibmpc_host_recv() != -1) {
                xprintf("W ");
                init_time = timer_read();
                converter_state = STATE_WAIT_AABFBF;
            }
            break;

        case STATE_WAIT_AABFBF:
            if (timer_elapsed(init_time) > 500) { converter_state = STATE_READ_ID; break; }
            if (ibmpc_host_recv() != -1) { xprintf("W "); converter_state = STATE_READ_ID; }
            break;

        case STATE_READ_ID: {
            keyboard_id = ibmpc_host_keyboard_id();
            xprintf("\nID:%04X ", keyboard_id);

            if      (keyboard_id == 0x0000) keyboard_kind = PC_AT;
            else if (keyboard_id == 0xFFFF) keyboard_kind = PC_XT;
            else if (keyboard_id == 0xFFFE) keyboard_kind = PC_AT;
            else if (keyboard_id == 0xFFFD) keyboard_kind = PC_AT;   /* Zenith Z-150 */
            else if (keyboard_id == 0xFFFC) keyboard_kind = PC_XT;   /* IBM XT       */
            else if (keyboard_id == 0xFFFB) keyboard_kind = PC_XT;   /* Clone XT     */
            else if (keyboard_id == 0x00FF) keyboard_kind = PC_MOUSE;
            else if (keyboard_id == 0xAB85 || /* IBM 122-key / NCD N-97 */
                     keyboard_id == 0xAB86 || /* Cherry G80-2551         */
                     keyboard_id == 0xAB92) { /* IBM 5576-001            */
                if ((IBMPC_ACK == ibmpc_host_send(IBMPC_CMD_SCANCODE)) &&
                    (IBMPC_ACK == ibmpc_host_send(0x03)))
                    keyboard_kind = PC_TERMINAL;
                else
                    keyboard_kind = PC_AT;
            }
            else if (keyboard_id == 0xAB90 || /* IBM 5576-002             */
                     keyboard_id == 0xAB91) { /* IBM 5576-003 / Televideo */
                xprintf("5576_CS82h: ");
                keyboard_kind = PC_AT;
                if ((IBMPC_ACK == ibmpc_host_send(IBMPC_CMD_SCANCODE)) &&
                    (IBMPC_ACK == ibmpc_host_send(0x82))) {
                    xprintf("OK ");
                } else {
                    xprintf("NG ");
                    if (keyboard_id == 0xAB91) {
                        xprintf("Televideo: ");
                        if ((IBMPC_ACK == ibmpc_host_send(IBMPC_CMD_SCANCODE)) &&
                            (IBMPC_ACK == ibmpc_host_send(0x03))) {
                            xprintf("OK "); keyboard_kind = PC_TERMINAL;
                        } else { xprintf("NG "); }
                    }
                }
            }
            else if (keyboard_id == 0xBFB0)                   keyboard_kind = PC_TERMINAL;
            else if ((keyboard_id & 0xFF00) == 0xAB00)        keyboard_kind = PC_AT;
            else if ((keyboard_id & 0xFF00) == 0xBF00)        keyboard_kind = PC_TERMINAL;
            else if ((keyboard_id & 0xFF00) == 0x7F00)        keyboard_kind = PC_TERMINAL;
            else {
                xprintf("Unknown: ");
                if ((IBMPC_ACK == ibmpc_host_send(IBMPC_CMD_SCANCODE)) &&
                    (IBMPC_ACK == ibmpc_host_send(0x02)))
                    keyboard_kind = PC_AT;
                else if ((IBMPC_ACK == ibmpc_host_send(IBMPC_CMD_SCANCODE)) &&
                         (IBMPC_ACK == ibmpc_host_send(0x03)))
                    keyboard_kind = PC_TERMINAL;
                else
                    keyboard_kind = PC_AT;
            }

            xprintf("(%s)\n", KEYBOARD_KIND_STR(keyboard_kind));
            converter_state = STATE_SETUP;
            break;
        }

        case STATE_SETUP:
            xprintf("S ");
            switch (keyboard_kind) {
                case PC_XT:
                    ibmpc_host_set_protocol(IBMPC_PROTOCOL_XT);
                    break;
                case PC_AT:
                    ibmpc_host_set_protocol(IBMPC_PROTOCOL_AT);
                    led_update_kb(host_keyboard_led_state());
                    break;
                case PC_TERMINAL:
                    ibmpc_host_set_protocol(IBMPC_PROTOCOL_AT);
                    ibmpc_host_send(IBMPC_CMD_ALLM_TMB); /* all keys make/break */
                    led_update_kb(host_keyboard_led_state());
                    break;
                default: break;
            }
            converter_state = STATE_LOOP;
            xprintf("L ");
            /* FALLTHRU */

        case STATE_LOOP: {
            int16_t code = ibmpc_host_recv();
            if (code == -1) break;
            xprintf("%02X ", (uint8_t)code);
            int8_t ret = 0;
            switch (keyboard_kind) {
                case PC_XT:       ret = process_cs1((uint8_t)code); break;
                case PC_AT:       ret = process_cs2((uint8_t)code); break;
                case PC_TERMINAL: ret = process_cs3((uint8_t)code); break;
                default: break;
            }
            if (ret == -1) converter_state = STATE_ERROR;
            break;
        }

        case STATE_ERROR_PARITY_AA: {
            xprintf("P ");
            /* AT/XT auto-switch: resend last byte; 0xAA means XT keyboard */
            int16_t code = ibmpc_host_send(IBMPC_RESEND);
            if (code == IBMPC_BAT_OK) { converter_state = STATE_READ_ID; break; }
            /* FALLTHRU to full re-init */
        }
        /* FALLTHRU */

        case STATE_ERROR:
            xprintf("E ");
            keyboard_id      = 0x0000;
            keyboard_kind    = PC_NONE;
            current_protocol = 0;
            matrix_clear();
            clear_keyboard();
            ibmpc_host_isr_clear();
            converter_state = STATE_INIT;
            break;

        default:
            converter_state = STATE_INIT;
            break;
    }
}

/******************************************************************************
 * QMK matrix interface
 ******************************************************************************/

void matrix_init(void) {
    debug_enable     = true;
    debug_matrix     = false;  /* per-scan matrix diff — too noisy; toggle via IS_COMMAND() */
    keyboard_id      = 0x0000;
    keyboard_kind    = PC_NONE;
    current_protocol = 0;
    converter_state  = STATE_INIT;
    matrix_clear();
    ibmpc_host_init();
    xprintf("[INIT] IBM PC USB converter ready\n");
    matrix_init_kb();
}

uint8_t matrix_scan(void) {
    process_interface();
    matrix_scan_kb();
    return 1;
}

uint8_t matrix_get_row(uint8_t row) {
    return matrix[row];
}

#if (MATRIX_COLS <= 8)
#    define print_matrix_header() print("\nr/c 01234567\n")
#    define print_matrix_row(row) print_bin_reverse8(matrix_get_row(row))
#elif (MATRIX_COLS <= 16)
#    define print_matrix_header() print("\nr/c 0123456789ABCDEF\n")
#    define print_matrix_row(row) print_bin_reverse16(matrix_get_row(row))
#elif (MATRIX_COLS <= 32)
#    define print_matrix_header() print("\nr/c 0123456789ABCDEF0123456789ABCDEF\n")
#    define print_matrix_row(row) print_bin_reverse32(matrix_get_row(row))
#endif

void matrix_print(void) {
    print_matrix_header();
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        print_hex8(row);
        print(": ");
        print_matrix_row(row);
        print("\n");
    }
}

/* Forward host LED state to the IBM/PC keyboard (QMK modern LED API) */
bool led_update_kb(led_t led_state) {
    bool res = led_update_user(led_state);
    if (res) {
        if (keyboard_kind == PC_NONE || keyboard_kind == PC_XT || keyboard_kind == PC_MOUSE)
            return res;
        uint8_t ibmpc_led = 0;
        if (led_state.scroll_lock) ibmpc_led |= (1 << IBMPC_LED_SCROLL_LOCK);
        if (led_state.num_lock)    ibmpc_led |= (1 << IBMPC_LED_NUM_LOCK);
        if (led_state.caps_lock)   ibmpc_led |= (1 << IBMPC_LED_CAPS_LOCK);
        dprintf("LED scr=%d num=%d cap=%d -> ibmpc=0x%02X\n",
                led_state.scroll_lock, led_state.num_lock,
                led_state.caps_lock, ibmpc_led);
        ibmpc_host_set_led(ibmpc_led);
    }
    return res;
}

