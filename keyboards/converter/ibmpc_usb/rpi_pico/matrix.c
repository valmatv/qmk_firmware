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

#include <stdint.h>
#include <stdbool.h>
#include "action.h"
#include "print.h"
#include "util.h"
#include "debug.h"
#include "ibmpc.h"
#include "matrix.h"

static void matrix_make(uint8_t code);
static void matrix_break(uint8_t code);

static uint8_t matrix[MATRIX_ROWS];

#define ROW(code) ((code) >> 3)
#define COL(code) ((code) & 0x07)

__attribute__((weak))
void matrix_init_kb(void) {
    matrix_init_user();
}

__attribute__((weak))
void matrix_scan_kb(void) {
    matrix_scan_user();
}

__attribute__((weak))
void matrix_init_user(void) {
}

__attribute__((weak))
void matrix_scan_user(void) {
}

/*--------------------------------------------------------------------
 * Scan Code Set 1 (XT) processing
 *
 * Set 1 scancodes: make < 0x80, break = make | 0x80
 * E0 prefix: extended keys remapped to unused area
 * E1 prefix: Pause key (E1 1D 45 / E1 9D C5)
 *------------------------------------------------------------------*/

// Convert E0-escaped Set 1 codes into unused internal area
static uint8_t set1_move_e0code(uint8_t code) {
    switch (code) {
        // Original IBM XT keyboard
        case 0x37: return 0x54; // Print Screen
        case 0x46: return 0x55; // Ctrl + Pause
        case 0x1C: return 0x6F; // Keypad Enter
        case 0x35: return 0x7F; // Keypad /

        // Extended keys
        case 0x5B: return 0x5A; // Left GUI
        case 0x5C: return 0x5B; // Right GUI
        case 0x5D: return 0x5C; // Application
        case 0x5E: return 0x5D; // Power
        case 0x5F: return 0x5E; // Sleep
        case 0x63: return 0x5F; // Wake
        case 0x48: return 0x60; // Up
        case 0x4B: return 0x61; // Left
        case 0x50: return 0x62; // Down
        case 0x4D: return 0x63; // Right
        case 0x52: return 0x71; // Insert
        case 0x53: return 0x72; // Delete
        case 0x47: return 0x74; // Home
        case 0x4F: return 0x75; // End
        case 0x49: return 0x77; // Page Up
        case 0x51: return 0x78; // Page Down
        case 0x1D: return 0x7A; // Right Ctrl
        case 0x38: return 0x7C; // Right Alt
    }
    return 0x00;
}

static void process_set1_scancode(uint8_t code) {
    static enum {
        S1_INIT,
        S1_E0,
        S1_E1,
        S1_E1_1D,
        S1_E1_9D,
    } state = S1_INIT;

    switch (state) {
        case S1_INIT:
            switch (code) {
                case 0xE0:
                    state = S1_E0;
                    break;
                case 0xE1:
                    state = S1_E1;
                    break;
                default:
                    if (code < 0x80) {
                        matrix_make(code);
                    } else {
                        matrix_break(code & 0x7F);
                    }
                    break;
            }
            break;

        case S1_E0:
            switch (code) {
                case 0x2A:
                case 0xAA:
                case 0x36:
                case 0xB6:
                    // Ignore fake shift
                    state = S1_INIT;
                    break;
                default:
                    if (code < 0x80) {
                        matrix_make(set1_move_e0code(code));
                    } else {
                        matrix_break(set1_move_e0code(code & 0x7F));
                    }
                    state = S1_INIT;
                    break;
            }
            break;

        case S1_E1:
            switch (code) {
                case 0x1D:
                    state = S1_E1_1D;
                    break;
                case 0x9D:
                    state = S1_E1_9D;
                    break;
                default:
                    state = S1_INIT;
                    break;
            }
            break;

        case S1_E1_1D:
            if (code == 0x45) {
                matrix_make(0x55); // Pause make
            }
            state = S1_INIT;
            break;

        case S1_E1_9D:
            if (code == 0xC5) {
                matrix_break(0x55); // Pause break
            }
            state = S1_INIT;
            break;

        default:
            state = S1_INIT;
    }
}

/*--------------------------------------------------------------------
 * Scan Code Set 2 (AT/PS2) processing
 *
 * Set 2 scancodes: F0 prefix for break, E0 prefix for extended,
 * E1 prefix for Pause.
 *
 * Set 2 codes must be translated to internal keycodes (same space
 * as Set 1 internal codes) so they map to the same matrix positions.
 *------------------------------------------------------------------*/

// Scan Code Set 2 to internal keycode translation table
// Index = Set 2 scan code, Value = internal keycode (same as Set 1 internal)
// 0x00 = unused/invalid
static const uint8_t set2_to_internal[128] = {
    // 0x00-0x0F
    0x00, 0x43, 0x00, 0x3F, 0x3D, 0x3B, 0x3C, 0x58, // 00-07
    0x00, 0x44, 0x42, 0x40, 0x3E, 0x0F, 0x29, 0x00, // 08-0F
    // 0x10-0x1F
    0x00, 0x38, 0x2A, 0x00, 0x1D, 0x10, 0x02, 0x00, // 10-17
    0x00, 0x00, 0x2C, 0x1F, 0x1E, 0x11, 0x03, 0x00, // 18-1F
    // 0x20-0x2F
    0x00, 0x2E, 0x2D, 0x20, 0x12, 0x05, 0x04, 0x00, // 20-27
    0x00, 0x39, 0x2F, 0x21, 0x14, 0x13, 0x06, 0x00, // 28-2F
    // 0x30-0x3F
    0x00, 0x31, 0x30, 0x23, 0x22, 0x15, 0x07, 0x00, // 30-37
    0x00, 0x00, 0x32, 0x24, 0x16, 0x08, 0x09, 0x00, // 38-3F
    // 0x40-0x4F
    0x00, 0x33, 0x25, 0x17, 0x18, 0x0B, 0x0A, 0x00, // 40-47
    0x00, 0x34, 0x35, 0x26, 0x27, 0x19, 0x0C, 0x00, // 48-4F
    // 0x50-0x5F
    0x00, 0x00, 0x28, 0x00, 0x1A, 0x0D, 0x00, 0x00, // 50-57
    0x3A, 0x36, 0x1C, 0x1B, 0x00, 0x2B, 0x00, 0x00, // 58-5F
    // 0x60-0x6F
    0x00, 0x56, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, // 60-67
    0x00, 0x4F, 0x00, 0x4B, 0x47, 0x00, 0x00, 0x00, // 68-6F
    // 0x70-0x7F
    0x52, 0x53, 0x50, 0x4C, 0x4D, 0x48, 0x01, 0x45, // 70-77
    0x57, 0x4E, 0x51, 0x4A, 0x37, 0x49, 0x46, 0x00, // 78-7F
};

// E0-prefixed Set 2 codes -> internal keycode
static uint8_t set2_e0_to_internal(uint8_t code) {
    switch (code) {
        case 0x11: return 0x7C; // Right Alt
        case 0x14: return 0x7A; // Right Ctrl
        case 0x1F: return 0x5A; // Left GUI
        case 0x27: return 0x5B; // Right GUI
        case 0x2F: return 0x5C; // Application/Menu
        case 0x4A: return 0x7F; // KP /
        case 0x5A: return 0x6F; // KP Enter
        case 0x69: return 0x75; // End
        case 0x6B: return 0x61; // Left
        case 0x6C: return 0x74; // Home
        case 0x70: return 0x71; // Insert
        case 0x71: return 0x72; // Delete
        case 0x72: return 0x62; // Down
        case 0x74: return 0x63; // Right
        case 0x75: return 0x60; // Up
        case 0x7A: return 0x78; // Page Down
        case 0x7D: return 0x77; // Page Up
        case 0x7C: return 0x54; // Print Screen (E0 7C is part of the make sequence)
        default:   return 0x00;
    }
}

static void process_set2_scancode(uint8_t code) {
    static enum {
        S2_INIT,
        S2_F0,       // Break prefix
        S2_E0,       // Extended prefix
        S2_E0_F0,    // Extended break prefix
        S2_E1,       // Pause sequence step 1
        S2_E1_14,    // Pause sequence step 2
        S2_E1_F0,    // Pause sequence step 3
        S2_E1_F0_14, // Pause sequence step 4
        S2_E1_F0_14_F0, // Pause sequence step 5
    } state = S2_INIT;

    switch (state) {
        case S2_INIT:
            switch (code) {
                case 0xF0:
                    state = S2_F0;
                    break;
                case 0xE0:
                    state = S2_E0;
                    break;
                case 0xE1:
                    state = S2_E1;
                    break;
                default:
                    if (code < 0x80) {
                        uint8_t internal = set2_to_internal[code];
                        if (internal) matrix_make(internal);
                    }
                    break;
            }
            break;

        case S2_F0: // Break
            if (code < 0x80) {
                uint8_t internal = set2_to_internal[code];
                if (internal) matrix_break(internal);
            }
            state = S2_INIT;
            break;

        case S2_E0: // Extended
            switch (code) {
                case 0xF0:
                    state = S2_E0_F0;
                    break;
                case 0x12: // Fake shift (part of Print Screen make)
                case 0x59: // Fake shift
                    state = S2_INIT;
                    break;
                default: {
                    uint8_t internal = set2_e0_to_internal(code);
                    if (internal) matrix_make(internal);
                    state = S2_INIT;
                    break;
                }
            }
            break;

        case S2_E0_F0: // Extended break
            switch (code) {
                case 0x12: // Fake shift break (part of Print Screen break)
                case 0x59: // Fake shift break
                    state = S2_INIT;
                    break;
                default: {
                    uint8_t internal = set2_e0_to_internal(code);
                    if (internal) matrix_break(internal);
                    state = S2_INIT;
                    break;
                }
            }
            break;

        // Pause key: E1 14 77 E1 F0 14 F0 77
        case S2_E1:
            if (code == 0x14) {
                state = S2_E1_14;
            } else {
                state = S2_INIT;
            }
            break;

        case S2_E1_14:
            if (code == 0x77) {
                matrix_make(0x55); // Pause make
                state = S2_E1_F0;
            } else {
                state = S2_INIT;
            }
            break;

        case S2_E1_F0:
            if (code == 0xE1) {
                state = S2_E1_F0_14;
            } else {
                state = S2_INIT;
            }
            break;

        case S2_E1_F0_14:
            if (code == 0xF0) {
                state = S2_E1_F0_14_F0;
            } else {
                state = S2_INIT;
            }
            break;

        case S2_E1_F0_14_F0:
            if (code == 0x14) {
                // Next byte should be F0 77 for Pause break
                // For simplicity, break Pause immediately
                matrix_break(0x55);
            }
            state = S2_INIT;
            break;

        default:
            state = S2_INIT;
    }
}

/*--------------------------------------------------------------------
 * Matrix interface
 *------------------------------------------------------------------*/

void matrix_init(void) {
    debug_enable = true;
    ibmpc_host_init();

    // Initialize matrix state: all keys off
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0x00;
    }

    matrix_init_kb();
}

uint8_t matrix_scan(void) {
    uint8_t code = ibmpc_host_recv();

    if (!code) {
        return 0;
    }

    xprintf("%02X ", code);

    if (ibmpc_host_protocol() == IBMPC_PROTOCOL_XT) {
        process_set1_scancode(code);
    } else {
        process_set2_scancode(code);
    }

    matrix_scan_kb();
    return 1;
}

inline
uint8_t matrix_get_row(uint8_t row) {
    return matrix[row];
}

inline static void matrix_make(uint8_t code) {
    if (!matrix_is_on(ROW(code), COL(code))) {
        matrix[ROW(code)] |= 1 << COL(code);
    }
}

inline static void matrix_break(uint8_t code) {
    if (matrix_is_on(ROW(code), COL(code))) {
        matrix[ROW(code)] &= ~(1 << COL(code));
    }
}

void matrix_clear(void) {
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) matrix[i] = 0x00;
}

bool matrix_is_on(uint8_t row, uint8_t col) {
    return (matrix_get_row(row) & (1 << col));
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
