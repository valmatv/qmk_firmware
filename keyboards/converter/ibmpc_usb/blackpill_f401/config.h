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

#pragma once

/* key matrix size */
#define MATRIX_ROWS 16  // keycode bit: 3-0
#define MATRIX_COLS 8   // keycode bit: 6-4

/*
 * Pin configuration for the IBM PC Keyboard Converter on BlackPill F401
 *
 * A0 / PA0 — Clock line (must be an external-interrupt-capable pin, EXTI0)
 * A1 / PA1 — Data line
 * A2 / PA2 — Hardware reset output 0 (XT keyboards, optional)
 * A3 / PA3 — Hardware reset output 1 (XT keyboards, optional)
 *
 * Wiring:
 *   DIN-5 / PS/2 plug                 BlackPill
 *   ────────────────────────────────────────────
 *   Pin 1  Clock  ──── 4.7 kΩ ── 3.3V           (keyboard-side pull-up)
 *   Pin 2  Data   ──── 4.7 kΩ ── 3.3V           (keyboard-side pull-up)
 *   Pin 1  Clock  ─────────────────────────────► A0
 *   Pin 2  Data   ─────────────────────────────► A1
 *   Pin 4  GND    ─────────────────────────────► GND
 *   Pin 5  +5V    ─────────────────────────────► 5V (or regulated 5V supply)
 */
#define IBMPC_CLOCK_PIN   A0
#define IBMPC_DATA_PIN    A1

/* Optional: hardware XT reset lines — comment out if not wired */
#define IBMPC_RST_PIN0    A2
#define IBMPC_RST_PIN1    A3

/* key combination to enter Command mode (debug commands) */
#define IS_COMMAND() ( \
    get_mods() == (MOD_BIT(KC_LSFT) | MOD_BIT(KC_RSFT)) || \
    get_mods() == (MOD_BIT(KC_LCTL) | MOD_BIT(KC_RSFT)) \
)
