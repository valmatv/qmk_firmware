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

/* IBM PC keyboard clock and data pins */
#define IBMPC_CLOCK_PIN GP1
#define IBMPC_DATA_PIN  GP0

/* Optional: hardware reset pin for XT keyboards */
/* #define IBMPC_RST_PIN GP2 */

/* key combination for command */
#define IS_COMMAND() ( \
    get_mods() == (MOD_BIT(KC_LSFT) | MOD_BIT(KC_RSFT)) || \
    get_mods() == (MOD_BIT(KC_LCTL) | MOD_BIT(KC_RSFT)) \
)
