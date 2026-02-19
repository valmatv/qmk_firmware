// SPDX-License-Identifier: GPL-2.0-or-later
#include QMK_KEYBOARD_H

/*
 * VIA + Debug keymap for converter/ibmpc_usb/rpi_pico
 *
 * Layer 0 : default IBM XT layout (same as the default keymap)
 * Layers 1-3 : transparent — fully configurable via VIA at runtime
 *
 * Debug features enabled by this keymap:
 *   - debug_enable  : console prints from xprintf / dprintf  (always on via matrix_init)
 *   - debug_matrix  : logs every matrix make/break to console (enabled in post_init below)
 *   - DEBUG_MATRIX_SCAN_RATE (config.h): reports scans/sec to console every second
 *
 * View output with:  qmk console   (or hid_listen)
 */

/* Convenience shorthand */
#define ___ KC_TRNS

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    /* Layer 0 — IBM XT
     * ,-------.  ,--------------------------------------------------------------------------.
     * | F1| F2|  |Esc|  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|  BS  |NumLck |ScrLck |
     * |-------|  |--------------------------------------------------------------------------|
     * | F3| F4|  | Tab |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P|  [|  ] |   |  7|  8|  9|  -|
     * |-------|  |------------------------------------------------------|Ent|---------------|
     * | F5| F6|  | Ctrl |  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '|  `|   |  4|  5|  6|   |
     * |-------|  |----------------------------------------------------------------------|   |
     * | F7| F8|  |Shif|  \|  Z|  X|  C|  V|  B|  N|  M|  ,|  .|  /|Shift|  *|  1|  2|  3|  +|
     * |-------|  |----------------------------------------------------------------------|   |
     * | F9|F10|  |  Alt  |               Space                  |CapsLck|   0   |   .   |   |
     * `-------'  `--------------------------------------------------------------------------'
     */
    [0] = LAYOUT_xt(
        KC_F1,   KC_F2,     KC_ESC,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_NUM,           KC_SCRL,
        KC_F3,   KC_F4,     KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,          KC_P7,   KC_P8,   KC_P9,   KC_PMNS,
        KC_F5,   KC_F6,     KC_LCTL, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT, KC_GRV,  KC_ENT,  KC_P4,   KC_P5,   KC_P6,
        KC_F7,   KC_F8,     KC_LSFT, KC_BSLS, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT, KC_PAST, KC_P1,   KC_P2,   KC_P3,   KC_PPLS,
        KC_F9,   KC_F10,    KC_LALT,                                              KC_SPC,                                      KC_CAPS,          KC_P0,            KC_PDOT
    ),

    /* Layers 1–3 — transparent, fully remappable via VIA */
    [1] = LAYOUT_xt(
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,              ___,
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,              ___,     ___,     ___,     ___,
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,
        ___,     ___,       ___,                                                  ___,                                         ___,              ___,              ___
    ),

    [2] = LAYOUT_xt(
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,              ___,
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,              ___,     ___,     ___,     ___,
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,
        ___,     ___,       ___,                                                  ___,                                         ___,              ___,              ___
    ),

    [3] = LAYOUT_xt(
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,              ___,
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,              ___,     ___,     ___,     ___,
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,
        ___,     ___,       ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,     ___,
        ___,     ___,       ___,                                                  ___,                                         ___,              ___,              ___
    ),
};

/* Enable full matrix debug logging for this keymap on startup.
 * Output visible via: qmk console  (or hid_listen)
 * Toggle at runtime : LShift+RShift or LCtrl+RShift (Command feature shortcut, type 'd')
 */
void keyboard_post_init_user(void) {
    debug_enable = true;
    debug_matrix = true;
    xprintf("[via_debug] debug_enable=1 debug_matrix=1\n");
}
