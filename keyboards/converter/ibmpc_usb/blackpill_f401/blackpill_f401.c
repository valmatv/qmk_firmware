/* Copyright 2020 Sergey Vlasov (sigprof)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H

void board_init(void) {
    /*
     * The STM32F401 BlackPill board file configures B9 as I2C1_SDA (alternate
     * function AF4).  This AF assignment persists on boot and can interfere
     * with open-drain bus behaviour on any pin connected to that net.
     *
     * Reconfigure B9 as a plain floating input to release the AF so the rest
     * of the GPIO subsystem works correctly for the PS/2 / XT clock and data
     * lines on A0 / A1.
     *
     * Reference: purdeaandrei/vial-qmk-with-ibmpc-usb-converter
     *   keyboards/converter/ibmpc_usb/blackpill_f401/blackpill_f401.c
     */
    setPinInputHigh(B9);
}
