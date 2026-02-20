/* Copyright 2020 Nick Brassel (tzarc)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * STM32F401 MCU configuration overrides for the IBM PC keyboard converter.
 *
 * Only TIM5 is enabled for PWM — this matches the default BlackPill F401
 * QMK board file and is needed if a status LED or buzzer is desired.
 * I2C1 is left disabled (default) because board_init() reconfigures B9
 * which the board file maps to I2C1_SDA.
 */
#pragma once

#include_next "mcuconf.h"

/* Enable TIM5 for PWM (used by optional status indicators / backlight) */
#undef STM32_PWM_USE_TIM5
#define STM32_PWM_USE_TIM5 TRUE
