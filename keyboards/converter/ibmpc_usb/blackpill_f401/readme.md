# IBM PC Keyboard Converter — BlackPill STM32F401

IBM PC to USB converter firmware for the WeAct BlackPill (STM32F401CCU6).
Supports XT (Scan Code Set 1) and AT/PS2 (Scan Code Set 2) keyboards with
automatic protocol detection, plus Scan Code Set 3 terminal keyboards.

## Hardware

| Item | Detail |
|------|--------|
| MCU | STM32F401CCU6 (WeAct BlackPill v1.3 / v2.x) |
| Bootloader | STM32 DFU (`0483:df11`) |
| Clock | HSE 25 MHz → PLL → 84 MHz |

### Wiring

```
 DIN-5 (XT)     PS/2 Mini-DIN-6      BlackPill
 ─────────       ──────────────       ─────────
 Pin 1 CLK       Pin 5 CLK  ──[4.7kΩ]──3.3V
 Pin 2 DATA      Pin 1 DATA ──[4.7kΩ]──3.3V
 Pin 1 CLK  ─────Pin 5 CLK  ───────────────── A0  (EXTI0)
 Pin 2 DATA ─────Pin 1 DATA ───────────────── A1
 Pin 4 GND  ─────Pin 3 GND  ───────────────── GND
 Pin 5 +5V  ─────Pin 4 +5V  ───────────────── 5V rail
```

Optional hardware reset (XT keyboards):
- `A2` → XT reset line 0
- `A3` → XT reset line 1

## Features

- Auto-detects XT (10-bit) and PS/2 AT (11-bit) framing on power-up
- Full Scan Code Set 1, 2 and 3 processing (CS2 → CS1 internal re-mapping)
- IBM 5576-002/003 and Televideo DEC keyboard support
- LED indicators (NumLock, CapsLock, ScrollLock) forwarded to PS/2
- N-Key Rollover (NKRO) enabled
- QMK Command mode: LShift+RShift or LCtrl+RShift

## Keymaps

| Keymap | Description |
|--------|-------------|
| `default` | Single-layer IBM XT 84-key layout |
| `via` | 4 programmable layers; configure with [VIA](https://usevia.app) |

## Building

```bash
# Default keymap
qmk compile -kb converter/ibmpc_usb/blackpill_f401 -km default

# VIA keymap
qmk compile -kb converter/ibmpc_usb/blackpill_f401 -km via
```

## Flashing

1. Enter DFU bootloader: hold **BOOT0**, tap **NRST**, release **BOOT0**
2. Verify the device appears: `lsusb` should show `0483:df11 STMicroelectronics STM Device in DFU Mode`
3. Flash:
   ```bash
   dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave \
       -D converter_ibmpc_usb_blackpill_f401_via.bin
   ```
4. Alternatively: use [QMK Toolbox](https://github.com/qmk/qmk_toolbox)

## VIA / QMK Toolbox

- **Vendor ID**: `0xFEED`
- **Product ID**: `0x6514`  (rpi_pico variant uses `0x6513`)

## Notes

- `board_init()` in `blackpill_f401.c` reconfigures **B9** from the default
  I2C1_SDA alternate function to a plain floating input.  Without this the
  STM32F401 board file's AF assignment can interfere with open-drain bus
  behaviour on PA0/PA1.
- The PAL callback is registered **before** `palEnableLineEvent()` — the
  STM32 EXTI controller silently drops a falling edge if no callback is
  registered at the time the event fires; the RP2040 tolerates the reverse
  order.
- VIA EEPROM uses `wear_leveling` over embedded flash (2 KB logical / one
  16 KB STM32F4 flash sector consumed regardless of the setting).
