# MCU name
MCU = STM32F401

# Bootloader selection
BOOTLOADER = stm32-dfu

# Share USB endpoints — required when console+mouse+extrakey are all enabled
KEYBOARD_SHARED_EP = yes

# Custom matrix driven by ibmpc_host.c state machine
CUSTOM_MATRIX = yes

SRC += matrix.c ibmpc_host.c
