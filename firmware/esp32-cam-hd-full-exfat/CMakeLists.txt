cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Native USB device stack (Arduino USB-OTG / TinyUSB mode).
# Serial remains on UART0 / the board's USB-UART bridge.
list(APPEND compile_definitions "ARDUINO_USB_MODE=0")
list(APPEND compile_definitions "ARDUINO_USB_CDC_ON_BOOT=0")
list(APPEND compile_definitions "ARDUINO_USB_MSC_ON_BOOT=0")
list(APPEND compile_definitions "ARDUINO_USB_DFU_ON_BOOT=0")

project(esp32_cam_hd_full_exfat)
