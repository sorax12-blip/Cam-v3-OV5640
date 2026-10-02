cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Arduino native TinyUSB device mode (equivalent to Arduino IDE USB-OTG).
list(APPEND compile_definitions "ARDUINO_USB_MODE=0")
list(APPEND compile_definitions "ARDUINO_USB_CDC_ON_BOOT=0")
list(APPEND compile_definitions "ARDUINO_USB_MSC_ON_BOOT=0")
list(APPEND compile_definitions "ARDUINO_USB_DFU_ON_BOOT=0")

project(esp32_cam_hd_full_exfat)
