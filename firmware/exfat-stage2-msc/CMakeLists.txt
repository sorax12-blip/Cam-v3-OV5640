cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Native USB TinyUSB device mode (Arduino "USB-OTG" mode)
list(APPEND compile_definitions "ARDUINO_USB_MODE=0")
list(APPEND compile_definitions "ARDUINO_USB_CDC_ON_BOOT=0")
list(APPEND compile_definitions "ARDUINO_USB_MSC_ON_BOOT=0")
list(APPEND compile_definitions "ARDUINO_USB_DFU_ON_BOOT=0")

project(esp32_cam_hd_exfat_stage2_msc)
