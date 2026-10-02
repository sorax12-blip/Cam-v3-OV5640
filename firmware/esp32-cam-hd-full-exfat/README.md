# ESP32 Cam HD — Full exFAT Firmware

Full camera firmware built with ESP-IDF 5.5.5 + Arduino-ESP32 3.3.12.

Storage:
- exFAT microSD support
- built-in 1-bit SDMMC: CLK 39, CMD 38, D0 40
- physical card capacity exposed over native USB MSC
- USB MSC is read-only in the full camera build
- direct USB-C to USB-C is supported on the native USB-OTG connector

Camera:
- OV3660 / OV5640 PID auto-detect
- QXGA 2048x1536 saved JPEG profile (shared temporary profile)
- motion checks: 5 unsaved QQVGA comparison frames every 10 seconds
- >=10% changed area starts event capture
- event photos every 750 ms
- event recheck every 2 minutes

BLE:
- ESP32 Cam HD
- button GPIO21
- blue LED GPIO47
- Live default: HVGA 480x320, Q20, 250 ms target
- capture pauses during BLE

Startup:
- green LED GPIO14 flashes 3 Hz for 5 seconds before capture starts
