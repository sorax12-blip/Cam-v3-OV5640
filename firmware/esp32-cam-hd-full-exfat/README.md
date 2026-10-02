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
- OV3660 saved stills: QXGA 2048x1536, JPEG Q4
- OV5640 saved stills: QSXGA 2560x1920, JPEG Q4
- OV5640 sensor vertical flip enabled
- OV5640 saved JPEGs carry EXIF Orientation 8 (display 90° counter-clockwise)
- motion checks: 5 unsaved QQVGA comparison frames every 7.5 seconds
- >=10% changed area starts event capture
- event photos every 600 ms
- event recheck every 2 minutes
- nominal event block: 200 photos
- photo folders: 2,000 slots each (10 complete event blocks)

BLE:
- ESP32 Cam HD
- button GPIO21
- blue LED GPIO47
- Live default: HVGA 480x320, Q20, 250 ms target
- capture pauses during BLE

Startup:
- green LED GPIO14 flashes 3 Hz for 5 seconds before capture starts
