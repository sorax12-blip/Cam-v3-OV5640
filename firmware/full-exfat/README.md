# ESP32 Cam HD — Full exFAT Integration

Full camera firmware integrating the proven Stage 2 storage path.

## Retained camera behavior

- ESP32-S3-N16R8
- OV3660 / OV5640 PID auto-detection
- QXGA 2048x1536 saved JPEGs, quality 4
- OV3660 vertical flip ON; OV5640 vertical flip OFF
- Five unsaved QQVGA comparison frames every 10 seconds
- 10% motion threshold
- Event capture every 750 ms
- Two-minute event blocks with five-frame recheck
- 1,600 photo-number slots per folder and 160-photo block reservation rule
- BLE button GPIO21
- Blue BLE LED GPIO47
- Green startup LED GPIO14
- BLE device name: ESP32 Cam HD
- Live default: HVGA 480x320, Q20, 250 ms
- USB has priority over BLE/capture

## Storage changes

- FAT32 + exFAT capable FatFs build
- Full physical SD capacity advertised over USB MSC
- Multi-sector raw SD reads for USB
- Final USB MSC is read-only
- Native USB product: ESP32 Cam HD
- Direct USB-C-to-USB-C supported through the connector labeled USB-OTG

## microSD pins

- CLK GPIO39
- CMD GPIO38
- D0 GPIO40
- 1-bit SDMMC

## Important

The USB mass-storage side is intentionally READ-ONLY in this full build.
The phone/computer may browse and copy camera files, but cannot modify the card
through USB MSC while the camera filesystem is mounted.
