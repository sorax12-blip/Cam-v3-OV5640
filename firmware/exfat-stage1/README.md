# ESP32 Cam HD — exFAT Stage 1

Proof build only. This is not the final camera firmware.

It builds Arduino-ESP32 3.3.12 as an ESP-IDF 5.5.5 component and uses a project-local copy of ESP-IDF FatFs with exFAT enabled.

Hardware assumptions:
- ESP32-S3-N16R8
- Built-in microSD
- 1-bit SDMMC
- CLK GPIO39
- CMD GPIO38
- D0 GPIO40
- 64 GB to 256 GB exFAT card

Expected Serial output with a working exFAT card:

MOUNT OK.
WRITE OK
READBACK:
ESP32 Cam HD exFAT read/write test
...
PASS: exFAT mounted and file read/write succeeded.

The GitHub Actions workflow builds a merged ESP32-S3 binary for easier phone-side downloading/flashing.
