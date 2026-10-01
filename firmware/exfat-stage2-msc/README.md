# ESP32 Cam HD — exFAT Stage 2 USB MSC

Diagnostic build only.

Purpose:
- Mount the 128 GB exFAT microSD on the ESP32-S3
- Create/read a small Stage 2 marker file
- Expose the full physical card through native ESP32-S3 USB Mass Storage
- Test mounting from Samsung My Files

Hardware:
- SDMMC CLK GPIO39
- SDMMC CMD GPIO38
- SDMMC D0 GPIO40
- 1-bit SDMMC
- Native USB-OTG connector for phone storage access

Phone connection:
Samsung USB-C -> Samsung USB-C male to USB-A female OTG adapter ->
USB-A male to USB-C male data cable -> ESP32 connector labeled USB-OTG

Do NOT use the USB-UART connector for the mass-storage test.

This Stage 2 build is temporarily writable to behave like a normal USB card reader.
