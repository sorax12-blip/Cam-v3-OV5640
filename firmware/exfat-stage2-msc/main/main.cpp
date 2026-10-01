#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <USB.h>
#include <USBMSC.h>

extern "C" {
#include "diskio.h"
}

static constexpr int SD_CLK = 39;
static constexpr int SD_CMD = 38;
static constexpr int SD_D0  = 40;

static USBMSC MSC;

static uint8_t sdPdrv = 0xFF;
static uint32_t physicalSectors = 0;
static uint16_t physicalSectorSize = 512;

static volatile uint64_t mscReadBytes = 0;
static volatile uint64_t mscWriteBytes = 0;
static volatile uint32_t mscReadOps = 0;
static volatile uint32_t mscWriteOps = 0;
static volatile uint32_t mscReadFail = 0;
static volatile uint32_t mscWriteFail = 0;

static bool findSdPhysicalDrive() {
  const uint32_t wantedSectors =
      (uint32_t)(SD_MMC.cardSize() / (uint64_t)SD_MMC.sectorSize());
  const uint16_t wantedSectorSize = (uint16_t)SD_MMC.sectorSize();

  for (uint8_t pdrv = 0; pdrv < 10; ++pdrv) {
    LBA_t sectors = 0;
    WORD sectorSize = 0;

    if (disk_ioctl(pdrv, GET_SECTOR_COUNT, &sectors) != RES_OK) continue;
    if (disk_ioctl(pdrv, GET_SECTOR_SIZE, &sectorSize) != RES_OK) continue;

    if ((uint64_t)sectors == (uint64_t)wantedSectors &&
        sectorSize == wantedSectorSize) {
      sdPdrv = pdrv;
      physicalSectors = wantedSectors;
      physicalSectorSize = wantedSectorSize;
      return true;
    }
  }
  return false;
}

static int32_t onRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  ++mscReadOps;

  if (sdPdrv == 0xFF || physicalSectorSize == 0) {
    ++mscReadFail;
    return -1;
  }

  if (offset == 0 && (bufsize % physicalSectorSize) == 0) {
    const uint32_t count = bufsize / physicalSectorSize;
    if ((uint64_t)lba + count > physicalSectors) {
      ++mscReadFail;
      return -1;
    }
    if (disk_read(sdPdrv, static_cast<BYTE *>(buffer), lba, count) != RES_OK) {
      ++mscReadFail;
      return -1;
    }
    mscReadBytes += bufsize;
    return (int32_t)bufsize;
  }

  uint8_t sector[512];
  if (physicalSectorSize != sizeof(sector) ||
      lba >= physicalSectors ||
      offset + bufsize > physicalSectorSize) {
    ++mscReadFail;
    return -1;
  }

  if (disk_read(sdPdrv, sector, lba, 1) != RES_OK) {
    ++mscReadFail;
    return -1;
  }
  memcpy(buffer, sector + offset, bufsize);
  mscReadBytes += bufsize;
  return (int32_t)bufsize;
}

static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  ++mscWriteOps;

  if (sdPdrv == 0xFF || physicalSectorSize == 0) {
    ++mscWriteFail;
    return -1;
  }

  if (offset == 0 && (bufsize % physicalSectorSize) == 0) {
    const uint32_t count = bufsize / physicalSectorSize;
    if ((uint64_t)lba + count > physicalSectors) {
      ++mscWriteFail;
      return -1;
    }
    if (disk_write(sdPdrv, buffer, lba, count) != RES_OK) {
      ++mscWriteFail;
      return -1;
    }
    mscWriteBytes += bufsize;
    return (int32_t)bufsize;
  }

  uint8_t sector[512];
  if (physicalSectorSize != sizeof(sector) ||
      lba >= physicalSectors ||
      offset + bufsize > physicalSectorSize) {
    ++mscWriteFail;
    return -1;
  }

  if (disk_read(sdPdrv, sector, lba, 1) != RES_OK) {
    ++mscWriteFail;
    return -1;
  }
  memcpy(sector + offset, buffer, bufsize);
  if (disk_write(sdPdrv, sector, lba, 1) != RES_OK) {
    ++mscWriteFail;
    return -1;
  }

  mscWriteBytes += bufsize;
  return (int32_t)bufsize;
}

static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
  Serial.printf("MSC START/STOP power=%u start=%u eject=%u\n",
                power_condition, start, load_eject);
  return true;
}

static void usbEventCallback(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
  if (event_base != ARDUINO_USB_EVENTS) return;

  switch (event_id) {
    case ARDUINO_USB_STARTED_EVENT:
      Serial.println("USB EVENT: PLUGGED / CONFIGURED");
      break;
    case ARDUINO_USB_STOPPED_EVENT:
      Serial.println("USB EVENT: UNPLUGGED");
      break;
    case ARDUINO_USB_SUSPEND_EVENT:
      Serial.println("USB EVENT: SUSPENDED");
      break;
    case ARDUINO_USB_RESUME_EVENT:
      Serial.println("USB EVENT: RESUMED");
      break;
    default:
      break;
  }
}

static bool stage2FilesystemProof() {
  const char *path = "/EXFAT_STAGE2_MSC.TXT";

  File f = SD_MMC.open(path, FILE_WRITE);
  if (!f) {
    Serial.println("FAIL: could not create Stage 2 marker file");
    return false;
  }
  f.println("ESP32 Cam HD exFAT Stage 2 USB MSC");
  f.printf("boot_millis=%lu\n", (unsigned long)millis());
  f.flush();
  f.close();

  f = SD_MMC.open(path, FILE_READ);
  if (!f) {
    Serial.println("FAIL: could not reopen Stage 2 marker file");
    return false;
  }

  Serial.println("STAGE2 FILE READBACK:");
  while (f.available()) Serial.write(f.read());
  f.close();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("=== ESP32 Cam HD exFAT STAGE 2 USB MSC ===");
  Serial.println("Temporary diagnostic build: writable USB mass storage.");
  Serial.println("SD pins: CLK=39 CMD=38 D0=40, 1-bit.");

  if (!SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0)) {
    Serial.println("FAIL: SD_MMC.setPins()");
    return;
  }

  if (!SD_MMC.begin("/sdcard", true, false, 20000, 5)) {
    Serial.println("FAIL: exFAT SD_MMC.begin()");
    return;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("FAIL: no SD card");
    return;
  }

  physicalSectorSize = (uint16_t)SD_MMC.sectorSize();
  physicalSectors =
      (uint32_t)(SD_MMC.cardSize() / (uint64_t)physicalSectorSize);

  Serial.printf("exFAT MOUNT OK: %.2f GiB, sector=%u, physical sectors=%lu\n",
                (double)SD_MMC.cardSize() / (1024.0 * 1024.0 * 1024.0),
                physicalSectorSize,
                (unsigned long)physicalSectors);

  if (!stage2FilesystemProof()) return;
  Serial.println("Stage 2 marker file WRITE/READ OK.");

  if (!findSdPhysicalDrive()) {
    Serial.println("FAIL: could not locate FatFs SD physical drive");
    return;
  }

  Serial.printf("Fast raw SD layer: pdrv=%u, %lu sectors x %u bytes\n",
                sdPdrv,
                (unsigned long)physicalSectors,
                physicalSectorSize);

  Serial.println("Starting native USB MSC...");
  Serial.println("From this point, firmware will NOT access files through FatFs.");
  Serial.println("Connect Samsung to the connector labeled USB-OTG using the OTG adapter path.");

  USB.onEvent(usbEventCallback);

  USB.VID(0x303A);
  USB.PID(0x1001);
  USB.productName("ESP32 Cam HD exFAT");
  USB.manufacturerName("ESP32 Cam HD");
  USB.serialNumber("ESP32CAMHDEXFAT");

  MSC.vendorID("ESP32");
  MSC.productID("CAM_HD_EXFAT");
  MSC.productRevision("2.0");
  MSC.onStartStop(onStartStop);
  MSC.onRead(onRead);
  MSC.onWrite(onWrite);
  MSC.mediaPresent(true);
  MSC.isWritable(true);

  if (!MSC.begin(physicalSectors, physicalSectorSize)) {
    Serial.println("FAIL: MSC.begin()");
    return;
  }

  if (!USB.begin()) {
    Serial.println("FAIL: USB.begin()");
    return;
  }

  Serial.println("USB MSC READY.");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= 3000) {
    last = millis();
    Serial.printf("MSC stats: reads=%lu fail=%lu read=%.2f MiB, writes=%lu fail=%lu written=%.2f KiB\n",
                  (unsigned long)mscReadOps,
                  (unsigned long)mscReadFail,
                  (double)mscReadBytes / (1024.0 * 1024.0),
                  (unsigned long)mscWriteOps,
                  (unsigned long)mscWriteFail,
                  (double)mscWriteBytes / 1024.0);
  }
  delay(20);
}
