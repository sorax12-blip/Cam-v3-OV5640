#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>

// ESP32 Cam HD built-in microSD, 1-bit SDMMC
static constexpr int SD_CLK = 39;
static constexpr int SD_CMD = 38;
static constexpr int SD_D0  = 40;

static void listRoot() {
  File root = SD_MMC.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("FAIL: could not open root directory");
    return;
  }

  Serial.println("Root directory:");
  File f = root.openNextFile();
  while (f) {
    Serial.printf("  %s%s  %llu bytes\n",
                  f.name(),
                  f.isDirectory() ? "/" : "",
                  (unsigned long long)f.size());
    f = root.openNextFile();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("=== ESP32 Cam HD exFAT STAGE 1 ===");
  Serial.println("Goal: prove exFAT mount + read/write on built-in SDMMC.");
  Serial.println("Pins: CLK=39 CMD=38 D0=40, 1-bit mode.");

  if (!SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0)) {
    Serial.println("FAIL: SD_MMC.setPins()");
    return;
  }

  if (!SD_MMC.begin("/sdcard", true, false, 20000, 5)) {
    Serial.println("FAIL: SD_MMC.begin() - exFAT did not mount");
    return;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("FAIL: no SD card detected");
    return;
  }

  Serial.printf("MOUNT OK. Card: %.2f GiB, sector=%u\n",
                (double)SD_MMC.cardSize() / (1024.0 * 1024.0 * 1024.0),
                (unsigned)SD_MMC.sectorSize());

  const char *path = "/EXFAT_STAGE1_TEST.TXT";

  {
    File f = SD_MMC.open(path, FILE_WRITE);
    if (!f) {
      Serial.println("FAIL: could not create test file");
      return;
    }
    f.println("ESP32 Cam HD exFAT read/write test");
    f.printf("millis=%lu\n", (unsigned long)millis());
    f.flush();
    f.close();
    Serial.println("WRITE OK");
  }

  {
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) {
      Serial.println("FAIL: could not reopen test file");
      return;
    }
    Serial.println("READBACK:");
    while (f.available()) Serial.write(f.read());
    f.close();
  }

  listRoot();

  Serial.println();
  Serial.println("PASS: exFAT mounted and file read/write succeeded.");
  Serial.println("Next step: add USB MSC, then merge into full camera firmware.");
}

void loop() {
  delay(1000);
}
