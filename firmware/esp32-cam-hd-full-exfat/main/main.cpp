struct BleIndexStats;  // Forward declaration for Arduino .ino auto-generated prototypes

/*
 * Cam v3 Motion-Triggered Photo Logger - ESP32-S3 N16R8 + OV3660/OV5640 + microSD
 * Arduino-ESP32 3.x
 *
 * Capture behavior:
 *   MONITORING MODE
 *   - Every 7.5 seconds, capture 5 low-resolution comparison frames in rapid
 *     succession. These frames are NEVER written to the SD card.
 *   - The 5 frames are reduced to coarse grayscale signatures and compared.
 *   - If the average changed-area score is >= 10%, enter EVENT CAPTURE mode.
 *
 *   EVENT CAPTURE MODE
 *   - Save one full-quality photo every 600 ms.
 *   - After 2 minutes of event photos, stop briefly and capture another 5-frame
 *     comparison burst (also not saved).
 *   - If the comparison score is still >= 10%, continue for another 2-minute
 *     event block. Otherwise return to MONITORING MODE.
 *
 * Motion comparison:
 *   - Comparison frames are QQVGA (160x120) JPEGs used only in RAM.
 *   - Each is decoded and reduced to a 40x30 grayscale signature (1200 cells).
 *   - Frame #1 is the reference; frames #2-#5 are each compared with it.
 *   - A cell counts as changed when its normalized brightness differs by >= 18/255.
 *   - Uniform whole-frame brightness shifts are compensated before comparison
 *     to reduce false triggers from auto-exposure or lighting changes.
 *   - The four changed-cell percentages are averaged. >=10% triggers motion.
 *
 * Saved photos:
 *   - OV3660 and OV5640 are identified automatically from the sensor PID at startup.
 *   - OV3660 keeps its proven 2048x1536 (QXGA), JPEG quality 4 profile.
 *   - OV5640 uses its maximum esp32-camera supported still size,
 *     2560x1920 (QSXGA), JPEG quality 4.
 *   - Existing /Photos/00001... folder and F#_Pic_#.JPG naming is retained.
 *   - Each numbered folder holds up to 2,000 photos (10 full 2-minute event blocks).
 *   - Video recording has been removed completely.
 *
 * Bluetooth LE SD browser:
 *   - BLE broadcast name is ESP32 Cam HD. The web viewer and Android app use the same name.
 *     The actual camera sensor is detected and reported over Serial at startup.
 *   - GPIO21 normally-open momentary button to GND requests BLE.
 *   - A press during a comparison burst or saved photo is latched; that complete
 *     operation finishes before BLE starts.
 *   - BLE advertises for 30 seconds; blue LED GPIO47 blinks 4 times/sec while
 *     waiting and stays solid while connected. No connection -> BLE shuts off.
 *   - Capture is paused while BLE is active.
 *   - LIVE ON starts an unsaved BLE camera preview; LIVE OFF returns to SD browsing.
 *   - LIVE SET lets the diagnostic web viewer tune QVGA/HVGA/VGA/SVGA, JPEG
 *     quality and target interval at runtime. The firmware remains dual-sensor;
 *     Live_Test_01 is simply being tuned first on the OV3660 hardware.
 *   - Live preview defaults to HVGA (480x320), JPEG quality 20, 250 ms target,
 *     and sends frames over the existing data characteristic. Nothing from Live is saved.
 *
 * Startup LED:
 *   - Green LED GPIO14 blinks 3 times/sec for 5 seconds at power-up.
 *   - No camera capture starts until the 5-second sequence is complete.
 *
 * USB-C read-only exFAT SD mass storage is retained. USB has priority over BLE.
 *   - Android/phone-first MSC: the SD LUN is reported present before USB
 *     enumeration so Samsung/Android can mount it immediately over direct USB-C or USB-OTG.
 */
#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <FS.h>
#include <SD_MMC.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#if !SOC_USB_OTG_SUPPORTED || ARDUINO_USB_MODE
#error "USB Mass Storage requires ESP32-S3 native USB in USB-OTG (TinyUSB) mode. In Arduino IDE select Tools > USB Mode > USB-OTG (TinyUSB)."
#endif
#include <USB.h>
#include <USBMSC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "esp_heap_caps.h"
#include "driver/sdmmc_host.h"
extern "C" {
#include "diskio.h"
}

// ESP32-S3-N16R8 + OV3660 / OV5640 Camera Pin Configuration
static const int CAM_PWDN = -1;
static const int CAM_RESET = -1;

static const int CAM_XCLK = 15;
static const int CAM_SIOD = 4;
static const int CAM_SIOC = 5;

static const int CAM_D0 = 11;
static const int CAM_D1 = 9;
static const int CAM_D2 = 8;
static const int CAM_D3 = 10;
static const int CAM_D4 = 12;
static const int CAM_D5 = 18;
static const int CAM_D6 = 17;
static const int CAM_D7 = 16;

static const int CAM_VSYNC = 6;
static const int CAM_HREF = 7;
static const int CAM_PCLK = 13;

// MicroSD Card Pin Configuration
static const int SD_CLK = 39;
static const int SD_CMD = 38;
static const int SD_D0 = 40;

// V3 external controls / LEDs. These pins are unused by the camera, SD card,
// PSRAM and native USB on the pinout supplied with the new board.
static const int GREEN_STATUS_LED_PIN = 14;
static const int BLUE_BLE_LED_PIN = 47;
static const int BLE_MOMENTARY_BUTTON_PIN = 21;

// Momentary BLE button: normally open between GPIO21 and GND. INPUT_PULLUP keeps
// the pin HIGH until pressed. A FALLING-edge interrupt latches presses even if
// they occur during a blocking comparison burst or saved-photo operation.
static constexpr uint32_t BLE_BUTTON_DEBOUNCE_MS = 250;
static constexpr uint32_t BLE_ADVERTISING_TIMEOUT_MS = 30000;

// 4 complete blue flashes per second = 250 ms per flash cycle, so toggle the
// LED every 125 ms for a 50% duty cycle while advertising.
static constexpr uint32_t BLUE_LED_TOGGLE_MS = 125;

// Startup green indicator: 15 complete flashes over exactly ~5 seconds.
static constexpr uint8_t GREEN_STARTUP_FLASHES = 15;
static constexpr uint32_t GREEN_LED_ON_MS = 166;
static constexpr uint32_t GREEN_LED_OFF_MS = 167;

// -------------------------------------------------------------------------

// Motion / event capture timing.
static constexpr uint32_t MOTION_CHECK_INTERVAL_MS = 7500;
static constexpr uint8_t MOTION_COMPARE_FRAMES = 5;
static constexpr float MOTION_TRIGGER_PERCENT = 10.0f;
static constexpr uint8_t MOTION_CELL_BRIGHTNESS_DELTA = 18;
static constexpr uint32_t EVENT_PHOTO_INTERVAL_MS = 600;
static constexpr uint32_t EVENT_RECHECK_INTERVAL_MS = 120000;
// One uninterrupted 2-minute block nominally saves 200 photos:
// t=0, 0.60 s, ... 119.40 s, then the 120 s comparison takes priority.
static constexpr uint32_t PHOTOS_PER_EVENT_BLOCK =
    EVENT_RECHECK_INTERVAL_MS / EVENT_PHOTO_INTERVAL_MS;

// Comparison resolution/signature geometry. QQVGA is 160x120.
static constexpr uint16_t MOTION_COMPARE_WIDTH = 160;
static constexpr uint16_t MOTION_COMPARE_HEIGHT = 120;
static constexpr uint8_t MOTION_SIG_WIDTH = 40;
static constexpr uint8_t MOTION_SIG_HEIGHT = 30;
static constexpr uint16_t MOTION_SIG_CELLS = MOTION_SIG_WIDTH * MOTION_SIG_HEIGHT;
static constexpr uint8_t MOTION_COMPARE_JPEG_QUALITY = 20;

// -------------------------------------------------------------------------
// LOW-POWER WAIT SETTINGS (no artificial rest)
//
// Full speed is retained for camera capture, JPEG comparison and SD writes.
// Between scheduled captures the CPU drops to 80 MHz and the camera is put in
// a tiny low-data-rate mode. This is ordinary between-event idling, not an
// added post-capture rest.
static constexpr uint32_t BATTERY_IDLE_CPU_MHZ = 80;
static constexpr uint32_t BATTERY_ACTIVE_CPU_MHZ = 240;
static constexpr bool BATTERY_CAMERA_LOW_RATE_BETWEEN_PHOTOS = true;
static constexpr uint32_t BATTERY_IDLE_SLICE_MS = 100;
// -------------------------------------------------------------------------
// 2,000 photos per folder = 10 complete 2-minute event blocks at
// one saved photo every 600 ms (200 photos per 2-minute block).
// If a new/restarted block would begin with fewer than 200 slots left in the
// current folder, those remaining numbers are intentionally skipped and the
// next block starts at the first number of the next folder. Exactly 200 free
// slots is allowed so a complete nominal 2-minute block can still fit.
static constexpr uint32_t PHOTOS_PER_FOLDER = 2000;
static_assert(PHOTOS_PER_EVENT_BLOCK == 200, "2-minute block math changed");
static_assert((PHOTOS_PER_FOLDER % PHOTOS_PER_EVENT_BLOCK) == 0,
              "Folder size must contain whole event blocks");
static constexpr uint32_t MAX_MEDIA_FOLDERS = 99999;
static constexpr uint32_t MAX_PHOTO_NUMBER = MAX_MEDIA_FOLDERS * PHOTOS_PER_FOLDER;

// Saved still-photo quality. Lower JPEG quality number = less compression.
// OV3660 stays on its proven QXGA profile. OV5640 uses the maximum
// esp32-camera supported still size for this sensor: QSXGA (2560x1920).
static constexpr uint8_t PHOTO_JPEG_QUALITY = 4;
static constexpr framesize_t OV3660_SAVED_FRAME_SIZE = FRAMESIZE_QXGA;
static constexpr framesize_t OV5640_SAVED_FRAME_SIZE = FRAMESIZE_QSXGA;
// EXIF orientation 8 = display 90 degrees counter-clockwise.
// This rotates presentation without decoding/re-encoding the 5 MP JPEG.
static constexpr uint16_t OV5640_SAVED_EXIF_ORIENTATION = 8;

// Shared minimal JPEG APP1 Exif segment.
// Orientation 8 = display 90 degrees counter-clockwise.
static const uint8_t OV5640_EXIF_ORIENTATION_90_CCW[] = {
  0xFF, 0xE1, 0x00, 0x22,
  0x45, 0x78, 0x69, 0x66, 0x00, 0x00,       // "Exif\0\0"
  0x49, 0x49, 0x2A, 0x00,                   // little-endian TIFF header
  0x08, 0x00, 0x00, 0x00,                   // IFD0 offset
  0x01, 0x00,                               // one IFD entry
  0x12, 0x01,                               // tag 0x0112: Orientation
  0x03, 0x00,                               // type SHORT
  0x01, 0x00, 0x00, 0x00,                   // count = 1
  (uint8_t)OV5640_SAVED_EXIF_ORIENTATION, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00                    // no next IFD
};

static constexpr uint8_t CAMERA_FB_COUNT = 3;

static const char* PHOTO_ROOT = "/Photos";

static bool cameraReady = false;
static bool sdReady = false;
static bool qsxgaInitSucceeded = true;
static bool qxgaInitSucceeded = true;
static bool cameraWasInitializedOnce = false;

// Sensor is identified from sensor_t::id.PID immediately after camera init.
// This lets one firmware image run with either supported camera module.
enum CameraSensorModel : uint8_t {
  CAMERA_SENSOR_UNKNOWN = 0,
  CAMERA_SENSOR_OV3660,
  CAMERA_SENSOR_OV5640
};

static CameraSensorModel detectedCameraSensor = CAMERA_SENSOR_UNKNOWN;
static uint16_t detectedCameraPid = 0;

static CameraSensorModel cameraModelFromPid(uint16_t pid) {
  if (pid == OV3660_PID) return CAMERA_SENSOR_OV3660;
  if (pid == OV5640_PID) return CAMERA_SENSOR_OV5640;
  return CAMERA_SENSOR_UNKNOWN;
}

static const char* cameraModelName(CameraSensorModel model) {
  switch (model) {
    case CAMERA_SENSOR_OV3660: return "OV3660";
    case CAMERA_SENSOR_OV5640: return "OV5640";
    default: return "UNKNOWN";
  }
}

static framesize_t preferredSavedPhotoFrameSize(uint16_t pid) {
  if (pid == OV5640_PID) return OV5640_SAVED_FRAME_SIZE;
  return OV3660_SAVED_FRAME_SIZE;
}

// Native USB Mass Storage state. MSC is intentionally read-only.
// Android/phone-first behavior: the SD medium is advertised as PRESENT from
// the moment USB MSC starts so Android can mount it during initial enumeration.
// Capture/BLE still pause as soon as the native USB host is mounted.
static USBMSC usbMsc;
static volatile bool usbHostConnected = false;
static bool usbMassStorageActive = false;
static uint8_t usbSdPdrv = 0xFF;
static uint32_t usbPhysicalSectors = 0;
static uint16_t usbPhysicalSectorSize = 512;
static volatile uint32_t usbMscReadOps = 0;
static volatile uint32_t usbMscReadFail = 0;

// BLE is initialized only for a momentary-button-requested session.
static const char* BLE_DEVICE_NAME = "ESP32 Cam HD";
static const char* BLE_SERVICE_UUID = "b6a30001-6e8c-4b74-a4c2-3b33d2c7a001";
static const char* BLE_COMMAND_UUID = "b6a30002-6e8c-4b74-a4c2-3b33d2c7a001";
static const char* BLE_STATUS_UUID  = "b6a30003-6e8c-4b74-a4c2-3b33d2c7a001";
static const char* BLE_DATA_UUID    = "b6a30004-6e8c-4b74-a4c2-3b33d2c7a001";

// BLE live-preview TEST profile. Full-resolution saved photos are unchanged.
// The firmware remains dual OV3660/OV5640. Live_Test_01 is being tuned on the
// OV3660 first, so these values can be changed at runtime from the diagnostic
// web viewer without reflashing between every throughput experiment.
static constexpr framesize_t BLE_LIVE_DEFAULT_FRAME_SIZE = FRAMESIZE_HVGA;
static constexpr uint8_t BLE_LIVE_DEFAULT_JPEG_QUALITY = 20;
static constexpr uint32_t BLE_LIVE_DEFAULT_FRAME_INTERVAL_MS = 250;
static constexpr size_t BLE_LIVE_DATA_HEADER_BYTES = 6; // uint16 frame ID + uint32 byte offset

static framesize_t bleLiveFrameSize = BLE_LIVE_DEFAULT_FRAME_SIZE;
static uint16_t bleLiveWidth = 480;
static uint16_t bleLiveHeight = 320;
static uint8_t bleLiveJpegQuality = BLE_LIVE_DEFAULT_JPEG_QUALITY;
static uint32_t bleLiveFrameIntervalMs = BLE_LIVE_DEFAULT_FRAME_INTERVAL_MS;

static bool bleModeActive = false;
static volatile bool bleClientConnected = false;
static volatile bool bleClientDisconnectedEvent = false;
static BLEServer* bleServer = nullptr;
static BLECharacteristic* bleStatusCharacteristic = nullptr;
static BLECharacteristic* bleDataCharacteristic = nullptr;
static uint32_t blueLedLastToggleMs = 0;
static bool blueLedState = false;
static uint32_t bleAdvertisingStartedMs = 0;

// Live preview exists only inside an active BLE session. Preview frames are
// never written to the SD card and use the same BLE data characteristic as GET.
static bool bleLiveModeActive = false;
static uint16_t bleLiveFrameId = 0;
static uint32_t bleNextLiveFrameDueMs = 0;

// Set by the GPIO ISR. Main-loop code consumes and debounces this flag.
static volatile bool bleButtonPressPending = false;
static uint32_t bleButtonLastAcceptedMs = 0;

struct BleCommandMessage {
  char text[192];
};
static QueueHandle_t bleCommandQueue = nullptr;

static uint32_t nextPhotoNumber = 1;

enum CaptureMode : uint8_t {
  CAPTURE_MODE_MONITORING = 0,
  CAPTURE_MODE_EVENT = 1
};

static CaptureMode captureMode = CAPTURE_MODE_MONITORING;
static uint32_t nextMotionCheckDueMs = 0;
static uint32_t nextEventPhotoDueMs = 0;
static uint32_t eventBlockStartedMs = 0;
static uint32_t eventPhotosThisBlock = 0;

// Comparison scratch storage. Five signatures use only 6000 bytes.
static uint8_t motionSignatures[MOTION_COMPARE_FRAMES][MOTION_SIG_CELLS];
static uint8_t* motionRgbBuffer = nullptr;

// CPU power-state tracking.
static bool batteryActivePower = true;

// SD diagnostics.
static uint32_t sdRequestedFreqKHz = 0;
static int sdRealFreqKHz = 0;

class V3BleServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    bleClientConnected = true;
    bleClientDisconnectedEvent = false;
    blueLedState = true;
    digitalWrite(BLUE_BLE_LED_PIN, HIGH);
    Serial.println("BLE client connected; capture remains paused");
  }

  void onDisconnect(BLEServer* server) override {
    bleClientConnected = false;
    bleClientDisconnectedEvent = true;
    blueLedState = false;
    digitalWrite(BLUE_BLE_LED_PIN, LOW);
    Serial.println("BLE client disconnected; BLE session will close and capture will resume");
  }
};

class V3BleCommandCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    if (!bleCommandQueue) return;
    String value = characteristic->getValue();
    value.trim();
    if (value.length() == 0) return;

    BleCommandMessage msg = {};
    value.toCharArray(msg.text, sizeof(msg.text));
    if (xQueueSend(bleCommandQueue, &msg, 0) != pdTRUE) {
      Serial.println("BLE command queue full; command dropped");
    }
  }
};

static V3BleServerCallbacks v3BleServerCallbacks;
static V3BleCommandCallbacks v3BleCommandCallbacks;

// Manual prototypes for the state-machine helpers.
static bool capturePhoto();
static bool runMotionComparison(float& averageChangedPercent, float& peakChangedPercent);
static void batteryEnterActivePower();
static void batteryEnterIdlePower();
static void batteryCameraIdleMode();
static void usbMassStorageBegin();
static void enterUsbMassStorageMode();
static void exitUsbMassStorageMode();
static int32_t usbMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize);
static int32_t usbMscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize);
static bool usbMscStartStop(uint8_t power_condition, bool start, bool load_eject);
static void usbEventCallback(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void blinkGreenLedAtBoot();
static void IRAM_ATTR onBleMomentaryButtonPressed();
static bool consumeBleButtonPress();
static bool startBluetoothBrowser();
static void stopBluetoothBrowser();
static void updateBluetoothBlueLed();
static void processBluetoothCommands();
static void bleSendStatus(const String& message);
static void bleListPath(const String& path);
static void bleStatPath(const String& path);
static void bleGetFile(const String& path);
static bool bleStartLivePreview();
static void bleStopLivePreview(bool notifyClient);
static void bleUpdateLivePreview();
static bool configureBleLivePreviewMode(bool flushAfterChange);
static bool bleApplyLiveProfileCommand(const String& upperCommand);
static void bleSendLiveInfo();
static size_t blePayloadBytes();
static bool advanceToFreshFolderIfNeeded(const char* reason);

// -------------------------------------------------------------------------
// USB Mass Storage (read-only, exFAT-safe, physical-capacity-backed)
// -------------------------------------------------------------------------
static bool findUsbSdPhysicalDrive() {
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
      usbSdPdrv = pdrv;
      usbPhysicalSectors = wantedSectors;
      usbPhysicalSectorSize = wantedSectorSize;
      return true;
    }
  }
  return false;
}

static int32_t usbMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  ++usbMscReadOps;
  if (!sdReady || usbSdPdrv == 0xFF || !buffer || bufsize == 0) {
    ++usbMscReadFail;
    return -1;
  }

  // A real block read proves that a USB host is present. This intentionally
  // allows Android's very first probe instead of presenting an empty LUN.
  usbHostConnected = true;

  const uint32_t sectorSize = usbPhysicalSectorSize;
  if (sectorSize == 0) {
    ++usbMscReadFail;
    return -1;
  }

  // Fast path: TinyUSB normally asks for one or more complete sectors.
  if (offset == 0 && (bufsize % sectorSize) == 0) {
    const uint32_t count = bufsize / sectorSize;
    if ((uint64_t)lba + count > usbPhysicalSectors ||
        disk_read(usbSdPdrv, static_cast<BYTE*>(buffer), lba, count) != RES_OK) {
      ++usbMscReadFail;
      return -1;
    }
    return (int32_t)bufsize;
  }

  // Safe fallback for an unaligned request.
  uint8_t sector[512];
  if (sectorSize != sizeof(sector)) {
    ++usbMscReadFail;
    return -1;
  }

  uint64_t absolute = (uint64_t)lba * sectorSize + offset;
  uint8_t* dst = static_cast<uint8_t*>(buffer);
  uint32_t remaining = bufsize;
  while (remaining) {
    const uint32_t sectorNumber = (uint32_t)(absolute / sectorSize);
    const uint32_t inside = (uint32_t)(absolute % sectorSize);
    if (sectorNumber >= usbPhysicalSectors ||
        disk_read(usbSdPdrv, sector, sectorNumber, 1) != RES_OK) {
      ++usbMscReadFail;
      return -1;
    }

    uint32_t chunk = sectorSize - inside;
    if (chunk > remaining) chunk = remaining;
    memcpy(dst, sector + inside, chunk);
    dst += chunk;
    absolute += chunk;
    remaining -= chunk;
  }
  return (int32_t)bufsize;
}

static int32_t usbMscWrite(uint32_t, uint32_t, uint8_t*, uint32_t) {
  // Final camera firmware intentionally exposes the card read-only. This keeps
  // Android/PC from modifying raw exFAT sectors while the camera VFS is mounted.
  return -1;
}

static bool usbMscStartStop(uint8_t, bool, bool) {
  return true;
}

static void usbEventCallback(void*, esp_event_base_t event_base,
                             int32_t event_id, void*) {
  if (event_base != ARDUINO_USB_EVENTS) return;
  switch (event_id) {
    case ARDUINO_USB_STARTED_EVENT:
      usbHostConnected = true;
      Serial.println("USB EVENT: PLUGGED / CONFIGURED");
      break;
    case ARDUINO_USB_STOPPED_EVENT:
      usbHostConnected = false;
      Serial.println("USB EVENT: UNPLUGGED");
      break;
    default:
      break;
  }
}

static void usbMassStorageBegin() {
  if (!sdReady) {
    Serial.println("USB MSC not started: SD unavailable");
    return;
  }

  if (!findUsbSdPhysicalDrive()) {
    Serial.println("USB MSC not started: could not locate SD FatFs physical drive");
    return;
  }

  Serial.printf("USB MSC backing device: pdrv=%u, %lu sectors x %u bytes (%.2f GiB)\n",
                usbSdPdrv,
                (unsigned long)usbPhysicalSectors,
                (unsigned)usbPhysicalSectorSize,
                ((double)usbPhysicalSectors * usbPhysicalSectorSize) /
                    (1024.0 * 1024.0 * 1024.0));

  USB.VID(0x303A);
  USB.PID(0x1001);
  USB.productName("ESP32 Cam HD exFAT");
  USB.manufacturerName("ESP32 Cam HD");
  USB.serialNumber("ESP32CAMHDEXFAT");

  usbMsc.vendorID("ESP32");
  usbMsc.productID("CAM_HD_EXFAT");
  usbMsc.productRevision("3.0");
  usbMsc.onRead(usbMscRead);
  usbMsc.onWrite(usbMscWrite);
  usbMsc.onStartStop(usbMscStartStop);
  usbMsc.isWritable(false);
  usbMsc.mediaPresent(true);

  if (!usbMsc.begin(usbPhysicalSectors, usbPhysicalSectorSize)) {
    Serial.println("USB MSC initialization failed");
    return;
  }

  USB.onEvent(usbEventCallback);
  if (!USB.begin()) {
    Serial.println("Native USB initialization failed");
    return;
  }

  Serial.println("USB-C mass storage ready: exFAT / full physical card / READ-ONLY / direct USB-C supported");
}

static void enterUsbMassStorageMode() {
  if (usbMassStorageActive || !usbHostConnected || !sdReady) return;

  batteryEnterActivePower();
  if (cameraReady) {
    esp_camera_deinit();
    cameraReady = false;
  }

  usbMassStorageActive = true;
  Serial.printf("USB host connected: capture PAUSED; exFAT SD exposed READ-ONLY (reads=%lu failures=%lu)\n",
                (unsigned long)usbMscReadOps, (unsigned long)usbMscReadFail);
}

static void exitUsbMassStorageMode() {
  if (!usbMassStorageActive || usbHostConnected) return;

  usbMassStorageActive = false;

  const uint32_t now = millis();
  if (captureMode == CAPTURE_MODE_EVENT) {
    advanceToFreshFolderIfNeeded("USB pause ended / block restart");
    eventBlockStartedMs = now;
    eventPhotosThisBlock = 0;
    nextEventPhotoDueMs = now + EVENT_PHOTO_INTERVAL_MS;
    Serial.println("USB host disconnected: EVENT capture resumes in 600 ms; 2-minute block restarted");
  } else {
    nextMotionCheckDueMs = now + MOTION_CHECK_INTERVAL_MS;
    Serial.println("USB host disconnected: MONITORING resumes; next 5-frame check in 7.5 seconds");
  }
  batteryEnterIdlePower();
}

static void mountSD() {
  // setPins order is CLK, CMD, D0. This board is intentionally wired in
  // 1-bit SDMMC mode, so D1-D3 are not needed.
  if (!SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0)) {
    Serial.println("SD pin setup failed");
    return;
  }

  sdRequestedFreqKHz = 40000;
  if (!SD_MMC.begin("/sdcard", true, false, (int)sdRequestedFreqKHz, 5)) {
    Serial.println("SD 40 MHz mount failed; retrying at 20 MHz");
    SD_MMC.end();
    SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
    sdRequestedFreqKHz = 20000;
    if (!SD_MMC.begin("/sdcard", true, false, (int)sdRequestedFreqKHz, 5)) {
      Serial.println("SD mount failed - check board pinout and exFAT/FAT card format");
      return;
    }
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("SD card not detected");
    return;
  }

  sdRealFreqKHz = 0;
#if defined(CONFIG_IDF_TARGET_ESP32P4) && defined(BOARD_SDMMC_SLOT) && (BOARD_SDMMC_SLOT == 0)
  const int sdHostSlot = SDMMC_HOST_SLOT_0;
#else
  const int sdHostSlot = SDMMC_HOST_SLOT_1;
#endif
  const esp_err_t freqErr = sdmmc_host_get_real_freq(sdHostSlot, &sdRealFreqKHz);
  if (freqErr == ESP_OK && sdRealFreqKHz > 0) {
    Serial.printf("SD bus: 1-bit SDMMC, requested %.1f MHz, actual %.2f MHz\n",
                  sdRequestedFreqKHz / 1000.0f, sdRealFreqKHz / 1000.0f);
  } else {
    Serial.printf("SD bus: 1-bit SDMMC, requested %.1f MHz; actual clock query failed (0x%x)\n",
                  sdRequestedFreqKHz / 1000.0f, (unsigned)freqErr);
  }

  if (!SD_MMC.exists(PHOTO_ROOT)) SD_MMC.mkdir(PHOTO_ROOT);
  sdReady = SD_MMC.exists(PHOTO_ROOT);
  Serial.printf("SD ready: %s; card=%.1f GB, used=%.1f GB\n",
                sdReady ? "yes" : "no",
                SD_MMC.cardSize() / (1024.0 * 1024.0 * 1024.0),
                SD_MMC.usedBytes() / (1024.0 * 1024.0 * 1024.0));
}

// Return only the final filename component from a full SD path.
// This helper is used by the media-number scanner and is unrelated to BLE/Wi-Fi.
static String basenameOf(const String& full) {
  const int pos = full.lastIndexOf('/');
  return pos < 0 ? full : full.substring(pos + 1);
}

static bool allDigits(const String& s) {
  if (s.isEmpty()) return false;
  for (size_t i = 0; i < s.length(); ++i) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

static uint32_t highestSequentialFolder(const char* root) {
  // Folders are created sequentially, so binary search avoids scanning up to
  // 99,999 directory entries at every boot.
  uint32_t low = 1, high = MAX_MEDIA_FOLDERS, last = 0;
  char path[40];
  while (low <= high) {
    uint32_t mid = low + (high - low) / 2;
    snprintf(path, sizeof(path), "%s/%05lu", root, (unsigned long)mid);
    if (SD_MMC.exists(path)) {
      last = mid;
      low = mid + 1;
    } else {
      if (mid == 0) break;
      high = mid - 1;
    }
  }
  return last;
}

static uint32_t scanNextPhotoNumber() {
  if (!sdReady) return 1;

  const uint32_t lastFolder = highestSequentialFolder(PHOTO_ROOT);
  if (lastFolder == 0) return 1;

  char folderPath[40];
  snprintf(folderPath, sizeof(folderPath), "%s/%05lu", PHOTO_ROOT, (unsigned long)lastFolder);
  File dir = SD_MMC.open(folderPath);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return (lastFolder - 1) * PHOTOS_PER_FOLDER + 1;
  }

  uint32_t highest = (lastFolder - 1) * PHOTOS_PER_FOLDER;
  File item = dir.openNextFile();
  while (item) {
    if (!item.isDirectory()) {
      String name = basenameOf(String(item.name()));
      const int tokenPos = name.indexOf("_Pic_");
      if (tokenPos >= 0 && name.endsWith(".JPG")) {
        const int numberStart = tokenPos + 5;
        const int numberEnd = name.length() - 4;
        if (numberEnd > numberStart) {
          String digits = name.substring(numberStart, numberEnd);
          if (allDigits(digits)) {
            const uint32_t id = (uint32_t)strtoul(digits.c_str(), nullptr, 10);
            if (id > highest) highest = id;
          }
        }
      }
    }
    item.close();
    item = dir.openNextFile();
  }
  dir.close();
  return highest + 1;
}

// Keep each newly started/restarted 2-minute block inside one folder whenever
// possible. If the current folder has fewer than 200 unused photo numbers
// remaining, abandon those remaining numbers and begin at the next folder's
// first number. Exactly 200 remaining slots is valid and is used for one full
// block. Example: after F1_Pic_1800.JPG, the next photo remains 1801; after
// F1_Pic_1801.JPG, the next number becomes 2001.
static bool advanceToFreshFolderIfNeeded(const char* reason) {
  if (nextPhotoNumber < 1 || nextPhotoNumber > MAX_PHOTO_NUMBER) return false;

  const uint32_t folder = ((nextPhotoNumber - 1) / PHOTOS_PER_FOLDER) + 1;
  const uint32_t folderFirst = (folder - 1) * PHOTOS_PER_FOLDER + 1;
  const uint32_t folderLast = folder * PHOTOS_PER_FOLDER;
  const uint32_t slotsRemaining = folderLast - nextPhotoNumber + 1;

  // A brand-new folder should never be skipped. Otherwise, reserve enough room
  // for a complete nominal 2-minute / 200-photo block.
  if (nextPhotoNumber != folderFirst && slotsRemaining < PHOTOS_PER_EVENT_BLOCK) {
    if (folder >= MAX_MEDIA_FOLDERS) {
      Serial.println("Folder reserve rule reached maximum media folder; cannot advance");
      return false;
    }

    const uint32_t oldNext = nextPhotoNumber;
    const uint32_t nextFolder = folder + 1;
    nextPhotoNumber = folderLast + 1;

    // Create the fresh folder immediately when the boundary is taken. If the
    // mkdir fails, makePhotoPath() will retry when the first photo is saved.
    if (sdReady) {
      char nextFolderPath[40];
      snprintf(nextFolderPath, sizeof(nextFolderPath), "%s/%05lu",
               PHOTO_ROOT, (unsigned long)nextFolder);
      if (!SD_MMC.exists(nextFolderPath) && !SD_MMC.mkdir(nextFolderPath)) {
        Serial.printf("WARNING: could not pre-create %s; will retry on first photo\n",
                      nextFolderPath);
      }
    }

    Serial.printf("Folder reserve rule (%s): skipping photo numbers %lu-%lu; next photo is %lu in folder %05lu\n",
                  reason ? reason : "new block",
                  (unsigned long)oldNext,
                  (unsigned long)folderLast,
                  (unsigned long)nextPhotoNumber,
                  (unsigned long)nextFolder);
    return true;
  }
  return false;
}

static void initializeMediaSequences() {
  nextPhotoNumber = scanNextPhotoNumber();
  // A reboot/power loss ends the old partial block. If fewer than one full
  // block's worth of slots remain, start fresh in the next numbered folder.
  advanceToFreshFolderIfNeeded("boot/restart");
  Serial.printf("Next photo number: %lu\n", (unsigned long)nextPhotoNumber);
}

static bool makePhotoPath(uint32_t id, char* path, size_t pathSize) {
  if (id < 1 || id > MAX_PHOTO_NUMBER) return false;

  const uint32_t folder = ((id - 1) / PHOTOS_PER_FOLDER) + 1;
  char folderPath[40];
  snprintf(folderPath, sizeof(folderPath), "%s/%05lu", PHOTO_ROOT, (unsigned long)folder);
  if (!SD_MMC.exists(folderPath) && !SD_MMC.mkdir(folderPath)) {
    Serial.printf("Failed to create folder %s\n", folderPath);
    return false;
  }

  snprintf(path, pathSize, "%s/F%lu_Pic_%02lu.JPG",
           folderPath, (unsigned long)folder, (unsigned long)id);
  return true;
}

static void cameraBegin() {
  if (!psramFound()) {
    Serial.println("No PSRAM: camera disabled because highest-resolution capture requires PSRAM");
    return;
  }

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_D0; c.pin_d1 = CAM_D1; c.pin_d2 = CAM_D2; c.pin_d3 = CAM_D3;
  c.pin_d4 = CAM_D4; c.pin_d5 = CAM_D5; c.pin_d6 = CAM_D6; c.pin_d7 = CAM_D7;
  c.pin_xclk = CAM_XCLK; c.pin_pclk = CAM_PCLK;
  c.pin_vsync = CAM_VSYNC; c.pin_href = CAM_HREF;
  c.pin_sccb_sda = CAM_SIOD; c.pin_sccb_scl = CAM_SIOC;
  c.pin_pwdn = CAM_PWDN; c.pin_reset = CAM_RESET;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_QSXGA;
  c.jpeg_quality = PHOTO_JPEG_QUALITY;
  c.fb_count = CAMERA_FB_COUNT;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera QSXGA init failed: 0x%x; retry QXGA\n", err);
    qsxgaInitSucceeded = false;
    c.frame_size = FRAMESIZE_QXGA;
    err = esp_camera_init(&c);
  }
  if (err != ESP_OK) {
    Serial.printf("Camera QXGA init failed: 0x%x; retry UXGA\n", err);
    qxgaInitSucceeded = false;
    c.frame_size = FRAMESIZE_UXGA;
    err = esp_camera_init(&c);
  }
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("Camera init returned no sensor descriptor");
    esp_camera_deinit();
    return;
  }

  detectedCameraPid = s->id.PID;
  detectedCameraSensor = cameraModelFromPid(detectedCameraPid);

  if (detectedCameraSensor == CAMERA_SENSOR_OV3660) {
    // Existing OV3660 orientation correction.
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
  } else if (detectedCameraSensor == CAMERA_SENSOR_OV5640) {
    // OV5640 saved stills use the sensor's maximum esp32-camera profile:
    // QSXGA 2560x1920 at JPEG Q4. Apply vertical flip at the sensor.
    // A 90-degree counter-clockwise display orientation is added to saved
    // JPEGs via EXIF so burst timing is not penalized by 5 MP re-encoding.
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);
  } else {
    Serial.printf("WARNING: unsupported/unrecognized camera PID 0x%x; using generic fallback behavior\n",
                  detectedCameraPid);
  }

  // Both sensors use JPEG Q4; saved resolution is selected per sensor.
  s->set_quality(s, PHOTO_JPEG_QUALITY);

  cameraReady = true;
  if (!cameraWasInitializedOnce) {
    Serial.printf("Detected camera: %s (PID 0x%x), frame size enum %d\n",
                  cameraModelName(detectedCameraSensor), detectedCameraPid,
                  s->status.framesize);
    if (detectedCameraSensor == CAMERA_SENSOR_OV5640) {
      Serial.println("OV5640 detected: saved-photo target QSXGA 2560x1920, JPEG Q4");
    }
    Serial.printf("Camera initialized with %u PSRAM frame buffers\n", (unsigned)CAMERA_FB_COUNT);
    cameraWasInitializedOnce = true;
  }
}

static void flushCameraFrames(uint8_t count) {
  // With triple buffering, frames captured before a resolution change may still
  // be queued. Drain them so the next returned frame matches the new setting.
  for (uint8_t i = 0; i < count; ++i) {
    camera_fb_t* stale = esp_camera_fb_get();
    if (stale) esp_camera_fb_return(stale);
    delay(2);
  }
}

static bool configureSavedPhotoMode(bool flushAfterChange) {
  if (!cameraReady) return false;
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return false;

  framesize_t wanted = preferredSavedPhotoFrameSize(sensor->id.PID);
  if (sensor->id.PID == OV5640_PID && !qsxgaInitSucceeded) {
    wanted = qxgaInitSucceeded ? FRAMESIZE_QXGA : FRAMESIZE_UXGA;
  } else if (sensor->id.PID != OV5640_PID && !qxgaInitSucceeded) {
    wanted = FRAMESIZE_UXGA;
  }
  bool changed = false;
  if (sensor->status.framesize != wanted) {
    if (sensor->set_framesize(sensor, wanted) != 0) return false;
    changed = true;
  }
  if (sensor->status.quality != PHOTO_JPEG_QUALITY) {
    sensor->set_quality(sensor, PHOTO_JPEG_QUALITY);
    changed = true;
  }  if (changed && flushAfterChange) flushCameraFrames(CAMERA_FB_COUNT);
  return true;
}

static camera_fb_t* getHighestResolutionFrame() {
  if (!cameraReady) return nullptr;
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return nullptr;

  // Fast path: keep the camera in the selected full-resolution mode during
  // 750-ms event capture so we do not throw away three frames every cycle.
  if (configureSavedPhotoMode(true)) {
    camera_fb_t* frame = esp_camera_fb_get();
    if (frame && frame->format == PIXFORMAT_JPEG && frame->len > 256) return frame;
    if (frame) esp_camera_fb_return(frame);
  }

  // Recovery path: progressively lower the still resolution if the sensor or
  // frame buffers fail at the preferred setting.
  const framesize_t frameSizes[] = {
    FRAMESIZE_QSXGA, FRAMESIZE_QXGA, FRAMESIZE_UXGA,
    FRAMESIZE_SXGA, FRAMESIZE_XGA, FRAMESIZE_SVGA, FRAMESIZE_VGA
  };
  int start = 1; // OV3660 starts at QXGA.
  if (sensor->id.PID == OV5640_PID) {
    start = qsxgaInitSucceeded ? 0 : (qxgaInitSucceeded ? 1 : 2);
  } else if (!qxgaInitSucceeded) {
    start = 2;
  }
  for (int stage = start; stage < 7; ++stage) {
    if (sensor->set_framesize(sensor, frameSizes[stage]) != 0) continue;
    sensor->set_quality(sensor, PHOTO_JPEG_QUALITY);
    flushCameraFrames(CAMERA_FB_COUNT);
    camera_fb_t* frame = esp_camera_fb_get();
    if (frame && frame->format == PIXFORMAT_JPEG && frame->len > 256) return frame;
    if (frame) esp_camera_fb_return(frame);
  }
  return nullptr;
}

// Save a camera JPEG. OV5640 files receive a minimal EXIF Orientation tag
// (value 8 = 90 degrees counter-clockwise for display) immediately after the
// JPEG SOI marker. Pixel data is left untouched, avoiding an expensive 5 MP
// decode/rotate/re-encode pass that would make the 600-ms burst target impossible.
static bool writeSavedJpeg(File& output, const camera_fb_t* frame, size_t& savedBytes) {
  savedBytes = 0;
  if (!output || !frame || !frame->buf || frame->len < 2) return false;

  if (detectedCameraSensor != CAMERA_SENSOR_OV5640) {
    savedBytes = output.write(frame->buf, frame->len);
    return savedBytes == frame->len;
  }

  if (frame->buf[0] != 0xFF || frame->buf[1] != 0xD8) {
    Serial.println("OV5640 JPEG missing SOI marker; saving without EXIF rotation tag");
    savedBytes = output.write(frame->buf, frame->len);
    return savedBytes == frame->len;
  }

  // JPEG APP1 Exif segment, little-endian TIFF, one Orientation SHORT entry.
  // APP1 length = 0x0022 (34 bytes including its two-byte length field).
  const size_t a = output.write(frame->buf, 2);
  const size_t b = output.write(OV5640_EXIF_ORIENTATION_90_CCW,
                                sizeof(OV5640_EXIF_ORIENTATION_90_CCW));
  const size_t d = output.write(frame->buf + 2, frame->len - 2);
  savedBytes = a + b + d;

  return a == 2 &&
         b == sizeof(OV5640_EXIF_ORIENTATION_90_CCW) &&
         d == (frame->len - 2) &&
         savedBytes == frame->len + sizeof(OV5640_EXIF_ORIENTATION_90_CCW);
}

static bool capturePhoto() {
  if (!sdReady || !cameraReady) {
    Serial.println("Photo skipped: camera or card unavailable");
    return false;
  }
  if (nextPhotoNumber > MAX_PHOTO_NUMBER) {
    Serial.println("Photo folder limit reached (99999 folders)");
    return false;
  }

  camera_fb_t* frame = getHighestResolutionFrame();
  if (!frame) {
    Serial.println("Photo failed at all supported resolutions");
    return false;
  }

  uint32_t id = nextPhotoNumber;
  char fileName[96];
  while (id <= MAX_PHOTO_NUMBER) {
    if (!makePhotoPath(id, fileName, sizeof(fileName))) {
      esp_camera_fb_return(frame);
      return false;
    }
    if (!SD_MMC.exists(fileName)) break;
    ++id;
  }
  if (id > MAX_PHOTO_NUMBER) {
    esp_camera_fb_return(frame);
    Serial.println("Photo numbering limit reached");
    return false;
  }

  File output = SD_MMC.open(fileName, FILE_WRITE);
  bool ok = false;
  size_t savedBytes = 0;
  if (output) {
    ok = writeSavedJpeg(output, frame, savedBytes);
    output.flush();
    output.close();
  }

  Serial.printf("%s %s (%ux%u, %u bytes%s)\n",
                ok ? "SAVED" : "FAILED", fileName,
                frame->width, frame->height, (unsigned)savedBytes,
                (detectedCameraSensor == CAMERA_SENSOR_OV5640)
                    ? ", EXIF rotate 90 CCW" : "");
  esp_camera_fb_return(frame);

  if (!ok) {
    SD_MMC.remove(fileName);
    return false;
  }

  nextPhotoNumber = id + 1;
  return true;
}

static bool buildMotionSignature(const camera_fb_t* frame, uint8_t* signature) {
  if (!frame || !signature || frame->format != PIXFORMAT_JPEG ||
      frame->width != MOTION_COMPARE_WIDTH || frame->height != MOTION_COMPARE_HEIGHT) {
    return false;
  }

  const size_t rgbBytes = (size_t)MOTION_COMPARE_WIDTH * MOTION_COMPARE_HEIGHT * 3U;
  if (!motionRgbBuffer) {
    motionRgbBuffer = (uint8_t*)heap_caps_malloc(rgbBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!motionRgbBuffer) {
      // A small internal-RAM fallback is acceptable if PSRAM is fragmented.
      motionRgbBuffer = (uint8_t*)heap_caps_malloc(rgbBytes, MALLOC_CAP_8BIT);
    }
    if (!motionRgbBuffer) {
      Serial.println("Motion compare failed: unable to allocate RGB decode buffer");
      return false;
    }
  }

  if (!fmt2rgb888(frame->buf, frame->len, PIXFORMAT_JPEG, motionRgbBuffer)) {
    Serial.println("Motion compare failed: JPEG decode error");
    return false;
  }

  const uint16_t cellW = MOTION_COMPARE_WIDTH / MOTION_SIG_WIDTH;   // 4
  const uint16_t cellH = MOTION_COMPARE_HEIGHT / MOTION_SIG_HEIGHT; // 4
  for (uint16_t gy = 0; gy < MOTION_SIG_HEIGHT; ++gy) {
    for (uint16_t gx = 0; gx < MOTION_SIG_WIDTH; ++gx) {
      uint32_t sum = 0;
      for (uint16_t py = 0; py < cellH; ++py) {
        const uint16_t y = gy * cellH + py;
        for (uint16_t px = 0; px < cellW; ++px) {
          const uint16_t x = gx * cellW + px;
          const size_t i = ((size_t)y * MOTION_COMPARE_WIDTH + x) * 3U;
          const uint8_t c0 = motionRgbBuffer[i + 0];
          const uint8_t c1 = motionRgbBuffer[i + 1];
          const uint8_t c2 = motionRgbBuffer[i + 2];
          // Average all three decoded channels. Channel order is irrelevant for
          // motion detection and this avoids depending on RGB-vs-BGR conventions.
          sum += ((uint16_t)c0 + c1 + c2) / 3U;
        }
      }
      signature[(size_t)gy * MOTION_SIG_WIDTH + gx] =
          (uint8_t)(sum / (cellW * cellH));
    }
  }
  return true;
}

static float changedPercentBetweenSignatures(const uint8_t* reference,
                                             const uint8_t* current) {
  uint32_t refSum = 0;
  uint32_t curSum = 0;
  for (uint16_t i = 0; i < MOTION_SIG_CELLS; ++i) {
    refSum += reference[i];
    curSum += current[i];
  }

  // Remove a uniform brightness shift between frames. This makes the spatial
  // change score much less sensitive to auto-exposure or a cloud passing.
  const int32_t refMean = (int32_t)(refSum / MOTION_SIG_CELLS);
  const int32_t curMean = (int32_t)(curSum / MOTION_SIG_CELLS);
  const int32_t globalShift = curMean - refMean;

  uint32_t changed = 0;
  for (uint16_t i = 0; i < MOTION_SIG_CELLS; ++i) {
    int32_t adjusted = (int32_t)current[i] - globalShift;
    if (adjusted < 0) adjusted = 0;
    if (adjusted > 255) adjusted = 255;
    int32_t delta = adjusted - (int32_t)reference[i];
    if (delta < 0) delta = -delta;
    if (delta >= MOTION_CELL_BRIGHTNESS_DELTA) ++changed;
  }
  return (100.0f * changed) / MOTION_SIG_CELLS;
}

static bool runMotionComparison(float& averageChangedPercent, float& peakChangedPercent) {
  averageChangedPercent = 0.0f;
  peakChangedPercent = 0.0f;
  if (!cameraReady) return false;

  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return false;

  batteryEnterActivePower();
  if (sensor->set_framesize(sensor, FRAMESIZE_QQVGA) != 0) {
    Serial.println("Motion compare failed: unable to set QQVGA");
    return false;
  }
  sensor->set_quality(sensor, MOTION_COMPARE_JPEG_QUALITY);

  // Drain old full-resolution/idle frames. None of these are saved.
  flushCameraFrames(CAMERA_FB_COUNT);

  Serial.println("Motion compare: capturing 5 unsaved frames...");
  for (uint8_t n = 0; n < MOTION_COMPARE_FRAMES; ++n) {
    camera_fb_t* frame = esp_camera_fb_get();
    if (!frame) {
      Serial.printf("Motion compare failed: frame %u unavailable\n", (unsigned)(n + 1));
      configureSavedPhotoMode(true);
      return false;
    }
    const bool ok = buildMotionSignature(frame, motionSignatures[n]);
    esp_camera_fb_return(frame);
    if (!ok) {
      configureSavedPhotoMode(true);
      return false;
    }
  }

  float sumPercent = 0.0f;
  Serial.print("Motion differences vs frame 1:");
  for (uint8_t n = 1; n < MOTION_COMPARE_FRAMES; ++n) {
    const float pct = changedPercentBetweenSignatures(motionSignatures[0], motionSignatures[n]);
    sumPercent += pct;
    if (pct > peakChangedPercent) peakChangedPercent = pct;
    Serial.printf(" %.1f%%", pct);
  }
  averageChangedPercent = sumPercent / (MOTION_COMPARE_FRAMES - 1U);
  Serial.printf(" | average=%.1f%%, peak=%.1f%%, threshold=%.1f%% -> %s\n",
                averageChangedPercent, peakChangedPercent, MOTION_TRIGGER_PERCENT,
                averageChangedPercent >= MOTION_TRIGGER_PERCENT ? "MOTION" : "CLEAR");

  // Return to full-resolution still mode now. Event mode can immediately save
  // its first photo; monitoring mode will later drop to tiny idle frames.
  if (!configureSavedPhotoMode(true)) {
    Serial.println("Warning: unable to restore full-resolution photo mode after comparison");
  }
  return true;
}

// -------------------------------------------------------------------------
// V3 momentary BLE SD browser + status LEDs
// -------------------------------------------------------------------------
static void blinkGreenLedAtBoot() {
  Serial.println("Startup indicator: green LED blinking 3 times/sec for 5 seconds; capture inhibited");
  digitalWrite(GREEN_STATUS_LED_PIN, LOW);
  const uint32_t started = millis();
  for (uint8_t i = 0; i < GREEN_STARTUP_FLASHES; ++i) {
    digitalWrite(GREEN_STATUS_LED_PIN, HIGH);
    delay(GREEN_LED_ON_MS);
    digitalWrite(GREEN_STATUS_LED_PIN, LOW);
    delay(GREEN_LED_OFF_MS);
  }
  // 15 x (166 + 167) = 4995 ms. Fill the last few milliseconds so the
  // startup capture inhibit is at least a full 5.000 seconds.
  while ((uint32_t)(millis() - started) < 5000U) delay(1);
  digitalWrite(GREEN_STATUS_LED_PIN, LOW);
  Serial.println("Startup indicator complete; camera/SD initialization may continue");
}

static void IRAM_ATTR onBleMomentaryButtonPressed() {
  // ISR only latches the event. Debounce and all BLE work happen in normal code.
  bleButtonPressPending = true;
}

static bool consumeBleButtonPress() {
  if (!bleButtonPressPending) return false;

  noInterrupts();
  const bool pending = bleButtonPressPending;
  bleButtonPressPending = false;
  interrupts();
  if (!pending) return false;

  const uint32_t now = millis();
  if ((uint32_t)(now - bleButtonLastAcceptedMs) < BLE_BUTTON_DEBOUNCE_MS) {
    return false;
  }
  bleButtonLastAcceptedMs = now;
  return true;
}

static size_t blePayloadBytes() {
  if (!bleServer || !bleClientConnected) return 20;
  uint16_t mtu = bleServer->getPeerMTU(bleServer->getConnId());
  if (mtu < 23) mtu = 23;
  size_t payload = (size_t)mtu - 3U;
  if (payload > 244U) payload = 244U;  // local MTU target is 247
  if (payload < 20U) payload = 20U;
  return payload;
}

static void bleSendCharacteristicChunks(BLECharacteristic* characteristic,
                                        const uint8_t* data, size_t bytes) {
  if (!characteristic || !data || bytes == 0 || !bleClientConnected) return;
  const size_t chunkMax = blePayloadBytes();
  size_t offset = 0;
  while (offset < bytes && bleClientConnected && bleModeActive && !usbHostConnected) {
    const size_t chunk = min(chunkMax, bytes - offset);
    characteristic->setValue(data + offset, chunk);
    characteristic->notify();
    offset += chunk;
    delay(6);
  }
}

static void bleSendStatus(const String& message) {
  if (!bleStatusCharacteristic) return;
  String line = message;
  if (!line.endsWith("\n")) line += "\n";
  bleStatusCharacteristic->setValue(line);
  if (bleClientConnected) {
    bleSendCharacteristicChunks(bleStatusCharacteristic,
                                (const uint8_t*)line.c_str(), line.length());
  }
}


static String bleLowerPath(String path) {
  path.toLowerCase();
  return path;
}

static bool bleIsImagePath(const String& path) {
  const String lower = bleLowerPath(path);
  return lower.endsWith(".jpg") || lower.endsWith(".jpeg") ||
         lower.endsWith(".png") || lower.endsWith(".bmp");
}

static bool bleIsVideoPath(const String& path) {
  const String lower = bleLowerPath(path);
  return lower.endsWith(".avi") || lower.endsWith(".mjpeg") ||
         lower.endsWith(".mjpg") || lower.endsWith(".mp4");
}

static char bleFileKind(const String& path) {
  if (bleIsImagePath(path)) return 'P';
  if (bleIsVideoPath(path)) return 'V';
  return 'F';
}

struct BleIndexStats {
  uint32_t folders;
  uint32_t pictures;
  uint32_t videos;
  uint32_t otherFiles;
  bool truncated;
};

static constexpr uint32_t BLE_INDEX_MAX_FOLDERS = 10000;
static constexpr uint8_t BLE_INDEX_MAX_DEPTH = 16;

static void bleIndexDirectory(const String& requestedPath, uint8_t depth, BleIndexStats& stats) {
  if (!bleClientConnected || !bleModeActive || usbHostConnected || stats.truncated) return;
  if (depth > BLE_INDEX_MAX_DEPTH) {
    bleSendStatus("INDEX WARN depth limit " + requestedPath);
    return;
  }
  if (stats.folders >= BLE_INDEX_MAX_FOLDERS) {
    stats.truncated = true;
    return;
  }

  File dir = SD_MMC.open(requestedPath.c_str());
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return;
  }

  uint32_t directDirs = 0;
  uint32_t directPictures = 0;
  uint32_t directVideos = 0;
  uint32_t directOther = 0;

  File item = dir.openNextFile();
  while (item && bleClientConnected && bleModeActive && !usbHostConnected) {
    String name = basenameOf(String(item.name()));
    String child = requestedPath;
    if (child.length() == 0) child = "/";
    if (!child.endsWith("/")) child += "/";
    child += name;

    const bool isDir = item.isDirectory();
    item.close();

    if (isDir) {
      ++directDirs;
      bleIndexDirectory(child, depth + 1, stats);
    } else {
      const char kind = bleFileKind(child);
      if (kind == 'P') {
        ++directPictures;
        ++stats.pictures;
      } else if (kind == 'V') {
        ++directVideos;
        ++stats.videos;
      } else {
        ++directOther;
        ++stats.otherFiles;
      }
    }

    if (stats.truncated) break;
    item = dir.openNextFile();
  }
  if (item) item.close();
  dir.close();

  // INDEX intentionally sends folder metadata only. File names are loaded only
  // when the viewer issues LIST for the folder currently being viewed.
  if (!stats.truncated && bleClientConnected && bleModeActive && !usbHostConnected) {
    String line = "I\tD\t" + requestedPath;
    line += "\t" + String(directDirs);
    line += "\t" + String(directPictures);
    line += "\t" + String(directVideos);
    line += "\t" + String(directOther);
    bleSendStatus(line);
    ++stats.folders;
  }
}

static void bleIndexCard() {
  if (!sdReady) {
    bleSendStatus("ERR SD unavailable");
    return;
  }

  BleIndexStats stats = {};
  bleSendStatus("BEGIN INDEX /");
  bleIndexDirectory("/", 0, stats);

  String summary = "END INDEX folders=" + String(stats.folders);
  summary += " pics=" + String(stats.pictures);
  summary += " videos=" + String(stats.videos);
  summary += " other=" + String(stats.otherFiles);
  summary += stats.truncated ? " TRUNCATED" : " COMPLETE";
  bleSendStatus(summary);
}

static void bleListPath(const String& requestedPath) {
  if (!sdReady) {
    bleSendStatus("ERR SD unavailable");
    return;
  }

  String path = requestedPath;
  path.trim();
  if (path.length() == 0) path = "/";
  if (!path.startsWith("/")) path = "/" + path;

  File dir = SD_MMC.open(path.c_str());
  if (!dir) {
    bleSendStatus("ERR cannot open " + path);
    return;
  }
  if (!dir.isDirectory()) {
    const uint32_t bytes = (uint32_t)dir.size();
    dir.close();
    bleSendStatus("FILE\t" + path + "\t" + String(bytes));
    return;
  }

  bleSendStatus("BEGIN LIST " + path);
  uint32_t count = 0;
  File item = dir.openNextFile();
  while (item && bleClientConnected && bleModeActive && !usbHostConnected) {
    String name = basenameOf(String(item.name()));
    String child = path;
    if (!child.endsWith("/")) child += "/";
    child += name;
    if (item.isDirectory()) {
      bleSendStatus("D\t" + child);
    } else {
      const char kind = bleFileKind(child);
      String line;
      line += kind;
      line += "\t";
      line += child;
      line += "\t";
      line += String((uint32_t)item.size());
      bleSendStatus(line);
    }
    ++count;
    item.close();
    if (count >= 500) {
      bleSendStatus("TRUNCATED after 500 entries");
      break;
    }
    item = dir.openNextFile();
  }
  if (item) item.close();
  dir.close();
  bleSendStatus("END LIST " + String(count));
}

static void bleStatPath(const String& requestedPath) {
  if (!sdReady) {
    bleSendStatus("ERR SD unavailable");
    return;
  }
  String path = requestedPath;
  path.trim();
  if (!path.startsWith("/")) path = "/" + path;
  File f = SD_MMC.open(path.c_str());
  if (!f) {
    bleSendStatus("ERR not found " + path);
    return;
  }
  if (f.isDirectory()) {
    bleSendStatus("DIR " + path);
  } else {
    bleSendStatus("FILE " + path + " " + String((uint32_t)f.size()) + " bytes");
  }
  f.close();
}

static void bleGetFile(const String& requestedPath) {
  if (!sdReady || !bleDataCharacteristic || !bleClientConnected) {
    bleSendStatus("ERR transfer unavailable");
    return;
  }

  String path = requestedPath;
  path.trim();
  if (!path.startsWith("/")) path = "/" + path;

  if (bleIsVideoPath(path)) {
    bleSendStatus("ERR video files are listing-only; open/download disabled");
    return;
  }
  if (!bleIsImagePath(path)) {
    bleSendStatus("ERR GET is restricted to picture files");
    return;
  }

  File f = SD_MMC.open(path.c_str(), FILE_READ);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    bleSendStatus("ERR file not found " + path);
    return;
  }

  const uint32_t fileBytes = (uint32_t)f.size();
  bleSendStatus("BEGIN GET " + path + " " + String(fileBytes));

  uint8_t packet[244];
  uint32_t offset = 0;
  bool cancelled = false;
  while (f.available() && bleClientConnected && !usbHostConnected) {
    updateBluetoothBlueLed();
    if (!bleModeActive) {
      cancelled = true;
      break;
    }

    size_t mtuPayload = blePayloadBytes();
    if (mtuPayload > sizeof(packet)) mtuPayload = sizeof(packet);
    if (mtuPayload <= 4) mtuPayload = 20;
    const size_t wanted = mtuPayload - 4;
    const size_t got = f.read(packet + 4, wanted);
    if (got == 0) break;

    packet[0] = (uint8_t)(offset);
    packet[1] = (uint8_t)(offset >> 8);
    packet[2] = (uint8_t)(offset >> 16);
    packet[3] = (uint8_t)(offset >> 24);
    bleDataCharacteristic->setValue(packet, got + 4);
    bleDataCharacteristic->notify();
    offset += (uint32_t)got;
    delay(6);
  }
  f.close();

  if (cancelled || !bleClientConnected) {
    bleSendStatus("CANCEL GET at " + String(offset));
  } else {
    bleSendStatus("END GET " + String(offset));
  }
}

static const char* bleLiveProfileName(framesize_t frameSize) {
  switch (frameSize) {
    case FRAMESIZE_QVGA: return "QVGA";   // 320x240
    case FRAMESIZE_HVGA: return "HVGA";   // 480x320
    case FRAMESIZE_VGA: return "VGA";     // 640x480
    case FRAMESIZE_SVGA: return "SVGA";   // 800x600
    default: return "CUSTOM";
  }
}

static bool bleResolveLiveProfile(const String& requested,
                                  framesize_t& frameSize,
                                  uint16_t& width,
                                  uint16_t& height) {
  String profile = requested;
  profile.trim();
  profile.toUpperCase();

  if (profile == "QVGA" || profile == "320X240") {
    frameSize = FRAMESIZE_QVGA; width = 320; height = 240; return true;
  }
  if (profile == "HVGA" || profile == "480X320") {
    frameSize = FRAMESIZE_HVGA; width = 480; height = 320; return true;
  }
  if (profile == "VGA" || profile == "640X480") {
    frameSize = FRAMESIZE_VGA; width = 640; height = 480; return true;
  }
  if (profile == "SVGA" || profile == "800X600") {
    frameSize = FRAMESIZE_SVGA; width = 800; height = 600; return true;
  }
  return false;
}

static void bleSendLiveInfo() {
  String info = "LIVE INFO camera=";
  info += cameraModelName(detectedCameraSensor);
  info += " profile=";
  info += bleLiveProfileName(bleLiveFrameSize);
  info += " width=" + String(bleLiveWidth);
  info += " height=" + String(bleLiveHeight);
  info += " quality=" + String(bleLiveJpegQuality);
  info += " interval=" + String(bleLiveFrameIntervalMs);
  if (detectedCameraSensor == CAMERA_SENSOR_OV5640) info += " rotation=CCW90";
  info += bleLiveModeActive ? " state=ON" : " state=OFF";
  bleSendStatus(info);
}

static bool bleApplyLiveProfileCommand(const String& upperCommand) {
  char profileText[16] = {};
  int requestedQuality = 0;
  unsigned long requestedInterval = 0;
  const int parsed = sscanf(upperCommand.c_str(), "LIVE SET %15s %d %lu",
                            profileText, &requestedQuality, &requestedInterval);
  if (parsed != 3) {
    bleSendStatus("ERR LIVE SET syntax: LIVE SET <QVGA|HVGA|VGA|SVGA> <quality 4-63> <interval_ms 100-5000>");
    return false;
  }

  framesize_t requestedFrameSize;
  uint16_t requestedWidth = 0;
  uint16_t requestedHeight = 0;
  if (!bleResolveLiveProfile(String(profileText), requestedFrameSize,
                             requestedWidth, requestedHeight)) {
    bleSendStatus("ERR LIVE SET profile must be QVGA, HVGA, VGA, or SVGA");
    return false;
  }
  if (requestedQuality < 4 || requestedQuality > 63) {
    bleSendStatus("ERR LIVE SET quality must be 4-63 (lower number = higher JPEG quality)");
    return false;
  }
  if (requestedInterval < 100UL || requestedInterval > 5000UL) {
    bleSendStatus("ERR LIVE SET interval must be 100-5000 ms");
    return false;
  }

  bleLiveFrameSize = requestedFrameSize;
  bleLiveWidth = requestedWidth;
  bleLiveHeight = requestedHeight;
  bleLiveJpegQuality = (uint8_t)requestedQuality;
  bleLiveFrameIntervalMs = (uint32_t)requestedInterval;

  // Commands are processed between complete preview frames, so it is safe to
  // switch the sensor profile here. Drain old buffered frames after a change.
  if (bleLiveModeActive && cameraReady && !configureBleLivePreviewMode(true)) {
    bleSendStatus("ERR LIVE SET sensor rejected requested profile");
    return false;
  }
  if (bleLiveModeActive) bleNextLiveFrameDueMs = millis();

  String status = "LIVE CONFIG ";
  status += bleLiveProfileName(bleLiveFrameSize);
  status += " " + String(bleLiveWidth);
  status += " " + String(bleLiveHeight);
  status += " " + String(bleLiveJpegQuality);
  status += " " + String(bleLiveFrameIntervalMs);
  bleSendStatus(status);
  return true;
}

static bool configureBleLivePreviewMode(bool flushAfterChange) {
  if (!cameraReady) return false;
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return false;

  bool changed = false;
  if (sensor->status.framesize != bleLiveFrameSize) {
    if (sensor->set_framesize(sensor, bleLiveFrameSize) != 0) return false;
    changed = true;
  }
  if (sensor->status.quality != bleLiveJpegQuality) {
    sensor->set_quality(sensor, bleLiveJpegQuality);
    changed = true;
  }
  if (changed && flushAfterChange) flushCameraFrames(CAMERA_FB_COUNT);
  return true;
}

static bool bleStartLivePreview() {
  if (!bleModeActive || !bleClientConnected || usbHostConnected) return false;
  if (bleLiveModeActive) {
    bleSendLiveInfo();
    return true;
  }

  batteryEnterActivePower();
  if (!cameraReady) cameraBegin();
  if (!cameraReady || !configureBleLivePreviewMode(true)) {
    bleSendStatus("ERR LIVE camera unavailable");
    return false;
  }

  bleLiveModeActive = true;
  bleLiveFrameId = 0;
  bleNextLiveFrameDueMs = millis();

  String status = "LIVE ON ";
  status += bleLiveProfileName(bleLiveFrameSize);
  status += " " + String(bleLiveWidth);
  status += " " + String(bleLiveHeight);
  status += " " + String(bleLiveJpegQuality);
  status += " " + String(bleLiveFrameIntervalMs);
  bleSendStatus(status);
  Serial.printf("BLE LIVE ON: %s %ux%u Q%u target=%lu ms%s; frames unsaved; normal capture paused\n",
                bleLiveProfileName(bleLiveFrameSize),
                (unsigned)bleLiveWidth, (unsigned)bleLiveHeight,
                (unsigned)bleLiveJpegQuality, (unsigned long)bleLiveFrameIntervalMs,
                (detectedCameraSensor == CAMERA_SENSOR_OV5640) ? "; display rotate 90 CCW" : "");
  return true;
}

static void bleStopLivePreview(bool notifyClient) {
  if (!bleLiveModeActive && !cameraReady) return;

  bleLiveModeActive = false;
  bleNextLiveFrameDueMs = 0;

  // BLE browsing does not need the camera. Release DMA/frame buffers again so
  // SD browsing and the radio have maximum memory available.
  if (cameraReady) {
    esp_camera_deinit();
    cameraReady = false;
  }

  if (notifyClient && bleClientConnected && bleModeActive) {
    bleSendStatus("LIVE OFF");
  }
  Serial.println("BLE LIVE OFF: camera released; SD browser remains connected");
}

// Copy a range from the logical OV5640 Live JPEG with the same EXIF
// Orientation 8 tag used by saved photos. This keeps the existing JPEG pixels
// and BLE packet framing intact, so there is effectively no Live FPS penalty.
// Logical byte stream:
//   original SOI (2 bytes) + EXIF APP1 + remainder of original JPEG.
static void copyOv5640OrientedLiveJpegRange(const camera_fb_t* frame,
                                           size_t logicalOffset,
                                           uint8_t* dst,
                                           size_t len) {
  if (!frame || !dst || len == 0) return;

  const size_t exifLen = sizeof(OV5640_EXIF_ORIENTATION_90_CCW);
  const size_t soiEnd = 2;
  const size_t exifEnd = soiEnd + exifLen;

  size_t out = 0;
  while (out < len) {
    const size_t pos = logicalOffset + out;
    if (pos < soiEnd) {
      const size_t chunk = min(len - out, soiEnd - pos);
      memcpy(dst + out, frame->buf + pos, chunk);
      out += chunk;
    } else if (pos < exifEnd) {
      const size_t exifPos = pos - soiEnd;
      const size_t chunk = min(len - out, exifLen - exifPos);
      memcpy(dst + out, OV5640_EXIF_ORIENTATION_90_CCW + exifPos, chunk);
      out += chunk;
    } else {
      const size_t srcPos = pos - exifLen;
      const size_t chunk = min(len - out, frame->len - srcPos);
      memcpy(dst + out, frame->buf + srcPos, chunk);
      out += chunk;
    }
  }
}

static void bleUpdateLivePreview() {
  if (!bleLiveModeActive || !bleModeActive || !bleClientConnected || usbHostConnected) return;

  const uint32_t now = millis();
  if ((int32_t)(now - bleNextLiveFrameDueMs) < 0) return;

  batteryEnterActivePower();
  if (!cameraReady) cameraBegin();
  if (!cameraReady || !configureBleLivePreviewMode(true)) {
    bleSendStatus("ERR LIVE camera unavailable");
    bleStopLivePreview(false);
    return;
  }

  camera_fb_t* frame = esp_camera_fb_get();
  if (!frame || frame->format != PIXFORMAT_JPEG || frame->len < 256) {
    if (frame) esp_camera_fb_return(frame);
    bleSendStatus("ERR LIVE frame failed");
    bleNextLiveFrameDueMs = millis() + bleLiveFrameIntervalMs;
    return;
  }

  const bool orientLive90Ccw =
      detectedCameraSensor == CAMERA_SENSOR_OV5640 &&
      frame->buf[0] == 0xFF && frame->buf[1] == 0xD8;
  const size_t liveJpegLen =
      frame->len + (orientLive90Ccw ? sizeof(OV5640_EXIF_ORIENTATION_90_CCW) : 0U);
  const uint16_t liveDisplayWidth = orientLive90Ccw ? frame->height : frame->width;
  const uint16_t liveDisplayHeight = orientLive90Ccw ? frame->width : frame->height;

  uint16_t frameId = ++bleLiveFrameId;
  if (frameId == 0) frameId = ++bleLiveFrameId; // reserve zero as invalid in the test viewer
  bleSendStatus("BEGIN LIVE " + String(frameId) + " " + String((uint32_t)liveJpegLen) +
                " " + String(liveDisplayWidth) + " " + String(liveDisplayHeight) +
                " " + String(bleLiveJpegQuality));

  uint8_t packet[244];
  uint32_t offset = 0;
  bool cancelled = false;
  while ((size_t)offset < liveJpegLen && bleClientConnected && bleModeActive &&
         bleLiveModeActive && !usbHostConnected) {
    updateBluetoothBlueLed();
    size_t mtuPayload = blePayloadBytes();
    if (mtuPayload > sizeof(packet)) mtuPayload = sizeof(packet);
    if (mtuPayload <= BLE_LIVE_DATA_HEADER_BYTES) mtuPayload = 20;
    const size_t payloadCapacity = mtuPayload - BLE_LIVE_DATA_HEADER_BYTES;
    const size_t frameBytesRemaining = liveJpegLen - (size_t)offset;
    const size_t wanted = (payloadCapacity < frameBytesRemaining)
                              ? payloadCapacity
                              : frameBytesRemaining;

    // LIVE packets are self-identifying so an interrupted frame can never be
    // accidentally merged with the next one by the browser:
    //   byte 0..1 = uint16 little-endian frame ID
    //   byte 2..5 = uint32 little-endian JPEG byte offset
    //   byte 6..  = JPEG payload
    packet[0] = (uint8_t)(frameId);
    packet[1] = (uint8_t)(frameId >> 8);
    packet[2] = (uint8_t)(offset);
    packet[3] = (uint8_t)(offset >> 8);
    packet[4] = (uint8_t)(offset >> 16);
    packet[5] = (uint8_t)(offset >> 24);
    if (orientLive90Ccw) {
      copyOv5640OrientedLiveJpegRange(frame, offset,
                                     packet + BLE_LIVE_DATA_HEADER_BYTES, wanted);
    } else {
      memcpy(packet + BLE_LIVE_DATA_HEADER_BYTES, frame->buf + offset, wanted);
    }
    bleDataCharacteristic->setValue(packet, wanted + BLE_LIVE_DATA_HEADER_BYTES);
    bleDataCharacteristic->notify();
    offset += (uint32_t)wanted;
    delay(6);
  }

  esp_camera_fb_return(frame);

  if (!bleLiveModeActive || !bleClientConnected || usbHostConnected) {
    cancelled = true;
  }

  if (!cancelled) {
    bleSendStatus("END LIVE " + String(frameId) + " " + String(offset));
  }

  // Target the requested interval between frame starts. BLE transfer itself is  // the backpressure: if transfer exceeds the interval, no frames queue up and
  // the next capture starts only after this complete JPEG has finished sending.
  bleNextLiveFrameDueMs += bleLiveFrameIntervalMs;
  if ((int32_t)(millis() - bleNextLiveFrameDueMs) >= 0) {
    bleNextLiveFrameDueMs = millis();
  }
}

static void processBluetoothCommands() {
  if (!bleModeActive || !bleClientConnected || !bleCommandQueue) return;

  BleCommandMessage msg = {};
  if (xQueueReceive(bleCommandQueue, &msg, 0) != pdTRUE) return;
  String command(msg.text);
  command.trim();
  String upper = command;
  upper.toUpperCase();

  if (upper == "HELP") {
    bleSendStatus("Commands: HELP | INDEX | LIST [path] | STAT <path> | GET <picture-path> | LIVE INFO | LIVE SET <QVGA|HVGA|VGA|SVGA> <Q4-63> <100-5000ms> | LIVE ON | LIVE OFF");
    return;
  }

  if (upper == "LIVE INFO") {
    bleSendLiveInfo();
    return;
  }
  if (upper.startsWith("LIVE SET ")) {
    bleApplyLiveProfileCommand(upper);
    return;
  }
  if (upper == "LIVE" || upper == "LIVE ON") {
    bleStartLivePreview();
    return;
  }
  if (upper == "LIVE OFF" || upper == "STOP LIVE") {
    bleStopLivePreview(true);
    return;
  }

  if (bleLiveModeActive) {
    bleSendStatus("ERR LIVE active; send LIVE OFF before SD browser commands");
    return;
  }

  if (upper == "INDEX") {
    bleIndexCard();
    return;
  }

  if (upper == "LIST" || upper == "DIR") {
    bleListPath("/");
    return;
  }
  if (upper.startsWith("LIST ") || upper.startsWith("DIR ")) {
    const int split = command.indexOf(' ');
    bleListPath(command.substring(split + 1));
    return;
  }
  if (upper.startsWith("STAT ")) {
    bleStatPath(command.substring(5));
    return;
  }
  if (upper.startsWith("GET ")) {
    bleGetFile(command.substring(4));
    return;
  }

  bleSendStatus("ERR unknown command; send HELP");
}

static bool startBluetoothBrowser() {
  if (bleModeActive) return true;
  if (!sdReady || usbHostConnected || usbMassStorageActive) return false;

  // Starting BLE is only done between complete comparison/photo operations. Stop
  // camera DMA/XCLK and release its buffers before enabling the radio stack.
  batteryEnterActivePower();
  if (cameraReady) {
    esp_camera_deinit();
    cameraReady = false;
  }

  while (bleCommandQueue && uxQueueMessagesWaiting(bleCommandQueue)) {
    BleCommandMessage discard = {};
    xQueueReceive(bleCommandQueue, &discard, 0);
  }

  // Arduino-ESP32 3.3.x BLEDevice::init() returns void.
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(247);

  bleServer = BLEDevice::createServer();
  if (!bleServer) {
    Serial.println("BLE server creation failed");
    BLEDevice::deinit(false);
    batteryEnterIdlePower();
    return false;
  }
  bleServer->setCallbacks(&v3BleServerCallbacks);
  bleServer->advertiseOnDisconnect(false);

  BLEService* service = bleServer->createService(BLE_SERVICE_UUID);
  BLECharacteristic* commandCharacteristic = service->createCharacteristic(
      BLE_COMMAND_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  commandCharacteristic->setCallbacks(&v3BleCommandCallbacks);

  bleStatusCharacteristic = service->createCharacteristic(
      BLE_STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  bleStatusCharacteristic->addDescriptor(new BLE2902());
  String readyStatus = String("ESP32 Cam HD ready; camera=") +
                       cameraModelName(detectedCameraSensor) + "; send LIVE INFO, INDEX, or LIVE ON";
  bleStatusCharacteristic->setValue(readyStatus.c_str());

  bleDataCharacteristic = service->createCharacteristic(
      BLE_DATA_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleDataCharacteristic->addDescriptor(new BLE2902());

  service->start();
  BLEAdvertising* advertising = bleServer->getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  bleClientConnected = false;
  bleClientDisconnectedEvent = false;
  bleLiveModeActive = false;
  bleLiveFrameId = 0;
  bleNextLiveFrameDueMs = 0;
  bleModeActive = true;
  bleAdvertisingStartedMs = millis();
  blueLedLastToggleMs = bleAdvertisingStartedMs;
  blueLedState = false;
  digitalWrite(BLUE_BLE_LED_PIN, LOW);
  Serial.printf("BLE mode ON: capture PAUSED; advertising as ESP32 Cam HD; detected camera=%s\n",
                cameraModelName(detectedCameraSensor));
  Serial.printf("BLE connection window: %u seconds; blue LED blinks 4 times/sec until connected\n",
                (unsigned)(BLE_ADVERTISING_TIMEOUT_MS / 1000U));
  Serial.println("BLE SD browser is READ-ONLY. LIVE preview profile is runtime-tunable; frames are never saved.");
  return true;
}

static void stopBluetoothBrowser() {
  if (!bleModeActive && !BLEDevice::getInitialized()) {
    digitalWrite(BLUE_BLE_LED_PIN, LOW);
    return;
  }

  if (BLEDevice::getInitialized()) {
    BLEDevice::stopAdvertising();
    if (bleServer && bleClientConnected) {
      bleServer->disconnect(bleServer->getConnId());
      delay(30);
    }
    BLEDevice::deinit(false);
  }

  // If Live was active, release the camera before leaving BLE mode.
  bleLiveModeActive = false;
  bleNextLiveFrameDueMs = 0;
  if (cameraReady) {
    esp_camera_deinit();
    cameraReady = false;
  }

  bleModeActive = false;
  bleClientConnected = false;
  bleClientDisconnectedEvent = false;
  bleAdvertisingStartedMs = 0;
  bleServer = nullptr;
  bleStatusCharacteristic = nullptr;
  bleDataCharacteristic = nullptr;
  blueLedState = false;
  digitalWrite(BLUE_BLE_LED_PIN, LOW);

  // Resume whichever capture state was active before BLE. An EVENT block is
  // restarted so the 2-minute timer counts only uninterrupted active capture.
  const uint32_t now = millis();
  if (captureMode == CAPTURE_MODE_EVENT) {
    advanceToFreshFolderIfNeeded("BLE pause ended / block restart");
    eventBlockStartedMs = now;
    eventPhotosThisBlock = 0;
    nextEventPhotoDueMs = now + EVENT_PHOTO_INTERVAL_MS;
    Serial.println("BLE mode OFF: EVENT capture resumes in 600 ms; 2-minute block restarted");
  } else {
    nextMotionCheckDueMs = now + MOTION_CHECK_INTERVAL_MS;
    Serial.println("BLE mode OFF: MONITORING resumes; next 5-frame check in 7.5 seconds");
  }
  batteryEnterIdlePower();
}

static void updateBluetoothBlueLed() {
  if (!bleModeActive) {
    if (blueLedState) {
      blueLedState = false;
      digitalWrite(BLUE_BLE_LED_PIN, LOW);
    }
    return;
  }

  if (bleClientConnected) {
    if (!blueLedState) {
      blueLedState = true;
      digitalWrite(BLUE_BLE_LED_PIN, HIGH);
    }
    return;
  }

  const uint32_t now = millis();
  if ((uint32_t)(now - blueLedLastToggleMs) >= BLUE_LED_TOGGLE_MS) {
    blueLedLastToggleMs = now;
    blueLedState = !blueLedState;
    digitalWrite(BLUE_BLE_LED_PIN, blueLedState ? HIGH : LOW);
  }
}

static void batteryEnterActivePower() {
  if (batteryActivePower) return;
  // Full CPU speed is used for camera capture, JPEG comparison and SD writes.
  setCpuFrequencyMhz(BATTERY_ACTIVE_CPU_MHZ);
  batteryActivePower = true;
}

static void batteryEnterIdlePower() {
  if (!batteryActivePower) return;
  // 80 MHz reduces CPU dynamic power during ordinary waits between captures.
  // Full 240 MHz is restored before camera capture and SD writes.
  setCpuFrequencyMhz(BATTERY_IDLE_CPU_MHZ);
  batteryActivePower = false;
}

static void batteryCameraIdleMode() {
  if (!BATTERY_CAMERA_LOW_RATE_BETWEEN_PHOTOS || !cameraReady) return;
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) return;

  // The camera remains initialized (avoids heap churn between every photo),
  // but tiny JPEG frames greatly reduce DMA/PSRAM traffic while waiting.
  sensor->set_framesize(sensor, FRAMESIZE_QQVGA);
  sensor->set_quality(sensor, 63);
}

void setup() {
  pinMode(GREEN_STATUS_LED_PIN, OUTPUT);
  pinMode(BLUE_BLE_LED_PIN, OUTPUT);
  pinMode(BLE_MOMENTARY_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(GREEN_STATUS_LED_PIN, LOW);
  digitalWrite(BLUE_BLE_LED_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(BLE_MOMENTARY_BUTTON_PIN),
                  onBleMomentaryButtonPressed, FALLING);

  Serial.begin(115200);
  Serial.println("\nESP32 Cam HD FULL exFAT - OV3660/OV5640 / motion stills / BLE Live / native USB MSC");
  blinkGreenLedAtBoot();
  Serial.printf("Motion monitor: 5 unsaved frames every %u sec; trigger >= %.1f%% changed area\n",
                (unsigned)(MOTION_CHECK_INTERVAL_MS / 1000U), MOTION_TRIGGER_PERCENT);
  Serial.printf("Event mode: saved photo every %.2f sec; recheck every %u sec\n",
                EVENT_PHOTO_INTERVAL_MS / 1000.0f,
                (unsigned)(EVENT_RECHECK_INTERVAL_MS / 1000U));
  Serial.printf("CPU: %u MHz active / %u MHz while waiting\n",
                (unsigned)BATTERY_ACTIVE_CPU_MHZ,
                (unsigned)BATTERY_IDLE_CPU_MHZ);
  Serial.printf("V3 pins: momentary BLE button GPIO%d (press=GND), blue LED GPIO%d, green LED GPIO%d\n",
                BLE_MOMENTARY_BUTTON_PIN, BLUE_BLE_LED_PIN, GREEN_STATUS_LED_PIN);

  setCpuFrequencyMhz(BATTERY_ACTIVE_CPU_MHZ);
  batteryActivePower = true;

  bleCommandQueue = xQueueCreate(4, sizeof(BleCommandMessage));
  if (!bleCommandQueue) Serial.println("WARNING: BLE command queue allocation failed");

  mountSD();
  initializeMediaSequences();
  usbMassStorageBegin();
  cameraBegin();

  batteryCameraIdleMode();
  batteryEnterIdlePower();

  captureMode = CAPTURE_MODE_MONITORING;
  nextMotionCheckDueMs = millis() + MOTION_CHECK_INTERVAL_MS;
  nextEventPhotoDueMs = 0;
  eventBlockStartedMs = 0;
  eventPhotosThisBlock = 0;
  Serial.println("MONITORING mode active; first 5-frame comparison in 7.5 seconds");
}

void loop() {
  static bool bleStartRequested = false;

  const bool bleButtonPressed = consumeBleButtonPress();
  if (bleButtonPressed) {
    if (bleModeActive) {
      Serial.println("BLE momentary button pressed while BLE is active; shutting Bluetooth off");
      stopBluetoothBrowser();
      bleStartRequested = false;
    } else {
      Serial.println("BLE momentary button pressed; BLE will start after the current comparison/photo operation completes");
      bleStartRequested = true;
    }
  }
  updateBluetoothBlueLed();

  // USB-C mass storage has priority over BLE. A request that arrives during a
  // comparison burst or SD photo write is handled after that operation returns.
  if (usbHostConnected) {
    if (bleModeActive) stopBluetoothBrowser();
    enterUsbMassStorageMode();
    if (!batteryActivePower) batteryEnterActivePower();
    delay(20);
    return;
  }

  if (usbMassStorageActive) exitUsbMassStorageMode();

  if (bleStartRequested && !bleModeActive) {
    bleStartRequested = false;
    startBluetoothBrowser();
  }

  if (bleModeActive) {
    processBluetoothCommands();
    if (bleLiveModeActive) bleUpdateLivePreview();
    updateBluetoothBlueLed();

    if (bleClientDisconnectedEvent && !bleClientConnected) {
      Serial.println("BLE client session ended; shutting Bluetooth off");
      stopBluetoothBrowser();
      delay(10);
      return;
    }

    if (!bleClientConnected && bleAdvertisingStartedMs != 0 &&
        (uint32_t)(millis() - bleAdvertisingStartedMs) >= BLE_ADVERTISING_TIMEOUT_MS) {
      Serial.println("No BLE client connected within 30 seconds; shutting Bluetooth off");
      stopBluetoothBrowser();
      delay(10);
      return;
    }

    delay(10);
    return;
  }

  if (!sdReady) {
    delay(250);
    return;
  }

  const uint32_t now = millis();

  if (captureMode == CAPTURE_MODE_MONITORING) {
    if ((int32_t)(now - nextMotionCheckDueMs) >= 0) {
      batteryEnterActivePower();
      if (!cameraReady) cameraBegin();

      const uint32_t checkStartedMs = millis();
      float averagePct = 0.0f;
      float peakPct = 0.0f;
      const bool valid = cameraReady && runMotionComparison(averagePct, peakPct);

      if (valid && averagePct >= MOTION_TRIGGER_PERCENT) {
        // Never start a fresh 2-minute block when fewer than 200 slots remain
        // in a partially used folder. This also handles a prior interrupted block.
        advanceToFreshFolderIfNeeded("new motion block");
        captureMode = CAPTURE_MODE_EVENT;
        eventBlockStartedMs = millis();
        eventPhotosThisBlock = 0;
        nextEventPhotoDueMs = millis();  // first saved event photo immediately
        Serial.printf("MOTION TRIGGERED (avg %.1f%%): entering EVENT mode for at least 2 minutes\n",
                      averagePct);
      } else {
        if (!valid) Serial.println("Motion comparison invalid; staying in MONITORING mode");
        batteryCameraIdleMode();
        batteryEnterIdlePower();
        // Keep approximately 7.5 seconds between starts of comparison bursts.
        nextMotionCheckDueMs = checkStartedMs + MOTION_CHECK_INTERVAL_MS;
        if ((int32_t)(millis() - nextMotionCheckDueMs) >= 0) {
          nextMotionCheckDueMs = millis() + MOTION_CHECK_INTERVAL_MS;
        }
      }
    }
  } else {  // CAPTURE_MODE_EVENT
    // At the 2-minute boundary, comparison takes priority over the next photo.
    if ((uint32_t)(now - eventBlockStartedMs) >= EVENT_RECHECK_INTERVAL_MS) {
      batteryEnterActivePower();
      if (!cameraReady) cameraBegin();

      float averagePct = 0.0f;
      float peakPct = 0.0f;
      const bool valid = cameraReady && runMotionComparison(averagePct, peakPct);

      if (!valid) {
        // Fail safe toward continuing capture. A transient JPEG/decode problem
        // should not terminate an already-active event. This is a new 2-minute
        // block, so enforce the folder reserve boundary first.
        advanceToFreshFolderIfNeeded("event recheck failed / next block");
        eventBlockStartedMs = millis();
        eventPhotosThisBlock = 0;
        nextEventPhotoDueMs = millis();
        Serial.println("EVENT recheck failed: continuing another 2-minute block as a safety fallback");
      } else if (averagePct >= MOTION_TRIGGER_PERCENT) {
        Serial.printf("EVENT recheck still active (avg %.1f%%; %lu photos in last block): continuing another 2 minutes\n",
                      averagePct, (unsigned long)eventPhotosThisBlock);
        advanceToFreshFolderIfNeeded("next 2-minute event block");
        eventBlockStartedMs = millis();
        eventPhotosThisBlock = 0;
        nextEventPhotoDueMs = millis();
      } else {
        Serial.printf("EVENT ended (avg %.1f%%; %lu photos in last block): returning to MONITORING mode\n",
                      averagePct, (unsigned long)eventPhotosThisBlock);
        // If this block stopped near the folder boundary, reserve the remainder
        // now so the next event begins at the next folder's first number.
        advanceToFreshFolderIfNeeded("event stopped");
        captureMode = CAPTURE_MODE_MONITORING;
        nextMotionCheckDueMs = millis() + MOTION_CHECK_INTERVAL_MS;
        batteryCameraIdleMode();
        batteryEnterIdlePower();
      }
    } else if ((int32_t)(now - nextEventPhotoDueMs) >= 0) {
      batteryEnterActivePower();
      if (!cameraReady) cameraBegin();

      const uint32_t photoStartedMs = millis();
      if (cameraReady && capturePhoto()) {
        ++eventPhotosThisBlock;
      }

      // Keep full-resolution camera mode during EVENT capture so the next shot
      // does not pay the resolution-switch/flush penalty every 600 ms.
      batteryEnterIdlePower();

      nextEventPhotoDueMs += EVENT_PHOTO_INTERVAL_MS;
      // If capture + SD write itself exceeds 600 ms, the requested cadence is
      // physically impossible. Start the next shot immediately rather than
      // adding another 600-ms delay, so event capture runs as fast as hardware allows.
      if ((int32_t)(millis() - nextEventPhotoDueMs) >= 0) {
        const uint32_t elapsed = millis() - photoStartedMs;
        Serial.printf("Event photo cadence overrun: operation took %u ms; target is %u ms; next shot ASAP\n",
                      (unsigned)elapsed, (unsigned)EVENT_PHOTO_INTERVAL_MS);
        nextEventPhotoDueMs = millis();
      }
    }
  }

  if (batteryActivePower) batteryEnterIdlePower();

  uint32_t waitMs = BATTERY_IDLE_SLICE_MS;
  uint32_t targetMs = (captureMode == CAPTURE_MODE_EVENT) ? nextEventPhotoDueMs : nextMotionCheckDueMs;
  if (captureMode == CAPTURE_MODE_EVENT) {
    const uint32_t recheckDue = eventBlockStartedMs + EVENT_RECHECK_INTERVAL_MS;
    if ((int32_t)(recheckDue - targetMs) < 0) targetMs = recheckDue;
  }
  const int32_t untilNext = (int32_t)(targetMs - millis());
  if (untilNext > 0 && waitMs > (uint32_t)untilNext) waitMs = (uint32_t)untilNext;
  if (waitMs == 0) waitMs = 1;
  delay(waitMs);
}