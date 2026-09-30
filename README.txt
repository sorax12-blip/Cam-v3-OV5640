Cam v3 - OV5640 Android companion source

Open this folder in Android Studio and build app > Build APK(s).
Requirements: Android SDK 35 / JDK 17.

Behavior:
- Native BLE connection to device name: Cam v3 - OV5640
- Indexes the entire SD card's folder tree/counts.
- Requests filenames only for the folder currently open.
- Pictures: view and download.
- Videos: visible/listed only; no open or download command is exposed.
- Downloads save to Downloads/Cam v3 on Android 10+.

The embedded viewer is app/src/main/assets/index.html.
