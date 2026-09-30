# Cam v3 momentary BLE + LED behavior

Firmware revision behavior:

- BLE device name remains **Cam v3 - OV5640**.
- GPIO21 uses a **normally-open momentary push button to GND** with the ESP32 internal pull-up.
- A falling-edge GPIO interrupt latches the button press, so a press during a photo or 15-second video is remembered.
- The current photo/video operation always finishes before BLE starts.
- Once requested, BLE advertises for **30 seconds**.
- If no client connects within 30 seconds, BLE shuts off automatically and capture resumes after the normal 3-second delay.
- If a client connects, BLE remains enabled for that connection.
- When the client disconnects, BLE shuts off and capture resumes after the normal 3-second delay.
- Capture remains paused while BLE is advertising or connected.
- Blue LED (GPIO47): **4 complete blinks/second while advertising**, solid ON while connected, OFF when BLE is disabled.
- Green LED (GPIO14): **3 complete blinks/second for 5 seconds at startup** (15 flashes total). Capture cannot begin before this startup indication completes.

Wiring:
- Momentary button: GPIO21 -> normally-open button -> GND
- Blue LED: GPIO47 -> 150–220 ohm resistor -> LED anode; LED cathode -> GND
- Green LED: GPIO14 -> 150–220 ohm resistor -> LED anode; LED cathode -> GND
