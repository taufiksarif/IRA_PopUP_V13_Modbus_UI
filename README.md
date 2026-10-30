# Interactive Relay Automation
ESP32 Relay Controller

## Build

This sketch targets a classic ESP32 Dev Module with Arduino-ESP32 core 2.0.17
(ESP-IDF 4.x and the legacy `driver/twai.h` API). Install Arduino CLI, then run
these commands from this folder in PowerShell:

```powershell
arduino-cli core update-index
arduino-cli core install esp32:esp32@2.0.17
arduino-cli compile --fqbn esp32:esp32:esp32 --warnings all .
```

The Wi-Fi, web server, DNS, Preferences, FreeRTOS, and TWAI dependencies are
provided by the ESP32 core; no additional Arduino libraries are required.
Core 3.x is not the pinned build target because it changes the TWAI API used
by this sketch.
