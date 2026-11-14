# wl_pro_robot — custom build (ESP32-WROOM-32D + 2x DRV8313 + 2805 gimbal + PWM leg servos, 3S)

Modified from the NavBot-EN01 firmware. **Everything specific to your hardware is in `hw_config.h`.**

## Build steps
1. Arduino IDE with **arduino-esp32 core 2.0.x or 3.x** (the original project targeted 2.0.x; on 3.x use a SimpleFOC version that supports core 3).
2. Board: **ESP32 Dev Module**. Partition scheme: **Huge APP (3MB No OTA/1MB SPIFFS)** if the sketch is too big for the default.
3. Libraries: install the same ones as the original README from its `libraries/` folder
   (Simple_FOC, MPU6050_tockn, WebSockets, ArduinoJson, ...) plus "Queue" from the Library Manager.
   The SCSservo / OLED code is no longer used.
4. Open `wl_pro_robot.ino`, Serial Monitor at **921600 baud, line ending "Newline"**.
5. Before the first power-up read the hardware checklist in the tuning guide.

## Files
| File | Purpose |
|---|---|
| `hw_config.h` | pins, 3S supply, torque limit, servo angles, jump timing, IMU signs (EDIT THIS) |
| `wl_wifi.h/.cpp` | the original `wifi.h/.cpp`, renamed because on Windows `<WiFi.h>` was picking up the sketch file instead of the ESP32 library |
| `leg_math.h` | pure servo/height math (unit-tested on a PC) |
| `leg_servo.h/.cpp` | PWM leg servo driver (LEDC 50 Hz, slew-limited) |
| `wl_pro_robot.ino` | main sketch (balance, yaw, legs, jump, serial commands) |
| `robot.cpp/.h` | web/BLE protocol (servo commands adapted, config-save bug fixed) |
| others | unchanged from the original (wifi, ble, web UI, eeprom, feedback) |

Removed: OLED/expression task (`cpu0_task`, `ssd1362`, `oled_font.h`, `data/*.bin`), STS3032 bus-servo driver.

## Serial commands (Newline terminated)
`S` status · `T1`/`T0` telemetry for Serial Plotter · `Z` set balance zero from current angle ·
`V6` driver voltage limit · `M0.7` speed-loop base P · `X` kill drivers · `Y` re-arm ·
`AP1.0` etc. = PID letters A..L as in the original (P/I/D/R/L sub-letters; `AP` alone prints the value).
