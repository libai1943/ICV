# Chapter 9: experimental-vehicle firmware

The chapter-local firmware is [Firmware/main/main.ino](Firmware/main/main.ino). It contains the vehicle message handling, encoder readings, incremental PI wheel-speed control, steering-servo command, and display logic. The supplied display library is in `Firmware/libraries/SSD1306`.

The initial firmware snapshot and its integration were supplied through Yueshuo Sun's repository. [Firmware/README.md](Firmware/README.md) records the exact source commit and attribution. `Chapter9-source.zip` contains the full chapter directory.

## Prepare the Arduino libraries

Install the Arduino IDE and use the board and connection settings described in the book. The documented board is the Arduino Uno. Select the actual device port for the connected vehicle.

The sketch includes these libraries:

- SSD1306: use the supplied project library.
- Servo.
- PinChangeInt.
- MsTimer2.
- Tinyproto, providing `TinyProtocol.h`.

The book records MsTimer2 1.1.0 and Arduino Tinyproto 0.9.3 for its firmware baseline. These are separate from the host-side dependency snapshot supplied with Chapter 11. Use the library versions and serial framing/checksum configuration documented for the physical platform.

For a Linux installation with the default Arduino sketchbook location:

```bash
export ICV_BOOK_ROOT="$HOME/src/ICV"
export ICV_ARDUINO_SKETCHBOOK="$HOME/Arduino"
mkdir -p "$ICV_ARDUINO_SKETCHBOOK/libraries"
cp -R "$ICV_BOOK_ROOT/Chapter9/Firmware/libraries/SSD1306" \
  "$ICV_ARDUINO_SKETCHBOOK/libraries/"
```

If the IDE uses a different sketchbook directory, set `ICV_ARDUINO_SKETCHBOOK` to that directory. Install the remaining libraries through the IDE's library workflow using the platform's recorded versions.

## Compile and upload

Open `Firmware/main/main.ino` in the Arduino IDE, select Arduino Uno and the connected device port, and run **Verify**. Configure the vehicle identifier, pin connections, calibration constants, and serial settings for the target chassis using the book's platform procedure. Then use **Upload** for that connected vehicle.

The migration preserves the sketch's pin assignments, timing, message layouts, numerical constants, and control equations. Chinese comments were translated into English. Library authorship and notices remain in the supplied library files.

## Verification scope

Source comparison confirmed that the firmware's executable tokens were unchanged by the comment translation. The review environment did not compile the Arduino sketch, upload firmware, or operate a vehicle. Complete the platform calibration and device-specific checks described in the book before a physical experiment.
