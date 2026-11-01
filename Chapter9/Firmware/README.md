# Chapter 9: experimental-vehicle firmware

This directory preserves the vehicle-firmware snapshot supplied by Yueshuo Sun for the experimental platform. The initial firmware and integration work are credited to Yueshuo Sun.

## Provenance

- Original repository: https://github.com/YueshuoSun/Firmware
- Original branch: `main`
- Exact imported commit: `936148e043ffdc2218ebc52688cf23ccbacb01b8`
- Snapshot: https://github.com/YueshuoSun/Firmware/tree/936148e043ffdc2218ebc52688cf23ccbacb01b8
- Book repository: https://github.com/libai1943/ICV

## Editorial changes

- Translated Chinese comments and user-facing messages into English.
- Consolidated Yueshuo Sun's first-party contribution headers in this README.
- Preserved mathematical expressions, identifiers, ROS interfaces, configuration values, and third-party notices, except for the specific changes listed below.

The Arduino sketch is `main/main.ino`; the supplied SSD1306 display library is in `libraries/SSD1306`. Pin assignments, timing, serial message layout, calibration constants, and control equations are unchanged. The other Arduino libraries named by the sketch must be installed separately. Existing library attribution has been retained. No hardware upload or vehicle test was performed during this import.
