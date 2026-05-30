# Changelog

## 2.2.1

- Added `version`, `description`, `url`, and `license` to `idf_component.yml`.
- Bumped `library.json` version to `2.2.1` and tightened `frameworks` to `["arduino", "espidf"]`.
- Rewrote `CMakeLists.txt` to dynamically detect `esp_driver_i2c` availability, matching QuickSPI's approach for ESP-IDF 5/6 compatibility.
- Added `platformio.ini` with dual Arduino and ESP-IDF environments.
- Added an ESP-IDF GitHub Actions smoke-test matrix for `v5.4` and `v6.0`.
- Added an Arduino GitHub Actions smoke-test build.
- Added minimal ESP-IDF and Arduino smoke-test applications used by CI.
- Aligned repository version metadata for the next release.
