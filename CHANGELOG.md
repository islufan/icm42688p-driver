# Changelog

All notable changes to this project are documented in this file.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-11-28

First public release. The public API is stable from this version on.

### Added

- Platform-independent bus: read, write and microsecond-delay callbacks injected by the caller
- Accelerometer full scale: ±16 / ±8 / ±4 / ±2 g
- Gyroscope full scale: ±2000 / ±1000 / ±500 / ±250 dps
- ODR selection shared by both sensors, `0x01`–`0x0B` plus `0x0F`
- 12-byte burst read of all six axes
- Conversion helpers for g, dps and degrees Celsius
- Soft reset, `WHO_AM_I` verification and low-noise enable
- Register bank selection, 0 to 4
- Direct register read/write for options not covered by the API
- STM32 HAL SPI adapter in `port/`
- STM32G474 example in `examples/`
- Mock-bus unit tests that need no hardware
- GitHub Actions workflow building and testing on Ubuntu

### Fixed

- Full-scale bits are written to `CONFIG0[7:5]`; the earlier layout clobbered the ODR field
- Temperature conversion uses `TEMP_DATA / 132.48 + 25`; `2.07` only applies to the 8-bit FIFO temperature field
- ODR encoding matches DS-000347: `0x07` is 200 Hz and `0x0F` is 500 Hz, `0x0C`–`0x0E` are reserved on the gyroscope
- Added the missing `<stddef.h>` include so GCC builds succeed

### Notes

- The driver is not thread-safe; guard a shared `icm_t` instance if several tasks use it.
- Initialization applies ±16 g, ±2000 dps, 1 kHz, low-noise mode and leaves the 45 ms gyro start-up wait to the caller.
