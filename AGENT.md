# AI Agent Context — Noise Guardian ESP32

## What this project is

A noise traffic light (Lärmampel) on a Waveshare ESP32-S3-Touch-LCD-1.85C V2.
Measures ambient volume via microphone → displays Green / Yellow / Red on a round 360×360 display.

## Critical architecture facts

### Display
- Controller: **ST77916** via **QSPI** (4 data lines) — NOT standard SPI
- **TFT_eSPI is incompatible** — always use `Arduino_GFX` (`moononournation/GFX Library for Arduino`)
- Bus: `Arduino_ESP32QSPI`, driver: `Arduino_ST77916`

### Microphone
- Audio goes through an **ES7210 Audio Codec**, NOT a direct MEMS I²S connection
- Init sequence: **I²C first** (codec config), then **I²S** (audio data stream)
- ES7210 library (`lib/es7210/`) uses the **IDF v5 new I2C master driver** (`driver/i2c_master.h`)
  → `i2c_new_master_bus()` creates the bus handle, passed into `es7210_i2c_config_t.bus_handle`
  → Do NOT use the legacy `driver/i2c.h` — conflicts with Arduino Core 3.x `driver_ng`

### Framework
- **pioarduino** (community fork), NOT the official Espressif PlatformIO platform
- Provides Arduino Core 3.3.x / IDF 5.5.x — required for `ESP_I2S.h` and QSPI support

## Verified pin assignments

### Display QSPI (ST77916)
| Signal | GPIO |
|--------|------|
| CS     | 21   |
| CLK    | 40   |
| D0     | 46   |
| D1     | 45   |
| D2     | 42   |
| D3     | 41   |
| BL     | 5    |
| RST    | -1 (software reset via `gfx->begin()`) |

### Microphone (ES7210 Codec)
| Signal | GPIO | Role |
|--------|------|------|
| MCK    | 2    | I²S Master Clock |
| BCK    | 48   | I²S Bit Clock |
| WS     | 38   | I²S Word Select / LRCLK |
| DIN    | 39   | I²S Data IN (mic → ESP32) |
| DOUT   | 47   | I²S Data OUT (unused) |
| SDA    | 11   | I²C codec init |
| SCL    | 10   | I²C codec init |

ES7210 I²C address: `0x40`

## Key files

| File | Purpose |
|------|---------|
| `src/main.cpp` | All application logic |
| `lib/es7210/es7210.h` | Codec driver API + config structs |
| `lib/es7210/es7210.c` | Codec driver implementation (IDF v5 I2C master) |
| `lib/es7210/es7210_reg.h` | Register address macros |
| `platformio.ini` | Build config (pioarduino, Arduino_GFX, qio_opi) |

## Audio pipeline (main.cpp)

```
ES7210 (I²C init) → I²S 16 kHz stereo 16-bit
→ read 512 stereo frames (32 ms)
→ RMS of left channel only (index 0,2,4… in buffer)
→ moving average over 8 readings (~256 ms)
→ compare to THRESHOLD_LOW / THRESHOLD_HIGH
→ fillScreen() on state change only
```

Threshold constants at `src/main.cpp:43–44`. Debug line at `src/main.cpp:477`.

## platformio.ini summary

```ini
platform = pioarduino (stable zip URL)
board = esp32-s3-devkitc1-n16r8
board_build.flash_size = 16MB
board_build.psram = enabled
board_build.arduino.memory_type = qio_opi   ← required for ESP32-S3R8 OPI PSRAM
lib_deps = moononournation/GFX Library for Arduino@1.6.5
build_flags = -DBOARD_HAS_PSRAM -DARDUINO_USB_CDC_ON_BOOT=1
```

## Build

```bash
pio run                    # build only
pio run --target upload    # flash
pio device monitor         # serial monitor (115200 baud)
```
