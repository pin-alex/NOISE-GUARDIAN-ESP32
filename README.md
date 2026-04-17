# Noise Guardian ESP32

A visual noise traffic light based on the Waveshare ESP32-S3-Touch-LCD-1.85C V2.

The device continuously measures ambient volume via a MEMS microphone and visualises the result as a full-screen color on the round display — giving instant, glanceable feedback without any numbers or dials:

- 🟢 **Green** — quiet environment
- 🟡 **Yellow** — moderate noise level
- 🔴 **Red** — too loud

Typical use case: keeping noise levels in check in a living room, classroom, or office.
No calibrated dB value is needed — the system works with relative loudness thresholds that you tune once to your environment.

---

## Hardware

**Board:** [Waveshare ESP32-S3-Touch-LCD-1.85C V2](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.85C)

**PlatformIO target:** `esp32-s3-devkitc1-n16r8` (matching the board's 16 MB flash / 8 MB PSRAM layout)

| Component | Details |
|-----------|---------|
| MCU | ESP32-S3R8 · Dual-Core LX7 · 240 MHz |
| Flash | 16 MB |
| PSRAM | 8 MB |
| Display | Round TFT · ST77916 · **QSPI** · 360×360 px |
| Microphone | MEMS via **ES7210** Audio Codec (I²S + I²C) |
| USB | USB-C (CDC) |

---

## Pin Reference

### Display (QSPI — ST77916)

| Signal | GPIO |
|--------|------|
| CS | 21 |
| CLK | 40 |
| D0 | 46 |
| D1 | 45 |
| D2 | 42 |
| D3 | 41 |
| Backlight (BL) | 5 |
| RST | via TCA9554PWR IO Expander · software reset is sufficient |

> ⚠️ The display uses **QSPI** (4 data lines), not standard SPI. TFT_eSPI is **not compatible** — use Arduino_GFX.

### Microphone (ES7210 Codec)

| Signal | GPIO | Note |
|--------|------|------|
| MCK | 2 | Master Clock |
| BCK | 48 | Bit Clock |
| WS | 38 | Word Select / LRCLK |
| DIN | 39 | Audio Data IN (Mic → ESP32) |
| DOUT | 47 | Audio Data OUT (Speaker) |
| SDA | 11 | I²C Codec Init |
| SCL | 10 | I²C Codec Init |

---

## How It Works

```
[Mic] → [ES7210 Codec] → [I²S 16 kHz Stereo]
                                  ↓
                         RMS over 512 samples
                                  ↓
                      Moving average (8 readings)
                                  ↓
               smooth < LOW  →  🟢 Green
               smooth < HIGH →  🟡 Yellow
               smooth ≥ HIGH →  🔴 Red
```

**Audio processing:**
- ES7210 initialized via I²C (address `0x40`), gain 36 dB, bias 2.87 V
- I²S: 16 kHz · 16-bit · stereo · MCLK = 4.096 MHz
- RMS calculated from left channel only (512 frames = 32 ms per measurement)
- Smoothing: moving average over 8 measurements ≈ 256 ms

**Threshold constants** (`src/main.cpp`):
```cpp
#define THRESHOLD_LOW   0.008f   // below → Green
#define THRESHOLD_HIGH  0.025f   // above → Red
```

---

## Quick Start

### Requirements

- [PlatformIO](https://platformio.org/) (VSCode Extension)
- Board connected via USB-C

### Build & Flash

```bash
# Build
pio run

# Flash
pio run --target upload

# Serial Monitor (for calibration)
pio device monitor
```

### Verified toolchain

This repository is pinned to the following versions for reproducible public builds:

- pioarduino `platform-espressif32` `55.03.38`
- Arduino Core for ESP32 `3.3.8`
- ESP-IDF libraries `5.5.4`
- Arduino_GFX (`GFX Library for Arduino`) `1.6.5`

---

## Calibration

Thresholds need to be tuned to your environment once after first flash.

1. In [src/main.cpp](src/main.cpp), temporarily uncomment the debug output in `loop()`:
   ```cpp
   Serial.printf("rms=%.5f smooth=%.5f\n", rms, smooth);
   ```
2. Flash · open Serial Monitor (115200 baud)
3. Read the values:

   | Situation | Target state | Action |
   |-----------|-------------|--------|
   | Silence / background noise | Green | set `THRESHOLD_LOW` = value × 2–3 |
   | Normal conversation | Yellow | set `THRESHOLD_HIGH` just above |
   | Loud voice / clapping | Red | verify it triggers |

4. Update `THRESHOLD_LOW` and `THRESHOLD_HIGH` near the top of [src/main.cpp](src/main.cpp#L42-L43), comment the debug line back out, then reflash.

---

## Software Stack

| Layer | Library |
|-------|---------|
| Framework | [pioarduino](https://github.com/pioarduino/platform-espressif32) · pinned to `55.03.38` (Arduino Core `3.3.8` / IDF `5.5.4`) |
| Display | [Arduino_GFX](https://github.com/moononournation/Arduino_GFX) `1.6.5` · `Arduino_ST77916` + `Arduino_ESP32QSPI` |
| I²S Audio | `ESP_I2S.h` (Arduino Core 3.x) |
| Codec Driver | ES7210 Library (Espressif · Apache 2.0) · local in `lib/es7210/` |

---

## Project Structure

```
├── src/
│   └── main.cpp          # Application logic
├── lib/
│   └── es7210/           # ES7210 codec driver (Apache 2.0, Espressif)
│       ├── es7210.h
│       ├── es7210.c
│       └── es7210_reg.h
├── .github/workflows/    # GitHub Actions build workflow
├── CONTRIBUTING.md       # Contribution guidelines
├── SECURITY.md           # Vulnerability reporting policy
├── platformio.ini         # Build configuration
└── .gitignore
```

---

## License

This project is released under the **MIT License**.

The ES7210 driver in `lib/es7210/` is copyright **Espressif Systems (Shanghai) CO LTD**, licensed under the **Apache License 2.0**.

For redistribution clarity:

- Main project license: [LICENSE](LICENSE)
- Third-party notices: [NOTICE](NOTICE)
- Full Apache 2.0 text for the bundled ES7210 driver: [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt)
