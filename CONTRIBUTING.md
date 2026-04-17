# Contributing

Thanks for your interest in improving Noise Guardian ESP32.

## Before you open a pull request

- Make sure the firmware still builds with PlatformIO:
  - `pio run`
- Keep changes focused and avoid unrelated reformatting.
- Update [README.md](README.md) when hardware setup, behavior, or build steps change.
- Preserve third-party notices for bundled code in [lib/es7210/](lib/es7210/).

## Pull request guidelines

- Describe the hardware used for testing.
- Mention any threshold or calibration changes.
- Include logs or screenshots when UI or hardware behavior changes.
- Prefer small, reviewable pull requests.

## Development notes

- Main firmware entry point: [src/main.cpp](src/main.cpp)
- Build configuration: [platformio.ini](platformio.ini)
- Bundled third-party codec driver: [lib/es7210/](lib/es7210/)

By submitting a contribution, you agree that your changes may be distributed under the repository license.
