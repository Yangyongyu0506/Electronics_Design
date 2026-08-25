# AGENTS.md

ESP32 course projects (Electronics Design curriculum). Each top-level directory is a separate ESP-IDF project; there is no root build.

## Environment & build

- ESP-IDF v5.4.4, target `esp32s3`, 2MB flash. Toolchain lives at `~/.espressif/v5.4.4/` (not on PATH by default).
- To build/flash a project (e.g. `blink-led`):
  ```sh
  source ~/.espressif/v5.4.4/esp-idf/export.sh
  cd blink-led && idf.py build        # then: idf.py flash monitor
  ```
- No tests, linter, or CI in this repo. `idf.py build` is the only verification.

## Conventions & gotchas

- `managed_components/` (downloaded deps, e.g. `espressif__led_strip`) and `sdkconfig`/`sdkconfig.old` are committed. Never hand-edit them; change `main/idf_component.yml` instead and let `idf.py` regenerate.
- `blink-led` drives a WS2812 (GRB) LED on GPIO 38. Wiring tables in `README.md` are the source of truth for hardware connections (TB6612 motor driver pins); keep them in sync with `main/main.c` defines.
- Build artifacts (`build/`) are gitignored.
