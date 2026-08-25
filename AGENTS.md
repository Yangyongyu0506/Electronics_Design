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
- Wiring tables in `README.md` are the source of truth for hardware connections; keep them in sync with the pin defines in `main/*.c`.
- Board is an ESP32-S3-WROOM-2 (32MB octal flash): GPIO33-37 are taken by the flash and must never be used. GPIO0/3/45/46 are strapping pins; 19/20 are USB-JTAG (wired to the USB connector); 38 is the DevKitC-1 v1.1 onboard RGB LED; 43/44 are the UART console. Encoder pairs must stay on clean pins (A=41/40, B=8/3, D=17/18; E2B on GPIO3 is a strapping pin but works as input).
- New projects follow `idf.py create-project <name>` + `sdkconfig.defaults` (`CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y`) + `idf.py set-target esp32s3`.
- Project roles: `blink-led` reads the LQ-R4CHVB 4-channel line sensor (GPIO11-14, 0=black 1=white, CH1 rightmost). `motor-test` drives the 3 TB6612 motors with encoder PID speed control and owns `set_speed(vx, vy, wz)` (cm/s, cm/s, rad/s, ROS2 frame: x forward, y left, CCW+; omni wheels, drive dirs A=30° B=270° D=150°, base 9 cm, 512 counts/wheel-rev measured (spec "512 AB" already includes ×4), wheels 5.5 cm). `line-follow` reuses the same PID closed loop + `set_speed` (kept in sync by copy-paste) plus the line sensor for black-line following (P control on wz, stop on all-white). `encoder-test` only reads the 3 wheel encoders (no motor output) to verify PCNT counts by hand.
- Build artifacts (`build/`) are gitignored.
