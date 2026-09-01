# cam-ball-track

Camera ball tracking: the USB UVC camera (patched `usb_host_uvc` stack) decodes MJPEG at 1/8 scale, classifies target-colored pixels by HSV-style hue, computes the blob centroid, and drives the car with a two-phase PD controller — rotate to center the ball in x first, then drive forward/backward to center it in y.

## Color thresholds

The pixel classifier works in HSV-ish space: `value = max(r,g,b)`, `saturation = max-min`, `hue` in degrees from the standard hue formula. A pixel is a target pixel when `value >= MIN_VALUE`, `saturation >= MIN_SATURATION` and its hue falls inside the target band.

| Parameter | Green ball (current) |
|-----------|----------------------|
| `MIN_VALUE` | 129 |
| `MIN_SATURATION` | 19 |
| Hue band | `[167, 188]` |
| `HUE_MIN_ANGLE` | 167 |
| `HUE_MAX_ANGLE` | 187 |
| `DETECT_RATIO_PERCENT` | 0.2 |

Previous experiments: red ball used `MIN_VALUE=80`, `MIN_SATURATION=25`, hue band `[0,12] ∪ [300,360]`, `DETECT_RATIO_PERCENT=0.5`; earlier green sets were `[165,190]` with value 120 / ratio 1.0.

The centroid is published only when the target-pixel ratio of the frame exceeds `DETECT_RATIO_PERCENT`; otherwise the ball is considered not visible (car stops after the aim timeout).

## Wiring

Motors: same as `motor-test` (3x TB6612, encoders 41/40, 8/3, 17/18).
Camera: D+→GPIO20, D-→GPIO19.

## Control

- `err_x = cx - frame_center_x`, `err_y = cy - frame_center_y` (full-res pixels, sign corrected by `CAMERA_FLIPPED`)
- Phase 1: `|err_x| > AIM_DEADBAND_PX` → rotate in place, `wz = AIM_KP*err_x + AIM_KD*d(err_x)/dt` (clamped ±`AIM_MAX_WZ`)
- Phase 2: x aligned → `vx = AIM_Y_SIGN * (AIM_KP_Y*err_y + AIM_KD_Y*d(err_y)/dt)` (clamped ±`AIM_MAX_VX_CM_S`)
- Ball lost for `AIM_TIMEOUT_US` → stop

## Tunables (`main/cam-ball-track.c`)

- `GREEN_HUE_MIN_ANGLE` / `GREEN_HUE_MAX_ANGLE` (168 / 188): green band centered on the camera's rendering of the ball (teal green, hue ~170-183°)
- `GREEN_MIN_SATURATION` (20), `GREEN_MIN_VALUE` (130): exclude gray/white and dark noise
- `GREEN_DETECT_RATIO_PERCENT` (1.2): minimum target-pixel ratio for "ball present"
- `AIM_KP`/`AIM_KD` (0.01 / 0.00015): rotation PD
- `AIM_KP_Y`/`AIM_KD_Y` (0.5 / 0.0): forward/backward PD
- `AIM_Y_SIGN` (-1): flip if the car drives the wrong way along y
- `CAMERA_FLIPPED` (0): set 1 if the camera is mounted upside down

## Binary mask streaming

With `STREAM_BINARY_MASK=1` the firmware prints one line per decoded frame on the UART console:

```
BIN <width> <height> <seq> <base64>
```

`<base64>` is the binarized green mask: one bit per pixel, MSB first, row-major (1 = passes the green threshold). View it live on the PC:

```sh
pip install pyserial
python3 tools/mask_viewer.py -p /dev/ttyUSB0 -b 115200 -s 8
```

## Build & flash

```sh
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
idf.py flash monitor
```
