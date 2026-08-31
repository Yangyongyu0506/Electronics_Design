# cam-line-follow

Camera-based black-line following: the USB UVC camera (same stack as `cam-ball-track`, incl. patched `usb_host_uvc` files) decodes MJPEG at 1/8 scale (60x40) and the frame is split into left/right halves; the black-pixel count difference drives a PD controller on `wz` while the car drives forward at constant speed. When the total black-pixel count drops below a threshold the car stops.

## Wiring

Motors: same as `motor-test` (3x TB6612, encoders on 41/40, 8/3, 17/18).
Camera: D+→GPIO20, D-→GPIO19 (native USB).

## Logic

- ROI: the `1/LINE_ROI_FRACTION` (1/4) of the decoded frame nearest to the car — with the camera mounted upside down (`CAMERA_FLIPPED=1`) the near ground is at the TOP of the frame; with an upright camera (`CAMERA_FLIPPED=0`) it is at the bottom
- `CAMERA_FLIPPED` (1): flips both the steering sign of the left/right diff and the ROI side
- Black pixel: `lum = (r + 2g + b)/4 < LINE_LUM_THRESHOLD`
- `diff = black_left - black_right` (image halves; sign corrected by `CAMERA_FLIPPED`)
- `wz = LINE_STEER_SIGN * (LINE_KP*diff + LINE_KD*d(diff)/dt)`, clamped to ±LINE_MAX_WZ
- `vx = LINE_SPEED_CM_S` while `total = left+right >= LINE_MIN_BLACK_PIXELS`
- Stop when `total < LINE_MIN_BLACK_PIXELS`

## Tunables (`main/cam-line-follow.c`)

- `LINE_LUM_THRESHOLD` (220): raise if the floor is detected as black, lower if the line is not detected
- `LINE_MIN_BLACK_PIXELS` (6): minimum black pixels in the ROI to keep driving
- `LINE_ROI_FRACTION` (4): bottom fraction of the frame used for detection
- `LINE_KP` / `LINE_KD` (0.003 / 0.0005): PD on the left-right diff
- `LINE_SPEED_CM_S` (10), `LINE_MAX_WZ` (1.0)
- `LINE_STEER_SIGN` (1.0): flip to -1 if the car steers away from the line
- `CAMERA_FLIPPED` (0): set 1 if the camera is mounted upside down

## Build & flash

```sh
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
idf.py flash monitor
```

Logs: `line: left=.. right=.. total=..` (detection) and `follow: diff=.. total=.. wz=..` (control).
