# cam-line-follow

Camera-based black-line following for a camera whose useful ground view is only about 2 cm in front of the car. The USB UVC camera decodes MJPEG at 1/8 scale. Steering uses only the nearest 1/8 of the image so that a 90-degree corner visible farther ahead cannot make the car cut across the corner. The controller slows down as error grows and pivots in place for large errors.

## Wiring

Motors: same as `motor-test` (3x TB6612, encoders on 41/40, 8/3, 17/18).
Camera: D+ -> GPIO20, D- -> GPIO19 (native USB).
HC-SR04: TRIG -> GPIO9, ECHO -> GPIO10. The ECHO signal must use a voltage divider or level shifter to 3.3 V.

## Logic

- ROI: only the nearest `1/LINE_ROI_FRACTION` (1/8) of the decoded frame. With the camera upside down (`CAMERA_FLIPPED=1`), physical near ground is at the top of the decoded image; otherwise it is at the bottom.
- Black pixel: `lum = (r + 2g + b)/4 < LINE_LUM_THRESHOLD`.
- `lateral` is the horizontal centroid of black pixels in this near-only ROI, normalized to roughly -1..1. There is deliberately no far-band or heading look-ahead.
- `error = lateral`.
- A PD controller produces `wz`, clamped to `+-LINE_MAX_WZ`.
- Forward speed falls from `LINE_SPEED_MAX_CM_S` to `LINE_SPEED_MIN_CM_S` as error increases. At `LINE_PIVOT_ERROR`, the car rotates without advancing.
- When the near ROI contains fewer than `LINE_MIN_BLACK_PIXELS` black pixels, the car keeps its last motion command for `LINE_LOST_STOP_DELAY_MS` (500 ms), then stops if the line is still missing. Seeing the line again within that interval cancels the pending stop. A stale camera frame still stops the car immediately.

At 3 cm/s and 10 FPS the car advances about 0.3 cm per frame, giving the controller several observations within the 2 cm visible area.

## Tunables (`main/cam-line-follow.c`)

- `LINE_LUM_THRESHOLD` (100): raise it if the line is missed; lower it if the floor is detected as black.
- `LINE_MIN_BLACK_PIXELS` (4): minimum black pixels required.
- `LINE_ROI_FRACTION` (8): fraction of the frame used as the near-only ROI.
- `LINE_KP` / `LINE_KD` (1.8 / 0.035): PD gains for normalized error.
- `LINE_SPEED_MAX_CM_S` / `LINE_SPEED_MIN_CM_S` (3.0 / 1.5).
- `LINE_PIVOT_ERROR` (0.72), `LINE_MAX_WZ` (1.2).
- `LINE_LOST_STOP_DELAY_MS` (500): time to keep moving after losing the line.
- `LINE_STEER_SIGN` (1.0): change to -1 if the car steers away from the line.
- `CAMERA_FLIPPED` (1): set to 1 for a camera mounted upside down.

## Build & flash

```sh
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
idf.py flash monitor
```

Useful logs are `line: ... lateral=.. heading=..` and `follow: ... vx=.. wz=..`.

## Obstacle avoidance

The HC-SR04 is sampled every 60 ms. Three consecutive valid readings below 6 cm start the same avoidance sequence used by the previous `line-follow` obstacle course:

1. Stop for 200 ms.
2. Strafe left at 15 cm/s until three readings indicate that the obstacle is clear.
3. Continue left for another 700 ms.
4. Drive forward at 18 cm/s for 2 seconds. If the obstacle appears again, return to the left-strafe step.
5. Strafe right until the camera sees the black line in three consecutive new frames.
6. Drive straight (no steering) until the camera misses the line in three consecutive new frames, then stop.

Obstacle avoidance has priority over camera steering. Camera line-loss handling is suspended while avoidance is active, including the final straight-ahead segment; the camera steering controller does not regain control after avoidance.
