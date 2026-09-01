# cam-line-follow-and-ball-track

Combined mission: follow the black line with the camera (logic from `cam-line-follow`), and when the line is lost and the car has stopped, switch to the ball-tracking mission (logic from `cam-ball-track`, red ball first, then green ball).

## Mission flow

1. **Line following** (`line_ctrl_task`, 20 ms): black-pixel detection in the near 1/6 ROI, PD on the normalized lateral error, speed scaled by error magnitude, pivot when the error is large. On line loss the car drives straight forward for 0.5 s and then stops and hands over.
2. **Obstacle avoidance** (`ultrasonic_avoid_task`, 60 ms, HC-SR04 on GPIO9/10): the full `cam-line-follow` state machine — IDLE (obstacle <6 cm x3) → STOP → LEFT → EXTRA_LEFT → FORWARD → SEARCH_LINE → ALIGN_LINE → hand back to the line follower → FOLLOW_LINE_UNTIL_LOST → FINISHED_STOP. `s_avoid_active` suspends the line task during the maneuver.
3. **Ball tracking** (`ball_task_ctrl`, 30 ms): after line-loss handover, the `cam-ball-track` state machine runs:
   `SEARCH` (rotate until ball found) → `ALIGN_DIRECTION` (P on x error) → `LOCK_DIRECTION` (hold 1 s) → `APPROACH_BALL` (drive straight 1 s) → `STOP_AFTER_APPROACH` (0.3 s) → `RETURN_TO_WAIT` (reverse 4 s) → switch to the green ball and repeat → `COMPLETE`.

## Architecture

- One decode per frame (`detect_mjpeg`): the same 1/8-scale RGB565 buffer feeds both the line analysis (ROI rows only) and the ball classifier (full frame).
- `s_avoid_active` / `s_ball_mode_active` gate the control tasks so they never fight over `set_speed`: the line task suspends itself during avoidance and after handover, the ball task waits for its flag before starting.
- Motor layer, camera stack (`usb_host_uvc` with local patches) and `cooperative_jpeg` decode are shared with the source projects.

## Tunables

- Line: `LINE_LUM_THRESHOLD` (100), `LINE_MIN_BLACK_PIXELS` (4), `LINE_ROI_FRACTION` (6), `LINE_KP`/`LINE_KD` (1.8/0.035), `LINE_SPEED_MAX_CM_S` (10), `LINE_PIVOT_ERROR` (0.72), `LINE_LOST_STOP_DELAY_MS` (500), `CAMERA_FLIPPED` (1)
- Ball: `RED_*` (80/25, hue `[0,12]∪[300,360]`, min 18 px), `GREEN_*` (129/19, hue `[167,187]`, ratio 0.2%), `SEARCH_WZ_RAD_S` (0.65), `ALIGN_KP` (0.012), `ALIGN_DEADBAND_PX` (8), `APPROACH_SPEED_CM_S` (50), `APPROACH_TIME_MS` (1000), `RETURN_SPEED_CM_S` (-10), `RETURN_TIME_MS` (4000), `BALL_CAMERA_FLIPPED` (0)

## Build & flash

```sh
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
idf.py flash monitor
```

Logs: `line: left=.. right=.. lateral=..` (line phase), `line lost; switching to ball tracking`, then the ball task phase transitions.
