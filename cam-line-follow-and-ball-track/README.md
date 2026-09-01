# cam-line-follow-and-ball-track

Combined mission: first follow the black line with the camera (logic from `cam-line-follow`). Only after the line mission has ended and the car has stopped does ball detection and ball tracking start (logic from `cam-ball-track`, red ball first, then green ball).

## Mission flow

1. **Line following** (`line_ctrl_task`, 20 ms): black-pixel detection in the near 1/6 ROI, PD on the normalized lateral error, speed scaled by error magnitude, pivot when the error is large. On line loss the car drives straight forward for 0.5 s, stops, then waits 3 s before handing over to ball tracking.
2. **Obstacle avoidance** (`ultrasonic_avoid_task`, 60 ms, HC-SR04 on GPIO9/10): the full `cam-line-follow` state machine — IDLE (obstacle <6 cm x3) → STOP → LEFT → EXTRA_LEFT → FORWARD → SEARCH_LINE → ALIGN_LINE → hand back to the line follower → FOLLOW_LINE_UNTIL_LOST → FINISHED_STOP. `s_avoid_active` suspends the line task during the maneuver.
3. **Ball tracking** (`ball_task_ctrl`, 30 ms): the MG90S is initialized with `SERVO_INITIAL_DUTY_PERCENT`. After line-loss handover, `start_ball_tracking()` clears prior ball results and enables the ball classifier. On first entry to `SEARCH`, PWM changes to `SERVO_SEARCH_DUTY_PERCENT`; LEDC then holds that duty cycle through all later ball-task states. The `cam-ball-track` state machine then runs:
   `SEARCH` (rotate until ball found) → `ALIGN_DIRECTION` (P on x error) → `LOCK_DIRECTION` (hold 1 s) → `APPROACH_BALL` (drive straight 1 s) → `STOP_AFTER_APPROACH` (0.3 s) → `RETURN_TO_WAIT` (reverse 4 s) → switch to the green ball and repeat → `COMPLETE`.

## Architecture

- One decode per frame (`detect_mjpeg`): the same 1/8-scale RGB565 buffer feeds line analysis (ROI rows only); ball-pixel classification (full frame) is enabled only after handover.
- `s_avoid_active` / `s_ball_mode_active` gate the control tasks so they never fight over `set_speed`: the line task suspends itself during avoidance and after handover; the ball task waits for its flag, and the avoidance task becomes idle after handover.
- Motor layer, camera stack (`usb_host_uvc` with local patches) and `cooperative_jpeg` decode are shared with the source projects.

## MG90S wiring

| MG90S lead | ESP32-S3 / power connection |
| --- | --- |
| Signal (orange/yellow) | `SERVO_PWM_GPIO` (`GPIO14`; change the define if wiring changes) |
| V+ (red) | External regulated 5 V supply |
| GND (brown/black) | External supply GND and ESP32-S3 GND must be common |

The servo uses its own LEDC timer/channel at 50 Hz, so its PWM is generated in hardware and does not block camera or chassis-control tasks.

## Tunables

- Line: `LINE_LUM_THRESHOLD` (100), `LINE_MIN_BLACK_PIXELS` (4), `LINE_ROI_FRACTION` (6), `LINE_KP`/`LINE_KD` (1.8/0.035), `LINE_SPEED_MAX_CM_S` (10), `LINE_PIVOT_ERROR` (0.72), `LINE_LOST_STOP_DELAY_MS` (500), `CAMERA_FLIPPED` (1)
- Ball: `RED_*` (80/25, hue `[0,12]∪[300,360]`, min 18 px), `GREEN_*` (60/10, hue `[90,155]`, ratio 0.1%), `BALL_STALE_US` (1500000), `BALL_START_DELAY_MS` (3000), `SEARCH_WZ_RAD_S` (0.65), `RED_ALIGN_KP`/`GREEN_ALIGN_KP` (0.012/0.010), `ALIGN_DEADBAND_PX` (8), `APPROACH_SPEED_CM_S` (50), `APPROACH_TIME_MS` (1000), `RETURN_SPEED_CM_S` (-10), `RETURN_TIME_MS` (4000), `BALL_CAMERA_FLIPPED` (0)

- MG90S: `SERVO_PWM_GPIO` (`GPIO14`), `SERVO_INITIAL_DUTY_PERCENT` (2.5%), and `SERVO_SEARCH_DUTY_PERCENT` (2.5%). Adjust these centralized defines for the installed servo and linkage.

## Build & flash

```sh
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
idf.py flash monitor
```

Logs: `line: left=.. right=.. lateral=..` (line phase), `line lost; switching to ball tracking`, then the ball task phase transitions.
