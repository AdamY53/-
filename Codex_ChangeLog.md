# Codex Change Log

## 2026-08-16 - Map Tracking Without Servo Avoidance

Files changed:
- `User/main.c`
- `接线说明.txt`

What changed:
- Removed all servo-based obstacle-avoidance logic from `main.c`.
- Restored fixed three-channel ultrasonic polling: front, left, and right
  modules are measured in rotation and displayed separately on OLED.
- Fixed 90-degree map turns now use two adjustable windows: encoder-count
  forward entry distance and reverse-direction differential motor PWM/time.
- OLED line debug page keeps `L`, `R`, and `C` in the same encoder-count
  unit, so the observed wheel counts can be copied directly into the forward
  entry-distance tuning window.
- Updated wiring notes to describe the three fixed ultrasonic modules and
  remove the obsolete servo scanning/avoidance text.

Build/verification:
- `User/main.c` compiled successfully with ARMCC to
  `Objects/codex_main_map_tracking.o`.

## 2026-08-16 - Fixed-Time 90-Degree Line Turn

Files changed:
- `User/main.c`

What changed:
- Replaced the previous 90-degree turn search/reacquire state machine with
  the simpler fixed-time turn approach.
- The sharp-turn trigger is still conservative: the left or right 3-sensor
  group must have at least 2 active sensors for 2 consecutive control ticks.
- Once triggered, the car first drives straight until the average left/right
  encoder increment reaches the adjustable `CAR_TURN_ENTRY_FORWARD_COUNT`
  window, then pivots in the detected direction for `CAR_FIXED_TURN_TICKS`
  ticks and resets to normal line following.
- Added Chinese comments for the adjustable 90-degree turn windows, including
  entry distance, entry tolerance, encoder fallback timeout, and fixed turn
  duration.
- The OLED line debug page now shows `C`, the accumulated encoder count used
  by the entry-distance window.
- The OLED line debug page now also shows left/right wheel cumulative encoder
  counts, with forward movement displayed as positive values.
- K1 start now resets the wheel cumulative counters, so each test run starts
  from zero.
- Kept the latest obstacle-avoidance, OLED, key, ultrasonic, and servo logic
  untouched in this change.

Build/verification:
- `User/main.c` compiled successfully with ARMCC to
  `Objects/codex_main_fixed_turn.o`.

## 2026-08-16 - Servo-Centered Obstacle Avoidance

Files changed:
- `User/main.c`
- `接线说明.txt`

What changed:
- Changed the normal tracking servo posture from 0 degrees to 90 degrees so
  the HC-SR04 faces forward during line following.
- Added a front-obstacle avoidance state machine: stop, scan left and right
  with the SG90-mounted front ultrasonic module, choose the clearer side,
  reverse briefly, turn around the obstacle, move past it, then search back
  for the center line.
- The OLED ultrasonic page now shows the live servo angle and the most recent
  forward/left/right scan distances from the servo-mounted front sensor.
- K1 start/stop now resets avoidance state and returns the servo to 90 degrees.

Build/verification:
- `User/main.c` compiled with ARMCC successfully.
- Linked with `armlink` successfully and regenerated `Objects/Project.hex`.
- Program size: `Code=17350 RO-data=3250 RW-data=92 ZI-data=2660`.

## 2026-08-16 - 90-Degree Line Turn State Machine

Files changed:
- `User/main.c`

What changed:
- Added a dedicated 90-degree turn state machine for line following.
- A left or right sharp turn is detected only after the matching side group
  has at least 2 active sensors for 2 consecutive control ticks.
- After detection, the car moves forward briefly, performs a minimum pivot
  turn, then slows down and keeps searching until L1/M/R1 reacquires the line
  for 2 consecutive ticks.
- During the turn state, temporary full line loss no longer stops the car
  immediately; timeout protection still stops the motors if the line is not
  found again.
- The OLED line debug page now shows signed left/right PWM and a compact
  mode indicator: `F` follow, `A` approach, `L/R` minimum turn, `S` search.

Build/verification:
- Pending local build verification.

## 2026-08-16 - Straight-Line-Only Control and Ultrasonic Diagnostics

Files changed:
- `User/main.c`
- `Hardware/Grayscale.c`
- `Hardware/Grayscale.h`
- `Hardware/Ultrasonic.c`
- `Hardware/Ultrasonic.h`

What changed:
- Removed obstacle avoidance, servo scanning, open-loop turns, and reverse
  motor commands from the application layer.
- Restricted line-following output to forward PWM only. Both wheels keep the
  configured minimum forward PWM while tracking; a lost line stops the car.
- Kept K1 as start/stop, K2 as the line-debug OLED page, and K3 as the three-
  channel ultrasonic OLED page.
- Fixed the servo at 0 degrees during initialization and removed later angle
  changes.
- Removed unused crossing, turn, and route-history state from the grayscale
  driver; it now only samples the seven sensors and counts active channels.
- Reworked ultrasonic timing to measure the ECHO pulse with one continuous
  SysTick counter, spaced channel sampling to reduce acoustic crosstalk, and
  exposed error status codes. OLED now shows `--- E:n` for a missing echo
  instead of presenting a timeout as `0 cm`.

Build/verification:
- Keil uVision build completed successfully with 0 errors and 0 warnings.
- Program size: `Code=15878 RO-data=3250 RW-data=80 ZI-data=2656`.

## 2026-08-15 15:30 - Straight Line PD Following

Files changed:
- `User/main.c`

Backup before change:
- `User/main_20260815_152818_before_straight_pd.c`

What changed:
- Replaced weighted open-loop line following with 7-sensor centroid error plus PD steering.
- Added light encoder balance to reduce left/right motor speed mismatch on straight lines.
- Added OLED debug values on the line test page:
  - `E`: line error, negative means line is on the left, positive means line is on the right.
  - `S`: PD steering PWM correction.
  - `B`: encoder balance PWM correction.
- Kept existing grayscale, motor, encoder, ultrasonic, servo, and key drivers unchanged.

Build/verification:
- `User/main.c` compiled to `Objects/main.o`.
- Linked with `armlink` successfully.
- Regenerated `Objects/Project.axf` and `Objects/Project.hex`.
- Program size: `Code=17054 RO-data=3250 RW-data=96 ZI-data=2816`.

Notes:
- UV4 command-line build stalled while closing, so final verification used `armcc` output plus direct `armlink/fromelf`.
- Initial tuning is conservative for stable straight-line tracking. Increase `CAR_BASE_PWM` only after the car can hold the line reliably.
