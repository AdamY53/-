# Codex Change Log

## 2026-08-24 - DSH: Remove All-Zero Line Stop for Debugging

Files changed:
- `User/main.c`

What changed:
- Deleted the "all grayscale sensors low -> stop" branch from the line
  following loop.
- The car now keeps running the line-following state machine even when the
  seven sensors are all zero, so you can check whether the 90-degree turn
  detector is being affected by an all-low signal pattern.
- No turn tuning windows were changed in this edit.

Build/verification:
- `User/main.c` compiled successfully with ARMCC to
  `Objects/codex_main_no_zero_stop.o`.

## 2026-08-20 - DSH: Remove Dead APPROACH Turn Code (trigger-then-pivot only)

Files changed:
- `User/main.c`

What changed:
- Removed the dead "forward entry" turn stage and its parameters:
  - `CAR_TURN_ENTRY_FORWARD_COUNT`, `CAR_TURN_ENTRY_COUNT_WINDOW`,
    `CAR_TURN_ENTRY_MAX_TICKS`, `CAR_LINE_STATE_APPROACH`,
    `Turn_Forward_Count`, `Turn_Last_Forward_Count`, `AbsEncoderCount`.
  - The APPROACH branch of `Car_RunSharpTurn` (drive straight while
    accumulating encoder counts) is gone: a T-turn now immediately enters
    `CAR_LINE_STATE_FIXED_TURN` and pivots in place for
    `CAR_FIXED_TURN_TICKS` loops, then cools down for
    `CAR_TURN_COOLDOWN_TICKS`.
  - OLED debug page no longer shows the `C`/`W` forward-count windows; only
    the `M` state character remains (F follow / L/R turning / K cooldown).
- Kept current tuning: `CAR_SHARP_CONFIRM_TICKS=1`,
  `CAR_FIXED_TURN_PWM=30`, `CAR_FIXED_TURN_TICKS=13`,
  `CAR_TURN_COOLDOWN_TICKS=18`.

Build/verification:
- Reviewed in DSH agent session (brace/paren balance 74/74, 177/177);
  ARMCC build to be re-run in Keil uVision before field testing.

## 2026-08-20 - DSH: Ultrasonic Fully Disabled, Focus on Basic Line Tracking

Files changed:
- `User/main.c`

What changed:
- Removed ALL ultrasonic usage from `main.c` to rule out the ultrasonic
  modules while tuning the basic line-following:
  - Deleted the ultrasonic OLED display page and `OLED_ShowUltrasonicLine`.
  - Deleted the 200ms ranging loop in `main()`: the line-following/T-turn
    state machine now runs every 20ms loop with no detection blind window.
  - Deleted the US mode 1/2 logic, `Car_UpdateUltrasonicOneStep`, the
    distance globals, the US macros, and the `Ultrasonic.h` include.
  - K3/K2 page switching removed; only K1 start/stop remains. The OLED shows
    only the line debug page (IR pattern, E/D, S/B, PL/PR, encoders, M/C/W).
- Kept the ultrasonic driver files (`Hardware/Ultrasonic.c/.h`) untouched so
  the modules can be re-enabled later.
- Kept all current tuning: `CAR_SHARP_CONFIRM_TICKS=1`,
  `CAR_TURN_ENTRY_FORWARD_COUNT=0`, `CAR_FIXED_TURN_PWM=34`,
  `CAR_FIXED_TURN_TICKS=13`, `CAR_TURN_COOLDOWN_TICKS=18`.

Build/verification:
- Reviewed in DSH agent session (brace/paren balance 79/79, 191/191);
  ARMCC build to be re-run in Keil uVision before field testing.

## 2026-08-20 - DSH: Single-Channel Ultrasonic Polling + US Mode 1/2 (K3 switch)

Files changed:
- `User/main.c`

What changed:
- Ultrasonic polling changed from "3 consecutive ranging loops every 200ms"
  to "1 ranging loop every 200ms" (front / enabled-side alternating,
  `ULTRASONIC_ROUND_GAP_LOOPS = 200/20 - 1 = 9`). The line-following and
  T-turn detection blind window drops from 3/10 (30%) to 1/10 (10%), which
  was the main cause of the ~60% missed T-turn detection observed on the
  car.
- Added ultrasonic mode 1/2: mode 1 = front + left, mode 2 = front + right
  (`CAR_US_MODE_LEFT` / `CAR_US_MODE_RIGHT`). Only the enabled side channel
  is measured (`Car_UpdateUltrasonicOneStep`), so the unplugged module no
  longer blocks for its 30ms echo timeout every round.
- K3 short press now switches the ultrasonic mode (1 <-> 2) and jumps to the
  ultrasonic display page; K3 long press (~1s) keeps the original
  "show ultrasonic page" function. The ultrasonic OLED page shows the active
  mode (`U1`/`U2`) and `L: OFF`/`R: OFF` for the disabled side.
- Boot hint text updated to "K3 US Mode".

Build/verification:
- Reviewed in DSH agent session (brace/paren balance 100/100, 227/227);
  ARMCC build to be re-run in Keil uVision before field testing.

## 2026-08-20 - DSH: T-Turn Detection Replaced with Pattern Matching (ported from 送药小车)

Files changed:
- `User/main.c`

What changed:
- Removed the old "count active sensors" 90-degree detection
  (`CAR_SHARP_GROUP_ACTIVE_MIN`, `Car_CountLeftTurnSensors`,
  `Car_CountRightTurnSensors`).
- Ported the left/right T-branch pattern matching from the
  送药小车_四段PID循迹版 project:
  - `Car_GetLineSensorMask` / `Car_MatchDisplayPatternByMask` /
    `Car_MatchEnabledDisplayPattern`: bit-order pattern strings
    (R3,R2,R1,M,L1,L2,L3; `1`=must be on, `0`=must be off, `-`=don't care),
    each pattern has an `ENABLE` switch.
  - Left T (physical left sensors on): `"000-111"` + `"0000011"` enabled.
  - Right T (physical right sensors on): `"111-000"` + `"1100000"` enabled.
  - Note: in the 送药小车 code the LEFT_T/RIGHT_T names are opposite to the
    physical bit order, and its turn direction comes from a pre-set route
    table, not from the T signal. Here the groups follow the 避障小车 physical
    direction (right side on -> turn right), and the direction mapping is
    exposed as `CAR_T_LEFT_ACTION` / `CAR_T_RIGHT_ACTION` so a reversed car
    only needs one macro change.
- Kept the confirm window `CAR_SHARP_CONFIRM_TICKS = 2` (a T signal must be
  seen for 2 consecutive loops or the counters reset), the hover guard
  (`Gray_ActiveCount < 2` clears both counters), and the existing
  two-stage turn action (approach straight by encoder window
  `CAR_TURN_ENTRY_FORWARD_COUNT`, then pivot with `CAR_FIXED_TURN_PWM` /
  `CAR_FIXED_TURN_TICKS`, then `CAR_TURN_COOLDOWN_TICKS` ignore window).
- The straight-line following logic (PD steering, encoder balance, lost-line
  stop) was NOT modified.

Build/verification:
- Reviewed in DSH agent session (brace/paren balance checked); ARMCC build
  to be re-run in Keil uVision before field testing.

## 2026-08-20 - DSH: Ultrasonic Out of Control Loop, Fixed 20ms Slice, Hover Guard

Files changed:
- `User/main.c`

What changed:
- Kicked the ultrasonic ranging out of the per-loop control path. A full
  front/left/right round now runs once every 200ms (3 consecutive ranging
  loops), and the tracking state machine is paused during those ranging loops
  so the blocking echo wait (up to ~30ms per channel) can no longer stretch
  the control period.
- After each full round the loop skips `ULTRASONIC_ROUND_GAP_LOOPS` (= 7)
  main loops before the next round, so each direction is refreshed every
  200ms (10 loops x 20ms). The gap is a tunable window: set it to 5 for the
  old ~160ms cadence.
- Fixed the time slice: `Delay_ms(20)` now runs at the top of `while(1)`,
  i.e. before the ranging call, so the fixed 20ms slice is always reserved
  and the overall loop period stays close to 20ms on non-ranging loops.
- Changed `CAR_SHARP_CONFIRM_TICKS` from 1.5 (float) to the integer 2.
- Added a hover guard in `Car_UpdateSharpTurnDetect()`: when
  `Gray_ActiveCount < 2`, both `Sharp_Left_Count` and `Sharp_Right_Count` are
  cleared immediately, preventing a false 90-degree trigger while the car is
  lifted off the track.

Build/verification:
- Reviewed in DSH agent session; ARMCC build to be re-run in Keil uVision
  (`Objects/` outputs regenerate on next project build).

## 2026-08-17 - Stronger 90-Degree Turn Trigger and Cooldown

Files changed:
- `User/main.c`

What changed:
- Strengthened 90-degree turn detection from side-only 2-of-3 to
  `M+L1+L2+L3` or `M+R1+R2+R3` 3-of-4 confirmation.
- Added the adjustable `CAR_TURN_COOLDOWN_TICKS` window after each completed
  90-degree turn. During cooldown, the car keeps normal line following but
  will not trigger another 90-degree turn.
- Kept all turn tuning windows annotated with Chinese comments.
- Changed the OLED line debug page to show `C` as the active or last saved
  forward-entry encoder count and `W` as the code target window, so `C` can
  be compared directly with `CAR_TURN_ENTRY_FORWARD_COUNT`.

Build/verification:
- `User/main.c` compiled successfully with ARMCC to
  `Objects/codex_main_turn_cooldown.o`.

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
