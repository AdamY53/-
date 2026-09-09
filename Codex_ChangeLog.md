# Codex ChangeLog

## 2026-08-24 - MODE C: obstacle 90° turns use separate CW/CCW encoder targets

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`
- `Codex_ChangeLog.md`

What changed:
- Mode C obstacle-bypass 90° turns no longer reuse the generic map-turn
  targets. Two independent groups are selected at runtime by `Obst_RouteDir`
  (0=CW clockwise, 1=CCW counter-clockwise):
  - CW group: `CAR_OBST_CW_LEFT/RIGHT_TURN_LEFT/RIGHT_TARGET`
  - CCW group: `CAR_OBST_CCW_LEFT/RIGHT_TURN_LEFT/RIGHT_TARGET`
- New loader `Car_LoadObstacleTurnTargets(Direction)`; `Obst_StartPivot`
  now calls it instead of `Car_LoadTurnEncoderTargets`, so every bypass
  pivot (T1..T4) uses the group matching the current lap direction.
- Both groups initialized to the previous generic values (-400/690/400/-690);
  user calibrates each group separately on the car.
- This commit also carries the user's in-Keil tuning: CAR_MB_* blind-turn
  targets -420/700/420/-700, CAR_OBST_TRIGGER_CM=22, D2_AVG=1920,
  D3_AVG=1300, LINE_MIN=3.

Build/verification:
- Reviewed (UTF-8 paren/brace 855/855, 341/341 balanced); ARMCC build to be
  confirmed by the user in Keil.

## 2026-08-24 - MODE A: first-lap result page (per-segment + total + blocks), frozen OLED

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`
- `Codex_ChangeLog.md`

What changed:
- New OLED page `CAR_DISPLAY_PAGE_RESULT` (4th page, K2 cycles to it in mode A only).
- When mode A finishes lap 1 (returns to start and starts lap 2), the four
  per-segment times, the total lap time, the block count, and the route mode
  are snapshotted (`Car_SnapshotLapResult` in `Car_AdvanceRouteSegment` when
  `Lap_Count == 0`).
- OLED auto-switches to the result page and FREEZES it (`Lap_Result_Frozen`):
  the page keeps showing lap-1 numbers while the car keeps looping; K2 page
  changes are ignored while frozen.
- Pressing K1 (stop) releases the freeze but keeps the data (K2 can return to
  the RESULT page). Pressing K1 again (start a new run) clears the snapshot
  and returns to the live Track page.
- Switching work modes (K3 long press) also clears the snapshot/page state.
- User confirmed current working-tree C-mode obstacle macros stay as-is:
  CAR_OBST_D1_AVG=1200, D2_AVG=2000, D3_AVG=2000, LINE_MIN=4 (committed
  together with this feature).

Build/verification:
- Reviewed (UTF-8 paren/brace 842/842, 332/332 balanced); ARMCC build to be
  confirmed by the user in Keil.


## 2026-08-24 - MODE B/C: B stops after blind rejoin; C rewritten to blind AVG run

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed:
- Mode B: after the blind (no-line) drive re-detects the black line and
  does the rejoin turn, the car now STOPS (`CAR_MODE_B_STATE_REJOIN_DONE`,
  RunModeBNav stops and holds) instead of continuing normal tracking.
- Mode C obstacle bypass rewritten to a blind fixed-AVG run (ultrasonic
  only used at trigger and D3 line-detection):
  stop -> T1(90, bypass side) -> settle -> G1 drive CAR_OBST_D1_AVG ->
  settle -> T2(opposite, back to heading) -> settle -> G2 drive
  CAR_OBST_D2_AVG -> settle -> T3(same as T2) -> settle -> G3 drive
  (grayscale line sight stops early, CAR_OBST_D3_AVG is the cap) -> settle
  -> T4(back to heading) -> hand back to tracking. All settles use
  CAR_OBST_STOP_TICKS. Directions: seq 1/4 = bypass side,
  seq 2/3 = opposite. Old X1/X2/WALL/FIND ultrasonic phases and their
  macros/helpers were removed.
- D1/D2/D3 defaults 1050/2100/700 AVG (~15/30/10 cm at 70 counts/cm);
  tune the macros on the car.
- Restored user values after an accidental checkout: CAR_STRAIGHT_PWM_TRIM=1,
  CAR_MB_* blind-turn targets -410/690/410/-690.

Build/verification:
- Reviewed (UTF-8 paren/brace 781/781, 317/317); ARMCC build to be
  re-run in Keil.

## 2026-08-24 - ARCH: mode roles split A/B/C, mode A laps forever

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed (per user + task doc 控制.docx):
- Mode roles: A = basic reqs 1+3 (line following + per-segment timing +
  perimeter block counting); B = basic req 2 (Q/O point navigation);
  C = extension (road obstacle bypass) as a NEW standalone mode
  (K3 long press now cycles A -> B -> C).
- Mode A now laps forever (`CAR_ROUTE_LOOP_ENABLE`): after finishing one
  lap the route restarts at segment 0 with Lap_Count++ (insurance, timing
  and segment display continue; OLED shows current lap). K1 stop ends it.
- Perimeter block counting stays first-lap only (`Lap_Count == 0`) then the
  count is frozen on the display, as before.
- Obstacle bypass moved to mode C only (`Car_ObstMonitor` now arms only in
  mode C, no Route requirement); A/B no longer auto-bypass. C is
  free line-following + bypass, no route insurance (insurance excludes
  mode C), no route timing. `Obst_BeginBlock` uses a fixed default turn
  side in mode C (`CAR_OBST_C_DEFAULT_TURN`, default LEFT/right sensor).
- K1 in mode C does not start route timing; switching modes resets bypass,
  servo to front, line controller.
- OLED Track page: shows `MODE:C / FREERUN` in mode C; in A/B shows the
  segment + lap number (e.g. `A-B L01`).

Build/verification:
- Reviewed (UTF-8 paren/brace 808/808, 326/326); ARMCC build to be
  re-run in Keil.

## 2026-08-24 - INSUR: route segment missed-turn insurance (A/B modes)

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed (per user):
- New "route insurance": if a 90-degree junction was not detected by the
  gray sensors, the car falls back by distance (AVG counts since the last
  completed fixed action) and forces the missing action, so it never runs
  off the track.
  - Value table (window = index in Route_SegmentTurnCount):
    forward normal segments AB/BC/CD = 6959; reverse BA/CB/DC = 7298;
    A-D = 1968,3420,3720,3720,1968; D-A = 1710,3720,3420,3420,1710.
  - Direction table: forward normal = LEFT; reverse = RIGHT; A-D = L,R,R,L,L;
    D-A = R,L,L,R,R (mirror; flip tables if real-car direction is reversed).
  - Triggered action: fixed 90-degree pivot with no entry drive
    (`Car_StartPivotTurnEx`, still counts the route turn and can end the
    segment, so normal route flow continues).
  - B-mode special window (A-D/D-A first segment, first special T that
    should NOT turn but drive straight to Q): if it was missed and AVG
    exceeds the window value, the rescue action is to enter the drive-to-Q
    phase (`Car_ModeBStartForwardToQ`), not a turn.
- Base AVG reset: at each finished fixed action (`Car_FinishSharpTurn`),
  at route start (`Car_StartRouteTiming`); insurance is checked in line
  following after sharp-turn detection (cooldown excluded). Enabled by
  `CAR_INSUR_ENABLE` (1).

Build/verification:
- Reviewed (UTF-8 paren/brace 731/731, 315/315); ARMCC build to be
  re-run in Keil.

## 2026-08-24 - OBST: FIND = see line -> drive 200 -> 4th 90 turn (one action)

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed (per user): the FIND phase is one combined fixed action:
1. wait until grayscale shows >= CAR_OBST_LINE_MIN for
   CAR_OBST_LINE_CONFIRM frames (line sight is the action START, at which
   point the drive distance resets);
2. drive straight CAR_OBST_FIND_FORWARD(200) AVG counts;
3. 4th 90-degree pivot back to the track heading;
then hand back to line following.
- No more "see line -> hand back immediately".
- `Obst_SideSeen` is cleared when entering FIND (reused as the
  "line sighted" flag) and reset when pivot 4 completes via Obst_ResetNav.
- Carries user's on-site values: CAR_ULTRASONIC_SAMPLE_TICKS 5,
  CAR_OBST_ENC_FIXED 550.

Build/verification:
- Reviewed (UTF-8 paren/brace 627/627, 287/287); ARMCC build to be
  re-run in Keil.

## 2026-08-24 - OBST: X1 exit >30cm; FIND = fixed straight + 90 turn

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed (per user review of the bypass flow table):
- X1 (cross-drive) now exits when the front US1 has no echo OR reads above
  `CAR_OBST_FRONT_GONE_CM` (new, 30cm) instead of echo-loss only.
- FIND no longer snakes for the line: it drives straight (balanced) for
  `CAR_OBST_FIND_FORWARD` (new, 200 AVG counts, ~2.9cm at 70/cm) then does
  the 4th 90-degree pivot (back to track heading) like a normal corner; if
  the line is seen early it hands straight back to line following (no extra
  pivot). The settle window between T3 and FIND was removed so the action is
  continuous (fixed forward + turn).
- Carries the user's current on-site values: CAR_LINE_KP 0.065,
  CAR_LINE_KD 0.053, CAR_TURN_ENTRY_FORWARD_COUNT 300,
  CAR_LOOP_PERIOD_MS 10, CAR_OBST_TRIGGER_CM 25, CAR_OBST_ENC_FIXED 350.

Build/verification:
- Reviewed (UTF-8 paren/brace 626/626, 286/286); ARMCC build to be
  re-run in Keil.

## 2026-08-24 - TURN: restore entry-straight for normal turns, zero-entry for special ones

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed (per user requirement "fixed entry drive then turn"):
- `CAR_TURN_ENTRY_FORWARD_COUNT` back to 100 (tunable start) for normal
  T/corner turns in mode A and regular mode B tracking; comment documents
  the encoder/cm conversion (2820 counts ~= 40cm => ~70 counts/cm) and
  warns that too-large values (>300) make corners appear missed. Start
  small (~100 = ~1.5cm) and raise until the car reaches the pivot point.
- Mode B "rejoin after no-line segment" turn now uses
  `Car_ModeBStartEncoderTurn` (zero entry, pivot in place) instead of
  `Car_StartSharpTurnEx` (which had an entry drive), per requirement that
  mode B turns after stopping off-line should not drive forward first.
- Obstacle-bypass turns already use zero-entry (`Obst_StartPivot`), no
  change needed.

Build/verification:
- Reviewed (UTF-8 paren/brace 618/618, 287/287); ARMCC build to be
  re-run in Keil.

## 2026-08-24 - DRIVE: TRIM default 0, direction note fixed, applied to line follow

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed:
- Reset `CAR_STRAIGHT_PWM_TRIM` back to 0 (user trial -88 stalled one wheel
  because the trim is a small static bias: +/-2..8 per step, never tens).
- Corrected the trim direction note: trim positive adds to the left wheel
  and subtracts from the right, which steers right; so fill a negative
  value if the car drifts right, positive if it drifts left.
- Trim now also applies to the line-following straight output (same
  left+TRIM/right-TRIM) in `Car_LineFollowStraight`, so the same static
  correction works during line tracking too.
- Restored garbled comment text on `CAR_OBJECT_CONFIRM_SAMPLES` (pasted
  chat text removed); value stays 1 as the user set (if blocks miscount,
  raise back to 3).

Build/verification:
- Reviewed (UTF-8 paren/brace 612/612, 287/287); ARMCC build to be
  re-run in Keil.

## 2026-08-24 - DRIVE: unified straight-drive balance (fixes drifting car)

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed:
- The car drifted to one side in every straight-driving scenario
  (open-loop equal PWM, obstacle-bypass straight segments, Mode B
  off-line drive). Fixes:
  - New `Car_DriveStraightBalance(BasePwm, Kp)`: encoder differential
    balance (left fast -> left wheel slows) plus static trim; reused by
    all open-line straight segments.
  - Obstacle bypass X1/X2/WALL straight drives now use it instead of raw
    `Car_SetSignedPWM(pwm, pwm)`.
  - Mode B no-line straight drive now calls it too.
  - New tunables: `CAR_STRAIGHT_PWM_TRIM` (static correction; +4..+8 if
    the car always drifts right, negative if left) and
    `CAR_STRAIGHT_BALANCE_KP` (0.6, used for obstacle bypass straight
    drives; Mode B keeps `CAR_ENCODER_BALANCE_KP`).
- Line-following balance logic untouched; if straight sections under line
  following still visibly drift after trim tuning, a trim can be added to
  the line-follow output the same way.

Build/verification:
- Reviewed in agent session (UTF-8 paren/brace 611/611, 287/287; no raw
  equal-PWM open-loop drive remains); ARMCC build to be re-run in Keil.

## 2026-08-24 - OBST: Rewrite bypass state machine to user field procedure

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed:
- Rewrote the obstacle(block) bypass to the user's exact field procedure
  (validated with user in Q&A; reverse route D-C right-bypass example):
  - Trigger: front servo-pan US1 (90 deg = ahead) < CAR_OBST_TRIGGER_CM(20)
    for CAR_OBST_CONFIRM_TIMES(2) consecutive front samples (counted in
    Car_UltrasonicTask, consumed by the line-follow monitor hook).
  - Bypass direction by route: forward 0-3 -> turn left, keep right US3;
    reverse 4-7 -> turn right, keep left US2; servo pre-pans the matching
    side during the stop window.
  - Sequence: STOP (0.5s, servo pre-pan) -> TURN1 (90 deg closed loop) ->
    X1 drive until front US1 has no echo -> X2 fixed drive of
    CAR_OBST_ENC_FIXED(400) encoders (OLED AVG units) -> TURN2 (back to
    heading) -> WAIT -> WALL drive while side US goes 0->value->0 (second
    zero stops, then WAIT) -> TURN3 -> WAIT -> FIND (snake until gray
    >= CAR_OBST_LINE_MIN) -> WAIT -> TURN4 (return to track heading) ->
    servo back to front, hand control back to line following.
  - All "settle" pauses share one CAR_OBST_STOP_TICKS window for tuning.
  - No distance PID in the wall segment (pure drive + zero-edge detect);
    direction/servo angles are the main field-calibration knobs
    (CAR_OBST_SERVO_RIGHT/LEFT).
- Removed old HUG/PD REJOIN variant and its macros/variables entirely.

Build/verification:
- Reviewed in agent session (UTF-8 paren/brace 598/598, 284/284; old
  symbol scan clean); ARMCC build to be re-run in Keil uVision.

## 2026-08-24 - OBST: Inline obstacle(block) bypass while line tracking

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed:
- Line tracking now auto-detects a 20x20cm block on the line (front servo
  pan ultrasonic US1, 90 degrees = straight ahead) and bypasses it, then
  returns to the track. Detection is hooked inside `Car_LineFollowStraight`
  (like Mode B) so tracking yields as soon as the front distance drops:
  - Trigger: US1 < CAR_OBST_TRIGGER_CM(25) held
    CAR_OBST_CONFIRM_TICKS(8) frames while mode A running on line.
  - Bypass direction by route: forward routes (mode 0-3) turn left and use
    right ultrasonic (US3); reverse routes (4-7) turn right and use left
    ultrasonic (US2).
  - Sequence: stop while servo pre-pans to the side -> 1st 90 deg pivot
    (closed-loop encoder) -> drive sideways until US1 leaves the block ->
    2nd opposite 90 deg + servo back to front -> wall-follow PD keeping the
    side ultrasonic at CAR_OBST_HOLD_CM(15) -> 3rd 90 deg -> snake to find
    the line (gray >= CAR_OBST_LINE_MIN) -> hand back to line following;
    if not found, a 4th 90 deg tries again, then timeout stop.
  - Car_UltrasonicTask uses Obst_SideChan while bypassing; OLED Track page
    shows OBS<state> + side/front distances during bypass; K1 resets bypass
    and returns servo to front.
- New servo init at boot (Servo_Init + front angle).
- Tuning knobs all in CAR_OBST_* macros; servo side angles
  CAR_OBST_SERVO_RIGHT/LEFT(180/0) need field calibration.

Build/verification:
- Reviewed in agent session (UTF-8 paren/brace balance 593/593, 274/274);
  ARMCC build to be re-run in Keil uVision before field testing.

## 2026-08-24 - WALL: Remove Wall-Following Mode C (prototype rejected on real car)

Branch: `feature/wall-follow-v1`

Files changed:
- `User/main.c`

What changed:
- Completely removed the wall-following mode C prototype (it could not run
  on the real car):
  - Deleted `CAR_WORK_MODE_C`, all `CAR_WALL_*` parameter/state macros,
    the `Wall_*` state variables, `Car_StartPivotTurn()`,
    `Car_ResetWallNav()` and `Car_WallTask()`.
  - `Car_ToggleWorkMode()` back to A/B cycling; `main()` loop and
    `Car_UltrasonicTask()` restored to pre-C logic; OLED Track page back to
    route-mode display.
- Kept the user's base tuning committed separately in `270589c`
  (CAR_BASE_PWM 31, MIN_FORWARD_PWM 15, MODE_B_LINE_ACTIVE_MIN 3,
  OBJECT_MIN/MAX_CM 23/35) untouched.
- Code state now equals the `codex/closed-loop-turn` logic plus those user
  tuning values only.

Build/verification:
- No CAR_WALL/CAR_WORK_MODE_C symbols remain; UTF-8 paren/brace balance
  439/439, 224/224; ARMCC build to be re-run in Keil uVision.

## 2026-08-24 - WALL: Port Wall-Following Mode C (from aqib-m31/Wall-Following-Robot-PID)

Branch: `feature/wall-follow-v1` (based on `codex/closed-loop-turn`)

Files changed:
- `User/main.c`

What changed:
- Added work mode C (wall following, K3 long-press cycles A/B/C).
  Algorithm ported from github.com/aqib-m31/Wall-Following-Robot-PID and
  adapted to this project:
  - Decision priority: front wall > side wall lost > PID following.
  - Fixed wall side (left by default; `CAR_WALL_DEFAULT_SIDE`, mirror by
    flipping that macro); front sensor + active-side sensor are polled by
    `Car_UltrasonicTask` in mode C.
  - Distance PID keeps the OLED side reading at `CAR_WALL_SETPOINT_CM` (15).
  - Front wall triggers a 90° pivot reusing the existing closed-loop
    encoder turn (`Car_StartPivotTurn` + `Car_RunSharpTurn`), with a short
    post-corner cooldown to avoid re-triggering on the new wall.
  - Side wall lost enters REACQUIRE (arc toward the wall); timeout stops
    the car instead of circling forever.
  - New `Car_WallTask()` runs from `main()` when mode C is active.
  - Gray sensors watched during wall following: when
    `Gray_ActiveCount >= CAR_WALL_LINE_EXIT_MIN`, wall mode exits back to
    normal line following.
- OLED Track page shows `MODE:WALL L/R` and side/front distance + state in
  mode C.

Notes / known v1 limitations:
- After a corner the car keeps the same wall side; if the wall ends up on
  the other side it first goes through REACQUIRE (simplified behavior).
- PID gains are starting points only: `CAR_WALL_KP_DIST/KD_DIST/KI_DIST`
  must be tuned on the real car (KP first, KD to kill weaving).

Build/verification:
- Reviewed in agent session (UTF-8 brace/paren balance 257/257, 528/528);
  ARMCC build to be re-run in Keil uVision before field testing.

## 2026-08-24 - H3M: T Branch Forward Entry by OLED Encoder Counts

Files changed:
- `User/main.c`

What changed:
- Created branch `H3M` from the current `DSH` line-tracking work.
- Replaced the T-branch detector with a simpler count-based rule:
  `M+L1+L2+L3` or `M+R1+R2+R3` must reach 3 active sensors, with
  consecutive confirmation before the turn begins.
- Restored a fixed forward-entry stage before the turn. The entry distance is
  measured from the OLED fifth-line left/right cumulative encoder counts, so
  the tuning window matches what you see during OLED testing.
- Kept the fixed differential turn stage and the post-turn cooldown window.
- OLED now shows `C` as the active or last saved forward-entry count and `W`
  as the target window.

Build/verification:
- `User/main.c` compiled successfully with ARMCC to `Objects/H3M_main.o`.

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
