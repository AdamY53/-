# Codex Change Log

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
