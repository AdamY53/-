# Codex Change Log

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
