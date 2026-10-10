# Faster straights and quicker turns

This version uses the same wiring and controller as NanoLineFollowerFast, with
higher speed targets and more responsive steering. The default is profile 2.

| Serial command | Straight target | Corner target | Maximum wheel PWM |
|---|---:|---:|---:|
| `1` — previous Fast default | 185 | 115 | 245 |
| `2` — Sport, new default | 220 | 150 | 255 |
| `3` — maximum trial | 245 | 165 | 255 |

PWM ranges from 0 to 255; it is not a measured speed. Corner values are targets,
not minimum speeds. Strong steering can reduce forward speed further to retain
steering authority. Profile 3 needs a track test; it is not necessarily fastest
around a tight track.

## Upload and run

1. Extract the ZIP and open `NanoLineFollowerSport.ino` inside its matching folder.
2. Copy across any local changes you made to `LEFT_REVERSED`, `RIGHT_REVERSED`,
   `SENSOR_PINS`, sensor weights, motor scales or button polarity in your working
   sketch. This version starts with the previously supplied wiring settings.
3. Use the same Nano board, processor/bootloader and port that worked before.
   Click Verify, then Upload.
4. Optional: press D2 once and sweep all sensors over black and white for six
   seconds. Calibration ends automatically and is lost on reset.
5. Center the sensor row on the line and press D10 to run profile 2. D10 stops it.
6. To select another profile, stop first, open Serial Monitor at **115200 baud**,
   send `1`, `2` or `3`, and then press D10. Reset restores profile 2.

Run profile 2 through the entire track before trying 3. If it zigzags or misses
corners, select profile 1. Profile 1 reproduces the previous Fast sketch's
default settings, not its original slow profile. The previous sketch remains
available separately.

## What makes the turns quicker

Profile 2 raises proportional steering gain from 380 to 460 and reduces the
derivative filter from 20 ms to 14 ms. This gives stronger correction and less
filter delay. Derivative gain stays at 7. Acceleration increases from 450 to
700 PWM/s, and deceleration from 1800 to 2600 PWM/s, so the base motor command
responds sooner entering and leaving curves. These are command ramps; actual
acceleration depends on your motors, battery, grip and robot mass.

Profile 3 uses gain 520, a 12 ms filter, and 900/3000 PWM/s ramps. It is more
aggressive and may oscillate with the coarse HIGH/LOW sensor readings.

Normal following still uses forward wheel commands: the inside wheel can stop
on a sharp turn. Lost-line recovery retains its existing reverse pivot. The
motor mixer, recovery timing, startup, buttons, calibration and motor test are
unchanged. A right-side line reading still speeds the left wheel for a right turn.

## Fine tuning profile 2

In the `SPEEDS` table, each row is:

```cpp
// straight, corner, maximum, kp, kd, filter seconds, accel PWM/s, decel PWM/s
{220, 150, 255, 460.0f, 7.0f, 0.014f, 700.0f, 2600.0f}
```

- If it wiggles on straights, try `kp = 420.0f` first. Change one setting at a time.
- If it still wiggles, restore the filter to `0.020f` and compare again.
- If it runs wide or loses the line in a corner, reduce corner target from 150
  to 135 before trying more speed. If that does not help, return to profile 1.

Exact turning speed cannot be guaranteed without testing on your track.

## Controls and connections

- D2 or Serial `c`: optional calibration, motors stopped.
- D10: start/stop. Serial `g` starts and `s` stops.
- Serial `m`: independent motor test; lift the wheels first.
- Serial `p`: sensor and button readings while stopped.
- Red steady: stopped. Green: following. Red flashing: fault stop.

Left motor: AIN1=D5, AIN2=D4, PWMA=D3. Right: BIN1=D7, BIN2=D8, PWMB=D9.
STBY=D6. Buttons D2/D10 to GND. LEDs D11 red and D12 green, active HIGH.
Sensors are assumed A7 through A0 from left to right, black HIGH/white LOW,
center between A4/A3, negative on the left. All eight use analogRead so the
Nano's ADC-only A6/A7 work with voltage-output sensors. Geometry remains
30 mm line, approximately 12 mm sensor pitch and 100 mm sensor-to-axle distance.

Validation: AVR-target syntax/type checks with Arduino API stubs passed. A full
Arduino compile/link and physical track performance were not verified here.
