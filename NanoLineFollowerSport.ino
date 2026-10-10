/*
  NanoLineFollowerSport -- classic Arduino Nano ATmega328P + TB6612FNG.
  8 DIGITAL voltage-output IR sensors: black HIGH, white LOW.
  Assumed LEFT -> RIGHT: A7 A6 A5 A4 | A3 A2 A1 A0.
  D2 = optional ONE-PRESS calibration; D10 = start/stop.
  Buttons connect to GND. D11 red LED; D12 green LED (active HIGH).

  Serial Monitor, 115200 baud:
    m = independent motor test (lift wheels first)
    g = go; s = stop; c = calibrate; p = sensor/button report
    1 / 2 / 3 = speed profile, selectable while stopped

  Calibration is NOT required to start. Default digital threshold is 512.
  D2 learns voltage thresholds during a 6s manual sweep, then finishes by
  itself. Calibration lasts until reset; no EEPROM settings or extra files.
  See SPEED_GUIDE.md for upload, speed and turning settings.
*/
#include <Arduino.h>
#include <math.h>

// ---------------- SETTINGS YOU MAY NEED TO CHANGE ----------------
const uint8_t SENSOR_PINS[8] = {A7, A6, A5, A4, A3, A2, A1, A0};
const float SENSOR_X_MM[8] = {-42, -30, -18, -6, 6, 18, 30, 42};
const float SENSOR_PITCH_MM = 12.0f; // Your approximate lateral spacing.
const float LINE_WIDTH_MM = 30.0f;
const float SENSOR_TO_AXLE_MM = 100.0f; // Sensors ahead of axle.
const bool LEFT_REVERSED = false;
const bool RIGHT_REVERSED = false;
const float LEFT_SCALE = 1.0f, RIGHT_SCALE = 1.0f;
const bool BUTTONS_ACTIVE_LOW = true; // true: button between pin and GND.
const uint8_t DEFAULT_SPEED = 1;      // Profile 2: sport. 0=previous, 1=sport, 2=max trial.

const uint8_t CAL_PIN = 2, START_PIN = 10, RED_PIN = 11, GREEN_PIN = 12;
const uint8_t AIN1_PIN = 5, AIN2_PIN = 4, PWMA_PIN = 3;
const uint8_t BIN1_PIN = 7, BIN2_PIN = 8, PWMB_PIN = 9, STBY_PIN = 6;
const uint16_t DEFAULT_THRESHOLD = 512;
const uint8_t SENSOR_HYSTERESIS = 15;
const uint8_t MOTOR_TEST_PWM = 150, SEARCH_PWM = 90;
const uint32_t TICK_US = 3000; // Nominal 333 Hz; standard analogRead timing.
const uint16_t CAL_MS = 6000, STOP_BRAKE_MS = 120;
const uint16_t SEARCH_BRAKE_MS = 70, LOST_LIMIT_MS = 450, WIDE_LIMIT_MS = 250;
struct SpeedProfile {
  uint8_t straight, corner, maximum;
  float kp, kd;
  float derivativeFilterSeconds, acceleration, deceleration;
};
// PWM targets, P, D, derivative filter (seconds), acceleration/deceleration (PWM/s).
// Profile 1 matches the previous Fast sketch's DEFAULT profile, including response.
// Profiles 2/3 add corner speed and steering response; verify on your track.
const SpeedProfile SPEEDS[3] = {
  {185, 115, 245, 380.0f, 7.0f, 0.020f, 450.0f, 1800.0f},
  {220, 150, 255, 460.0f, 7.0f, 0.014f, 700.0f, 2600.0f},
  {245, 165, 255, 520.0f, 7.0f, 0.012f, 900.0f, 3000.0f}
};
enum RobotMode : uint8_t { STOPPED, CALIBRATING, FOLLOWING, MOTOR_TEST };
struct Button { uint8_t pin; bool raw, stable; uint32_t changed; };
// Explicit prototype for Arduino's sketch preprocessor.
bool buttonPressed(Button &button, uint32_t now);

RobotMode mode = STOPPED;
Button calibrationButton = {CAL_PIN, false, false, 0};
Button startButton = {START_PIN, false, false, 0};
uint16_t raw[8], threshold[8], lowest[8], highest[8];
bool black[8] = {};
uint8_t blackCount = 0, speedIndex = DEFAULT_SPEED;
float positionMm = 0, basePWM = 0, previousError = 0, derivativeFiltered = 0;
float previousTurn = 0;
int8_t lastSide = 0;
bool fault = false, brakePending = false, tracking = false;
uint32_t lastTickUs = 0, phaseStarted = 0, brakeStarted = 0;
uint32_t lostStarted = 0, wideStarted = 0;

float clampFloat(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

bool buttonDown(uint8_t pin) {
  return digitalRead(pin) == (BUTTONS_ACTIVE_LOW ? LOW : HIGH);
}

bool buttonPressed(Button &button, uint32_t now) {
  bool down = buttonDown(button.pin);
  if (down != button.raw) { button.raw = down; button.changed = now; }
  if (down != button.stable && uint32_t(now - button.changed) >= 25) {
    button.stable = down;
    return down;
  }
  return false;
}

void readSensors() {
  blackCount = 0;
  float weighted = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    // A6 and A7 are ADC-only on classic Nano. Do not digitalRead them.
    raw[i] = analogRead(SENSOR_PINS[i]);
    if (raw[i] >= threshold[i] + SENSOR_HYSTERESIS) black[i] = true;
    else if (raw[i] <= threshold[i] - SENSOR_HYSTERESIS) black[i] = false;
    if (black[i]) { ++blackCount; weighted += SENSOR_X_MM[i]; }
  }
  positionMm = blackCount ? weighted / blackCount : 0;
}

void oneMotor(uint8_t in1, uint8_t in2, uint8_t pwm, int command, bool reversed) {
  command = constrain(command, -255, 255);
  if (reversed) command = -command;
  digitalWrite(in1, command >= 0 ? HIGH : LOW);
  digitalWrite(in2, command <= 0 ? HIGH : LOW);
  analogWrite(pwm, abs(command)); // command=0 gives short brake with STBY HIGH.
}

void drive(float left, float right) {
  brakePending = false;
  digitalWrite(STBY_PIN, HIGH);
  oneMotor(AIN1_PIN, AIN2_PIN, PWMA_PIN,
    int(roundf(clampFloat(left * LEFT_SCALE, -255, 255))), LEFT_REVERSED);
  oneMotor(BIN1_PIN, BIN2_PIN, PWMB_PIN,
    int(roundf(clampFloat(right * RIGHT_SCALE, -255, 255))), RIGHT_REVERSED);
}

void stopRobot(bool error) {
  mode = STOPPED;
  fault = error;
  basePWM = previousTurn = derivativeFiltered = 0;
  tracking = false;
  drive(0, 0);
  brakePending = true;
  brakeStarted = millis();
}

void startFollowing() {
  readSensors();
  if (blackCount == 0 || blackCount == 8) {
    stopRobot(true);
    if (blackCount == 0) Serial.println(F("STOP: all sensors WHITE. Center on black line; p=readings, m=motor test."));
    else Serial.println(F("STOP: all sensors BLACK. Check polarity/wiring; p=readings, m=motor test."));
    return;
  }
  fault = false;
  tracking = true;
  mode = FOLLOWING;
  basePWM = derivativeFiltered = previousTurn = 0;
  previousError = positionMm / SENSOR_TO_AXLE_MM;
  lastSide = fabsf(positionMm) > SENSOR_PITCH_MM * 0.2f ? (positionMm > 0 ? 1 : -1) : 0;
  lostStarted = wideStarted = 0;
  Serial.print(F("RUN: profile ")); Serial.println(speedIndex + 1);
  lastTickUs = micros(); // Set after status printing, avoiding a false overrun.
  drive(0, 0);
}

void startCalibration() {
  stopRobot(false);
  for (uint8_t i = 0; i < 8; ++i) { lowest[i] = 1023; highest[i] = 0; }
  mode = CALIBRATING;
  phaseStarted = millis();
  Serial.println(F("CAL: sweep ALL sensors over black and white for 6 seconds. Motors stopped."));
}

void calibrationTick(uint32_t now) {
  for (uint8_t i = 0; i < 8; ++i) {
    lowest[i] = min(lowest[i], raw[i]);
    highest[i] = max(highest[i], raw[i]);
  }
  if (uint32_t(now - phaseStarted) < CAL_MS) return;
  uint8_t learned = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    if (highest[i] - lowest[i] >= 100) {
      threshold[i] = (highest[i] + lowest[i]) / 2;
      ++learned;
    } // Weak channels retain their previous usable threshold.
    black[i] = false;
  }
  mode = STOPPED;
  Serial.print(F("CAL complete: learned ")); Serial.print(learned);
  Serial.println(F("/8 channels. Center on line, then D10 or g to run."));
  if (learned < 8) Serial.println(F("Some channels did not see both colors; p shows raw values. Starting is still allowed."));
}

void startMotorTest() {
  stopRobot(false);
  Serial.println(F("MOTOR TEST: left 1s, right 1s, both 1s, with pauses. D10 or s stops."));
  mode = MOTOR_TEST;
  phaseStarted = millis();
  lastTickUs = micros();
}

void motorTestTick(uint32_t now) {
  uint32_t elapsed = uint32_t(now - phaseStarted);
  if (elapsed < 200) drive(0, 0);
  else if (elapsed < 1200) drive(MOTOR_TEST_PWM, 0);
  else if (elapsed < 1500) drive(0, 0);
  else if (elapsed < 2500) drive(0, MOTOR_TEST_PWM);
  else if (elapsed < 2800) drive(0, 0);
  else if (elapsed < 3800) drive(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
  else { stopRobot(false); Serial.println(F("Motor test done. Both wheels should have moved FORWARD.")); }
}

void followTick(float dt, uint32_t now) {
  const SpeedProfile &profile = SPEEDS[speedIndex];
  if (blackCount == 0) {
    wideStarted = 0;
    if (tracking) { lostStarted = now; tracking = false; }
    uint32_t lostMs = uint32_t(now - lostStarted);
    basePWM = derivativeFiltered = 0;
    if (lostMs >= LOST_LIMIT_MS || (lastSide == 0 && lostMs >= SEARCH_BRAKE_MS)) {
      stopRobot(true);
      Serial.println(F("STOP: line lost. Reposition; D10 or g to restart."));
    } else if (lostMs < SEARCH_BRAKE_MS) drive(0, 0);
    else drive(lastSide * int(SEARCH_PWM), -lastSide * int(SEARCH_PWM));
    return;
  }
  if (blackCount == 8) {
    if (!wideStarted) wideStarted = now;
    if (uint32_t(now - wideStarted) >= WIDE_LIMIT_MS) {
      stopRobot(true); Serial.println(F("STOP: all 8 sensors stay black. Check track/polarity."));
    } else {
      float turn = clampFloat(previousTurn, -35, 35);
      drive(profile.corner + turn, profile.corner - turn);
    }
    // Reinitialize PD and any future line-loss timer after the wide patch.
    tracking = true;
    derivativeFiltered = 0;
    previousError = 0;
    return;
  }
  bool recovering = !tracking || wideStarted != 0;
  tracking = true;
  lostStarted = wideStarted = 0;
  float error = positionMm / SENSOR_TO_AXLE_MM;
  if (recovering) { previousError = error; derivativeFiltered = 0; }
  float rate = (error - previousError) / dt;
  previousError = error;
  derivativeFiltered += dt / (profile.derivativeFilterSeconds + dt) * (rate - derivativeFiltered);
  if (fabsf(positionMm) > SENSOR_PITCH_MM * 0.2f) lastSide = error > 0 ? 1 : -1;

  float severity = clampFloat(fabsf(positionMm) / LINE_WIDTH_MM +
    0.06f * fabsf(derivativeFiltered), 0, 1);
  float requestedBase = profile.straight - severity * (profile.straight - profile.corner);
  if (blackCount >= 5) requestedBase = profile.corner;
  float gainScale = clampFloat(basePWM / profile.straight, 0.65f, 1);
  float turn = gainScale * (profile.kp * error + profile.kd * derivativeFiltered);
  turn = clampFloat(turn, -profile.maximum, profile.maximum);
  requestedBase = min(requestedBase, max(0.0f, profile.maximum - fabsf(turn)));
  float ramp = requestedBase > basePWM ? profile.acceleration : profile.deceleration;
  basePWM += clampFloat(requestedBase - basePWM, -ramp * dt, ramp * dt);
  previousTurn = turn;
  // Positive error = line RIGHT. Left motor faster makes a RIGHT turn.
  drive(clampFloat(basePWM + turn, 0, profile.maximum),
        clampFloat(basePWM - turn, 0, profile.maximum));
}

void printDiagnostics() {
  readSensors();
  Serial.println(F("LEFT -> RIGHT: A7 A6 A5 A4 | A3 A2 A1 A0 (assumed wiring)"));
  Serial.print(F("Raw: "));
  for (uint8_t i = 0; i < 8; ++i) { Serial.print(raw[i]); Serial.print(' '); }
  Serial.print(F("\nThreshold: "));
  for (uint8_t i = 0; i < 8; ++i) { Serial.print(threshold[i]); Serial.print(' '); }
  Serial.print(F("\nBlack bits: "));
  for (uint8_t i = 0; i < 8; ++i) Serial.print(black[i] ? '1' : '0');
  Serial.print(F("  position mm=")); Serial.print(positionMm, 1);
  Serial.print(F("  sampled width mm~")); Serial.println(blackCount * SENSOR_PITCH_MM, 1);
  Serial.print(F("D2 raw=")); Serial.print(digitalRead(CAL_PIN));
  Serial.print(F("  D10 raw=")); Serial.println(digitalRead(START_PIN));
  Serial.println(F("For GND buttons: released=1, pressed=0. 1 in black bits means BLACK."));
}

void serialCommand() {
  if (!Serial.available()) return;
  char command = Serial.read();
  if (command >= 'A' && command <= 'Z') command += 'a' - 'A';
  if (command == 's') { stopRobot(false); Serial.println(F("Stopped.")); }
  else if (command == 'g') startFollowing();
  else if (command == 'c') startCalibration();
  else if (command == 'm') startMotorTest();
  else if (command == 'p' || command == 'b') {
    if (mode == STOPPED) printDiagnostics();
    else Serial.println(F("Send s before requesting diagnostics."));
  } else if (command >= '1' && command <= '3') {
    if (mode == STOPPED) {
      speedIndex = command - '1';
      Serial.print(F("Speed profile ")); Serial.println(speedIndex + 1);
    } else Serial.println(F("Stop first to change speed."));
  }
}

void showLEDs(uint32_t now) {
  bool red = false, green = false;
  if (mode == CALIBRATING) { red = (now / 150) % 2; green = !red; }
  else if (mode == MOTOR_TEST) red = green = (now / 120) % 2;
  else if (mode == FOLLOWING) { green = true; red = blackCount == 0 || blackCount == 8; }
  else red = fault ? (now / 150) % 2 : true;
  digitalWrite(RED_PIN, red ? HIGH : LOW);
  digitalWrite(GREEN_PIN, green ? HIGH : LOW);
}

void setup() {
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);
  const uint8_t outputs[] = {AIN1_PIN, AIN2_PIN, PWMA_PIN, BIN1_PIN, BIN2_PIN,
                             PWMB_PIN, RED_PIN, GREEN_PIN};
  for (uint8_t i = 0; i < sizeof(outputs); ++i) {
    pinMode(outputs[i], OUTPUT); digitalWrite(outputs[i], LOW);
  }
  pinMode(CAL_PIN, BUTTONS_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  pinMode(START_PIN, BUTTONS_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  calibrationButton.raw = calibrationButton.stable = buttonDown(CAL_PIN);
  startButton.raw = startButton.stable = buttonDown(START_PIN);
  for (uint8_t i = 0; i < 8; ++i) threshold[i] = DEFAULT_THRESHOLD;
  Serial.begin(115200);
  Serial.println(F("NanoLineFollowerSport. Calibration OPTIONAL. D2=calibrate D10=start/stop."));
  Serial.println(F("Lift wheels; m=motor test. p=readings, g=go, s=stop, c=calibrate, 1/2/3=speed."));
  lastTickUs = micros();
}

void loop() {
  uint32_t now = millis();
  bool calPress = buttonPressed(calibrationButton, now);
  bool runPress = buttonPressed(startButton, now);
  if (calPress && runPress) stopRobot(false);
  else if (runPress) {
    if (mode == STOPPED) startFollowing();
    else { stopRobot(false); Serial.println(F("Stopped by D10.")); }
  } else if (calPress) startCalibration();

  serialCommand();
  now = millis(); // Refresh after handlers; prevents unsigned timestamp wrap.
  if (brakePending && uint32_t(now - brakeStarted) >= STOP_BRAKE_MS) {
    digitalWrite(STBY_PIN, LOW);
    brakePending = false;
  }
  uint32_t currentUs = micros();
  uint32_t elapsedUs = uint32_t(currentUs - lastTickUs);
  if (elapsedUs >= TICK_US) {
    lastTickUs = currentUs;
    if ((mode == FOLLOWING || mode == MOTOR_TEST) && elapsedUs > 30000UL) {
      stopRobot(true); Serial.println(F("STOP: control timing overrun."));
    } else if (mode == MOTOR_TEST) motorTestTick(now);
    else if (mode == FOLLOWING || mode == CALIBRATING) {
      readSensors();
      if (mode == FOLLOWING) followTick(elapsedUs * 0.000001f, now);
      else calibrationTick(now);
    }
  }
  showLEDs(now);
}

