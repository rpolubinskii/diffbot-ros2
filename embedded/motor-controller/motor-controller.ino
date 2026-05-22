#define PWMA  D5   // GPIO14
#define AIN1  D1   // GPIO5
#define AIN2  D2   // GPIO4

#define PWMB  D6   // GPIO12
#define BIN1  D7   // GPIO13
#define BIN2  D0   // GPIO16

#define STBY  D8   // GPIO15 (must be LOW at boot!)

const int pwmFreq = 1000;    // ESP8266 PWM frequency (Hz)
const int pwmRange = 1023;   // ESP8266 PWM range (0-1023)
const int serialBaud = 115200;
const unsigned long commandTimeoutMs = 300;
const unsigned long stbyRecoveryPulseMs = 20;
const int pwmMaxAbs = 1023;

unsigned long lastValidCommandMs = 0;
bool timedOut = false;
bool stbyRecoveryPending = false;

void stopAllMotors();
void setStandby(bool enabled);
void maybeHandleTimeout();
void handleSerialInput();
bool parseCommand(const char* line, int &speedL, int &speedR);
int clampPwm(int speed);
void applyMotorCommands(int speedL, int speedR);

void setup() {
  // Set all pins as outputs
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  analogWriteFreq(pwmFreq);
  analogWriteRange(pwmRange);

  setStandby(true);
  stopAllMotors();

  Serial.begin(serialBaud);  // USB communication
  Serial.setTimeout(10);     // Keep parser responsive
  lastValidCommandMs = millis();
}

void loop() {
  handleSerialInput();
  maybeHandleTimeout();
}

void setStandby(bool enabled) {
  // TB6612: STBY HIGH = active, LOW = standby
  digitalWrite(STBY, enabled ? HIGH : LOW);
}

void stopAllMotors() {
  motorA(0);
  motorB(0);
}

void maybeHandleTimeout() {
  const unsigned long now = millis();
  if (!timedOut && (now - lastValidCommandMs > commandTimeoutMs)) {
    stopAllMotors();
    timedOut = true;
    stbyRecoveryPending = true;
  }
}

void handleSerialInput() {
  static char line[32];
  static size_t idx = 0;

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      line[idx] = '\0';
      idx = 0;

      int speedL = 0;
      int speedR = 0;
      if (!parseCommand(line, speedL, speedR)) {
        continue;
      }

      if (stbyRecoveryPending) {
        setStandby(false);
        delay(stbyRecoveryPulseMs);
        setStandby(true);
        stbyRecoveryPending = false;
      }

      applyMotorCommands(speedL, speedR);
      lastValidCommandMs = millis();
      timedOut = false;
      continue;
    }

    if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    } else {
      // Overflow: drop this line and wait for next newline.
      idx = 0;
    }
  }
}

bool parseCommand(const char* line, int &speedL, int &speedR) {
  return sscanf(line, "%d,%d", &speedL, &speedR) == 2;
}

int clampPwm(int speed) {
  if (speed > pwmMaxAbs) return pwmMaxAbs;
  if (speed < -pwmMaxAbs) return -pwmMaxAbs;
  return speed;
}

void applyMotorCommands(int speedL, int speedR) {
  motorA(clampPwm(speedL));
  motorB(clampPwm(speedR));
}


void motorA(int speed) {
  // Control motor A direction and speed
  if (speed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(speed));
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

void motorB(int speed) {
  // Control motor A direction and speed
  if (speed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(speed));
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}
