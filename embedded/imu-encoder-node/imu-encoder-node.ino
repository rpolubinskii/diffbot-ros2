#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>

// D1/D2/D5/D6 are already used.
constexpr uint8_t I2C_SDA_PIN = D7;
constexpr uint8_t I2C_SCL_PIN = D4;

// AD0 low selects 0x68.
constexpr bool kImuAd0High = false;
constexpr uint32_t kPublishPeriodMs = 10;
constexpr uint32_t kMinEncoderPulseUs = 200;
constexpr uint16_t kGyroCalibrationSamples = 800;
constexpr uint32_t kGyroCalibrationWarmupMs = 3000;
constexpr float kMilliGToG = 1.0f / 1000.0f;

struct ImuSample {
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
  float mx_ut;
  float my_ut;
  float mz_ut;
};

struct GyroBias {
  float x_dps;
  float y_dps;
  float z_dps;
};

ICM_20948_I2C imu;
static GyroBias gyroBias = {0.0f, 0.0f, 0.0f};
static bool imuConfigured = false;

// ESP8266 GPIO numbers.
const byte encoderLPinA = 5;
const byte encoderLPinB = 4;
const byte encoderRPinA = 14;
const byte encoderRPinB = 12;

volatile int pulseL = 0;
volatile int pulseR = 0;
volatile uint32_t lastL = 0;
volatile uint32_t lastR = 0;

bool imuInit() {
  imu.begin(Wire, kImuAd0High);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.print("IMU init failed: ");
    Serial.println(imu.statusString());
    return false;
  }

  return true;
}

bool readImu(ImuSample &sample) {
  imu.getAGMT();
  if (imu.status != ICM_20948_Stat_Ok) {
    return false;
  }

  sample.ax_g = imu.accX() * kMilliGToG;
  sample.ay_g = imu.accY() * kMilliGToG;
  sample.az_g = imu.accZ() * kMilliGToG;
  sample.gx_dps = imu.gyrX() - gyroBias.x_dps;
  sample.gy_dps = imu.gyrY() - gyroBias.y_dps;
  sample.gz_dps = imu.gyrZ() - gyroBias.z_dps;
  sample.mx_ut = imu.magX();
  sample.my_ut = imu.magY();
  sample.mz_ut = imu.magZ();
  return true;
}

bool calibrateGyro(uint16_t samples) {
  float sx = 0.0f;
  float sy = 0.0f;
  float sz = 0.0f;
  uint16_t collected = 0;
  uint32_t attempts = 0;

  while (collected < samples && attempts < samples * 20U) {
    ++attempts;
    if (!imu.dataReady()) {
      delay(5);
      continue;
    }

    imu.getAGMT();
    if (imu.status != ICM_20948_Stat_Ok) {
      return false;
    }

    sx += imu.gyrX();
    sy += imu.gyrY();
    sz += imu.gyrZ();
    ++collected;
    delay(5);
  }

  if (collected == 0) {
    return false;
  }

  gyroBias.x_dps = sx / static_cast<float>(collected);
  gyroBias.y_dps = sy / static_cast<float>(collected);
  gyroBias.z_dps = sz / static_cast<float>(collected);
  return collected == samples;
}

void IRAM_ATTR lISR() {
  const uint32_t now = micros();
  if (now - lastL < kMinEncoderPulseUs) {
    return;
  }
  lastL = now;

  if (digitalRead(encoderLPinB)) {
    pulseL--;
  } else {
    pulseL++;
  }
}

void IRAM_ATTR rISR() {
  const uint32_t now = micros();
  if (now - lastR < kMinEncoderPulseUs) {
    return;
  }
  lastR = now;

  if (digitalRead(encoderRPinB)) {
    pulseR++;
  } else {
    pulseR--;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  pinMode(encoderLPinB, INPUT_PULLUP);
  pinMode(encoderRPinB, INPUT_PULLUP);
  pinMode(encoderLPinA, INPUT_PULLUP);
  pinMode(encoderRPinA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLPinA), lISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRPinA), rISR, RISING);

  Serial.println("ICM-20948 IMU init...");
  imuConfigured = imuInit();
  if (!imuConfigured) {
    Serial.println("IMU init failed. Check wiring/address.");
    return;
  }

  Serial.println("IMU ready.");
  Serial.print("Waiting ");
  Serial.print(kGyroCalibrationWarmupMs);
  Serial.println(" ms before gyro calibration...");
  delay(kGyroCalibrationWarmupMs);
  Serial.println("Calibrating gyro... keep the board still.");
  if (!calibrateGyro(kGyroCalibrationSamples)) {
    Serial.println("Gyro calibration failed.");
    imuConfigured = false;
    return;
  }

  Serial.println("Gyro calibration done.");
  Serial.print("Gyro bias dps: ");
  Serial.print(gyroBias.x_dps, 4);
  Serial.print(", ");
  Serial.print(gyroBias.y_dps, 4);
  Serial.print(", ");
  Serial.println(gyroBias.z_dps, 4);
  Serial.println("Magnetometer ready via SparkFun library.");
}

void loop() {
  static uint32_t lastPublish = 0;
  const uint32_t now = millis();
  if (now - lastPublish < kPublishPeriodMs) {
    return;
  }
  lastPublish = now;

  int l = 0;
  int r = 0;
  noInterrupts();
  l = pulseL;
  r = pulseR;
  pulseL = 0;
  pulseR = 0;
  interrupts();

  if (!imuConfigured) {
    Serial.print(l);
    Serial.print(',');
    Serial.println(r);
    return;
  }

  ImuSample s{};
  if (!readImu(s)) {
    Serial.println("ERR_IMU");
    return;
  }

  // left_ticks,right_ticks,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT
  Serial.print(l);
  Serial.print(',');
  Serial.print(r);
  Serial.print(',');
  Serial.print(s.ax_g, 3);
  Serial.print(',');
  Serial.print(s.ay_g, 3);
  Serial.print(',');
  Serial.print(s.az_g, 3);
  Serial.print(',');
  Serial.print(s.gx_dps, 2);
  Serial.print(',');
  Serial.print(s.gy_dps, 2);
  Serial.print(',');
  Serial.print(s.gz_dps, 2);
  Serial.print(',');
  Serial.print(s.mx_ut, 2);
  Serial.print(',');
  Serial.print(s.my_ut, 2);
  Serial.print(',');
  Serial.println(s.mz_ut, 2);
}
