
#include <Wire.h>
#include <VL53L0X.h>
#include <DHT.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// Configurare hardware
// ---------------------------------------------------------------------------
namespace Pins {
  // Driver dreapta
  constexpr uint8_t STBY_R = 36;
  constexpr uint8_t PWMA_R = 4;
  constexpr uint8_t AIN1_R = 35;
  constexpr uint8_t AIN2_R = 34;
  constexpr uint8_t PWMB_R = 5;
  constexpr uint8_t BIN1_R = 37;
  constexpr uint8_t BIN2_R = 38;

  // Driver stanga
  constexpr uint8_t STBY_L = 41;
  constexpr uint8_t PWMA_L = 6;
  constexpr uint8_t AIN1_L = 40;
  constexpr uint8_t AIN2_L = 39;
  constexpr uint8_t PWMB_L = 7;
  constexpr uint8_t BIN1_L = 42;
  constexpr uint8_t BIN2_L = 43;

  constexpr uint8_t XSHUT[8] = {22, 23, 24, 25, 26, 27, 28, 29};

  // Senzori de mediu
  constexpr uint8_t DHT22_DATA = 44;
  constexpr uint8_t MQ135_DIGITAL = 45;
  constexpr uint8_t MQ7_DIGITAL = 46;
  constexpr uint8_t MQ8_DIGITAL = 47;
  constexpr uint8_t GP2Y_LED = 48; 
  constexpr uint8_t MQ135_ANALOG = A1;
  constexpr uint8_t MQ7_ANALOG = A2;
  constexpr uint8_t MQ8_ANALOG = A3;
  constexpr uint8_t GP2Y_ANALOG = A4;

  // Urmarire solara: LDR-uri simple, fiecare cu divizor de 10 kOhm spre GND.
  constexpr uint8_t LDR_TOP_LEFT = A8;
  constexpr uint8_t LDR_TOP_RIGHT = A7;
  constexpr uint8_t LDR_BOTTOM_LEFT = A6;
  constexpr uint8_t LDR_BOTTOM_RIGHT = A5;
  constexpr uint8_t SERVO_TILT = 8;
  constexpr uint8_t SERVO_ROTATION = 9;
}

namespace Config {
  constexpr uint32_t SERIAL_BAUD = 115200;
  constexpr uint32_t ESP_SERIAL_BAUD = 38400;
  constexpr uint8_t MPU_ADDR = 0x68;
  constexpr uint8_t TOF_FIRST_ADDR = 0x30;

  constexpr uint16_t TOF_TIMING_BUDGET_US = 20000;
  constexpr uint16_t TOF_PERIOD_MS = 25;
  constexpr uint16_t TOF_TIMEOUT_MS = 45;
  constexpr uint16_t SENSOR_STALE_MS = 350;
  constexpr uint16_t SENSOR_MIN_MM = 30;
  constexpr int16_t SLOW_MM = 450;
  constexpr int16_t CRITICAL_MM = 200;
  constexpr int16_t RELEASE_MM = 520;
  constexpr int16_t SIDE_SAFE_MM = 260;

  constexpr int16_t CRUISE_PWM = 145;
  constexpr int16_t SLOW_PWM = 85;
  constexpr int16_t TURN_PWM = 135;
  constexpr int16_t MAX_PWM = 200;
  constexpr uint8_t PWM_STEP = 10;
  constexpr uint16_t MOTOR_UPDATE_MS = 10;

  constexpr float GYRO_SCALE_250DPS = 1.0f / 131.0f;
  constexpr uint16_t GYRO_PERIOD_US = 10000;
  constexpr float KP = 2.65f;
  constexpr float KD = 0.18f;
  constexpr float MAX_HEADING_CORRECTION = 70.0f;

  constexpr float TURN_STEP_DEG = 45.0f;
  constexpr uint16_t BRAKE_MS = 180;
  constexpr uint16_t CORNER_SCAN_MS = 650;
  constexpr uint16_t TURN_COMMIT_MS = 1800;
  constexpr int16_t TURN_COMMIT_PWM = 95;
  constexpr uint16_t CRUISE_RAMP_MS = 800;
  constexpr int16_t CRUISE_START_PWM = 105;
  constexpr uint16_t TURN_TIMEOUT_MS = 3200;
  constexpr uint16_t REVERSE_MS = 550;
  constexpr int16_t REVERSE_PWM = 90;
  constexpr int16_t REAR_CRITICAL_MM = 220;
  constexpr uint16_t FAILED_PATH_WINDOW_MS = 8000;
  constexpr uint16_t DIRECTION_MEMORY_MS = 25000;
  constexpr uint16_t START_DELAY_MS = 2000;
  constexpr uint16_t TELEMETRY_MS = 250;

  constexpr uint8_t MS5611_ADDR = 0x77;
  constexpr uint32_t MQ_SAMPLE_MS = 250;
  constexpr uint32_t DHT_SAMPLE_MS = 2200;
  constexpr uint32_t DUST_SAMPLE_MS = 500;
  constexpr uint32_t PRESSURE_SAMPLE_MS = 1000;
  constexpr uint32_t DATA_SEND_MS = 2000;
  constexpr float ADC_VOLTAGE = 5.0f;
  constexpr float GP2Y_ZERO_VOLTAGE = 0.90f;
  constexpr float GP2Y_V_PER_UG_M3 = 0.005f;

  constexpr int16_t MANUAL_PWM = 120;
  constexpr int16_t MANUAL_TURN_PWM = 110;
  constexpr uint32_t MANUAL_WATCHDOG_MS = 650;

  // OLED SSD1306 128x64. 
  constexpr uint8_t OLED_ADDR = 0x3C;
  constexpr uint16_t OLED_FRAME_MS = 240;
  constexpr uint16_t MOOD_CONFIRM_MS = 5000;
  constexpr uint16_t AIR_GOOD_MAX = 350;
  constexpr uint16_t AIR_MODERATE_MAX = 700;

  // Urmarire solara .
  constexpr uint16_t SOLAR_SAMPLE_MS = 350;
  constexpr uint16_t SOLAR_DARK_LEVEL = 110;
  constexpr int16_t SOLAR_DEADBAND = 38;
  constexpr int16_t SOLAR_MIN_IMPROVEMENT = 8;
  constexpr uint8_t SOLAR_MAX_NO_IMPROVEMENT = 7;
  constexpr uint32_t SOLAR_RETRY_COOLDOWN_MS = 30000;

  // Servo inclinare MG90S normal: centru 90, maximum +/-40 grade.
  constexpr uint8_t TILT_CENTER_DEG = 90;
  constexpr uint8_t TILT_MIN_DEG = 50;
  constexpr uint8_t TILT_MAX_DEG = 130;
  constexpr uint8_t TILT_STEP_DEG = 2;
  constexpr uint16_t TILT_SETTLE_MS = 320;
  constexpr bool TILT_REVERSED = false;

  // Servo rotatie continua. Ajustati ROTATION_STOP_US daca se misca la repaus.
  constexpr uint16_t ROTATION_STOP_US = 1500;
  constexpr uint16_t ROTATION_LEFT_US = 1450;
  constexpr uint16_t ROTATION_RIGHT_US = 1550;
  constexpr uint16_t ROTATION_BURST_MS = 110;
  constexpr uint16_t ROTATION_SETTLE_MS = 260;
  constexpr int32_t ROTATION_MAX_BALANCE_MS = 4500;
}


enum SensorId : uint8_t {
  FRONT_LEFT = 0,
  FRONT_CENTER,
  FRONT_RIGHT,
  LEFT_SIDE,
  RIGHT_SIDE,
  REAR_LEFT,
  REAR_CENTER,
  REAR_RIGHT,
  SENSOR_COUNT
};

// Offseturile din autov1, in milimetri.
constexpr int16_t TOF_OFFSET[SENSOR_COUNT] = {-51, -47, -28, -14, -50, -27, -30, -50};

// Limita maxima utila pentru fiecare pozitie. O citire mai mare este marcata
// drept spatiu liber (8000 mm), fiind cel mai probabil reflexia podelei.
// Valorile pot fi calibrate separat dupa telemetria robotului real.
constexpr int16_t TOF_FLOOR_CUTOFF_MM[SENSOR_COUNT] = {
  420, // fata-stanga
  430, // fata-centru
  420, // fata-dreapta
  430, // lateral-stanga
  430, // lateral-dreapta
  440, // spate-stanga
  450, // spate-centru
  440  // spate-dreapta
};
constexpr uint8_t SCAN_PATTERN[] = {
  FRONT_CENTER, FRONT_LEFT, FRONT_CENTER, FRONT_RIGHT,
  FRONT_CENTER, LEFT_SIDE, RIGHT_SIDE, FRONT_CENTER,
  REAR_LEFT, REAR_CENTER, REAR_RIGHT
};
constexpr uint8_t SCAN_COUNT = sizeof(SCAN_PATTERN) / sizeof(SCAN_PATTERN[0]);

VL53L0X tof[SENSOR_COUNT];

struct RangeData {
  int16_t mm;
  uint32_t updatedAt;
  bool valid;
  uint8_t failures;
};
RangeData rangeData[SENSOR_COUNT];
int16_t tofHistory[SENSOR_COUNT][3];
uint8_t tofHistoryIndex[SENSOR_COUNT] = {0};
uint8_t tofHistoryCount[SENSOR_COUNT] = {0};

struct GyroData {
  float offsetRaw;
  float yawDeg;
  float rateDps;
  uint32_t lastUs;
  bool healthy;
} gyro = {0.0f, 0.0f, 0.0f, 0, false};

DHT dht(Pins::DHT22_DATA, DHT22);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
Servo tiltServo;
Servo rotationServo;
bool oledReady = false;

struct AirData {
  float temperature = NAN;
  float humidity = NAN;
  float pressureHpa = NAN;
  float dustUgM3 = 0.0f;
  float mq135Filtered = 0.0f;
  float mq7Filtered = 0.0f;
  float mq8Filtered = 0.0f;
  uint16_t mq135Raw = 0;
  uint16_t mq7Raw = 0;
  uint16_t mq8Raw = 0;
  bool dhtValid = false;
  bool pressureValid = false;
} air;

enum class AirMood : uint8_t { HAPPY, NEUTRAL, SAD };
AirMood airMood = AirMood::NEUTRAL;
AirMood pendingAirMood = AirMood::NEUTRAL;
uint32_t pendingMoodSince = 0;
uint32_t lastOledFrame = 0;

struct SolarData {
  float topLeft = 0.0f;
  float topRight = 0.0f;
  float bottomLeft = 0.0f;
  float bottomRight = 0.0f;
  bool initialized = false;
  int16_t horizontalError = 0;
  int16_t verticalError = 0;
  uint8_t tiltAngle = Config::TILT_CENTER_DEG;
  int8_t rotationDirection = 0;
  int8_t lastRotationDirection = 0;
  int16_t previousHorizontalAbs = 32767;
  uint8_t noImprovementCount = 0;
  int32_t rotationBalanceMs = 0;
  uint32_t motionStartedAt = 0;
  uint32_t lastSampleAt = 0;
  uint32_t cooldownUntil = 0;
  bool rotationActive = false;
  bool tiltActive = false;
} solar;

uint16_t msProm[7] = {0};
uint32_t msD1 = 0;
uint32_t msD2 = 0;
uint32_t msConversionStarted = 0;
uint32_t lastMsSample = 0;
enum class MsState : uint8_t { IDLE, WAIT_PRESSURE, WAIT_TEMPERATURE };
MsState msState = MsState::IDLE;

enum class ControlMode : uint8_t { AUTO, MANUAL };
enum class ManualCommand : uint8_t { STOP, FORWARD, BACKWARD, LEFT, RIGHT };
// Pornire sigura: robotul ramane oprit pana la o comanda explicita de pe site.
ControlMode controlMode = ControlMode::MANUAL;
ManualCommand manualCommand = ManualCommand::STOP;
uint32_t lastManualCommandAt = 0;
char espCommandBuffer[40];
uint8_t espCommandLength = 0;
uint32_t lastMqSample = 0;
uint32_t lastDhtSample = 0;
uint32_t lastDustSample = 0;
uint32_t lastDataSend = 0;
uint32_t dataSequence = 0;

enum class RobotState : uint8_t {
  STARTUP,
  CRUISE,
  AVOID,
  BRAKE,
  SCAN_CORNER,
  REVERSE,
  TURN_LEFT,
  TURN_RIGHT,
  ESCAPE,
  FAULT
};

RobotState state = RobotState::STARTUP;
RobotState stateAfterBrake = RobotState::CRUISE;
uint32_t stateSince = 0;
float headingReference = 0.0f;
float turnStartYaw = 0.0f;
float turnGoalDeg = Config::TURN_STEP_DEG;
int16_t requestedLeft = 0;
int16_t requestedRight = 0;
int16_t appliedLeft = 0;
int16_t appliedRight = 0;
uint8_t scanIndex = 0;
uint32_t lastMotorUpdate = 0;
uint32_t lastTelemetry = 0;
bool systemReady = false;
int8_t lastTurnDirection = 0; // +1 stanga, -1 dreapta
uint32_t lastTurnFinishedAt = 0;
uint32_t avoidLeftUntil = 0;
uint32_t avoidRightUntil = 0;
bool lastTurnFailureRegistered = true;
int8_t plannedEscapeDirection = 1;
uint8_t reverseAttempts = 0;
uint32_t directionCommitUntil = 0;
float pdPreviousError = 0.0f;
uint32_t pdPreviousUs = 0;

// ---------------------------------------------------------------------------
// Utilitare
// ---------------------------------------------------------------------------
int16_t clampPwm(int16_t value) {
  return constrain(value, -Config::MAX_PWM, Config::MAX_PWM);
}

int16_t approach(int16_t current, int16_t target, uint8_t step) {
  if (current < target) {
    int16_t next = current + step;
    return (next < target) ? next : target;
  }
  if (current > target) {
    int16_t next = current - step;
    return (next > target) ? next : target;
  }
  return current;
}

bool elapsedMs(uint32_t since, uint32_t interval) {
  return (uint32_t)(millis() - since) >= interval;
}

bool timeStillActive(uint32_t deadline) {
  return deadline != 0 && (int32_t)(deadline - millis()) > 0;
}

const __FlashStringHelper* stateName(RobotState value) {
  switch (value) {
    case RobotState::STARTUP:    return F("STARTUP");
    case RobotState::CRUISE:     return F("CRUISE");
    case RobotState::AVOID:      return F("AVOID");
    case RobotState::BRAKE:      return F("BRAKE");
    case RobotState::SCAN_CORNER:return F("SCAN_CORNER");
    case RobotState::REVERSE:    return F("REVERSE");
    case RobotState::TURN_LEFT:  return F("TURN_LEFT");
    case RobotState::TURN_RIGHT: return F("TURN_RIGHT");
    case RobotState::ESCAPE:     return F("ESCAPE");
    default:                     return F("FAULT");
  }
}

void enterState(RobotState next) {
  if (state == next) return;
  state = next;
  stateSince = millis();

  if (next == RobotState::CRUISE) {
    // Noua directie curenta devine reper; robotul nu revine la un 0 vechi.
    headingReference = gyro.yawDeg;
    reverseAttempts = 0;
    pdPreviousError = 0.0f;
    pdPreviousUs = micros();
  }
  if (next == RobotState::TURN_LEFT || next == RobotState::TURN_RIGHT || next == RobotState::ESCAPE) {
    turnStartYaw = gyro.yawDeg;
  }
}

int16_t distanceMm(SensorId id) {
  const RangeData &r = rangeData[id];
  if (!r.valid || elapsedMs(r.updatedAt, Config::SENSOR_STALE_MS)) return 8000;
  return r.mm;
}

bool frontDataFresh() {
  const SensorId ids[] = {FRONT_LEFT, FRONT_CENTER, FRONT_RIGHT};
  for (uint8_t i = 0; i < 3; ++i) {
    if (!rangeData[ids[i]].valid || elapsedMs(rangeData[ids[i]].updatedAt, Config::SENSOR_STALE_MS)) {
      return false;
    }
  }
  return true;
}

bool rearDataFresh() {
  const SensorId ids[] = {REAR_LEFT, REAR_CENTER, REAR_RIGHT};
  for (uint8_t i = 0; i < 3; ++i) {
    if (!rangeData[ids[i]].valid || elapsedMs(rangeData[ids[i]].updatedAt,
                                               Config::SENSOR_STALE_MS)) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Motoare
// ---------------------------------------------------------------------------
void writeChannel(uint8_t in1, uint8_t in2, uint8_t pwmPin, int16_t speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); 
  }
  analogWrite(pwmPin, abs(speed));
}

void applyMotors(int16_t left, int16_t right) {
  // Motoarele din dreapta sunt montate in oglinda fata de cele din stanga.
  writeChannel(Pins::AIN1_L, Pins::AIN2_L, Pins::PWMA_L, left, true);
  writeChannel(Pins::BIN1_L, Pins::BIN2_L, Pins::PWMB_L, left, true);
  writeChannel(Pins::AIN1_R, Pins::AIN2_R, Pins::PWMA_R, right, false);
  writeChannel(Pins::BIN1_R, Pins::BIN2_R, Pins::PWMB_R, right, false);
}

void requestMotors(int16_t left, int16_t right) {
  requestedLeft = clampPwm(left);
  requestedRight = clampPwm(right);
}

void updateMotors() {
  if (!elapsedMs(lastMotorUpdate, Config::MOTOR_UPDATE_MS)) return;
  lastMotorUpdate = millis();
  appliedLeft = approach(appliedLeft, requestedLeft, Config::PWM_STEP);
  appliedRight = approach(appliedRight, requestedRight, Config::PWM_STEP);
  applyMotors(appliedLeft, appliedRight);
}

void emergencyStop() {
  requestedLeft = requestedRight = 0;
  appliedLeft = appliedRight = 0;
  applyMotors(0, 0);
}

// ---------------------------------------------------------------------------
// MPU6050
// ---------------------------------------------------------------------------
bool readGyroZ(int16_t &raw) {
  Wire.beginTransmission(Config::MPU_ADDR);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(Config::MPU_ADDR, (uint8_t)2, (uint8_t)true) != 2) return false;
  raw = (int16_t)((uint16_t)Wire.read() << 8 | Wire.read());
  return true;
}

bool initMpu() {
  Wire.beginTransmission(Config::MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) return false;

  delay(100); // numai in setup
  int32_t sum = 0;
  uint16_t good = 0;
  for (uint16_t i = 0; i < 500; ++i) {
    int16_t raw;
    if (readGyroZ(raw)) {
      sum += raw;
      ++good;
    }
    delay(2); // numai la calibrare
  }
  if (good < 450) return false;
  gyro.offsetRaw = (float)sum / good;
  gyro.yawDeg = 0.0f;
  gyro.rateDps = 0.0f;
  gyro.lastUs = micros();
  gyro.healthy = true;
  return true;
}

void updateGyro() {
  const uint32_t now = micros();
  if ((uint32_t)(now - gyro.lastUs) < Config::GYRO_PERIOD_US) return;
  float dt = (now - gyro.lastUs) * 0.000001f;
  gyro.lastUs = now;

  int16_t raw;
  if (!readGyroZ(raw)) {
    gyro.healthy = false;
    return;
  }
  gyro.healthy = true;
  gyro.rateDps = (raw - gyro.offsetRaw) * Config::GYRO_SCALE_250DPS;
  // Zona moarta reduce deriva atunci cand robotul este aproape nemiscat.
  if (fabs(gyro.rateDps) < 0.35f) gyro.rateDps = 0.0f;
  gyro.yawDeg += gyro.rateDps * dt;
}

// ---------------------------------------------------------------------------
// Senzori de aer si mediu
// ---------------------------------------------------------------------------
bool msWriteCommand(uint8_t command) {
  Wire.beginTransmission(Config::MS5611_ADDR);
  Wire.write(command);
  return Wire.endTransmission(true) == 0;
}

uint16_t msReadProm(uint8_t command) {
  Wire.beginTransmission(Config::MS5611_ADDR);
  Wire.write(command);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(Config::MS5611_ADDR, (uint8_t)2, (uint8_t)true) != 2) return 0;
  return (uint16_t)Wire.read() << 8 | Wire.read();
}

uint32_t msReadAdc() {
  Wire.beginTransmission(Config::MS5611_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(Config::MS5611_ADDR, (uint8_t)3, (uint8_t)true) != 3) return 0;
  return (uint32_t)Wire.read() << 16 | (uint32_t)Wire.read() << 8 | Wire.read();
}

bool initMs5611() {
  if (!msWriteCommand(0x1E)) return false;
  delay(4); // numai in setup
  for (uint8_t i = 1; i <= 6; ++i) {
    msProm[i] = msReadProm(0xA0 + i * 2);
    if (msProm[i] == 0 || msProm[i] == 0xFFFF) return false;
  }
  msState = MsState::IDLE;
  return true;
}

void calculateMs5611() {
  if (msD1 == 0 || msD2 == 0) return;
  int64_t dT = (int64_t)msD2 - ((int64_t)msProm[5] << 8);
  int64_t temp = 2000 + ((dT * msProm[6]) >> 23);
  int64_t off = ((int64_t)msProm[2] << 16) + ((dT * msProm[4]) >> 7);
  int64_t sens = ((int64_t)msProm[1] << 15) + ((dT * msProm[3]) >> 8);

  if (temp < 2000) {
    const int64_t t2 = (dT * dT) >> 31;
    const int64_t delta = temp - 2000;
    int64_t off2 = (5 * delta * delta) >> 1;
    int64_t sens2 = (5 * delta * delta) >> 2;
    if (temp < -1500) {
      const int64_t cold = temp + 1500;
      off2 += 7 * cold * cold;
      sens2 += (11 * cold * cold) >> 1;
    }
    temp -= t2;
    off -= off2;
    sens -= sens2;
  }

  const int64_t pressurePa = ((((int64_t)msD1 * sens) >> 21) - off) >> 15;
  air.pressureHpa = pressurePa / 100.0f;
  air.pressureValid = air.pressureHpa > 300.0f && air.pressureHpa < 1200.0f;
}

void updateMs5611() {
  switch (msState) {
    case MsState::IDLE:
      if (elapsedMs(lastMsSample, Config::PRESSURE_SAMPLE_MS) && msWriteCommand(0x48)) { // D1, OSR 4096
        lastMsSample = millis();
        msConversionStarted = millis();
        msState = MsState::WAIT_PRESSURE;
      }
      break;
    case MsState::WAIT_PRESSURE:
      if (elapsedMs(msConversionStarted, 11)) {
        msD1 = msReadAdc();
        if (msWriteCommand(0x58)) { // D2, OSR 4096
          msConversionStarted = millis();
          msState = MsState::WAIT_TEMPERATURE;
        } else {
          msState = MsState::IDLE;
        }
      }
      break;
    case MsState::WAIT_TEMPERATURE:
      if (elapsedMs(msConversionStarted, 11)) {
        msD2 = msReadAdc();
        calculateMs5611();
        msState = MsState::IDLE;
      }
      break;
  }
}

void updateMqSensors() {
  if (!elapsedMs(lastMqSample, Config::MQ_SAMPLE_MS)) return;
  lastMqSample = millis();
  air.mq135Raw = analogRead(Pins::MQ135_ANALOG);
  air.mq7Raw = analogRead(Pins::MQ7_ANALOG);
  air.mq8Raw = analogRead(Pins::MQ8_ANALOG);
  if (air.mq135Filtered == 0) {
    air.mq135Filtered = air.mq135Raw;
    air.mq7Filtered = air.mq7Raw;
    air.mq8Filtered = air.mq8Raw;
  } else {
    air.mq135Filtered = 0.8f * air.mq135Filtered + 0.2f * air.mq135Raw;
    air.mq7Filtered = 0.8f * air.mq7Filtered + 0.2f * air.mq7Raw;
    air.mq8Filtered = 0.8f * air.mq8Filtered + 0.2f * air.mq8Raw;
  }
}

void updateDht22() {
  if (!elapsedMs(lastDhtSample, Config::DHT_SAMPLE_MS)) return;
  lastDhtSample = millis();
  const float humidity = dht.readHumidity();
  const float temperature = dht.readTemperature();
  if (!isnan(humidity) && !isnan(temperature)) {
    air.humidity = humidity;
    air.temperature = temperature;
    air.dhtValid = true;
  }
}

void updateDustSensor() {
  if (!elapsedMs(lastDustSample, Config::DUST_SAMPLE_MS)) return;
  lastDustSample = millis();
  digitalWrite(Pins::GP2Y_LED, LOW);
  delayMicroseconds(280);
  const uint16_t raw = analogRead(Pins::GP2Y_ANALOG);
  delayMicroseconds(40);
  digitalWrite(Pins::GP2Y_LED, HIGH);

  const float voltage = raw * (Config::ADC_VOLTAGE / 1023.0f);
  float density = (voltage - Config::GP2Y_ZERO_VOLTAGE) / Config::GP2Y_V_PER_UG_M3;
  air.dustUgM3 = constrain(density, 0.0f, 1000.0f);
}

void updateAirSensors() {
  updateMqSensors();
  updateDht22();
  updateDustSensor();
  updateMs5611();
}

// ---------------------------------------------------------------------------
// Fata OLED si stare aer
// ---------------------------------------------------------------------------
AirMood measuredAirMood() {
  if (air.mq135Filtered <= 0.0f) return AirMood::NEUTRAL;

  if (airMood == AirMood::HAPPY && air.mq135Filtered < Config::AIR_GOOD_MAX + 30) {
    return AirMood::HAPPY;
  }
  if (airMood == AirMood::SAD && air.mq135Filtered > Config::AIR_MODERATE_MAX - 40) {
    return AirMood::SAD;
  }
  if (air.mq135Filtered < Config::AIR_GOOD_MAX) return AirMood::HAPPY;
  if (air.mq135Filtered < Config::AIR_MODERATE_MAX) return AirMood::NEUTRAL;
  return AirMood::SAD;
}

void updateAirMood() {
  const AirMood measured = measuredAirMood();
  if (measured == airMood) {
    pendingAirMood = measured;
    pendingMoodSince = millis();
    return;
  }
  if (measured != pendingAirMood) {
    pendingAirMood = measured;
    pendingMoodSince = millis();
    return;
  }
  if (elapsedMs(pendingMoodSince, Config::MOOD_CONFIRM_MS)) airMood = pendingAirMood;
}

void drawOledEye(int16_t x, int16_t y, int16_t w, int16_t h, AirMood mood, bool blink) {
  if (blink) {
    oled.fillRoundRect(x, y + h / 2 - 2, w, 5, 2, SSD1306_WHITE);
    return;
  }

  oled.fillRoundRect(x, y, w, h, 7, SSD1306_WHITE);
  if (mood == AirMood::HAPPY) {
    // Taietura de jos produce ochi veseli, usor arcuiti.
    oled.fillTriangle(x, y + h - 9, x + w, y + h - 9,
                      x + w / 2, y + h + 2, SSD1306_BLACK);
  } else if (mood == AirMood::SAD) {
    // Coltul interior coborat formeaza expresia trista.
    if (x < 64) {
      oled.fillTriangle(x, y, x + w, y, x + w, y + 10, SSD1306_BLACK);
    } else {
      oled.fillTriangle(x, y, x + w, y, x, y + 10, SSD1306_BLACK);
    }
  }
}

void updateOledFace() {
  updateAirMood();
  if (!oledReady || !elapsedMs(lastOledFrame, Config::OLED_FRAME_MS)) return;
  lastOledFrame = millis();

  int8_t look = 0;
  if (state == RobotState::TURN_LEFT ||
      (controlMode == ControlMode::MANUAL && manualCommand == ManualCommand::LEFT)) look = -4;
  else if (state == RobotState::TURN_RIGHT ||
           (controlMode == ControlMode::MANUAL && manualCommand == ManualCommand::RIGHT)) look = 4;

  const bool blink = (millis() % 4300UL) < 190UL;
  oled.clearDisplay();
  drawOledEye(18 + look, 17, 38, 30, airMood, blink);
  drawOledEye(72 + look, 17, 38, 30, airMood, blink);
  oled.display();
}

// ---------------------------------------------------------------------------
// Urmarire solara cu 4 LDR-uri si doua servo-uri
// ---------------------------------------------------------------------------
uint16_t readLdrStable(uint8_t pin) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 4; ++i) sum += analogRead(pin);
  return sum / 4;
}

void sampleSolarLdrs() {
  const uint16_t tl = readLdrStable(Pins::LDR_TOP_LEFT);
  const uint16_t tr = readLdrStable(Pins::LDR_TOP_RIGHT);
  const uint16_t bl = readLdrStable(Pins::LDR_BOTTOM_LEFT);
  const uint16_t br = readLdrStable(Pins::LDR_BOTTOM_RIGHT);

  if (!solar.initialized) {
    solar.topLeft = tl;
    solar.topRight = tr;
    solar.bottomLeft = bl;
    solar.bottomRight = br;
    solar.initialized = true;
  } else {
    constexpr float keep = 0.72f;
    constexpr float add = 1.0f - keep;
    solar.topLeft = keep * solar.topLeft + add * tl;
    solar.topRight = keep * solar.topRight + add * tr;
    solar.bottomLeft = keep * solar.bottomLeft + add * bl;
    solar.bottomRight = keep * solar.bottomRight + add * br;
  }

  const int16_t left = (solar.topLeft + solar.bottomLeft) * 0.5f;
  const int16_t right = (solar.topRight + solar.bottomRight) * 0.5f;
  const int16_t top = (solar.topLeft + solar.topRight) * 0.5f;
  const int16_t bottom = (solar.bottomLeft + solar.bottomRight) * 0.5f;
  solar.horizontalError = left - right;
  solar.verticalError = top - bottom;
}

void stopRotationServo() {
  if (!solar.rotationActive) return;
  rotationServo.writeMicroseconds(Config::ROTATION_STOP_US);
  rotationServo.detach();
  solar.rotationActive = false;
  solar.rotationBalanceMs += (int32_t)solar.rotationDirection * Config::ROTATION_BURST_MS;
  solar.rotationDirection = 0;
  solar.lastSampleAt = millis();
}

void finishTiltServo() {
  if (!solar.tiltActive) return;
  tiltServo.detach();
  solar.tiltActive = false;
  solar.lastSampleAt = millis();
}

bool rotationBudgetAllows(int8_t direction) {
  const int32_t next = solar.rotationBalanceMs +
                       (int32_t)direction * Config::ROTATION_BURST_MS;
  return next >= -Config::ROTATION_MAX_BALANCE_MS &&
         next <= Config::ROTATION_MAX_BALANCE_MS;
}

void startRotationPulse(int8_t direction) {
  rotationServo.attach(Pins::SERVO_ROTATION);
  rotationServo.writeMicroseconds(direction > 0 ? Config::ROTATION_LEFT_US
                                                : Config::ROTATION_RIGHT_US);
  solar.rotationDirection = direction;
  solar.lastRotationDirection = direction;
  solar.rotationActive = true;
  solar.motionStartedAt = millis();
}

void startTiltMove(uint8_t target) {
  solar.tiltAngle = constrain(target, Config::TILT_MIN_DEG, Config::TILT_MAX_DEG);
  tiltServo.attach(Pins::SERVO_TILT);
  tiltServo.write(solar.tiltAngle);
  solar.tiltActive = true;
  solar.motionStartedAt = millis();
}

void updateSolarTracker() {
  if (solar.rotationActive) {
    if (elapsedMs(solar.motionStartedAt, Config::ROTATION_BURST_MS)) stopRotationServo();
    return;
  }
  if (solar.tiltActive) {
    if (elapsedMs(solar.motionStartedAt, Config::TILT_SETTLE_MS)) finishTiltServo();
    return;
  }
  if (!elapsedMs(solar.lastSampleAt, Config::SOLAR_SAMPLE_MS)) return;
  solar.lastSampleAt = millis();
  sampleSolarLdrs();

  const int16_t brightness = (solar.topLeft + solar.topRight +
                              solar.bottomLeft + solar.bottomRight) * 0.25f;
  if (brightness < Config::SOLAR_DARK_LEVEL) {
    solar.noImprovementCount = 0;
    solar.previousHorizontalAbs = 32767;
    return;
  }

  const int16_t horizontalAbs = abs(solar.horizontalError);
  if (solar.lastRotationDirection != 0) {
    if (horizontalAbs + Config::SOLAR_MIN_IMPROVEMENT < solar.previousHorizontalAbs) {
      solar.noImprovementCount = 0;
    } else if (solar.noImprovementCount < 255) {
      ++solar.noImprovementCount;
    }
    solar.lastRotationDirection = 0;
  }
  solar.previousHorizontalAbs = horizontalAbs;

  if (solar.noImprovementCount >= Config::SOLAR_MAX_NO_IMPROVEMENT) {
    solar.cooldownUntil = millis() + Config::SOLAR_RETRY_COOLDOWN_MS;
    solar.noImprovementCount = 0;
  }

  if (horizontalAbs > Config::SOLAR_DEADBAND && !timeStillActive(solar.cooldownUntil)) {
    const int8_t direction = solar.horizontalError > 0 ? 1 : -1;
    if (rotationBudgetAllows(direction)) {
      startRotationPulse(direction);
      return;
    }
  }

  if (abs(solar.verticalError) > Config::SOLAR_DEADBAND) {
    int8_t step = solar.verticalError > 0 ? Config::TILT_STEP_DEG : -Config::TILT_STEP_DEG;
    if (Config::TILT_REVERSED) step = -step;
    const int16_t target = constrain((int16_t)solar.tiltAngle + step,
                                     (int16_t)Config::TILT_MIN_DEG,
                                     (int16_t)Config::TILT_MAX_DEG);
    if (target != solar.tiltAngle) startTiltMove((uint8_t)target);
  } else if (horizontalAbs <= Config::SOLAR_DEADBAND) {
    solar.noImprovementCount = 0;
    solar.previousHorizontalAbs = 32767;
  }
}

void initOledAndSolar() {
  oledReady = oled.begin(SSD1306_SWITCHCAPVCC, Config::OLED_ADDR);
  if (oledReady) {
    oled.clearDisplay();
    oled.display();
    Serial.println(F("[OK] OLED SSD1306 initializat."));
  } else {
    Serial.println(F("[WARN] OLED absent; roverul continua fara ecran."));
  }

  pinMode(Pins::LDR_TOP_LEFT, INPUT);
  pinMode(Pins::LDR_TOP_RIGHT, INPUT);
  pinMode(Pins::LDR_BOTTOM_LEFT, INPUT);
  pinMode(Pins::LDR_BOTTOM_RIGHT, INPUT);

  // Centrare initiala a inclinarii, apoi updateSolarTracker il elibereaza.
  tiltServo.attach(Pins::SERVO_TILT);
  tiltServo.write(solar.tiltAngle);
  solar.tiltActive = true;
  solar.motionStartedAt = millis();

  rotationServo.attach(Pins::SERVO_ROTATION);
  rotationServo.writeMicroseconds(Config::ROTATION_STOP_US);
  delay(40); // numai in setup, pentru un impuls de STOP sigur
  rotationServo.detach();
}

// ---------------------------------------------------------------------------
// ESP32, mod manual si transmisie
// ---------------------------------------------------------------------------
void setControlMode(ControlMode next) {
  emergencyStop();
  manualCommand = ManualCommand::STOP;
  controlMode = next;
  if (next == ControlMode::AUTO) {
    state = RobotState::STARTUP;
    stateSince = millis();
  }
}

void processEspCommand(char *frame) {
  if (strcmp(frame, "MODE,AUTO") == 0) {
    setControlMode(ControlMode::AUTO);
    return;
  }
  if (strcmp(frame, "MODE,MANUAL") == 0) {
    setControlMode(ControlMode::MANUAL);
    return;
  }
  if (controlMode != ControlMode::MANUAL) return;

  if (strcmp(frame, "MOVE,FWD") == 0) manualCommand = ManualCommand::FORWARD;
  else if (strcmp(frame, "MOVE,BACK") == 0) manualCommand = ManualCommand::BACKWARD;
  else if (strcmp(frame, "MOVE,LEFT") == 0) manualCommand = ManualCommand::LEFT;
  else if (strcmp(frame, "MOVE,RIGHT") == 0) manualCommand = ManualCommand::RIGHT;
  else manualCommand = ManualCommand::STOP;
  lastManualCommandAt = millis();
}

void readEspCommands() {
  while (Serial2.available()) {
    const char c = (char)Serial2.read();
    if (c == '\n') {
      espCommandBuffer[espCommandLength] = '\0';
      processEspCommand(espCommandBuffer);
      espCommandLength = 0;
    } else if (c != '\r') {
      if (espCommandLength < sizeof(espCommandBuffer) - 1) {
        espCommandBuffer[espCommandLength++] = c;
      } else {
        espCommandLength = 0;
      }
    }
  }
}

void runManualControl() {
  if (elapsedMs(lastManualCommandAt, Config::MANUAL_WATCHDOG_MS)) {
    manualCommand = ManualCommand::STOP;
  }
  const int16_t frontNearest = min(distanceMm(FRONT_CENTER),
                               min(distanceMm(FRONT_LEFT), distanceMm(FRONT_RIGHT)));
  const int16_t rearNearest = min(distanceMm(REAR_CENTER),
                              min(distanceMm(REAR_LEFT), distanceMm(REAR_RIGHT)));

  switch (manualCommand) {
    case ManualCommand::FORWARD:
      if (frontDataFresh() && frontNearest >= Config::CRITICAL_MM)
        requestMotors(Config::MANUAL_PWM, Config::MANUAL_PWM);
      else requestMotors(0, 0);
      break;
    case ManualCommand::BACKWARD:
      if (rearDataFresh() && rearNearest >= Config::REAR_CRITICAL_MM)
        requestMotors(-Config::MANUAL_PWM, -Config::MANUAL_PWM);
      else requestMotors(0, 0);
      break;
    case ManualCommand::LEFT:
      if (distanceMm(LEFT_SIDE) >= Config::CRITICAL_MM)
        requestMotors(-Config::MANUAL_TURN_PWM, Config::MANUAL_TURN_PWM);
      else requestMotors(0, 0);
      break;
    case ManualCommand::RIGHT:
      if (distanceMm(RIGHT_SIDE) >= Config::CRITICAL_MM)
        requestMotors(Config::MANUAL_TURN_PWM, -Config::MANUAL_TURN_PWM);
      else requestMotors(0, 0);
      break;
    default:
      requestMotors(0, 0);
      break;
  }
}

void sendDataToEsp() {
  if (!elapsedMs(lastDataSend, Config::DATA_SEND_MS)) return;
  lastDataSend = millis();
  ++dataSequence;
  // DATA,seq,temp,hum,mq135,mq7,mq8,dust,pressure,mode,state
  Serial2.print(F("DATA,")); Serial2.print(dataSequence);
  Serial2.print(','); Serial2.print(air.dhtValid ? air.temperature : 0.0f, 1);
  Serial2.print(','); Serial2.print(air.dhtValid ? air.humidity : 0.0f, 1);
  Serial2.print(','); Serial2.print((int)air.mq135Filtered);
  Serial2.print(','); Serial2.print((int)air.mq7Filtered);
  Serial2.print(','); Serial2.print((int)air.mq8Filtered);
  Serial2.print(','); Serial2.print(air.dustUgM3, 1);
  Serial2.print(','); Serial2.print(air.pressureValid ? air.pressureHpa : 0.0f, 1);
  Serial2.print(','); Serial2.print(controlMode == ControlMode::AUTO ? F("auto") : F("manual"));
  Serial2.print(','); Serial2.println(stateName(state));

  // Diagnostic pe USB: confirma ca Mega citeste si transmite senzorii.
  Serial.print(F("[AIR TX] seq=")); Serial.print(dataSequence);
  Serial.print(F(" MQ=")); Serial.print((int)air.mq135Filtered);
  Serial.print(','); Serial.print((int)air.mq7Filtered);
  Serial.print(','); Serial.print((int)air.mq8Filtered);
  Serial.print(F(" DHT=")); Serial.print(air.dhtValid ? air.temperature : 0.0f, 1);
  Serial.print(','); Serial.print(air.dhtValid ? air.humidity : 0.0f, 1);
  Serial.print(F(" DUST=")); Serial.print(air.dustUgM3, 1);
  Serial.print(F(" P=")); Serial.println(air.pressureValid ? air.pressureHpa : 0.0f, 1);
}

// ---------------------------------------------------------------------------
// VL53L0X
// ---------------------------------------------------------------------------
bool initTofArray() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(Pins::XSHUT[i], OUTPUT);
    digitalWrite(Pins::XSHUT[i], LOW);
    rangeData[i] = {8000, 0, false, 0};
    for (uint8_t j = 0; j < 3; ++j) tofHistory[i][j] = 8000;
  }
  delay(20);

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    digitalWrite(Pins::XSHUT[i], HIGH);
    delay(15);

    tof[i].setTimeout(Config::TOF_TIMEOUT_MS);
    if (!tof[i].init(true)) {
      Serial.print(F("[EROARE] VL53L0X #")); Serial.println(i);
      return false;
    }
    tof[i].setAddress(Config::TOF_FIRST_ADDR + i);
    tof[i].setMeasurementTimingBudget(Config::TOF_TIMING_BUDGET_US);
    tof[i].startContinuous(Config::TOF_PERIOD_MS);
  }
  return true;
}

int16_t median3(int16_t a, int16_t b, int16_t c) {
  if (a > b) { int16_t t = a; a = b; b = t; }
  if (b > c) { int16_t t = b; b = c; c = t; }
  if (a > b) { int16_t t = a; a = b; b = t; }
  return b;
}

void updateOneTof() {
  const uint8_t id = SCAN_PATTERN[scanIndex];
  scanIndex = (scanIndex + 1) % SCAN_COUNT;

  uint16_t raw = tof[id].readRangeContinuousMillimeters();
  RangeData &r = rangeData[id];
  if (tof[id].timeoutOccurred() || raw == 0 || raw == 65535) {
    if (r.failures < 255) ++r.failures;
    if (r.failures >= 3) r.valid = false;
    return;
  }

  int16_t corrected = (int16_t)raw + TOF_OFFSET[id];
  int16_t sample;
  if (corrected < Config::SENSOR_MIN_MM) {
    // Un obiect extrem de apropiat ramane obstacol, nu este declarat liber.
    sample = Config::SENSOR_MIN_MM;
  } else if (corrected > TOF_FLOOR_CUTOFF_MM[id]) {
    
    sample = 8000;
  } else {
    sample = corrected;
  }

  tofHistory[id][tofHistoryIndex[id]] = sample;
  tofHistoryIndex[id] = (tofHistoryIndex[id] + 1) % 3;
  if (tofHistoryCount[id] < 3) ++tofHistoryCount[id];

  if (sample <= Config::CRITICAL_MM) {
    
    r.mm = sample;
  } else if (tofHistoryCount[id] == 1) {
    r.mm = sample;
  } else if (tofHistoryCount[id] == 2) {
    
    r.mm = min(tofHistory[id][0], tofHistory[id][1]);
  } else {
    r.mm = median3(tofHistory[id][0], tofHistory[id][1], tofHistory[id][2]);
  }
  r.updatedAt = millis();
  r.valid = true;
  r.failures = 0;
}

// ---------------------------------------------------------------------------
// Navigatie reactiva
// ---------------------------------------------------------------------------
int16_t scoreDistance(SensorId id) {
  // 8000 inseamna ca nu exista obstacol in domeniul util; limitam contributia
  // ca o singura citire sa nu domine intreaga decizie.
  return constrain(distanceMm(id), 0, 550);
}

void rememberRecentFailure() {
  if (lastTurnFailureRegistered || lastTurnDirection == 0) return;
  if (elapsedMs(lastTurnFinishedAt, Config::FAILED_PATH_WINDOW_MS)) return;

  const uint32_t deadline = millis() + Config::DIRECTION_MEMORY_MS;
  if (lastTurnDirection > 0) avoidLeftUntil = deadline;
  else avoidRightUntil = deadline;
  lastTurnFailureRegistered = true;
}

RobotState chooseTurnAfterScan() {
  const int16_t leftSide = distanceMm(LEFT_SIDE);
  const int16_t rightSide = distanceMm(RIGHT_SIDE);

  
  int32_t leftScore = scoreDistance(FRONT_LEFT)
                    + 2L * scoreDistance(LEFT_SIDE)
                    + scoreDistance(REAR_LEFT);
  int32_t rightScore = scoreDistance(FRONT_RIGHT)
                     + 2L * scoreDistance(RIGHT_SIDE)
                     + scoreDistance(REAR_RIGHT);

  const bool leftRememberedBad = timeStillActive(avoidLeftUntil);
  const bool rightRememberedBad = timeStillActive(avoidRightUntil);
 
  if (leftRememberedBad && !rightRememberedBad) leftScore -= 3000;
  if (rightRememberedBad && !leftRememberedBad) rightScore -= 3000;

  if (leftRememberedBad && rightSide >= Config::SIDE_SAFE_MM) {
    return RobotState::TURN_RIGHT;
  }
  if (rightRememberedBad && leftSide >= Config::SIDE_SAFE_MM) {
    return RobotState::TURN_LEFT;
  }

 
  if (leftSide < Config::SIDE_SAFE_MM && rightSide < Config::SIDE_SAFE_MM) {
    const int16_t rearNearest = min(distanceMm(REAR_CENTER),
                              min(distanceMm(REAR_LEFT), distanceMm(REAR_RIGHT)));
    if (rearDataFresh() && rearNearest > Config::REAR_CRITICAL_MM && reverseAttempts < 2) {
      return RobotState::REVERSE;
    }
    plannedEscapeDirection = (leftScore >= rightScore) ? 1 : -1;
    return RobotState::ESCAPE;
  }
  return (leftScore >= rightScore) ? RobotState::TURN_LEFT : RobotState::TURN_RIGHT;
}

void recordCompletedTurn(int8_t direction) {
  lastTurnDirection = direction;
  lastTurnFinishedAt = millis();
  lastTurnFailureRegistered = false;
 
}

int16_t turnPwmForRemaining(float remainingDeg) {
  
  if (remainingDeg > 16.0f) return Config::TURN_PWM;
  if (remainingDeg > 6.0f) return 100;
  return 80;
}

void beginBrakeFor(RobotState next) {
  stateAfterBrake = next;
  requestMotors(0, 0);
  enterState(RobotState::BRAKE);
}

void driveStraightPd(int16_t basePwm) {
  const uint32_t now = micros();
  float dt = pdPreviousUs ? (now - pdPreviousUs) * 0.000001f : 0.01f;
  pdPreviousUs = now;
  dt = constrain(dt, 0.005f, 0.1f);

  const float error = headingReference - gyro.yawDeg;
  float correction = Config::KP * error + Config::KD * (error - pdPreviousError) / dt;
  pdPreviousError = error;
  correction = constrain(correction, -Config::MAX_HEADING_CORRECTION, Config::MAX_HEADING_CORRECTION);
  requestMotors(basePwm - (int16_t)correction, basePwm + (int16_t)correction);
}

void runNavigation() {
  const int16_t fl = distanceMm(FRONT_LEFT);
  const int16_t fc = distanceMm(FRONT_CENTER);
  const int16_t fr = distanceMm(FRONT_RIGHT);
  const int16_t nearestFront = min(fc, min(fl, fr));

  // Daca datele frontale lipsesc, robotul se opreste; nu merge "orb".
  if (!frontDataFresh() && state != RobotState::STARTUP && state != RobotState::FAULT) {
    emergencyStop();
    enterState(RobotState::FAULT);
  }

  switch (state) {
    case RobotState::STARTUP:
      requestMotors(0, 0);
      if (elapsedMs(stateSince, Config::START_DELAY_MS) && frontDataFresh()) {
        enterState(RobotState::CRUISE);
      }
      break;

    case RobotState::CRUISE:
      if (nearestFront < Config::CRITICAL_MM) {
        rememberRecentFailure();
        beginBrakeFor(RobotState::SCAN_CORNER);
      } else if (timeStillActive(directionCommitUntil)) {
        
        driveStraightPd(Config::TURN_COMMIT_PWM);
      } else if (nearestFront < Config::SLOW_MM) {
        enterState(RobotState::AVOID);
      } else {
        const int16_t startPwm = elapsedMs(stateSince, Config::CRUISE_RAMP_MS)
                               ? Config::CRUISE_PWM : Config::CRUISE_START_PWM;
        driveStraightPd(startPwm);
      }
      break;

    case RobotState::AVOID: {
      if (nearestFront < Config::CRITICAL_MM) {
        rememberRecentFailure();
        beginBrakeFor(RobotState::SCAN_CORNER);
        break;
      }
      if (nearestFront > Config::RELEASE_MM) {
        enterState(RobotState::CRUISE);
        break;
      }
      int16_t speed = map(constrain(nearestFront, Config::CRITICAL_MM, Config::SLOW_MM),
                          Config::CRITICAL_MM, Config::SLOW_MM,
                          Config::SLOW_PWM, Config::CRUISE_PWM);
      
      int16_t steer = constrain((fl - fr) / 3, -65, 65);
      if (abs(fl - fr) < 35) {
        steer = (distanceMm(LEFT_SIDE) > distanceMm(RIGHT_SIDE)) ? 45 : -45;
      }
      requestMotors(speed - steer, speed + steer);
      break;
    }

    case RobotState::BRAKE:
      requestMotors(0, 0);
      if (elapsedMs(stateSince, Config::BRAKE_MS) && appliedLeft == 0 && appliedRight == 0) {
        enterState(stateAfterBrake);
      }
      break;

    case RobotState::SCAN_CORNER:
      
      requestMotors(0, 0);
      if (elapsedMs(stateSince, Config::CORNER_SCAN_MS)) {
        if (nearestFront > Config::SLOW_MM) {
          enterState(RobotState::CRUISE);
        } else if (nearestFront >= Config::CRITICAL_MM) {
          
          if (timeStillActive(directionCommitUntil)) enterState(RobotState::CRUISE);
          else enterState(RobotState::AVOID);
        } else {
          RobotState decision = chooseTurnAfterScan();
          turnGoalDeg = Config::TURN_STEP_DEG;
          enterState(decision);
        }
      }
      break;

    case RobotState::REVERSE: {
      const int16_t rearNearest = min(distanceMm(REAR_CENTER),
                                min(distanceMm(REAR_LEFT), distanceMm(REAR_RIGHT)));
      if (!rearDataFresh() || rearNearest <= Config::REAR_CRITICAL_MM) {
        beginBrakeFor(RobotState::SCAN_CORNER);
      } else if (elapsedMs(stateSince, Config::REVERSE_MS)) {
        ++reverseAttempts;
        beginBrakeFor(RobotState::SCAN_CORNER);
      } else {
        requestMotors(-Config::REVERSE_PWM, -Config::REVERSE_PWM);
      }
      break;
    }

    case RobotState::TURN_LEFT: {
      const float turned = fabs(gyro.yawDeg - turnStartYaw);
      const int16_t turnPwm = turnPwmForRemaining(max(0.0f, turnGoalDeg - turned));
      requestMotors(-turnPwm, turnPwm);
      if (turned >= turnGoalDeg) {
        recordCompletedTurn(1);
        beginBrakeFor(RobotState::SCAN_CORNER);
      } else if (elapsedMs(stateSince, Config::TURN_TIMEOUT_MS)) {
        emergencyStop();
        enterState(RobotState::FAULT);
      }
      break;
    }

    case RobotState::TURN_RIGHT: {
      const float turned = fabs(gyro.yawDeg - turnStartYaw);
      const int16_t turnPwm = turnPwmForRemaining(max(0.0f, turnGoalDeg - turned));
      requestMotors(turnPwm, -turnPwm);
      if (turned >= turnGoalDeg) {
        recordCompletedTurn(-1);
        beginBrakeFor(RobotState::SCAN_CORNER);
      } else if (elapsedMs(stateSince, Config::TURN_TIMEOUT_MS)) {
        emergencyStop();
        enterState(RobotState::FAULT);
      }
      break;
    }

    case RobotState::ESCAPE: {
      
      const bool goLeft = plannedEscapeDirection > 0;
      const float turned = fabs(gyro.yawDeg - turnStartYaw);
      const int16_t turnPwm = turnPwmForRemaining(max(0.0f, turnGoalDeg - turned));
      requestMotors(goLeft ? -turnPwm : turnPwm,
                    goLeft ? turnPwm : -turnPwm);
      if (turned >= turnGoalDeg) {
        recordCompletedTurn(goLeft ? 1 : -1);
        beginBrakeFor(RobotState::SCAN_CORNER);
      } else if (elapsedMs(stateSince, Config::TURN_TIMEOUT_MS)) {
        emergencyStop();
        enterState(RobotState::FAULT);
      }
      break;
    }

    case RobotState::FAULT:
      emergencyStop();
      
      if (frontDataFresh() && gyro.healthy && elapsedMs(stateSince, 1000)) {
        enterState(RobotState::CRUISE);
      }
      break;
  }
}

void printTelemetry() {
  if (!elapsedMs(lastTelemetry, Config::TELEMETRY_MS)) return;
  lastTelemetry = millis();
  Serial.print(F("STATE=")); Serial.print(stateName(state));
  Serial.print(F(" YAW=")); Serial.print(gyro.yawDeg, 1);
  Serial.print(F(" TOF="));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(distanceMm((SensorId)i));
    if (i + 1 < SENSOR_COUNT) Serial.print(',');
  }
  Serial.print(F(" PWM=")); Serial.print(appliedLeft);
  Serial.print(','); Serial.println(appliedRight);
}

void configureMotorPins() {
  const uint8_t outputs[] = {
    Pins::STBY_R, Pins::PWMA_R, Pins::AIN1_R, Pins::AIN2_R,
    Pins::PWMB_R, Pins::BIN1_R, Pins::BIN2_R,
    Pins::STBY_L, Pins::PWMA_L, Pins::AIN1_L, Pins::AIN2_L,
    Pins::PWMB_L, Pins::BIN1_L, Pins::BIN2_L
  };
  for (uint8_t i = 0; i < sizeof(outputs); ++i) pinMode(outputs[i], OUTPUT);
  digitalWrite(Pins::STBY_R, HIGH);
  digitalWrite(Pins::STBY_L, HIGH);
  emergencyStop();
}

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  Serial2.begin(Config::ESP_SERIAL_BAUD);
  configureMotorPins();

  pinMode(Pins::DHT22_DATA, INPUT_PULLUP);
  pinMode(Pins::MQ135_DIGITAL, INPUT);
  pinMode(Pins::MQ7_DIGITAL, INPUT);
  pinMode(Pins::MQ8_DIGITAL, INPUT);
  pinMode(Pins::GP2Y_LED, OUTPUT);
  digitalWrite(Pins::GP2Y_LED, HIGH);
  dht.begin();

  Wire.begin();
  Wire.setClock(400000UL);
  Wire.setWireTimeout(3000, true);

  Serial.println(F("\n[AUTOV2] Initializare rover"));
  Serial.println(F("[MPU] Nu miscati robotul in timpul calibrarii..."));
  if (!initMpu()) {
    Serial.println(F("[FATAL] MPU6050 nu raspunde."));
    enterState(RobotState::FAULT);
    return;
  }
  Serial.print(F("[MPU] Offset Z=")); Serial.println(gyro.offsetRaw, 2);

  Serial.println(F("[TOF] Initializare 8 senzori..."));
  if (!initTofArray()) {
    Serial.println(F("[FATAL] Matricea VL53L0X nu a pornit."));
    enterState(RobotState::FAULT);
    return;
  }

  if (!initMs5611()) {
    Serial.println(F("[WARN] MS5611 nu raspunde; roverul continua fara presiune."));
  } else {
    Serial.println(F("[OK] MS5611 initializat."));
  }

  initOledAndSolar();

  stateSince = millis();
  state = RobotState::STARTUP;
  systemReady = true;
  Serial.println(F("[OK] Sistem pregatit in mod MANUAL, cu motoarele oprite."));
}

void loop() {
  if (!systemReady) {
    emergencyStop();
    return;
  }
  readEspCommands();
  updateGyro();
  updateOneTof();
  updateAirSensors();
  updateSolarTracker();
  updateOledFace();
  if (controlMode == ControlMode::AUTO) runNavigation();
  else runManualControl();
  updateMotors();
  sendDataToEsp();
  printTelemetry();
}
 