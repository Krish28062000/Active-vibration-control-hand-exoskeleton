/*
  ============================================================================
  KK-EXOSKELETON — TEST 01: DUAL-ADXL345 STATIC GRAVITY VALIDATION V1
  ESP32 + 2x ADXL345 on the EXISTING project wiring
  ============================================================================

  PURPOSE
    Thesis Test 4.2 — verify sign, scale, bias, repeatability and basic integrity
    of ADXL1 and ADXL2 using six static orientations: +X, -X, +Y, -Y, +Z, -Z.

  EXISTING HARDWARE ARCHITECTURE — UNCHANGED
    ADXL1 TOOL / reference:  CS = GPIO5
    ADXL2 HAND / error:      CS = GPIO17
    Shared SPI SCK:          GPIO18
    Shared SPI MISO:         GPIO19
    Shared SPI MOSI:         GPIO23
    Visaton DAC output:      GPIO25 (NOT USED in this test; held at midscale)

  ADXL345 SETTINGS — SAME VALIDATED PROJECT SETTINGS
    Full resolution, +/-16 g        DATA_FORMAT = 0x0B
    Nominal output data rate        BW_RATE = 0x0E = 1600 Hz
    Measurement mode               POWER_CTL = 0x08
    SPI                            5 MHz, MODE3

  TEST CONDITIONS
    Tool OFF.
    Class-D amplifier OFF.
    Visaton unplugged / not driven.
    Keep the selected ADXL rigidly still during each record.

  SERIAL MONITOR
    115200 baud
    Newline or Both NL & CR

  COMMANDS
    g 1 +X   -> ADXL1, +X upward, automatically records 3 valid 2-s repeats
    g 1 -X
    g 1 +Y
    g 1 -Y
    g 1 +Z
    g 1 -Z

    g 2 +X   -> same sequence for ADXL2
    ...

    c        -> print stored orientation summaries and +/- calibration pairs
    v        -> verify both sensor registers now
    p        -> print fixed hardware/test configuration
    r        -> clear stored results in RAM
    h        -> help

  ORIENTATION DEFINITION
    "+X" means orient the selected sensor so its +X axis points upward and the
    expected static reading on X is approximately +g. Likewise for the other
    five orientations.

  DATA OUTPUT
    Machine-readable CSV-style rows are printed with these prefixes:
      REGISTER_CHECK
      STATIC_RUN
      ORIENTATION_SUMMARY
      CAL_PAIR

    Save the COMPLETE Serial Monitor output. It can be imported directly for
    thesis tables/plots and reviewed for data validity.
  ============================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ============================================================================
// EXISTING PROJECT HARDWARE — DO NOT CHANGE
// ============================================================================
const uint8_t PIN_SPI_SCK  = 18;
const uint8_t PIN_SPI_MISO = 19;
const uint8_t PIN_SPI_MOSI = 23;
const uint8_t PIN_CS_ADXL1 = 5;
const uint8_t PIN_CS_ADXL2 = 17;
const uint8_t PIN_VISATON_DAC = 25;

// ============================================================================
// ADXL345 REGISTERS / SETTINGS
// ============================================================================
const uint8_t REG_DEVID       = 0x00;
const uint8_t REG_BW_RATE     = 0x2C;
const uint8_t REG_POWER_CTL   = 0x2D;
const uint8_t REG_DATA_FORMAT = 0x31;
const uint8_t REG_DATAX0      = 0x32;

const uint8_t SPI_READ_BIT = 0x80;
const uint8_t SPI_MB_BIT   = 0x40;
const uint8_t ADXL_DEVID_OK = 0xE5;

// Retain the established project SPI setting for this first verification test.
SPISettings adxlSPISettings(5000000, MSBFIRST, SPI_MODE3);

// ============================================================================
// STATIC TEST SETTINGS
// ============================================================================
const double SAMPLE_RATE_HZ = 1600.0;
const uint32_t SAMPLE_PERIOD_US = 625;
const uint32_t LATE_SAMPLE_WARNING_US = 10;

const double ADXL_G_PER_LSB = 0.0039;
const double GRAVITY_MS2 = 9.80665;
const double ADXL_MS2_PER_LSB = ADXL_G_PER_LSB * GRAVITY_MS2;

const uint16_t RECORD_SECONDS = 2;
const uint32_t SAMPLES_PER_RECORD =
    static_cast<uint32_t>(SAMPLE_RATE_HZ * RECORD_SECONDS);

const uint8_t REQUIRED_REPEATS = 3;
const uint8_t MAX_ATTEMPTS_PER_ORIENTATION = 6;
const uint16_t PRE_RECORD_SETTLE_MS = 600;
const uint16_t BETWEEN_REPEAT_MS = 600;

// Raw clipping is impossible in a correct static +/-1 g test. This merely
// catches corrupted or impossible data without using it as a calibration gate.
const int16_t RAW_CLIP_LIMIT = 32000;

// ============================================================================
// DATA TYPES
// ============================================================================
struct RawAcceleration {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct SensorDefinition {
  uint8_t csPin;
  const char *shortName;
  const char *roleName;
};

struct RunningStats {
  uint32_t n;
  double mean;
  double m2;

  void clear() {
    n = 0;
    mean = 0.0;
    m2 = 0.0;
  }

  void add(double value) {
    n++;
    double delta = value - mean;
    mean += delta / static_cast<double>(n);
    double delta2 = value - mean;
    m2 += delta * delta2;
  }

  double sd() const {
    if (n < 2) return 0.0;
    return sqrt(m2 / static_cast<double>(n - 1));
  }
};

struct StaticRecordResult {
  bool valid;
  uint32_t samples;
  double meanX;
  double meanY;
  double meanZ;
  double sdX;
  double sdY;
  double sdZ;
  double meanNorm;
  double sdNorm;
  uint32_t clipCount;
  uint32_t lateSampleCount;
  uint32_t maxLatenessUs;
  bool rawChanged;
  bool registerOkBefore;
  bool registerOkAfter;
};

struct StoredRun {
  bool valid;
  double meanX;
  double meanY;
  double meanZ;
  double meanNorm;
};

// 2 sensors x 6 orientations x 3 repeats.
StoredRun storedRuns[2][6][REQUIRED_REPEATS];
uint8_t storedRunCount[2][6] = {};

SensorDefinition SENSOR1 = {
  PIN_CS_ADXL1,
  "ADXL1",
  "TOOL_REFERENCE"
};

SensorDefinition SENSOR2 = {
  PIN_CS_ADXL2,
  "ADXL2",
  "HAND_ERROR"
};

const char *ORIENTATIONS[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

// ============================================================================
// LOW-LEVEL SPI
// ============================================================================
void deselectAllSensors() {
  digitalWrite(PIN_CS_ADXL1, HIGH);
  digitalWrite(PIN_CS_ADXL2, HIGH);
}

void writeRegister(uint8_t csPin, uint8_t reg, uint8_t value) {
  SPI.beginTransaction(adxlSPISettings);
  deselectAllSensors();
  digitalWrite(csPin, LOW);
  SPI.transfer(reg & 0x3F);
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(uint8_t csPin, uint8_t reg) {
  SPI.beginTransaction(adxlSPISettings);
  deselectAllSensors();
  digitalWrite(csPin, LOW);
  SPI.transfer(SPI_READ_BIT | (reg & 0x3F));
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return value;
}

RawAcceleration readRawXYZ(uint8_t csPin) {
  RawAcceleration r = {};

  SPI.beginTransaction(adxlSPISettings);
  deselectAllSensors();
  digitalWrite(csPin, LOW);
  SPI.transfer(SPI_READ_BIT | SPI_MB_BIT | REG_DATAX0);

  uint8_t x0 = SPI.transfer(0x00);
  uint8_t x1 = SPI.transfer(0x00);
  uint8_t y0 = SPI.transfer(0x00);
  uint8_t y1 = SPI.transfer(0x00);
  uint8_t z0 = SPI.transfer(0x00);
  uint8_t z1 = SPI.transfer(0x00);

  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  r.x = static_cast<int16_t>((static_cast<uint16_t>(x1) << 8) | x0);
  r.y = static_cast<int16_t>((static_cast<uint16_t>(y1) << 8) | y0);
  r.z = static_cast<int16_t>((static_cast<uint16_t>(z1) << 8) | z0);
  return r;
}

// ============================================================================
// SENSOR INITIALIZATION / INTEGRITY
// ============================================================================
bool verifySensorRegisters(const SensorDefinition &sensor, bool printRow) {
  uint8_t devid = readRegister(sensor.csPin, REG_DEVID);
  uint8_t format = readRegister(sensor.csPin, REG_DATA_FORMAT);
  uint8_t rate = readRegister(sensor.csPin, REG_BW_RATE);
  uint8_t power = readRegister(sensor.csPin, REG_POWER_CTL);

  bool valid =
      (devid == ADXL_DEVID_OK) &&
      ((format & 0x0F) == 0x0B) &&
      ((rate & 0x1F) == 0x0E) &&
      (power == 0x08);

  if (printRow) {
    Serial.print("REGISTER_CHECK,");
    Serial.print(sensor.shortName);
    Serial.print(",DEVID=0x"); Serial.print(devid, HEX);
    Serial.print(",DATA_FORMAT=0x"); Serial.print(format, HEX);
    Serial.print(",BW_RATE=0x"); Serial.print(rate, HEX);
    Serial.print(",POWER_CTL=0x"); Serial.print(power, HEX);
    Serial.print(",STATUS="); Serial.println(valid ? "PASS" : "FAIL");
  }

  return valid;
}

bool initializeSensor(const SensorDefinition &sensor) {
  writeRegister(sensor.csPin, REG_POWER_CTL, 0x00);
  delay(10);

  uint8_t devid = readRegister(sensor.csPin, REG_DEVID);
  if (devid != ADXL_DEVID_OK) {
    Serial.print("ERROR,");
    Serial.print(sensor.shortName);
    Serial.print(",DEVID=0x");
    Serial.println(devid, HEX);
    return false;
  }

  // FULL_RES = 1, +/-16 g, four-wire SPI.
  writeRegister(sensor.csPin, REG_DATA_FORMAT, 0x0B);

  // Nominal 1600 Hz output data rate.
  writeRegister(sensor.csPin, REG_BW_RATE, 0x0E);

  // Measurement mode.
  writeRegister(sensor.csPin, REG_POWER_CTL, 0x08);
  delay(30);

  return verifySensorRegisters(sensor, true);
}

bool ensureSensorReady(const SensorDefinition &sensor) {
  if (verifySensorRegisters(sensor, false)) return true;

  Serial.print("INTEGRITY_RETRY,");
  Serial.print(sensor.shortName);
  Serial.println(",reason=register_mismatch,reinitializing=1");

  if (!initializeSensor(sensor)) return false;
  return verifySensorRegisters(sensor, false);
}

// ============================================================================
// ORIENTATION / SENSOR HELPERS
// ============================================================================
int orientationIndex(const char *text) {
  for (int i = 0; i < 6; i++) {
    if (strcmp(text, ORIENTATIONS[i]) == 0) return i;
  }
  return -1;
}

SensorDefinition *sensorFromNumber(int sensorNumber) {
  if (sensorNumber == 1) return &SENSOR1;
  if (sensorNumber == 2) return &SENSOR2;
  return nullptr;
}

int sensorArrayIndex(int sensorNumber) {
  return sensorNumber - 1;
}

char activeAxisForOrientation(int orientation) {
  if (orientation <= 1) return 'X';
  if (orientation <= 3) return 'Y';
  return 'Z';
}

double expectedActiveValue(int orientation) {
  return (orientation % 2 == 0) ? GRAVITY_MS2 : -GRAVITY_MS2;
}

double activeMean(const StoredRun &r, char axis) {
  if (axis == 'X') return r.meanX;
  if (axis == 'Y') return r.meanY;
  return r.meanZ;
}

// ============================================================================
// ONE STATIC RECORD
// ============================================================================
StaticRecordResult acquireStaticRecord(const SensorDefinition &sensor) {
  StaticRecordResult result = {};
  result.valid = false;

  result.registerOkBefore = ensureSensorReady(sensor);
  if (!result.registerOkBefore) return result;

  delay(PRE_RECORD_SETTLE_MS);

  RunningStats sx, sy, sz, sn;
  sx.clear(); sy.clear(); sz.clear(); sn.clear();

  RawAcceleration previous = {};
  bool havePrevious = false;
  bool rawChanged = false;
  uint32_t clipCount = 0;
  uint32_t lateCount = 0;
  uint32_t maxLateUs = 0;

  uint32_t nextSampleUs = micros();

  for (uint32_t i = 0; i < SAMPLES_PER_RECORD; i++) {
    while (static_cast<int32_t>(micros() - nextSampleUs) < 0) {
      // deterministic wait
    }

    uint32_t nowUs = micros();
    uint32_t latenessUs = nowUs - nextSampleUs;
    if (latenessUs > maxLateUs) maxLateUs = latenessUs;
    if (latenessUs > LATE_SAMPLE_WARNING_US) lateCount++;
    nextSampleUs += SAMPLE_PERIOD_US;

    RawAcceleration raw = readRawXYZ(sensor.csPin);

    if (abs(static_cast<int>(raw.x)) >= RAW_CLIP_LIMIT ||
        abs(static_cast<int>(raw.y)) >= RAW_CLIP_LIMIT ||
        abs(static_cast<int>(raw.z)) >= RAW_CLIP_LIMIT) {
      clipCount++;
    }

    if (havePrevious) {
      if (raw.x != previous.x || raw.y != previous.y || raw.z != previous.z) {
        rawChanged = true;
      }
    } else {
      previous = raw;
      havePrevious = true;
    }
    previous = raw;

    double x = static_cast<double>(raw.x) * ADXL_MS2_PER_LSB;
    double y = static_cast<double>(raw.y) * ADXL_MS2_PER_LSB;
    double z = static_cast<double>(raw.z) * ADXL_MS2_PER_LSB;
    double norm = sqrt(x * x + y * y + z * z);

    sx.add(x);
    sy.add(y);
    sz.add(z);
    sn.add(norm);
  }

  result.registerOkAfter = verifySensorRegisters(sensor, false);
  result.samples = sx.n;
  result.meanX = sx.mean;
  result.meanY = sy.mean;
  result.meanZ = sz.mean;
  result.sdX = sx.sd();
  result.sdY = sy.sd();
  result.sdZ = sz.sd();
  result.meanNorm = sn.mean;
  result.sdNorm = sn.sd();
  result.clipCount = clipCount;
  result.lateSampleCount = lateCount;
  result.maxLatenessUs = maxLateUs;
  result.rawChanged = rawChanged;

  // A record is accepted only on digital integrity and non-stuck raw data.
  // No acceleration-magnitude criterion is used here; scale is what this test
  // is intended to measure.
  result.valid =
      result.registerOkBefore &&
      result.registerOkAfter &&
      result.rawChanged &&
      (result.samples == SAMPLES_PER_RECORD) &&
      (result.clipCount == 0);

  return result;
}

void printStaticRunRow(
    const SensorDefinition &sensor,
    int orientation,
    uint8_t repeatNumber,
    uint8_t attemptNumber,
    const StaticRecordResult &r) {

  char axis = activeAxisForOrientation(orientation);
  double expected = expectedActiveValue(orientation);
  double measuredActive =
      (axis == 'X') ? r.meanX : ((axis == 'Y') ? r.meanY : r.meanZ);
  double activeErrorPct =
      100.0 * (measuredActive - expected) / fabs(expected);
  double normErrorPct =
      100.0 * (r.meanNorm - GRAVITY_MS2) / GRAVITY_MS2;

  Serial.print("STATIC_RUN,");
  Serial.print("sensor="); Serial.print(sensor.shortName);
  Serial.print(",role="); Serial.print(sensor.roleName);
  Serial.print(",orientation="); Serial.print(ORIENTATIONS[orientation]);
  Serial.print(",repeat="); Serial.print(repeatNumber);
  Serial.print(",attempt="); Serial.print(attemptNumber);
  Serial.print(",N="); Serial.print(r.samples);
  Serial.print(",meanX_mps2="); Serial.print(r.meanX, 6);
  Serial.print(",sdX_mps2="); Serial.print(r.sdX, 6);
  Serial.print(",meanY_mps2="); Serial.print(r.meanY, 6);
  Serial.print(",sdY_mps2="); Serial.print(r.sdY, 6);
  Serial.print(",meanZ_mps2="); Serial.print(r.meanZ, 6);
  Serial.print(",sdZ_mps2="); Serial.print(r.sdZ, 6);
  Serial.print(",meanNorm_mps2="); Serial.print(r.meanNorm, 6);
  Serial.print(",sdNorm_mps2="); Serial.print(r.sdNorm, 6);
  Serial.print(",activeAxis="); Serial.print(axis);
  Serial.print(",expectedActive_mps2="); Serial.print(expected, 6);
  Serial.print(",measuredActive_mps2="); Serial.print(measuredActive, 6);
  Serial.print(",activeError_pct="); Serial.print(activeErrorPct, 3);
  Serial.print(",normError_pct="); Serial.print(normErrorPct, 3);
  Serial.print(",clipCount="); Serial.print(r.clipCount);
  Serial.print(",lateSamples="); Serial.print(r.lateSampleCount);
  Serial.print(",maxLateness_us="); Serial.print(r.maxLatenessUs);
  Serial.print(",rawChanged="); Serial.print(r.rawChanged ? 1 : 0);
  Serial.print(",regBefore="); Serial.print(r.registerOkBefore ? 1 : 0);
  Serial.print(",regAfter="); Serial.print(r.registerOkAfter ? 1 : 0);
  Serial.print(",STATUS="); Serial.println(r.valid ? "VALID" : "INVALID");
}

// ============================================================================
// STORED RESULT SUMMARIES
// ============================================================================
void clearStoredResults() {
  for (int s = 0; s < 2; s++) {
    for (int o = 0; o < 6; o++) {
      storedRunCount[s][o] = 0;
      for (int r = 0; r < REQUIRED_REPEATS; r++) {
        storedRuns[s][o][r] = {};
      }
    }
  }
}

bool aggregateOrientation(
    int sIdx,
    int oIdx,
    double &meanX,
    double &sdBetweenX,
    double &meanY,
    double &sdBetweenY,
    double &meanZ,
    double &sdBetweenZ,
    double &meanNorm,
    double &sdBetweenNorm) {

  RunningStats sx, sy, sz, sn;
  sx.clear(); sy.clear(); sz.clear(); sn.clear();

  for (uint8_t r = 0; r < storedRunCount[sIdx][oIdx]; r++) {
    const StoredRun &run = storedRuns[sIdx][oIdx][r];
    if (!run.valid) continue;
    sx.add(run.meanX);
    sy.add(run.meanY);
    sz.add(run.meanZ);
    sn.add(run.meanNorm);
  }

  if (sx.n == 0) return false;

  meanX = sx.mean; sdBetweenX = sx.sd();
  meanY = sy.mean; sdBetweenY = sy.sd();
  meanZ = sz.mean; sdBetweenZ = sz.sd();
  meanNorm = sn.mean; sdBetweenNorm = sn.sd();
  return true;
}

void printOrientationSummary(int sensorNumber, int orientation) {
  int sIdx = sensorArrayIndex(sensorNumber);
  SensorDefinition *sensor = sensorFromNumber(sensorNumber);
  if (!sensor) return;

  double mx, sdx, my, sdy, mz, sdz, mn, sdn;
  if (!aggregateOrientation(sIdx, orientation,
                            mx, sdx, my, sdy, mz, sdz, mn, sdn)) {
    Serial.print("ORIENTATION_SUMMARY,sensor="); Serial.print(sensor->shortName);
    Serial.print(",orientation="); Serial.print(ORIENTATIONS[orientation]);
    Serial.println(",STATUS=NO_DATA");
    return;
  }

  char axis = activeAxisForOrientation(orientation);
  double expected = expectedActiveValue(orientation);
  double active = (axis == 'X') ? mx : ((axis == 'Y') ? my : mz);
  double activeErrorPct = 100.0 * (active - expected) / fabs(expected);
  double normErrorPct = 100.0 * (mn - GRAVITY_MS2) / GRAVITY_MS2;

  Serial.print("ORIENTATION_SUMMARY,");
  Serial.print("sensor="); Serial.print(sensor->shortName);
  Serial.print(",orientation="); Serial.print(ORIENTATIONS[orientation]);
  Serial.print(",validRepeats="); Serial.print(storedRunCount[sIdx][orientation]);
  Serial.print(",meanX_mps2="); Serial.print(mx, 6);
  Serial.print(",betweenSdX_mps2="); Serial.print(sdx, 6);
  Serial.print(",meanY_mps2="); Serial.print(my, 6);
  Serial.print(",betweenSdY_mps2="); Serial.print(sdy, 6);
  Serial.print(",meanZ_mps2="); Serial.print(mz, 6);
  Serial.print(",betweenSdZ_mps2="); Serial.print(sdz, 6);
  Serial.print(",meanNorm_mps2="); Serial.print(mn, 6);
  Serial.print(",betweenSdNorm_mps2="); Serial.print(sdn, 6);
  Serial.print(",activeAxis="); Serial.print(axis);
  Serial.print(",expectedActive_mps2="); Serial.print(expected, 6);
  Serial.print(",activeError_pct="); Serial.print(activeErrorPct, 3);
  Serial.print(",normError_pct="); Serial.print(normErrorPct, 3);
  Serial.print(",STATUS=");
  Serial.println(storedRunCount[sIdx][orientation] == REQUIRED_REPEATS ?
                 "COMPLETE" : "PARTIAL");
}

bool getOrientationMean(int sensorNumber, int orientation,
                        double &mx, double &my, double &mz, double &mn) {
  int sIdx = sensorArrayIndex(sensorNumber);
  double sdx, sdy, sdz, sdn;
  return aggregateOrientation(sIdx, orientation,
                              mx, sdx, my, sdy, mz, sdz, mn, sdn);
}

void printCalibrationPair(int sensorNumber, char axis) {
  SensorDefinition *sensor = sensorFromNumber(sensorNumber);
  if (!sensor) return;

  int posIdx = 0;
  int negIdx = 1;
  if (axis == 'Y') { posIdx = 2; negIdx = 3; }
  if (axis == 'Z') { posIdx = 4; negIdx = 5; }

  double px, py, pz, pn;
  double nx, ny, nz, nn;

  bool havePos = getOrientationMean(sensorNumber, posIdx, px, py, pz, pn);
  bool haveNeg = getOrientationMean(sensorNumber, negIdx, nx, ny, nz, nn);

  if (!havePos || !haveNeg) {
    Serial.print("CAL_PAIR,sensor="); Serial.print(sensor->shortName);
    Serial.print(",axis="); Serial.print(axis);
    Serial.println(",STATUS=INCOMPLETE");
    return;
  }

  double posActive = (axis == 'X') ? px : ((axis == 'Y') ? py : pz);
  double negActive = (axis == 'X') ? nx : ((axis == 'Y') ? ny : nz);

  double bias = 0.5 * (posActive + negActive);
  double sensitivityRatio = (posActive - negActive) / (2.0 * GRAVITY_MS2);
  double scaleErrorPct = 100.0 * (sensitivityRatio - 1.0);
  double gainCorrection = (fabs(sensitivityRatio) > 1e-12) ?
                          (1.0 / sensitivityRatio) : NAN;

  double posCrossRss = 0.0;
  double negCrossRss = 0.0;
  if (axis == 'X') {
    posCrossRss = sqrt(py * py + pz * pz);
    negCrossRss = sqrt(ny * ny + nz * nz);
  } else if (axis == 'Y') {
    posCrossRss = sqrt(px * px + pz * pz);
    negCrossRss = sqrt(nx * nx + nz * nz);
  } else {
    posCrossRss = sqrt(px * px + py * py);
    negCrossRss = sqrt(nx * nx + ny * ny);
  }

  Serial.print("CAL_PAIR,");
  Serial.print("sensor="); Serial.print(sensor->shortName);
  Serial.print(",axis="); Serial.print(axis);
  Serial.print(",posMean_mps2="); Serial.print(posActive, 6);
  Serial.print(",negMean_mps2="); Serial.print(negActive, 6);
  Serial.print(",bias_mps2="); Serial.print(bias, 6);
  Serial.print(",sensitivityRatio="); Serial.print(sensitivityRatio, 6);
  Serial.print(",scaleError_pct="); Serial.print(scaleErrorPct, 3);
  Serial.print(",gainCorrection="); Serial.print(gainCorrection, 6);
  Serial.print(",posCrossAxisRSS_mps2="); Serial.print(posCrossRss, 6);
  Serial.print(",negCrossAxisRSS_mps2="); Serial.print(negCrossRss, 6);
  Serial.println(",STATUS=COMPLETE");
}

void printAllStoredSummaries() {
  Serial.println();
  Serial.println("===== STORED STATIC-GRAVITY SUMMARY =====");

  for (int sensorNumber = 1; sensorNumber <= 2; sensorNumber++) {
    for (int orientation = 0; orientation < 6; orientation++) {
      printOrientationSummary(sensorNumber, orientation);
    }
    printCalibrationPair(sensorNumber, 'X');
    printCalibrationPair(sensorNumber, 'Y');
    printCalibrationPair(sensorNumber, 'Z');
  }

  Serial.println("===== END SUMMARY =====");
}

// ============================================================================
// RUN ONE ORIENTATION
// ============================================================================
void runOrientationTest(int sensorNumber, int orientation) {
  SensorDefinition *sensor = sensorFromNumber(sensorNumber);
  if (!sensor || orientation < 0 || orientation >= 6) return;

  int sIdx = sensorArrayIndex(sensorNumber);

  // Rerunning an orientation deliberately replaces the old in-RAM result.
  storedRunCount[sIdx][orientation] = 0;
  for (int r = 0; r < REQUIRED_REPEATS; r++) {
    storedRuns[sIdx][orientation][r] = {};
  }

  char axis = activeAxisForOrientation(orientation);
  double expected = expectedActiveValue(orientation);

  Serial.println();
  Serial.println("============================================================");
  Serial.print("STATIC GRAVITY TEST: "); Serial.print(sensor->shortName);
  Serial.print(" / "); Serial.println(ORIENTATIONS[orientation]);
  Serial.print("Place "); Serial.print(sensor->shortName);
  Serial.print(" so +"); Serial.print(axis);
  if (expected > 0.0) {
    Serial.println(" axis points UPWARD.");
  } else {
    Serial.print(" axis points DOWNWARD (expected ");
    Serial.print(axis); Serial.println(" approximately -g).");
  }
  Serial.println("Keep the sensor completely still until all 3 valid repeats finish.");
  Serial.println("============================================================");

  uint8_t validRepeats = 0;
  uint8_t attempt = 0;

  while (validRepeats < REQUIRED_REPEATS &&
         attempt < MAX_ATTEMPTS_PER_ORIENTATION) {
    attempt++;

    Serial.print("RECORD_START,sensor="); Serial.print(sensor->shortName);
    Serial.print(",orientation="); Serial.print(ORIENTATIONS[orientation]);
    Serial.print(",targetRepeat="); Serial.print(validRepeats + 1);
    Serial.print(",attempt="); Serial.println(attempt);

    StaticRecordResult result = acquireStaticRecord(*sensor);
    printStaticRunRow(*sensor, orientation, validRepeats + 1, attempt, result);

    if (result.valid) {
      StoredRun &dest = storedRuns[sIdx][orientation][validRepeats];
      dest.valid = true;
      dest.meanX = result.meanX;
      dest.meanY = result.meanY;
      dest.meanZ = result.meanZ;
      dest.meanNorm = result.meanNorm;
      validRepeats++;
      storedRunCount[sIdx][orientation] = validRepeats;
    } else {
      Serial.println("RECORD_REJECTED,reason=integrity_or_stuck_or_clip,retrying=1");
    }

    if (validRepeats < REQUIRED_REPEATS) delay(BETWEEN_REPEAT_MS);
  }

  printOrientationSummary(sensorNumber, orientation);

  if (validRepeats == REQUIRED_REPEATS) {
    Serial.println("ORIENTATION_COMPLETE,STATUS=PASS_DATA_COLLECTION");
  } else {
    Serial.println("ORIENTATION_COMPLETE,STATUS=FAILED_TO_GET_3_VALID_REPEATS");
    Serial.println("Do not use this orientation in the thesis until the cause is fixed.");
  }
}

// ============================================================================
// USER INTERFACE
// ============================================================================
void printConfiguration() {
  Serial.println();
  Serial.println("TEST_META,project=KK_EXOSKELETON,test=01_static_gravity_validation,version=V1");
  Serial.println("TEST_META,ADXL1_role=TOOL_REFERENCE,ADXL1_CS_GPIO=5");
  Serial.println("TEST_META,ADXL2_role=HAND_ERROR,ADXL2_CS_GPIO=17");
  Serial.println("TEST_META,SCK_GPIO=18,MISO_GPIO=19,MOSI_GPIO=23");
  Serial.println("TEST_META,Visaton_DAC_GPIO=25,Visaton_used=0");
  Serial.println("TEST_META,ADXL_range=FULL_RES_+/-16g,ODR_Hz=1600,SPI_Hz=5000000,SPI_mode=3");
  Serial.print("TEST_META,record_seconds="); Serial.print(RECORD_SECONDS);
  Serial.print(",samples_per_record="); Serial.print(SAMPLES_PER_RECORD);
  Serial.print(",required_repeats="); Serial.println(REQUIRED_REPEATS);
  Serial.print("TEST_META,nominal_scale_mps2_per_LSB=");
  Serial.println(ADXL_MS2_PER_LSB, 9);
}

void printHelp() {
  Serial.println();
  Serial.println("================ TEST 01 COMMANDS ================");
  Serial.println("g 1 +X   ADXL1 +X orientation, 3 automatic repeats");
  Serial.println("g 1 -X   ADXL1 -X orientation");
  Serial.println("g 1 +Y   ADXL1 +Y orientation");
  Serial.println("g 1 -Y   ADXL1 -Y orientation");
  Serial.println("g 1 +Z   ADXL1 +Z orientation");
  Serial.println("g 1 -Z   ADXL1 -Z orientation");
  Serial.println("g 2 +X   Same six commands for ADXL2");
  Serial.println("c        Print stored summaries + calibration pairs");
  Serial.println("v        Verify ADXL1 and ADXL2 registers");
  Serial.println("p        Print test/hardware configuration");
  Serial.println("r        Clear stored in-RAM results");
  Serial.println("h        Help");
  Serial.println("==================================================");
}

void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("h")) {
    printHelp();
    return;
  }

  if (line.equalsIgnoreCase("p")) {
    printConfiguration();
    return;
  }

  if (line.equalsIgnoreCase("v")) {
    verifySensorRegisters(SENSOR1, true);
    verifySensorRegisters(SENSOR2, true);
    return;
  }

  if (line.equalsIgnoreCase("c")) {
    printAllStoredSummaries();
    return;
  }

  if (line.equalsIgnoreCase("r")) {
    clearStoredResults();
    Serial.println("RESULTS_CLEARED,STATUS=OK");
    return;
  }

  if (line.startsWith("g ") || line.startsWith("G ")) {
    int sensorNumber = 0;
    char orientationText[4] = {0};

    int parsed = sscanf(line.c_str(), "%*c %d %3s", &sensorNumber, orientationText);
    if (parsed != 2 || (sensorNumber != 1 && sensorNumber != 2)) {
      Serial.println("COMMAND_ERROR,use_example=g 1 +X");
      return;
    }

    // Normalize x/y/z to uppercase without touching +/-.
    if (orientationText[1] >= 'a' && orientationText[1] <= 'z') {
      orientationText[1] = orientationText[1] - 'a' + 'A';
    }

    int oIdx = orientationIndex(orientationText);
    if (oIdx < 0) {
      Serial.println("COMMAND_ERROR,orientation_must_be=+X|-X|+Y|-Y|+Z|-Z");
      return;
    }

    runOrientationTest(sensorNumber, oIdx);
    return;
  }

  Serial.println("COMMAND_ERROR,unknown_command=1,type_h_for_help=1");
}

// ============================================================================
// ARDUINO SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_CS_ADXL1, OUTPUT);
  pinMode(PIN_CS_ADXL2, OUTPUT);
  deselectAllSensors();

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  // Visaton is not part of Test 01. Holding DAC at midscale avoids accidental
  // waveform generation if the amplifier input happens to remain connected.
  dacWrite(PIN_VISATON_DAC, 128);

  clearStoredResults();

  Serial.println();
  Serial.println("============================================================");
  Serial.println("KK-EXOSKELETON — TEST 01 STATIC GRAVITY VALIDATION V1");
  Serial.println("ESP32 + existing dual-ADXL345 architecture");
  Serial.println("Tool OFF | Amplifier OFF | Visaton not driven");
  Serial.println("============================================================");

  bool ok1 = initializeSensor(SENSOR1);
  bool ok2 = initializeSensor(SENSOR2);

  printConfiguration();

  if (!ok1 || !ok2) {
    Serial.println("STARTUP_STATUS=FAIL");
    Serial.println("Fix sensor wiring before collecting any data.");
  } else {
    Serial.println("STARTUP_STATUS=PASS");
    Serial.println("Both ADXL345 sensors detected and configured correctly.");
  }

  printHelp();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    processCommand(line);
  }
}
