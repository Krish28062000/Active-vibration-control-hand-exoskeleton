#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <esp_arduino_version.h>

/*
  KK-EXOSKELETON DEDICATED ROBUST PHASE SWEEP V8.1
  ------------------------------------------------
  Purpose:
    Run the phase sweep without the over-strict "local/reference instability"
    gate that blocked V7.1 before any phase point was tested.

  This sketch intentionally reuses the already validated values from the log:
    Frequency = 271.436806 Hz
    DAC amplitude = 20.25
    ADXL1 reference axis = X
    ADXL2 local/control axis = X

  Hardware:
    ESP32 DAC GPIO25 -> amplifier input -> Visaton EX45S
    ADXL1 TOOL  CS GPIO5
    ADXL2 LOCAL CS GPIO17
    Shared SPI: SCK 18, MISO 19, MOSI 23

  Main command:
    w  = complete baseline -> coarse sweep -> fine sweep -> best-phase proof

  Important:
    Tool must remain ON continuously during w.
    Keep both accelerometers and the Visaton mechanically fixed.
*/

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------
static const uint8_t SPI_SCK  = 18;
static const uint8_t SPI_MISO = 19;
static const uint8_t SPI_MOSI = 23;
static const uint8_t ADXL1_CS = 5;   // Tool/reference
static const uint8_t ADXL2_CS = 17;  // Local/error sensor
static const uint8_t DAC_PIN  = 25;

static const uint8_t REG_DEVID       = 0x00;
static const uint8_t REG_BW_RATE     = 0x2C;
static const uint8_t REG_POWER_CTL   = 0x2D;
static const uint8_t REG_DATA_FORMAT = 0x31;
static const uint8_t REG_DATAX0      = 0x32;

SPISettings adxlSPI(5000000, MSBFIRST, SPI_MODE3);

// Full-resolution ADXL345 scale: approximately 3.9 mg/LSB.
static const float ADXL_SCALE_MS2 = 0.0039f * 9.80665f;

// -----------------------------------------------------------------------------
// Validated operating point from the supplied result
// These can also be changed from Serial Monitor with f and a commands.
// -----------------------------------------------------------------------------
static float operatingFrequencyHz = 271.436806f;
static float actuatorDacAmplitude = 20.25f;

// Axis: 0 = X, 1 = Y, 2 = Z
static uint8_t referenceAxis = 0;  // ADXL1 X
static uint8_t controlAxis   = 0;  // ADXL2 X

// -----------------------------------------------------------------------------
// Phase-sweep settings
// -----------------------------------------------------------------------------
static const uint32_t SAMPLE_RATE_HZ = 1600;
static const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ; // 625 us
static const uint16_t BLOCK_N = 128;  // 80 ms/block, fast enough for live phase tracking

static const uint32_t BASELINE_MS      = 4000;
static const uint32_t SETTLE_MS        = 720;
static const uint32_t COARSE_MEASURE_MS = 2400;
static const uint32_t FINE_MEASURE_MS   = 2800;
static const uint32_t BEST_PROOF_MS     = 5000;
static const uint32_t POST_BASELINE_MS  = 3500;

static const float MIN_REFERENCE_AMP = 0.15f; // m/s^2 peak
static const float TRACKING_ALPHA = 0.65f;    // circular phase smoothing

static const uint8_t MAX_WINDOW_BLOCKS = 80;

// -----------------------------------------------------------------------------
// 10 kHz hardware-timer NCO
// -----------------------------------------------------------------------------
static const uint32_t DAC_UPDATE_HZ = 10000;
static const uint32_t DAC_TIMER_US = 1000000UL / DAC_UPDATE_HZ; // 100 us

hw_timer_t *dacTimer = nullptr;
volatile uint32_t ncoPhaseAcc = 0;
volatile uint32_t ncoPhaseStep = 0;
volatile uint32_t ncoCommandPhase = 0;
volatile uint16_t ncoAmplitudeQ8 = 0;
volatile bool ncoOutputEnabled = false;

int8_t sineTable[256];

// -----------------------------------------------------------------------------
// Data structures
// -----------------------------------------------------------------------------
struct RawAccel {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct BlockResult {
  float refAmp;
  float refPhaseDeg;
  float localAmp[3];
  float localTotal;
  bool validReference;
};

struct WindowResult {
  bool valid;
  uint16_t validBlocks;
  float refMedian;
  float controlMedian;
  float xMedian;
  float yMedian;
  float zMedian;
  float totalMedian;
  float controlMadPct;
  float controlCvPct;
};

struct SweepPoint {
  float phaseDeg;
  WindowResult measurement;
};

// Static capture buffers keep stack use low and predictable.
static RawAccel refRaw[BLOCK_N];
static RawAccel localRaw[BLOCK_N];
static uint32_t refNcoPhase[BLOCK_N];
static uint32_t localNcoPhase[BLOCK_N];

// Live tracking state
static bool trackingInitialized = false;
static float trackedReferencePhaseDeg = 0.0f;
static float activeRelativePhaseDeg = 0.0f;
static bool continuousHoldActive = false;
static uint32_t lastHoldPrintMs = 0;

// -----------------------------------------------------------------------------
// Utility functions
// -----------------------------------------------------------------------------
static float wrap360(float value) {
  while (value >= 360.0f) value -= 360.0f;
  while (value < 0.0f) value += 360.0f;
  return value;
}

static float wrap180(float value) {
  while (value > 180.0f) value -= 360.0f;
  while (value <= -180.0f) value += 360.0f;
  return value;
}

static uint32_t degreesToPhaseWord(float degrees) {
  const double normalized = (double)wrap360(degrees) / 360.0;
  return (uint32_t)(normalized * 4294967296.0); // 2^32
}

static const char *axisName(uint8_t axis) {
  if (axis == 0) return "X";
  if (axis == 1) return "Y";
  return "Z";
}

static int16_t axisRaw(const RawAccel &a, uint8_t axis) {
  if (axis == 0) return a.x;
  if (axis == 1) return a.y;
  return a.z;
}

static void sortFloat(float *values, uint16_t count) {
  for (uint16_t i = 1; i < count; ++i) {
    float key = values[i];
    int j = (int)i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = key;
  }
}

static float medianOf(const float *values, uint16_t count) {
  if (count == 0) return NAN;
  float copy[MAX_WINDOW_BLOCKS];
  if (count > MAX_WINDOW_BLOCKS) count = MAX_WINDOW_BLOCKS;
  for (uint16_t i = 0; i < count; ++i) copy[i] = values[i];
  sortFloat(copy, count);
  if (count & 1U) return copy[count / 2];
  return 0.5f * (copy[count / 2 - 1] + copy[count / 2]);
}

static float madPercent(const float *values, uint16_t count, float median) {
  if (count == 0 || !isfinite(median) || fabsf(median) < 1e-6f) return NAN;
  float deviations[MAX_WINDOW_BLOCKS];
  if (count > MAX_WINDOW_BLOCKS) count = MAX_WINDOW_BLOCKS;
  for (uint16_t i = 0; i < count; ++i) deviations[i] = fabsf(values[i] - median);
  return 100.0f * medianOf(deviations, count) / fabsf(median);
}

static float cvPercent(const float *values, uint16_t count) {
  if (count < 2) return NAN;
  double sum = 0.0;
  for (uint16_t i = 0; i < count; ++i) sum += values[i];
  const double mean = sum / count;
  if (fabs(mean) < 1e-9) return NAN;
  double ss = 0.0;
  for (uint16_t i = 0; i < count; ++i) {
    const double d = values[i] - mean;
    ss += d * d;
  }
  const double sd = sqrt(ss / (count - 1));
  return (float)(100.0 * sd / fabs(mean));
}

// -----------------------------------------------------------------------------
// ADXL345 SPI
// -----------------------------------------------------------------------------
static void writeRegister(uint8_t csPin, uint8_t reg, uint8_t value) {
  SPI.beginTransaction(adxlSPI);
  digitalWrite(csPin, LOW);
  SPI.transfer(reg & 0x3F);
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

static uint8_t readRegister(uint8_t csPin, uint8_t reg) {
  SPI.beginTransaction(adxlSPI);
  digitalWrite(csPin, LOW);
  SPI.transfer(0x80 | (reg & 0x3F));
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return value;
}

static RawAccel readRawXYZ(uint8_t csPin) {
  RawAccel data = {};
  SPI.beginTransaction(adxlSPI);
  digitalWrite(csPin, LOW);
  SPI.transfer(0x80 | 0x40 | REG_DATAX0);
  uint8_t x0 = SPI.transfer(0x00);
  uint8_t x1 = SPI.transfer(0x00);
  uint8_t y0 = SPI.transfer(0x00);
  uint8_t y1 = SPI.transfer(0x00);
  uint8_t z0 = SPI.transfer(0x00);
  uint8_t z1 = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  data.x = (int16_t)((x1 << 8) | x0);
  data.y = (int16_t)((y1 << 8) | y0);
  data.z = (int16_t)((z1 << 8) | z0);
  return data;
}

static bool initializeADXL(uint8_t csPin, const char *name) {
  writeRegister(csPin, REG_POWER_CTL, 0x00);
  delay(10);
  writeRegister(csPin, REG_DATA_FORMAT, 0x0B); // Full resolution, +/-16 g
  writeRegister(csPin, REG_BW_RATE, 0x0E);     // Nominal 1600 Hz ODR
  writeRegister(csPin, REG_POWER_CTL, 0x08);   // Measurement mode
  delay(30);

  const uint8_t id = readRegister(csPin, REG_DEVID);
  const uint8_t fmt = readRegister(csPin, REG_DATA_FORMAT);
  const uint8_t rate = readRegister(csPin, REG_BW_RATE);
  const uint8_t pwr = readRegister(csPin, REG_POWER_CTL);

  Serial.printf("%s DEVID 0x%02X | FORMAT 0x%02X | RATE 0x%02X | POWER 0x%02X\n",
                name, id, fmt, rate, pwr);

  const bool ok = id == 0xE5 && (fmt & 0x0F) == 0x0B &&
                  (rate & 0x1F) == 0x0E && pwr == 0x08;
  Serial.printf("%s STATUS: %s\n", name, ok ? "PASS" : "FAIL");
  return ok;
}

// -----------------------------------------------------------------------------
// NCO and DAC
// -----------------------------------------------------------------------------
void ARDUINO_ISR_ATTR onDacTimer() {
  // Keep the virtual oscillator running even when DAC output is disabled.
  const uint32_t phase = ncoPhaseAcc + ncoCommandPhase;
  ncoPhaseAcc += ncoPhaseStep;

  if (!ncoOutputEnabled) {
    dacWrite(DAC_PIN, 128);
    return;
  }

  const uint8_t index = (uint8_t)(phase >> 24);
  const int32_t scaled = ((int32_t)sineTable[index] * (int32_t)ncoAmplitudeQ8) /
                         (127L * 256L);
  int32_t output = 128 + scaled;
  if (output < 0) output = 0;
  if (output > 255) output = 255;
  dacWrite(DAC_PIN, (uint8_t)output);
}

static void applyNcoSettings() {
  if (operatingFrequencyHz < 1.0f) operatingFrequencyHz = 1.0f;
  if (operatingFrequencyHz > 1000.0f) operatingFrequencyHz = 1000.0f;
  if (actuatorDacAmplitude < 0.0f) actuatorDacAmplitude = 0.0f;
  if (actuatorDacAmplitude > 120.0f) actuatorDacAmplitude = 120.0f;

  ncoPhaseStep = (uint32_t)((double)operatingFrequencyHz * 4294967296.0 /
                            (double)DAC_UPDATE_HZ);
  ncoAmplitudeQ8 = (uint16_t)lroundf(actuatorDacAmplitude * 256.0f);
}

static void setOutputEnabled(bool enabled) {
  ncoOutputEnabled = enabled;
  if (!enabled) dacWrite(DAC_PIN, 128);
}

static void setRelativePhase(float phaseDeg) {
  activeRelativePhaseDeg = wrap360(phaseDeg);
  const float commandDeg = trackedReferencePhaseDeg + activeRelativePhaseDeg;
  ncoCommandPhase = degreesToPhaseWord(commandDeg);
}

static void resetTracking() {
  trackingInitialized = false;
  trackedReferencePhaseDeg = 0.0f;
}

static void updateReferenceTracking(float measuredReferencePhaseDeg,
                                    float desiredRelativePhaseDeg) {
  if (!trackingInitialized) {
    trackedReferencePhaseDeg = wrap360(measuredReferencePhaseDeg);
    trackingInitialized = true;
  } else {
    const float error = wrap180(measuredReferencePhaseDeg - trackedReferencePhaseDeg);
    trackedReferencePhaseDeg = wrap360(trackedReferencePhaseDeg + TRACKING_ALPHA * error);
  }
  activeRelativePhaseDeg = wrap360(desiredRelativePhaseDeg);
  ncoCommandPhase = degreesToPhaseWord(trackedReferencePhaseDeg + activeRelativePhaseDeg);
}

// -----------------------------------------------------------------------------
// Synchronous quadrature block measurement
// -----------------------------------------------------------------------------
static void projectSignal(const int16_t *raw, const uint32_t *phaseWords,
                          uint16_t count, float &amplitude, float &phaseDeg) {
  double mean = 0.0;
  for (uint16_t i = 0; i < count; ++i) mean += raw[i];
  mean /= count;

  double sumSin = 0.0;
  double sumCos = 0.0;
  for (uint16_t i = 0; i < count; ++i) {
    const double theta = ((double)phaseWords[i] / 4294967296.0) * TWO_PI;
    const double value = ((double)raw[i] - mean) * ADXL_SCALE_MS2;
    sumSin += value * sin(theta);
    sumCos += value * cos(theta);
  }

  amplitude = (float)((2.0 / count) * sqrt(sumSin * sumSin + sumCos * sumCos));
  phaseDeg = (float)(atan2(sumCos, sumSin) * 180.0 / PI);
  phaseDeg = wrap360(phaseDeg);
}

static BlockResult captureBlock() {
  BlockResult result = {};
  uint32_t nextSampleUs = micros();

  for (uint16_t i = 0; i < BLOCK_N; ++i) {
    nextSampleUs += SAMPLE_PERIOD_US;
    while ((int32_t)(micros() - nextSampleUs) < 0) {
      delayMicroseconds(15);
    }

    refNcoPhase[i] = ncoPhaseAcc;
    refRaw[i] = readRawXYZ(ADXL1_CS);

    localNcoPhase[i] = ncoPhaseAcc;
    localRaw[i] = readRawXYZ(ADXL2_CS);
  }

  int16_t refAxisData[BLOCK_N];
  int16_t localAxisData[3][BLOCK_N];

  for (uint16_t i = 0; i < BLOCK_N; ++i) {
    refAxisData[i] = axisRaw(refRaw[i], referenceAxis);
    localAxisData[0][i] = localRaw[i].x;
    localAxisData[1][i] = localRaw[i].y;
    localAxisData[2][i] = localRaw[i].z;
  }

  float ignoredPhase = 0.0f;
  projectSignal(refAxisData, refNcoPhase, BLOCK_N,
                result.refAmp, result.refPhaseDeg);

  for (uint8_t axis = 0; axis < 3; ++axis) {
    projectSignal(localAxisData[axis], localNcoPhase, BLOCK_N,
                  result.localAmp[axis], ignoredPhase);
  }

  result.localTotal = sqrtf(result.localAmp[0] * result.localAmp[0] +
                            result.localAmp[1] * result.localAmp[1] +
                            result.localAmp[2] * result.localAmp[2]);
  result.validReference = isfinite(result.refAmp) &&
                          result.refAmp >= MIN_REFERENCE_AMP;
  return result;
}

static WindowResult measureWindow(uint32_t durationMs,
                                  bool trackReference,
                                  float desiredRelativePhaseDeg,
                                  bool printProgress) {
  WindowResult out = {};
  float refValues[MAX_WINDOW_BLOCKS];
  float xValues[MAX_WINDOW_BLOCKS];
  float yValues[MAX_WINDOW_BLOCKS];
  float zValues[MAX_WINDOW_BLOCKS];
  float totalValues[MAX_WINDOW_BLOCKS];
  float controlValues[MAX_WINDOW_BLOCKS];

  const uint32_t blockDurationMs = (1000UL * BLOCK_N) / SAMPLE_RATE_HZ;
  uint16_t requestedBlocks = (uint16_t)(durationMs / blockDurationMs);
  if (requestedBlocks < 4) requestedBlocks = 4;
  if (requestedBlocks > MAX_WINDOW_BLOCKS) requestedBlocks = MAX_WINDOW_BLOCKS;

  uint16_t validCount = 0;
  for (uint16_t block = 0; block < requestedBlocks; ++block) {
    BlockResult b = captureBlock();

    if (b.validReference) {
      if (trackReference) {
        updateReferenceTracking(b.refPhaseDeg, desiredRelativePhaseDeg);
      }

      if (validCount < MAX_WINDOW_BLOCKS) {
        refValues[validCount] = b.refAmp;
        xValues[validCount] = b.localAmp[0];
        yValues[validCount] = b.localAmp[1];
        zValues[validCount] = b.localAmp[2];
        totalValues[validCount] = b.localTotal;
        controlValues[validCount] = b.localAmp[controlAxis];
        ++validCount;
      }
    }

    if (printProgress && ((block + 1) % 10 == 0 || block + 1 == requestedBlocks)) {
      Serial.printf("  blocks %u/%u | valid %u\n",
                    block + 1, requestedBlocks, validCount);
    }
  }

  out.validBlocks = validCount;
  const uint16_t minimumValid = max((uint16_t)5, (uint16_t)(requestedBlocks / 3));
  out.valid = validCount >= minimumValid;

  if (validCount > 0) {
    out.refMedian = medianOf(refValues, validCount);
    out.xMedian = medianOf(xValues, validCount);
    out.yMedian = medianOf(yValues, validCount);
    out.zMedian = medianOf(zValues, validCount);
    out.totalMedian = medianOf(totalValues, validCount);
    out.controlMedian = medianOf(controlValues, validCount);
    out.controlMadPct = madPercent(controlValues, validCount, out.controlMedian);
    out.controlCvPct = cvPercent(controlValues, validCount);
  } else {
    out.refMedian = NAN;
    out.xMedian = NAN;
    out.yMedian = NAN;
    out.zMedian = NAN;
    out.totalMedian = NAN;
    out.controlMedian = NAN;
    out.controlMadPct = NAN;
    out.controlCvPct = NAN;
  }

  return out;
}

static void printWindow(const char *label, const WindowResult &w) {
  Serial.printf("%s | valid blocks %u | REF %s %.4f | LOCAL X/Y/Z/T %.4f / %.4f / %.4f / %.4f | CTRL %s %.4f | MAD %.2f%% | CV %.2f%% | %s\n",
                label,
                w.validBlocks,
                axisName(referenceAxis), w.refMedian,
                w.xMedian, w.yMedian, w.zMedian, w.totalMedian,
                axisName(controlAxis), w.controlMedian,
                w.controlMadPct, w.controlCvPct,
                w.valid ? "ACCEPTED" : "INSUFFICIENT REFERENCE");
}

// -----------------------------------------------------------------------------
// Sweep logic
// -----------------------------------------------------------------------------
static int findBestPoint(const SweepPoint *points, uint8_t count) {
  int best = -1;
  float bestValue = INFINITY;
  for (uint8_t i = 0; i < count; ++i) {
    if (points[i].measurement.valid &&
        isfinite(points[i].measurement.controlMedian) &&
        points[i].measurement.controlMedian < bestValue) {
      bestValue = points[i].measurement.controlMedian;
      best = i;
    }
  }
  return best;
}

static int findBestTotalPoint(const SweepPoint *points, uint8_t count) {
  int best = -1;
  float bestValue = INFINITY;
  for (uint8_t i = 0; i < count; ++i) {
    if (points[i].measurement.valid &&
        isfinite(points[i].measurement.totalMedian) &&
        points[i].measurement.totalMedian < bestValue) {
      bestValue = points[i].measurement.totalMedian;
      best = i;
    }
  }
  return best;
}

static WindowResult measurePhasePoint(float phaseDeg, uint32_t measureMs,
                                      const char *stageName) {
  setRelativePhase(phaseDeg);

  // Settling interval is still tracked, but not used for scoring.
  (void)measureWindow(SETTLE_MS, true, phaseDeg, false);

  WindowResult result = measureWindow(measureMs, true, phaseDeg, false);
  char label[80];
  snprintf(label, sizeof(label), "%s PHASE %6.1f deg", stageName, wrap360(phaseDeg));
  printWindow(label, result);
  return result;
}

static bool runBaselineOnly() {
  Serial.println();
  Serial.println("====================================================================");
  Serial.println("TOOL-ONLY BASELINE CHECK");
  Serial.println("Tool ON | Visaton OFF | no strict phase/coherence rejection");
  Serial.println("====================================================================");

  continuousHoldActive = false;
  setOutputEnabled(false);
  resetTracking();

  WindowResult baseline = measureWindow(BASELINE_MS, true, 0.0f, true);
  printWindow("BASELINE", baseline);

  if (!baseline.valid) {
    Serial.println("BASELINE FAILED ONLY BECAUSE ADXL1 REFERENCE AMPLITUDE WAS TOO LOW.");
    Serial.println("Check ADXL1 mounting and reference-axis selection; the local signal itself is not called unstable.");
    return false;
  }

  Serial.println("BASELINE ACCEPTED. You may now enter w for the complete sweep.");
  return true;
}

static void runFullPhaseSweep() {
  Serial.println();
  Serial.println("====================================================================");
  Serial.println("DEDICATED ROBUST LIVE-REFERENCED PHASE SWEEP V8");
  Serial.printf("Frequency %.6f Hz | DAC %.2f | REF ADXL1 %s | CTRL ADXL2 %s\n",
                operatingFrequencyHz, actuatorDacAmplitude,
                axisName(referenceAxis), axisName(controlAxis));
  Serial.println("Tool must remain ON continuously. Do not move sensors or actuator.");
  Serial.println("This program continues through noisy points instead of aborting the whole sweep.");
  Serial.println("====================================================================");

  continuousHoldActive = false;
  setOutputEnabled(false);
  resetTracking();

  Serial.println("\n1) PRE-SWEEP TOOL-ONLY BASELINE");
  WindowResult preBaseline = measureWindow(BASELINE_MS, true, 0.0f, true);
  printWindow("PRE BASELINE", preBaseline);

  if (!preBaseline.valid) {
    Serial.println("SWEEP STOPPED: ADXL1 reference amplitude is below the minimum threshold.");
    Serial.println("This is a genuine missing-reference condition, not the old generic instability gate.");
    return;
  }

  Serial.println("\n2) COARSE SWEEP: 0 to 330 deg in 30 deg steps");
  SweepPoint coarse[12];
  setOutputEnabled(true);

  for (uint8_t i = 0; i < 12; ++i) {
    coarse[i].phaseDeg = 30.0f * i;
    coarse[i].measurement = measurePhasePoint(coarse[i].phaseDeg,
                                               COARSE_MEASURE_MS,
                                               "COARSE");
  }

  const int bestCoarse = findBestPoint(coarse, 12);
  const int bestCoarseTotal = findBestTotalPoint(coarse, 12);
  if (bestCoarse < 0) {
    setOutputEnabled(false);
    Serial.println("NO COARSE POINT HAD ENOUGH ADXL1 REFERENCE BLOCKS.");
    return;
  }

  Serial.printf("BEST COARSE CONTROL-AXIS PHASE: %.1f deg | %s %.4f m/s^2\n",
                coarse[bestCoarse].phaseDeg, axisName(controlAxis),
                coarse[bestCoarse].measurement.controlMedian);
  if (bestCoarseTotal >= 0) {
    Serial.printf("BEST COARSE TOTAL PHASE: %.1f deg | total %.4f m/s^2\n",
                  coarse[bestCoarseTotal].phaseDeg,
                  coarse[bestCoarseTotal].measurement.totalMedian);
  }

  Serial.println("\n3) FINE SWEEP: best coarse phase +/-30 deg in 10 deg steps");
  SweepPoint fine[7];
  for (uint8_t i = 0; i < 7; ++i) {
    fine[i].phaseDeg = wrap360(coarse[bestCoarse].phaseDeg - 30.0f + 10.0f * i);
    fine[i].measurement = measurePhasePoint(fine[i].phaseDeg,
                                            FINE_MEASURE_MS,
                                            "FINE  ");
  }

  const int bestFine = findBestPoint(fine, 7);
  const int bestFineTotal = findBestTotalPoint(fine, 7);
  if (bestFine < 0) {
    setOutputEnabled(false);
    Serial.println("NO FINE POINT HAD ENOUGH ADXL1 REFERENCE BLOCKS.");
    return;
  }

  const float bestPhase = fine[bestFine].phaseDeg;

  Serial.println("\n4) BEST-PHASE CONFIRMATION");
  WindowResult bestProof = measurePhasePoint(bestPhase, BEST_PROOF_MS, "BEST  ");

  Serial.println("\n5) POST-SWEEP TOOL-ONLY BASELINE");
  setOutputEnabled(false);
  WindowResult postBaseline = measureWindow(POST_BASELINE_MS, true, 0.0f, true);
  printWindow("POST BASELINE", postBaseline);

  float baselineForReduction = preBaseline.controlMedian;
  if (postBaseline.valid) {
    baselineForReduction = 0.5f * (preBaseline.controlMedian + postBaseline.controlMedian);
  }

  const float reductionPct = 100.0f *
      (baselineForReduction - bestProof.controlMedian) / baselineForReduction;
  const float prePostDriftPct = postBaseline.valid ?
      100.0f * fabsf(postBaseline.controlMedian - preBaseline.controlMedian) /
      preBaseline.controlMedian : NAN;

  Serial.println();
  Serial.println("====================================================================");
  Serial.println("PHASE SWEEP COMPLETE");
  Serial.println("====================================================================");
  Serial.printf("BEST CONTROL-AXIS PHASE: %.1f deg\n", bestPhase);
  Serial.printf("Baseline used: %.5f m/s^2 peak on ADXL2 %s\n",
                baselineForReduction, axisName(controlAxis));
  Serial.printf("Best-phase result: %.5f m/s^2 peak on ADXL2 %s\n",
                bestProof.controlMedian, axisName(controlAxis));
  Serial.printf("CONTROL-AXIS REDUCTION: %.2f %%\n", reductionPct);
  Serial.printf("Best-phase X/Y/Z/TOTAL: %.5f / %.5f / %.5f / %.5f m/s^2\n",
                bestProof.xMedian, bestProof.yMedian,
                bestProof.zMedian, bestProof.totalMedian);
  if (postBaseline.valid) {
    Serial.printf("Pre/post tool-only baseline drift: %.2f %%\n", prePostDriftPct);
    if (prePostDriftPct > 15.0f) {
      Serial.println("WARNING: tool vibration level changed by more than 15%; repeat once with firmer mounting.");
    }
  }
  if (bestFineTotal >= 0) {
    Serial.printf("FINE-SWEEP MINIMUM-TOTAL PHASE: %.1f deg | total %.5f m/s^2\n",
                  fine[bestFineTotal].phaseDeg,
                  fine[bestFineTotal].measurement.totalMedian);
  }
  if (bestProof.controlCvPct > 12.0f) {
    Serial.println("WARNING: best-phase amplitude varied considerably, but the sweep was completed and not discarded.");
  }
  if (reductionPct <= 0.0f) {
    Serial.println("HONEST RESULT: no control-axis attenuation was demonstrated at this actuator amplitude/mounting.");
  } else {
    Serial.println("ATTENUATION DEMONSTRATED AT THE REPORTED BEST PHASE.");
  }
  Serial.printf("To live-hold this phase with tracking, enter: g %.1f\n", bestPhase);
  Serial.println("Visaton is OFF after the proof measurement.");
  Serial.println("====================================================================");
}

// -----------------------------------------------------------------------------
// Continuous best-phase hold service
// -----------------------------------------------------------------------------
static void serviceContinuousHold() {
  if (!continuousHoldActive) return;

  BlockResult b = captureBlock();
  if (b.validReference) {
    updateReferenceTracking(b.refPhaseDeg, activeRelativePhaseDeg);
  }

  if (millis() - lastHoldPrintMs >= 1000) {
    lastHoldPrintMs = millis();
    Serial.printf("HOLD phase %.1f deg | REF %s %.4f | LOCAL X/Y/Z/T %.4f / %.4f / %.4f / %.4f | CTRL %s %.4f\n",
                  activeRelativePhaseDeg,
                  axisName(referenceAxis), b.refAmp,
                  b.localAmp[0], b.localAmp[1], b.localAmp[2], b.localTotal,
                  axisName(controlAxis), b.localAmp[controlAxis]);
  }
}

// -----------------------------------------------------------------------------
// Serial commands
// -----------------------------------------------------------------------------
static void printHelp() {
  Serial.println();
  Serial.println("====================================================================");
  Serial.println("KK-EXOSKELETON DEDICATED ROBUST PHASE SWEEP V8.1");
  Serial.println("====================================================================");
  Serial.println("w          Tool ON: run complete baseline + coarse + fine phase sweep");
  Serial.println("b          Tool ON, Visaton OFF: baseline/reference check only");
  Serial.println("g 150      Tool ON: live-hold 150 deg relative phase with tracking");
  Serial.println("s          stop Visaton and live hold");
  Serial.println("f 271.44   set operating frequency in Hz; output must be stopped");
  Serial.println("a 20.25    set actuator DAC amplitude; output must be stopped");
  Serial.println("r x        select ADXL1 reference axis x/y/z");
  Serial.println("c x        select ADXL2 control axis x/y/z");
  Serial.println("p          print current settings");
  Serial.println("h          print help");
  Serial.println();
  Serial.println("Immediate procedure for the supplied result:");
  Serial.println("1. Keep frequency 271.436806 Hz and DAC 20.25.");
  Serial.println("2. Tool ON continuously; Visaton initially OFF.");
  Serial.println("3. Enter b. If accepted, enter w.");
  Serial.println("====================================================================");
}

static void printState() {
  Serial.println();
  Serial.println("================ CURRENT SETTINGS ================");
  Serial.printf("Frequency: %.6f Hz\n", operatingFrequencyHz);
  Serial.printf("DAC amplitude: %.2f\n", actuatorDacAmplitude);
  Serial.printf("ADXL1 reference axis: %s\n", axisName(referenceAxis));
  Serial.printf("ADXL2 control axis: %s\n", axisName(controlAxis));
  Serial.printf("Visaton output: %s\n", ncoOutputEnabled ? "ON" : "OFF");
  Serial.printf("Continuous hold: %s", continuousHoldActive ? "ON" : "OFF");
  if (continuousHoldActive) Serial.printf(" at %.1f deg", activeRelativePhaseDeg);
  Serial.println();
  Serial.println("==================================================");
}

static int axisFromChar(char value) {
  if (value == 'x' || value == 'X') return 0;
  if (value == 'y' || value == 'Y') return 1;
  if (value == 'z' || value == 'Z') return 2;
  return -1;
}

static void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  char command = line.charAt(0);
  command = (char)tolower(command);

  if (command == 'h') {
    printHelp();
    return;
  }
  if (command == 'p') {
    printState();
    return;
  }
  if (command == 's') {
    continuousHoldActive = false;
    setOutputEnabled(false);
    Serial.println("VISATON OFF. Tracking hold stopped.");
    return;
  }
  if (command == 'b') {
    runBaselineOnly();
    return;
  }
  if (command == 'w') {
    runFullPhaseSweep();
    return;
  }

  if (command == 'f') {
    if (ncoOutputEnabled) {
      Serial.println("Stop output with s before changing frequency.");
      return;
    }
    float value = line.substring(1).toFloat();
    if (value < 50.0f || value > 500.0f) {
      Serial.println("Frequency must be between 50 and 500 Hz for this test.");
      return;
    }
    operatingFrequencyHz = value;
    applyNcoSettings();
    resetTracking();
    Serial.printf("Frequency set to %.6f Hz\n", operatingFrequencyHz);
    return;
  }

  if (command == 'a') {
    if (ncoOutputEnabled) {
      Serial.println("Stop output with s before changing DAC amplitude.");
      return;
    }
    float value = line.substring(1).toFloat();
    if (value < 0.25f || value > 120.0f) {
      Serial.println("DAC amplitude must be between 0.25 and 120.00.");
      return;
    }
    actuatorDacAmplitude = value;
    applyNcoSettings();
    Serial.printf("DAC amplitude set to %.2f\n", actuatorDacAmplitude);
    return;
  }

  if (command == 'r' || command == 'c') {
    if (line.length() < 2) {
      Serial.println("Use r x/y/z or c x/y/z.");
      return;
    }
    char axisChar = 0;
    for (uint16_t i = 1; i < line.length(); ++i) {
      const char ch = line.charAt(i);
      if (ch == 'x' || ch == 'X' || ch == 'y' || ch == 'Y' ||
          ch == 'z' || ch == 'Z') {
        axisChar = ch;
        break;
      }
    }
    int axis = axisFromChar(axisChar);
    if (axis < 0) {
      Serial.println("Axis must be x, y, or z.");
      return;
    }
    if (command == 'r') {
      referenceAxis = (uint8_t)axis;
      Serial.printf("ADXL1 reference axis set to %s\n", axisName(referenceAxis));
    } else {
      controlAxis = (uint8_t)axis;
      Serial.printf("ADXL2 control axis set to %s\n", axisName(controlAxis));
    }
    resetTracking();
    return;
  }

  if (command == 'g') {
    float phase = line.substring(1).toFloat();
    resetTracking();
    activeRelativePhaseDeg = wrap360(phase);

    Serial.printf("Acquiring live ADXL1 %s reference before holding %.1f deg...\n",
                  axisName(referenceAxis), activeRelativePhaseDeg);
    WindowResult acquisition = measureWindow(1000, true,
                                              activeRelativePhaseDeg, false);
    if (!acquisition.valid) {
      Serial.println("Cannot start hold: ADXL1 reference amplitude is too low.");
      setOutputEnabled(false);
      continuousHoldActive = false;
      return;
    }

    setRelativePhase(activeRelativePhaseDeg);
    setOutputEnabled(true);
    continuousHoldActive = true;
    lastHoldPrintMs = millis();
    Serial.printf("LIVE HOLD ACTIVE AT %.1f DEG. Enter s to stop.\n",
                  activeRelativePhaseDeg);
    return;
  }

  Serial.println("Unknown command. Enter h for help.");
}

// -----------------------------------------------------------------------------
// Arduino setup/loop
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1200);

  pinMode(ADXL1_CS, OUTPUT);
  pinMode(ADXL2_CS, OUTPUT);
  digitalWrite(ADXL1_CS, HIGH);
  digitalWrite(ADXL2_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  pinMode(DAC_PIN, OUTPUT);
  dacWrite(DAC_PIN, 128);

  for (uint16_t i = 0; i < 256; ++i) {
    sineTable[i] = (int8_t)lroundf(127.0f * sinf(TWO_PI * i / 256.0f));
  }

  applyNcoSettings();

  // ESP32 Arduino timer API compatibility.
  // Core 3.x: timerBegin(frequency), timerAttachInterrupt(timer, ISR),
  //             timerAlarm(timer, ticks, autoreload, reload_count).
  // Core 2.x: legacy timer number/prescaler/count-up API.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  dacTimer = timerBegin(1000000UL);  // 1 MHz timer tick: 1 tick = 1 us
  if (dacTimer != nullptr) {
    timerAttachInterrupt(dacTimer, &onDacTimer);
    timerAlarm(dacTimer, DAC_TIMER_US, true, 0); // 100 us, repeat forever
  }
#else
  dacTimer = timerBegin(0, 80, true); // APB 80 MHz / 80 = 1 MHz
  if (dacTimer != nullptr) {
    timerAttachInterrupt(dacTimer, &onDacTimer, true);
    timerAlarmWrite(dacTimer, DAC_TIMER_US, true);
    timerAlarmEnable(dacTimer);
  }
#endif

  if (dacTimer == nullptr) {
    Serial.println("FATAL: DAC hardware timer could not be created.");
    Serial.println("The Visaton output is disabled. Check the ESP32 board package.");
    setOutputEnabled(false);
    while (true) delay(1000);
  }

  Serial.printf("ESP32 Arduino core: %d.%d.%d\n",
                ESP_ARDUINO_VERSION_MAJOR,
                ESP_ARDUINO_VERSION_MINOR,
                ESP_ARDUINO_VERSION_PATCH);
  Serial.println("DAC TIMER READY: 1 MHz timer, 100 us auto-reload, 10 kHz NCO update.");

  Serial.println();
  Serial.println("====================================================================");
  Serial.println("KK-EXOSKELETON DEDICATED ROBUST PHASE SWEEP V8.1");
  Serial.println("OLD GENERIC INSTABILITY ABORT REMOVED");
  Serial.println("====================================================================");

  bool adxl1Ok = initializeADXL(ADXL1_CS, "ADXL1 TOOL GPIO5");
  bool adxl2Ok = initializeADXL(ADXL2_CS, "ADXL2 LOCAL GPIO17");

  if (!adxl1Ok || !adxl2Ok) {
    Serial.println("STOP: one or both ADXL345 register checks failed.");
  } else {
    Serial.println("BOTH ADXL345 SENSORS READY.");
  }

  printHelp();
  printState();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    processCommand(line);
  }

  if (continuousHoldActive) {
    serviceContinuousHold();
  } else {
    delay(5);
  }
}
