#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <esp_arduino_version.h>

/*
  KK-EXOSKELETON SINGLE-ADXL PHASE SWEEP V11.2 RECOVERY + SAFE FALLBACK
  =================================================

  SIMPLE TEST SEQUENCE
  --------------------
  1) b  TOOL ON, VISATON OFF
        - ADXL: frozen 3-axis FFT (1024 samples, 1600 Hz, Hamming, DC removal)
        - harmonic-safe operating-frequency selection
        - two-pass 2560-sample quadrature refinement
        - ADXL: local X/Y/Z amplitude measured at the final frequency
        - ADXL X is stored as the actuator amplitude target

  2) m  TOOL OFF
        - same exact-frequency NCO-synchronous amplitude matcher
        - adaptive DAC search, bracketing/interpolation, 0.25-DAC resolution
        - two-of-three confirmation and one bounded trim
        - accepted DAC is stored; Visaton is switched OFF afterward

  3) w  TOOL ON, VISATON OFF INITIALLY
        - ADXL phase is reacquired briefly with Visaton OFF before each phase point
        - NO continuous live phase tracking while a phase point is measured
        - ADXL tool-only baseline
        - 30-degree coarse sweep, then 10-degree fine sweep around best X phase
        - best-phase confirmation and post baseline
        - if X attenuation is demonstrated, a periodically re-locked hold starts
        - ADXL X/Y/Z/total checked every 1 second

  4) s  Stop Visaton/hold

  HARDWARE
  --------
  ESP32 DAC GPIO25 -> amplifier -> Visaton EX45S
  ONE ADXL345 sensor: CS GPIO17
  Shared SPI: SCK 18, MISO 19, MOSI 23

  IMPORTANT PHYSICAL ROLES
  ------------------------
  The single ADXL345 is fixed at the exact tool/Visaton test point. It performs
  FFT, quadrature, local amplitude capture, actuator matching, phase anchoring,
  sweep scoring, and hold verification. X is the control objective; Y, Z and
  total are always reported diagnostically.

  This version does not continuously track phase. Instead, it briefly switches
  the Visaton OFF and reacquires the ADXL phase before each coarse/fine point
  and before each 1-second hold check. During every measured point, the phase
  is fixed open-loop. This prevents accumulated phase drift without implementing
  continuous live tracking.

  SINGLE-SENSOR VALIDITY
  ----------------------
  The sensor measures the tool alone during b, the Visaton alone during m, and
  their vector sum during w. Before every sweep point the Visaton is switched
  OFF briefly so the same sensor can measure the current tool phase, after which
  the requested actuator phase is applied and held open-loop for that short point.
  This is a controlled proof-of-attenuation test at one physical location.
*/

// =============================================================================
// Hardware and ADXL345 configuration
// =============================================================================
static const uint8_t SPI_SCK  = 18;
static const uint8_t SPI_MISO = 19;
static const uint8_t SPI_MOSI = 23;
static const uint8_t ADXL_CS = 17;  // Single sensor: FFT, quadrature, matcher, sweep and hold
static const uint8_t DAC_PIN  = 25;

static const uint8_t REG_DEVID       = 0x00;
static const uint8_t REG_BW_RATE     = 0x2C;
static const uint8_t REG_POWER_CTL   = 0x2D;
static const uint8_t REG_DATA_FORMAT = 0x31;
static const uint8_t REG_DATAX0      = 0x32;

SPISettings adxlSPI(1000000, MSBFIRST, SPI_MODE3); // Lowered from 5 MHz for noise margin

// Full-resolution ADXL345: approximately 3.9 mg/LSB.
static const float ADXL_SCALE_MS2 = 0.0039f * 9.80665f;

// =============================================================================
// Frozen sensing settings
// =============================================================================
static const uint32_t SAMPLE_RATE_HZ = 1600;
static const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ; // 625 us

static const uint16_t FFT_N = 1024;
static const uint8_t FFT_FRAMES = 3;
static const float FFT_MIN_HZ = 5.0f;
static const float FFT_MAX_HZ = 750.0f;
static const float FFT_MIN_AXIS_AMP = 0.050f;
static const float FFT_MIN_AXIS_SNR_DB = 6.0f;

static const uint16_t QUAD_N = 2560;       // 1.6 s at 1600 Hz
static const uint16_t QUAD_SEGMENT_N = 256;
static const uint8_t QUAD_SEGMENTS = QUAD_N / QUAD_SEGMENT_N;

// Stored phase-reference axis is selected automatically after the final quadrature pass.
// ADXL X remains the fixed control axis requested by the user.
static uint8_t referenceAxis = 0;
static const uint8_t CONTROL_AXIS = 0;

// =============================================================================
// Matcher settings retained from the robust sequence
// =============================================================================
static float amplitudeTolerancePct = 7.5f;
static const float INNER_TOLERANCE_PCT = 2.5f;
static const float DAC_MIN = 1.0f;
static const float DAC_MAX = 110.0f;
static const float DAC_RESOLUTION = 0.25f;
static const uint16_t MATCH_BLOCK_N = 256; // 160 ms
static const uint8_t MATCH_BLOCKS = 6;
static const uint32_t MATCH_SETTLE_MS = 340;
static const float MATCH_MAX_MAD_PCT = 20.0f;
static const float MATCH_MAX_CV_PCT = 20.0f;
static const float MATCH_MIN_COHERENCE_PCT = 45.0f;

static const float AXIS_TEST_DACS[] = {5.0f, 9.0f, 13.0f, 17.0f};
static const uint8_t AXIS_TEST_COUNT = sizeof(AXIS_TEST_DACS) / sizeof(AXIS_TEST_DACS[0]);
static const float AXIS_TEST_MIN_TOTAL = 0.12f;
static const float AXIS_TEST_MIN_X = 0.06f;
static const float AXIS_DOMINANCE_MARGIN = 1.05f;
static const float DROPOUT_TOTAL_THRESHOLD = 0.010f;

// Robustness additions. These do not change the frozen FFT/quadrature/matcher math.
static const uint32_t OUTPUT_OFF_RECOVERY_MS = 250;
static const uint32_t OUTPUT_POINT_COOLDOWN_MS = 180;
static const float EXPECTED_TOOL_MIN_HZ = 240.0f;
static const float EXPECTED_TOOL_MAX_HZ = 300.0f;
static const float MIN_PARTIAL_AUTHORITY_RATIO = 0.05f; // permit phase proof when exact match is unreachable
static const float RESPONSE_COLLAPSE_RATIO = 0.45f;     // amplifier/protection drop detector

// =============================================================================
// Phase-sweep and hold settings
// =============================================================================
static const uint16_t PHASE_BLOCK_N = 128; // 80 ms
static const uint8_t BASELINE_BLOCKS = 20; // 1.6 s
static const uint8_t COARSE_BLOCKS = 7;    // 0.56 s: keeps open-loop sweep short
static const uint8_t FINE_BLOCKS = 9;      // 0.72 s
static const uint8_t PROOF_BLOCKS = 24;    // 1.92 s
static const uint32_t PHASE_SETTLE_MS = 180;
static const uint32_t HOLD_INTERVAL_MS = 1000;
static const uint8_t HOLD_BLOCKS = 8;
static const float MIN_TOOL_REFERENCE_AMP = 0.15f;
static const uint8_t POINT_ANCHOR_BLOCKS = 10; // 0.8 s, Visaton OFF
static const float MAX_POINT_RESIDUAL_DF_HZ = 0.18f;
static const float MIN_HOLD_START_REDUCTION_PCT = 2.0f;

// =============================================================================
// 10 kHz hardware-timer NCO (Core 3.x and 2.x compatible)
// =============================================================================
static const uint32_t DAC_UPDATE_HZ = 10000;
static const uint32_t DAC_TIMER_US = 1000000UL / DAC_UPDATE_HZ; // 100 us

hw_timer_t *dacTimer = nullptr;
volatile uint32_t ncoPhaseAcc = 0;
volatile uint32_t ncoPhaseStep = 0;
volatile uint32_t ncoCommandPhase = 0;
volatile uint16_t ncoAmplitudeQ8 = 0;
volatile bool ncoOutputEnabled = false;
int8_t sineTable[256];

// =============================================================================
// State
// =============================================================================
static bool sensorsReady = false;
static bool profileStored = false;
static bool amplitudeMatched = false;
static bool phaseSweepCompleted = false;
static bool holdActive = false;
static bool measurementHardwareFault = false;
static bool measurementSensorFault = false;
static bool measurementOutputDropout = false;
static bool actuatorXAxisVerified = false;
static bool exactAmplitudeMatch = false;
static bool lastIqRawFault = false;

static float operatingFrequencyHz = 0.0f;
static float localTargetAmp[3] = {0.0f, 0.0f, 0.0f};
static float localTargetTotal = 0.0f;
static float matchedDac = 0.0f;
static float matchedControlAmp = 0.0f;

static float storedToolPhaseDeg = 0.0f; // latest brief phase anchor relative to base NCO
static float bestPhaseDeg = 0.0f;
static float holdBaselineAmp[3] = {0.0f, 0.0f, 0.0f};
static float holdBaselineTotal = 0.0f;
static uint32_t lastHoldCheckMs = 0;
static uint8_t consecutiveBadHoldChecks = 0;
static uint32_t holdCheckNumber = 0;

// =============================================================================
// Data structures
// =============================================================================
struct RawAccel {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct AxisFftResult {
  float frequencyHz;
  float amplitude;
  float snrDb;
  bool valid;
};

struct QuadRefineResult {
  bool valid;
  float frequencyHz;
  float dfHz;
  float amplitude;
  float phaseEndDeg;
  float phaseRmseDeg;
};

struct IQBlockResult {
  float amplitude[3];
  float phaseDeg[3];
  float total;
};

struct StableMeasurement {
  bool valid;
  uint8_t count;
  float amplitude[3];
  float total;
  float controlMadPct;
  float controlCvPct;
  float controlCoherencePct;
};

struct PhaseBaseline {
  bool valid;
  float referenceAmplitude;
  float referencePhaseEndDeg;
  float referencePhaseRmseDeg;
  float residualDfHz;
  float localAmp[3];
  float localTotal;
  float localControlMadPct;
  float localControlCvPct;
};

struct SweepPoint {
  float phaseDeg;
  StableMeasurement measurement;
};

// =============================================================================
// Static work buffers (avoid large stack allocations)
// =============================================================================
static int16_t fftRaw[3][FFT_N];
static float fftReal[FFT_N];
static float fftImag[FFT_N];
static float spectrumAvg[3][FFT_N / 2 + 1];
static float combinedSpectrum[FFT_N / 2 + 1];
static float sortWork[FFT_N / 2 + 1];

static int16_t quadTool[QUAD_N];
static int16_t quadLocal[3][QUAD_N];

static int16_t matchRaw[3][MATCH_BLOCK_N];
static uint32_t matchPhaseWords[MATCH_BLOCK_N];

static int16_t phaseRefRaw[PHASE_BLOCK_N];
static int16_t phaseLocalRaw[3][PHASE_BLOCK_N];
static uint32_t phaseWords[PHASE_BLOCK_N];

// =============================================================================
// Utility functions
// =============================================================================
static float wrap360(float value) {
  while (value >= 360.0f) value -= 360.0f;
  while (value < 0.0f) value += 360.0f;
  return value;
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

static float quantizeDac(float value) {
  if (value < DAC_MIN) value = DAC_MIN;
  if (value > DAC_MAX) value = DAC_MAX;
  return roundf(value / DAC_RESOLUTION) * DAC_RESOLUTION;
}

static uint32_t degreesToPhaseWord(float degrees) {
  const double normalized = (double)wrap360(degrees) / 360.0;
  return (uint32_t)(normalized * 4294967296.0);
}

static void sortFloat(float *values, uint16_t count) {
  for (uint16_t i = 1; i < count; ++i) {
    const float key = values[i];
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
  for (uint16_t i = 0; i < count; ++i) sortWork[i] = values[i];
  sortFloat(sortWork, count);
  if (count & 1U) return sortWork[count / 2];
  return 0.5f * (sortWork[count / 2 - 1] + sortWork[count / 2]);
}

static float madPercent(const float *values, uint16_t count, float median) {
  if (count == 0 || !isfinite(median) || fabsf(median) < 1e-7f) return NAN;
  for (uint16_t i = 0; i < count; ++i) sortWork[i] = fabsf(values[i] - median);
  sortFloat(sortWork, count);
  float mad;
  if (count & 1U) mad = sortWork[count / 2];
  else mad = 0.5f * (sortWork[count / 2 - 1] + sortWork[count / 2]);
  return 100.0f * mad / fabsf(median);
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
  return (float)(100.0 * sqrt(ss / (count - 1)) / fabs(mean));
}

static float circularCoherencePct(const float *phaseDeg, uint16_t count) {
  if (count == 0) return 0.0f;
  double sumC = 0.0;
  double sumS = 0.0;
  for (uint16_t i = 0; i < count; ++i) {
    const double radians = phaseDeg[i] * PI / 180.0;
    sumC += cos(radians);
    sumS += sin(radians);
  }
  return (float)(100.0 * sqrt(sumC * sumC + sumS * sumS) / count);
}


static void waitUntilMicros(uint32_t targetUs) {
  while ((int32_t)(micros() - targetUs) < 0) {
    delayMicroseconds(10);
  }
}

// =============================================================================
// ADXL345 SPI
// =============================================================================
static void forceSpiIdle() {
  digitalWrite(ADXL_CS, HIGH);
  delayMicroseconds(10);
}

static void writeRegister(uint8_t csPin, uint8_t reg, uint8_t value) {
  forceSpiIdle();
  SPI.beginTransaction(adxlSPI);
  delayMicroseconds(2);
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);
  SPI.transfer(reg & 0x3F);
  SPI.transfer(value);
  delayMicroseconds(2);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(2);
  SPI.endTransaction();
}

static uint8_t readRegisterOnce(uint8_t csPin, uint8_t reg) {
  forceSpiIdle();
  SPI.beginTransaction(adxlSPI);
  delayMicroseconds(2);
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);
  SPI.transfer(0x80 | (reg & 0x3F));
  const uint8_t value = SPI.transfer(0x00);
  delayMicroseconds(2);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(2);
  SPI.endTransaction();
  return value;
}

static uint8_t readRegister(uint8_t csPin, uint8_t reg) {
  // Majority vote rejects an occasional EMI-corrupted SPI byte.
  const uint8_t a = readRegisterOnce(csPin, reg);
  delayMicroseconds(8);
  const uint8_t b = readRegisterOnce(csPin, reg);
  delayMicroseconds(8);
  const uint8_t c = readRegisterOnce(csPin, reg);
  if (a == b || a == c) return a;
  if (b == c) return b;
  return c;
}

static RawAccel readRawXYZ(uint8_t csPin) {
  RawAccel data = {};
  forceSpiIdle();
  SPI.beginTransaction(adxlSPI);
  delayMicroseconds(2);
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);
  SPI.transfer(0x80 | 0x40 | REG_DATAX0);
  const uint8_t x0 = SPI.transfer(0x00);
  const uint8_t x1 = SPI.transfer(0x00);
  const uint8_t y0 = SPI.transfer(0x00);
  const uint8_t y1 = SPI.transfer(0x00);
  const uint8_t z0 = SPI.transfer(0x00);
  const uint8_t z1 = SPI.transfer(0x00);
  delayMicroseconds(2);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(2);
  SPI.endTransaction();
  data.x = (int16_t)((x1 << 8) | x0);
  data.y = (int16_t)((y1 << 8) | y0);
  data.z = (int16_t)((z1 << 8) | z0);
  return data;
}

static bool initializeADXL(uint8_t csPin, const char *name) {
  forceSpiIdle();
  writeRegister(csPin, REG_POWER_CTL, 0x00);
  delay(12);
  writeRegister(csPin, REG_DATA_FORMAT, 0x0B); // Full resolution, +/-16 g
  writeRegister(csPin, REG_BW_RATE, 0x0E);     // Nominal 1600 Hz ODR
  writeRegister(csPin, REG_POWER_CTL, 0x08);   // Measurement mode
  delay(40);

  const uint8_t id = readRegister(csPin, REG_DEVID);
  const uint8_t format = readRegister(csPin, REG_DATA_FORMAT);
  const uint8_t rate = readRegister(csPin, REG_BW_RATE);
  const uint8_t power = readRegister(csPin, REG_POWER_CTL);

  Serial.printf("%s DEVID 0x%02X | FORMAT 0x%02X | RATE 0x%02X | POWER 0x%02X\n",
                name, id, format, rate, power);

  const bool okay = id == 0xE5 && (format & 0x0F) == 0x0B &&
                    (rate & 0x1F) == 0x0E && (power & 0x08) != 0;
  Serial.printf("%s STATUS: %s\n", name, okay ? "PASS" : "FAIL");
  return okay;
}

static bool adxlRegisterHealthCheck(bool printFailure) {
  const uint8_t id = readRegister(ADXL_CS, REG_DEVID);
  const uint8_t format = readRegister(ADXL_CS, REG_DATA_FORMAT);
  const uint8_t rate = readRegister(ADXL_CS, REG_BW_RATE);
  const uint8_t power = readRegister(ADXL_CS, REG_POWER_CTL);
  const bool ok = (id == 0xE5) && ((format & 0x0F) == 0x0B) &&
                  ((rate & 0x1F) == 0x0E) && ((power & 0x08) != 0);
  if (!ok && printFailure) {
    Serial.printf("ADXL HEALTH FAIL | DEVID 0x%02X | FORMAT 0x%02X | RATE 0x%02X | POWER 0x%02X\n",
                  id, format, rate, power);
  }
  return ok;
}

static bool recoverADXL(const char *context) {
  // Recovery is always performed with the actuator physically commanded OFF.
  ncoOutputEnabled = false;
  dacWrite(DAC_PIN, 128);
  delay(OUTPUT_OFF_RECOVERY_MS);

  Serial.printf("ADXL RECOVERY START: %s\n", context);
  for (uint8_t attempt = 1; attempt <= 3; ++attempt) {
    digitalWrite(ADXL_CS, HIGH);
    SPI.end();
    delay(25);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    forceSpiIdle();
    delay(25);

    char name[48];
    snprintf(name, sizeof(name), "ADXL RECOVERY ATTEMPT %u", attempt);
    if (initializeADXL(ADXL_CS, name) && adxlRegisterHealthCheck(false)) {
      sensorsReady = true;
      Serial.println("ADXL RECOVERY SUCCESSFUL.");
      return true;
    }
    delay(150);
  }

  sensorsReady = false;
  Serial.println("ADXL RECOVERY FAILED AFTER 3 ATTEMPTS.");
  Serial.println("This is now a physical power/ground/SPI connection problem, not an FFT or matcher problem.");
  return false;
}

static bool ensureADXLReady(const char *context) {
  if (adxlRegisterHealthCheck(false)) {
    sensorsReady = true;
    return true;
  }
  adxlRegisterHealthCheck(true);
  return recoverADXL(context);
}

// =============================================================================
// NCO and DAC
// =============================================================================
void ARDUINO_ISR_ATTR onDacTimer() {
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

static void applyNcoFrequency() {
  if (operatingFrequencyHz < 1.0f) operatingFrequencyHz = 1.0f;
  if (operatingFrequencyHz > 750.0f) operatingFrequencyHz = 750.0f;
  ncoPhaseStep = (uint32_t)((double)operatingFrequencyHz * 4294967296.0 /
                            (double)DAC_UPDATE_HZ);
}

static void applyNcoAmplitude(float dacAmplitude) {
  if (dacAmplitude < 0.0f) dacAmplitude = 0.0f;
  if (dacAmplitude > 120.0f) dacAmplitude = 120.0f;
  ncoAmplitudeQ8 = (uint16_t)lroundf(dacAmplitude * 256.0f);
}

static void setOutputEnabled(bool enabled) {
  ncoOutputEnabled = enabled;
  if (!enabled) dacWrite(DAC_PIN, 128);
}

static void setOpenLoopRelativePhase(float relativePhaseDeg) {
  const float commandDeg = storedToolPhaseDeg + wrap360(relativePhaseDeg);
  ncoCommandPhase = degreesToPhaseWord(commandDeg);
}

static void setMatcherPhaseZero() {
  ncoCommandPhase = 0;
}

// =============================================================================
// FFT implementation
// =============================================================================
static void fftInPlace(float *real, float *imag, uint16_t n) {
  uint16_t j = 0;
  for (uint16_t i = 1; i < n; ++i) {
    uint16_t bit = n >> 1;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j ^= bit;
    if (i < j) {
      const float tr = real[i];
      real[i] = real[j];
      real[j] = tr;
      const float ti = imag[i];
      imag[i] = imag[j];
      imag[j] = ti;
    }
  }

  for (uint16_t length = 2; length <= n; length <<= 1) {
    const float angle = -TWO_PI / length;
    const float wLenReal = cosf(angle);
    const float wLenImag = sinf(angle);
    for (uint16_t i = 0; i < n; i += length) {
      float wReal = 1.0f;
      float wImag = 0.0f;
      const uint16_t half = length >> 1;
      for (uint16_t k = 0; k < half; ++k) {
        const uint16_t even = i + k;
        const uint16_t odd = even + half;
        const float oddReal = real[odd] * wReal - imag[odd] * wImag;
        const float oddImag = real[odd] * wImag + imag[odd] * wReal;
        const float evenReal = real[even];
        const float evenImag = imag[even];
        real[even] = evenReal + oddReal;
        imag[even] = evenImag + oddImag;
        real[odd] = evenReal - oddReal;
        imag[odd] = evenImag - oddImag;
        const float nextWReal = wReal * wLenReal - wImag * wLenImag;
        wImag = wReal * wLenImag + wImag * wLenReal;
        wReal = nextWReal;
      }
    }
  }
}

static void captureFftFrame() {
  uint32_t nextSampleUs = micros();
  for (uint16_t i = 0; i < FFT_N; ++i) {
    nextSampleUs += SAMPLE_PERIOD_US;
    waitUntilMicros(nextSampleUs);
    const RawAccel a = readRawXYZ(ADXL_CS);
    fftRaw[0][i] = a.x;
    fftRaw[1][i] = a.y;
    fftRaw[2][i] = a.z;
  }
}

static void addAxisSpectrum(uint8_t axis) {
  double mean = 0.0;
  for (uint16_t i = 0; i < FFT_N; ++i) mean += fftRaw[axis][i];
  mean /= FFT_N;

  double windowSum = 0.0;
  for (uint16_t i = 0; i < FFT_N; ++i) {
    const float window = 0.54f - 0.46f * cosf(TWO_PI * i / (FFT_N - 1));
    const float value = ((float)fftRaw[axis][i] - (float)mean) * ADXL_SCALE_MS2;
    fftReal[i] = value * window;
    fftImag[i] = 0.0f;
    windowSum += window;
  }

  fftInPlace(fftReal, fftImag, FFT_N);

  for (uint16_t k = 0; k <= FFT_N / 2; ++k) {
    const float magnitude = sqrtf(fftReal[k] * fftReal[k] + fftImag[k] * fftImag[k]);
    const float peakAmplitude = (float)(2.0 * magnitude / windowSum);
    spectrumAvg[axis][k] += peakAmplitude / FFT_FRAMES;
  }
}

static uint16_t strongestBin(const float *spectrum, uint16_t firstBin,
                             uint16_t lastBin) {
  uint16_t best = firstBin;
  float bestValue = -1.0f;
  for (uint16_t k = firstBin; k <= lastBin; ++k) {
    if (spectrum[k] > bestValue) {
      bestValue = spectrum[k];
      best = k;
    }
  }
  return best;
}

static float interpolatedBin(const float *spectrum, uint16_t bin) {
  if (bin == 0 || bin >= FFT_N / 2) return (float)bin;
  const float left = spectrum[bin - 1];
  const float center = spectrum[bin];
  const float right = spectrum[bin + 1];
  const float denominator = left - 2.0f * center + right;
  if (fabsf(denominator) < 1e-12f) return (float)bin;
  float delta = 0.5f * (left - right) / denominator;
  if (delta > 0.5f) delta = 0.5f;
  if (delta < -0.5f) delta = -0.5f;
  return bin + delta;
}

static float spectrumNoiseMedian(const float *spectrum, uint16_t peakBin,
                                 uint16_t firstBin, uint16_t lastBin) {
  uint16_t count = 0;
  for (uint16_t k = firstBin; k <= lastBin; ++k) {
    if (k + 3 >= peakBin && k <= peakBin + 3) continue;
    sortWork[count++] = spectrum[k];
  }
  if (count == 0) return 1e-9f;
  sortFloat(sortWork, count);
  if (count & 1U) return sortWork[count / 2];
  return 0.5f * (sortWork[count / 2 - 1] + sortWork[count / 2]);
}

static AxisFftResult analyzeAxisSpectrum(uint8_t axis, uint16_t firstBin,
                                         uint16_t lastBin) {
  AxisFftResult result = {};
  const uint16_t peakBin = strongestBin(spectrumAvg[axis], firstBin, lastBin);
  const float binFloat = interpolatedBin(spectrumAvg[axis], peakBin);
  result.frequencyHz = binFloat * SAMPLE_RATE_HZ / FFT_N;
  result.amplitude = spectrumAvg[axis][peakBin];
  const float noise = spectrumNoiseMedian(spectrumAvg[axis], peakBin,
                                          firstBin, lastBin);
  result.snrDb = 20.0f * log10f((result.amplitude + 1e-9f) / (noise + 1e-9f));
  result.valid = result.amplitude >= FFT_MIN_AXIS_AMP &&
                 result.snrDb >= FFT_MIN_AXIS_SNR_DB;
  return result;
}

static float selectHarmonicSafeFrequency(const AxisFftResult axisResult[3],
                                         uint16_t firstBin, uint16_t lastBin,
                                         const char *&sourceText) {
  for (uint16_t k = firstBin; k <= lastBin; ++k) {
    const float x = spectrumAvg[0][k];
    const float y = spectrumAvg[1][k];
    const float z = spectrumAvg[2][k];
    combinedSpectrum[k] = sqrtf(x * x + y * y + z * z);
  }

  uint16_t combinedBin = strongestBin(combinedSpectrum, firstBin, lastBin);
  float combinedFrequency = interpolatedBin(combinedSpectrum, combinedBin) *
                            SAMPLE_RATE_HZ / FFT_N;

  // Harmonic protection: examine f/2 when the strongest combined peak may be 2f.
  if (combinedFrequency > 80.0f) {
    const uint16_t nominalHalf = combinedBin / 2;
    const uint16_t halfStart = nominalHalf > 2 ? nominalHalf - 2 : firstBin;
    const uint16_t halfEnd = (nominalHalf + 2 < lastBin) ? nominalHalf + 2 : lastBin;
    const uint16_t halfBin = strongestBin(combinedSpectrum, halfStart, halfEnd);
    const float ratio = combinedSpectrum[halfBin] /
                        (combinedSpectrum[combinedBin] + 1e-9f);
    uint8_t axesSupportingHalf = 0;
    const float halfHz = (float)halfBin * SAMPLE_RATE_HZ / FFT_N;
    for (uint8_t axis = 0; axis < 3; ++axis) {
      if (axisResult[axis].valid &&
          fabsf(axisResult[axis].frequencyHz - halfHz) <= 4.0f) {
        ++axesSupportingHalf;
      }
    }
    if (ratio >= 0.22f && (axesSupportingHalf >= 2 || ratio >= 0.36f)) {
      combinedBin = halfBin;
      combinedFrequency = interpolatedBin(combinedSpectrum, combinedBin) *
                          SAMPLE_RATE_HZ / FFT_N;
      sourceText = "COMBINED FUNDAMENTAL / SECOND-HARMONIC REJECTED";
    }
  }

  // Three-axis consensus around the harmonic-safe combined candidate.
  double weightedFrequency = 0.0;
  double weightSum = 0.0;
  uint8_t consensusAxes = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    if (!axisResult[axis].valid) continue;
    if (fabsf(axisResult[axis].frequencyHz - combinedFrequency) <= 4.0f) {
      const double weight = axisResult[axis].amplitude * axisResult[axis].amplitude;
      weightedFrequency += axisResult[axis].frequencyHz * weight;
      weightSum += weight;
      ++consensusAxes;
    }
  }

  if (consensusAxes >= 2 && weightSum > 0.0) {
    sourceText = "THREE-AXIS CONSENSUS / HARMONIC-SAFE";
    return (float)(weightedFrequency / weightSum);
  }

  sourceText = "COMBINED THREE-AXIS SPECTRUM / HARMONIC-SAFE";
  return combinedFrequency;
}

static bool runFrozenFft(float &centerFrequencyHz) {
  for (uint8_t axis = 0; axis < 3; ++axis) {
    for (uint16_t k = 0; k <= FFT_N / 2; ++k) spectrumAvg[axis][k] = 0.0f;
  }

  for (uint8_t frame = 0; frame < FFT_FRAMES; ++frame) {
    captureFftFrame();
    for (uint8_t axis = 0; axis < 3; ++axis) addAxisSpectrum(axis);
    Serial.printf("ADXL FFT frame %u / %u acquired.\n", frame + 1, FFT_FRAMES);
  }

  const uint16_t firstBin = (uint16_t)ceilf(FFT_MIN_HZ * FFT_N / SAMPLE_RATE_HZ);
  const uint16_t lastBin = (uint16_t)floorf(FFT_MAX_HZ * FFT_N / SAMPLE_RATE_HZ);

  AxisFftResult axisResult[3];
  for (uint8_t axis = 0; axis < 3; ++axis) {
    axisResult[axis] = analyzeAxisSpectrum(axis, firstBin, lastBin);
    Serial.printf("ADXL %s | %.4f Hz | amp %.5f m/s^2 | SNR %.2f dB | %s\n",
                  axisName(axis), axisResult[axis].frequencyHz,
                  axisResult[axis].amplitude, axisResult[axis].snrDb,
                  axisResult[axis].valid ? "VALID" : "INVALID");
  }

  const char *sourceText = "";
  centerFrequencyHz = selectHarmonicSafeFrequency(axisResult, firstBin,
                                                   lastBin, sourceText);
  Serial.printf("FFT OPERATING FREQUENCY: %.6f Hz | source: %s\n",
                centerFrequencyHz, sourceText);

  uint8_t validAxes = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) if (axisResult[axis].valid) ++validAxes;
  return validAxes >= 1 && centerFrequencyHz >= FFT_MIN_HZ &&
         centerFrequencyHz <= FFT_MAX_HZ;
}

// =============================================================================
// Frequency-domain quadrature refinement
// =============================================================================
static void captureQuadrature(bool includeLocal) {
  uint32_t nextSampleUs = micros();
  for (uint16_t i = 0; i < QUAD_N; ++i) {
    nextSampleUs += SAMPLE_PERIOD_US;
    waitUntilMicros(nextSampleUs);
    const RawAccel sample = readRawXYZ(ADXL_CS);
    quadTool[i] = axisRaw(sample, referenceAxis);
    if (includeLocal) {
      quadLocal[0][i] = sample.x;
      quadLocal[1][i] = sample.y;
      quadLocal[2][i] = sample.z;
    }
  }
}

static void projectRawAtFrequency(const int16_t *raw, uint16_t count,
                                  float frequencyHz, float &amplitude,
                                  float &phaseEndDeg) {
  double mean = 0.0;
  for (uint16_t i = 0; i < count; ++i) mean += raw[i];
  mean /= count;

  double sumI = 0.0;
  double sumQ = 0.0;
  for (uint16_t i = 0; i < count; ++i) {
    const double t = (double)i / SAMPLE_RATE_HZ;
    const double theta = TWO_PI * frequencyHz * t;
    const double value = ((double)raw[i] - mean) * ADXL_SCALE_MS2;
    sumI += value * cos(theta);
    sumQ += value * sin(theta);
  }

  amplitude = (float)((2.0 / count) * sqrt(sumI * sumI + sumQ * sumQ));
  const double phaseAtZero = atan2(-sumQ, sumI);
  const double phaseAtEnd = phaseAtZero + TWO_PI * frequencyHz *
                            ((double)(count - 1) / SAMPLE_RATE_HZ);
  phaseEndDeg = wrap360((float)(phaseAtEnd * 180.0 / PI));
}

static QuadRefineResult analyzeQuadratureFrequency(const int16_t *raw,
                                                   float centerFrequencyHz,
                                                   float maxCorrectionHz) {
  QuadRefineResult result = {};
  float unwrappedPhase[QUAD_SEGMENTS];
  float segmentTime[QUAD_SEGMENTS];
  float segmentAmplitude[QUAD_SEGMENTS];

  for (uint8_t segment = 0; segment < QUAD_SEGMENTS; ++segment) {
    const uint16_t start = segment * QUAD_SEGMENT_N;
    double mean = 0.0;
    for (uint16_t i = 0; i < QUAD_SEGMENT_N; ++i) mean += raw[start + i];
    mean /= QUAD_SEGMENT_N;

    double sumI = 0.0;
    double sumQ = 0.0;
    for (uint16_t i = 0; i < QUAD_SEGMENT_N; ++i) {
      const uint16_t index = start + i;
      const double t = (double)index / SAMPLE_RATE_HZ;
      const double theta = TWO_PI * centerFrequencyHz * t;
      const double value = ((double)raw[index] - mean) * ADXL_SCALE_MS2;
      sumI += value * cos(theta);
      sumQ += value * sin(theta);
    }

    segmentAmplitude[segment] = (float)((2.0 / QUAD_SEGMENT_N) *
                                        sqrt(sumI * sumI + sumQ * sumQ));
    float phase = (float)atan2(-sumQ, sumI);
    if (segment > 0) {
      while (phase - unwrappedPhase[segment - 1] > PI) phase -= TWO_PI;
      while (phase - unwrappedPhase[segment - 1] < -PI) phase += TWO_PI;
    }
    unwrappedPhase[segment] = phase;
    segmentTime[segment] = ((float)start + 0.5f * (QUAD_SEGMENT_N - 1)) /
                           SAMPLE_RATE_HZ;
  }

  double sumT = 0.0;
  double sumP = 0.0;
  for (uint8_t i = 0; i < QUAD_SEGMENTS; ++i) {
    sumT += segmentTime[i];
    sumP += unwrappedPhase[i];
  }
  const double meanT = sumT / QUAD_SEGMENTS;
  const double meanP = sumP / QUAD_SEGMENTS;
  double numerator = 0.0;
  double denominator = 0.0;
  for (uint8_t i = 0; i < QUAD_SEGMENTS; ++i) {
    const double dt = segmentTime[i] - meanT;
    numerator += dt * (unwrappedPhase[i] - meanP);
    denominator += dt * dt;
  }
  const double slope = denominator > 0.0 ? numerator / denominator : 0.0;
  float dfHz = (float)(slope / TWO_PI);
  if (dfHz > maxCorrectionHz) dfHz = maxCorrectionHz;
  if (dfHz < -maxCorrectionHz) dfHz = -maxCorrectionHz;

  const double intercept = meanP - slope * meanT;
  double squaredError = 0.0;
  for (uint8_t i = 0; i < QUAD_SEGMENTS; ++i) {
    const double fitted = intercept + slope * segmentTime[i];
    const double error = unwrappedPhase[i] - fitted;
    squaredError += error * error;
  }

  result.dfHz = dfHz;
  result.frequencyHz = centerFrequencyHz + dfHz;
  result.phaseRmseDeg = (float)(sqrt(squaredError / QUAD_SEGMENTS) * 180.0 / PI);
  result.amplitude = medianOf(segmentAmplitude, QUAD_SEGMENTS);
  const double endTime = (double)(QUAD_N - 1) / SAMPLE_RATE_HZ;
  result.phaseEndDeg = wrap360((float)((intercept + slope * endTime) * 180.0 / PI));
  result.valid = isfinite(result.frequencyHz) && isfinite(result.amplitude) &&
                 result.amplitude >= 0.05f && result.phaseRmseDeg <= 35.0f;
  return result;
}


static bool runFrozenQuadrature(float fftCenterHz) {
  // Pick the strongest ADXL FFT axis at the center frequency before capturing.
  const uint16_t centerBin = (uint16_t)lroundf(fftCenterHz * FFT_N / SAMPLE_RATE_HZ);
  float strongest = -1.0f;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    uint16_t bin = centerBin;
    if (bin > FFT_N / 2) bin = FFT_N / 2;
    if (spectrumAvg[axis][bin] > strongest) {
      strongest = spectrumAvg[axis][bin];
      referenceAxis = axis;
    }
  }

  Serial.printf("ADXL quadrature reference axis: %s\n", axisName(referenceAxis));

  Serial.println("---------------- QUADRATURE PASS 1 ----------------");
  captureQuadrature(false);
  QuadRefineResult pass1 = analyzeQuadratureFrequency(quadTool, fftCenterHz, 1.0f);
  Serial.printf("df1 %+.6f Hz | refined %.6f Hz | amp %.5f | phase-fit RMSE %.2f deg | %s\n",
                pass1.dfHz, pass1.frequencyHz, pass1.amplitude,
                pass1.phaseRmseDeg, pass1.valid ? "VALID" : "INVALID");
  if (!pass1.valid) return false;

  Serial.println("---------------- QUADRATURE PASS 2 ----------------");
  captureQuadrature(true);
  QuadRefineResult pass2 = analyzeQuadratureFrequency(quadTool,
                                                       pass1.frequencyHz,
                                                       0.50f);
  Serial.printf("df2 %+.6f Hz | final %.6f Hz | amp %.5f | phase-fit RMSE %.2f deg | %s\n",
                pass2.dfHz, pass2.frequencyHz, pass2.amplitude,
                pass2.phaseRmseDeg, pass2.valid ? "VALID" : "INVALID");
  if (!pass2.valid) return false;

  operatingFrequencyHz = pass2.frequencyHz;
  applyNcoFrequency();

  float ignoredPhase = 0.0f;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    projectRawAtFrequency(quadLocal[axis], QUAD_N, operatingFrequencyHz,
                          localTargetAmp[axis], ignoredPhase);
  }
  localTargetTotal = sqrtf(localTargetAmp[0] * localTargetAmp[0] +
                           localTargetAmp[1] * localTargetAmp[1] +
                           localTargetAmp[2] * localTargetAmp[2]);


  Serial.printf("SINGLE ADXL X/Y/Z/TOTAL at final frequency: %.5f / %.5f / %.5f / %.5f m/s^2 peak\n",
                localTargetAmp[0], localTargetAmp[1], localTargetAmp[2],
                localTargetTotal);
  Serial.printf("STORED MATCHER TARGET: SINGLE-ADXL X = %.5f m/s^2 peak\n",
                localTargetAmp[CONTROL_AXIS]);

  if (localTargetAmp[CONTROL_AXIS] < 0.05f) {
    Serial.println("PROFILE REJECTED: ADXL X is below 0.05 m/s^2 at the selected frequency.");
    Serial.println("Fix the ADXL rigidly at the exact cancellation point and repeat b.");
    return false;
  }
  if (localTargetAmp[CONTROL_AXIS] < 0.20f) {
    Serial.println("WARNING: Single-ADXL X is small; cancellation will be sensitive to mounting/noise.");
  }
  return true;
}

// =============================================================================
// NCO-synchronous I/Q measurements for matcher and phase test
// =============================================================================
static void projectSignalAgainstNco(const int16_t *raw,
                                    const uint32_t *phaseWordArray,
                                    uint16_t count,
                                    float &amplitude,
                                    float &phaseDeg) {
  double mean = 0.0;
  for (uint16_t i = 0; i < count; ++i) mean += raw[i];
  mean /= count;

  double sumSin = 0.0;
  double sumCos = 0.0;
  for (uint16_t i = 0; i < count; ++i) {
    const double theta = ((double)phaseWordArray[i] / 4294967296.0) * TWO_PI;
    const double value = ((double)raw[i] - mean) * ADXL_SCALE_MS2;
    sumSin += value * sin(theta);
    sumCos += value * cos(theta);
  }

  amplitude = (float)((2.0 / count) * sqrt(sumSin * sumSin + sumCos * sumCos));
  phaseDeg = wrap360((float)(atan2(sumCos, sumSin) * 180.0 / PI));
}

static IQBlockResult captureLocalIqBlock(uint16_t count) {
  IQBlockResult result = {};
  uint32_t nextSampleUs = micros();
  uint16_t impossibleSamples = 0;
  uint16_t longestIdenticalRun = 0;
  uint16_t identicalRun = 0;
  RawAccel previous = {};

  lastIqRawFault = false;

  for (uint16_t i = 0; i < count; ++i) {
    nextSampleUs += SAMPLE_PERIOD_US;
    waitUntilMicros(nextSampleUs);
    matchPhaseWords[i] = ncoPhaseAcc;
    const RawAccel local = readRawXYZ(ADXL_CS);

    const bool allZero = local.x == 0 && local.y == 0 && local.z == 0;
    const bool allMinusOne = local.x == -1 && local.y == -1 && local.z == -1;
    if (allZero || allMinusOne) ++impossibleSamples;

    if (i > 0 && local.x == previous.x && local.y == previous.y && local.z == previous.z) {
      ++identicalRun;
      if (identicalRun > longestIdenticalRun) longestIdenticalRun = identicalRun;
    } else {
      identicalRun = 0;
    }
    previous = local;

    matchRaw[0][i] = local.x;
    matchRaw[1][i] = local.y;
    matchRaw[2][i] = local.z;
  }

  if (impossibleSamples > count / 20 || longestIdenticalRun > count / 2) {
    lastIqRawFault = true;
  }

  for (uint8_t axis = 0; axis < 3; ++axis) {
    projectSignalAgainstNco(matchRaw[axis], matchPhaseWords, count,
                            result.amplitude[axis], result.phaseDeg[axis]);
  }
  result.total = sqrtf(result.amplitude[0] * result.amplitude[0] +
                       result.amplitude[1] * result.amplitude[1] +
                       result.amplitude[2] * result.amplitude[2]);
  return result;
}

static StableMeasurement measureActuatorPoint(float dac, const char *label) {
  StableMeasurement result = {};
  measurementHardwareFault = false;
  measurementSensorFault = false;
  measurementOutputDropout = false;

  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    float axisValues[3][MATCH_BLOCKS];
    float totalValues[MATCH_BLOCKS];
    float controlPhases[MATCH_BLOCKS];
    bool captureFault = false;

    // Every point starts and ends with the actuator OFF. This prevents the
    // amplifier from remaining at a high DAC during printing and the next test.
    setOutputEnabled(false);
    delay(OUTPUT_POINT_COOLDOWN_MS + (attempt ? 500 : 0));

    if (!ensureADXLReady("before actuator measurement")) {
      measurementHardwareFault = true;
      measurementSensorFault = true;
      return result;
    }

    applyNcoAmplitude(dac);
    setMatcherPhaseZero();
    setOutputEnabled(true);
    delay(MATCH_SETTLE_MS + (attempt ? 180 : 0));

    for (uint8_t block = 0; block < MATCH_BLOCKS; ++block) {
      IQBlockResult iq = captureLocalIqBlock(MATCH_BLOCK_N);
      if (lastIqRawFault) captureFault = true;
      for (uint8_t axis = 0; axis < 3; ++axis) axisValues[axis][block] = iq.amplitude[axis];
      totalValues[block] = iq.total;
      controlPhases[block] = iq.phaseDeg[CONTROL_AXIS];
    }

    // Critical fix: never leave the Visaton running between DAC search points.
    setOutputEnabled(false);
    delay(120);

    const bool sensorHealthyAfterPoint = adxlRegisterHealthCheck(false);
    if (!sensorHealthyAfterPoint || captureFault) {
      if (!sensorHealthyAfterPoint) adxlRegisterHealthCheck(true);
      Serial.printf("%s | DAC %.2f | SENSOR/SPI CAPTURE FAULT attempt %u/2\n",
                    label, dac, attempt + 1);
      if (!recoverADXL("after actuator point")) {
        measurementHardwareFault = true;
        measurementSensorFault = true;
        return result;
      }
      if (attempt == 0) continue;
      measurementHardwareFault = true;
      measurementSensorFault = true;
      return result;
    }

    result.count = MATCH_BLOCKS;
    for (uint8_t axis = 0; axis < 3; ++axis) {
      result.amplitude[axis] = medianOf(axisValues[axis], MATCH_BLOCKS);
    }
    result.total = medianOf(totalValues, MATCH_BLOCKS);
    result.controlMadPct = madPercent(axisValues[CONTROL_AXIS], MATCH_BLOCKS,
                                      result.amplitude[CONTROL_AXIS]);
    result.controlCvPct = cvPercent(axisValues[CONTROL_AXIS], MATCH_BLOCKS);
    result.controlCoherencePct = circularCoherencePct(controlPhases, MATCH_BLOCKS);

    const bool finiteMeasurement =
        isfinite(result.amplitude[0]) && isfinite(result.amplitude[1]) &&
        isfinite(result.amplitude[2]) && isfinite(result.total) &&
        isfinite(result.controlMadPct) && isfinite(result.controlCvPct);

    const bool suspiciousDropout =
        dac >= 5.0f && (!finiteMeasurement ||
                       result.total < DROPOUT_TOTAL_THRESHOLD);

    if (suspiciousDropout) {
      Serial.printf("%s | DAC %.2f | OUTPUT RESPONSE DROPOUT attempt %u/2\n",
                    label, dac, attempt + 1);
      measurementOutputDropout = true;
      delay(700);
      if (attempt == 0) continue;
      measurementHardwareFault = true;
      return result;
    }

    const float minimumSignal =
        fmaxf(0.025f, 0.02f * localTargetAmp[CONTROL_AXIS]);
    result.valid =
        finiteMeasurement &&
        result.amplitude[CONTROL_AXIS] >= minimumSignal &&
        result.controlMadPct <= MATCH_MAX_MAD_PCT &&
        result.controlCvPct <= MATCH_MAX_CV_PCT &&
        result.controlCoherencePct >= MATCH_MIN_COHERENCE_PCT;

    const float errorPct =
        100.0f * (result.amplitude[CONTROL_AXIS] -
                  localTargetAmp[CONTROL_AXIS]) /
        fmaxf(localTargetAmp[CONTROL_AXIS], 1e-6f);

    Serial.printf("%s | DAC %.2f | X %.5f | Y/Z/T %.5f / %.5f / %.5f | error %+.2f%% | MAD %.2f%% | CV %.2f%% | coh %.1f%% | %s\n",
                  label, dac, result.amplitude[0], result.amplitude[1],
                  result.amplitude[2], result.total, errorPct,
                  result.controlMadPct, result.controlCvPct,
                  result.controlCoherencePct,
                  result.valid ? "VALID" : "UNSTABLE/INVALID");
    return result;
  }

  return result;
}

static bool verifyVisatonXAxisDominance() {
  Serial.println();
  Serial.println("====================================================================");
  Serial.println("VISATON X-AXIS PRECHECK — TOOL OFF");
  Serial.println("The program measures all safe low-DAC points and selects the strongest reliable one.");
  Serial.println("X must be the strongest actuator axis before matching.");
  Serial.println("====================================================================");

  actuatorXAxisVerified = false;
  measurementHardwareFault = false;
  StableMeasurement chosen = {};
  float chosenDac = 0.0f;
  uint8_t usablePoints = 0;
  uint8_t xDominantPoints = 0;

  for (uint8_t i = 0; i < AXIS_TEST_COUNT; ++i) {
    char label[32];
    snprintf(label, sizeof(label), "AXIS TEST %u", i + 1);
    StableMeasurement point = measureActuatorPoint(AXIS_TEST_DACS[i], label);

    if (measurementHardwareFault) {
      setOutputEnabled(false);
      Serial.println("X-AXIS PRECHECK STOPPED after repeated sensor/output fault.");
      return false;
    }

    const bool usable =
        point.valid && isfinite(point.total) &&
        point.total >= AXIS_TEST_MIN_TOTAL &&
        point.amplitude[0] >= AXIS_TEST_MIN_X;

    if (!usable) continue;

    ++usablePoints;
    const float strongestOther =
        fmaxf(point.amplitude[1], point.amplitude[2]);
    if (point.amplitude[0] >= AXIS_DOMINANCE_MARGIN * strongestOther) {
      ++xDominantPoints;
    }

    if (chosenDac <= 0.0f ||
        point.amplitude[0] > chosen.amplitude[0]) {
      chosen = point;
      chosenDac = AXIS_TEST_DACS[i];
    }
  }

  setOutputEnabled(false);

  if (chosenDac <= 0.0f) {
    Serial.println("X-AXIS PRECHECK FAILED: no reliable Visaton motion at the safe low-DAC points.");
    Serial.println("The program already retried and recovered the ADXL automatically.");
    Serial.println("Remaining causes are amplifier power/protection, battery/ground, output wiring, or loose mechanical coupling.");
    return false;
  }

  const float strongestOther =
      fmaxf(chosen.amplitude[1], chosen.amplitude[2]);
  const float xToOther =
      chosen.amplitude[0] / fmaxf(strongestOther, 1e-6f);

  Serial.printf("AXIS VERDICT at strongest reliable point, DAC %.2f | X %.5f | Y %.5f | Z %.5f | X/strongest-other %.3f\n",
                chosenDac, chosen.amplitude[0], chosen.amplitude[1],
                chosen.amplitude[2], xToOther);
  Serial.printf("Usable low-DAC points: %u | X-dominant points: %u\n",
                usablePoints, xDominantPoints);

  if (chosen.amplitude[0] < AXIS_DOMINANCE_MARGIN * strongestOther) {
    Serial.println("VISATON X-AXIS PRECHECK: FAIL");
    Serial.println("The tool may be strongest in Y or Z; that is allowed. But the Visaton itself must push mainly along ADXL X.");
    Serial.println("Rotate the ADXL/Visaton mounting so the strongest Visaton direction is labelled X, then repeat b and m.");
    return false;
  }

  actuatorXAxisVerified = true;
  Serial.println("VISATON X-AXIS PRECHECK: PASS");
  Serial.println("The strongest reliable actuator response is X-dominant.");
  return true;
}

static bool amplitudeInBand(float amplitude, float tolerancePct) {
  const float target = localTargetAmp[CONTROL_AXIS];
  const float lower = target * (1.0f - tolerancePct / 100.0f);
  const float upper = target * (1.0f + tolerancePct / 100.0f);
  return amplitude >= lower && amplitude <= upper;
}

static bool actuatorMeasurementXAxisDominant(const StableMeasurement &m,
                                               float margin = 1.0f) {
  const float strongestOther = fmaxf(m.amplitude[1], m.amplitude[2]);
  return isfinite(m.amplitude[0]) &&
         m.amplitude[0] >= margin * strongestOther;
}

static bool confirmFixedDac(float dac, StableMeasurement &accepted,
                            const char *heading) {
  Serial.println();
  Serial.printf("%s AT FIXED DAC %.2f\n", heading, dac);

  StableMeasurement checks[3];
  uint8_t acceptedIndex[3];
  uint8_t acceptedCount = 0;

  for (uint8_t i = 0; i < 3; ++i) {
    char label[32];
    snprintf(label, sizeof(label), "CONFIRM %u", i + 1);
    checks[i] = measureActuatorPoint(dac, label);
    if (measurementHardwareFault) return false;
    if (checks[i].valid &&
        amplitudeInBand(checks[i].amplitude[CONTROL_AXIS], amplitudeTolerancePct)) {
      acceptedIndex[acceptedCount++] = i;
    }
    if (acceptedCount >= 2) break;
  }

  if (acceptedCount < 2) return false;

  const float a = checks[acceptedIndex[0]].amplitude[CONTROL_AXIS];
  const float b = checks[acceptedIndex[1]].amplitude[CONTROL_AXIS];
  const float driftPct = 100.0f * fabsf(a - b) / fmaxf(0.5f * (a + b), 1e-6f);
  if (driftPct > 8.0f) {
    Serial.printf("CONFIRMATION REJECTED: accepted-pair drift %.2f%% > 8%%.\n",
                  driftPct);
    return false;
  }

  accepted = checks[acceptedIndex[0]];
  for (uint8_t axis = 0; axis < 3; ++axis) {
    accepted.amplitude[axis] = 0.5f *
        (checks[acceptedIndex[0]].amplitude[axis] +
         checks[acceptedIndex[1]].amplitude[axis]);
  }
  accepted.total = 0.5f *
      (checks[acceptedIndex[0]].total + checks[acceptedIndex[1]].total);
  Serial.printf("CONFIRMATION ACCEPTED | pair drift %.2f%%\n", driftPct);
  return true;
}

static bool confirmStableDac(float dac, StableMeasurement &accepted,
                             const char *heading) {
  Serial.println();
  Serial.printf("%s AT FIXED DAC %.2f\n", heading, dac);

  StableMeasurement checks[3];
  uint8_t acceptedIndex[3];
  uint8_t acceptedCount = 0;

  for (uint8_t i = 0; i < 3; ++i) {
    char label[32];
    snprintf(label, sizeof(label), "CONFIRM %u", i + 1);
    checks[i] = measureActuatorPoint(dac, label);
    if (measurementHardwareFault) return false;
    if (checks[i].valid && actuatorMeasurementXAxisDominant(checks[i], 0.95f)) {
      acceptedIndex[acceptedCount++] = i;
    }
    if (acceptedCount >= 2) break;
  }

  if (acceptedCount < 2) return false;

  const float a = checks[acceptedIndex[0]].amplitude[CONTROL_AXIS];
  const float b = checks[acceptedIndex[1]].amplitude[CONTROL_AXIS];
  const float driftPct =
      100.0f * fabsf(a - b) / fmaxf(0.5f * (a + b), 1e-6f);
  if (driftPct > 12.0f) {
    Serial.printf("STABLE-DAC CONFIRMATION REJECTED: pair drift %.2f%% > 12%%.\n",
                  driftPct);
    return false;
  }

  accepted = checks[acceptedIndex[0]];
  for (uint8_t axis = 0; axis < 3; ++axis) {
    accepted.amplitude[axis] =
        0.5f * (checks[acceptedIndex[0]].amplitude[axis] +
                checks[acceptedIndex[1]].amplitude[axis]);
  }
  accepted.total =
      0.5f * (checks[acceptedIndex[0]].total +
              checks[acceptedIndex[1]].total);
  Serial.printf("STABLE PARTIAL-AUTHORITY DAC ACCEPTED | pair drift %.2f%%\n",
                driftPct);
  return true;
}

static bool runAmplitudeMatcher() {
  if (!profileStored) {
    Serial.println("NO STORED PROFILE. Tool ON, Visaton OFF: enter b first.");
    return false;
  }

  holdActive = false;
  setOutputEnabled(false);
  setMatcherPhaseZero();
  exactAmplitudeMatch = false;
  amplitudeMatched = false;

  if (!ensureADXLReady("before X-axis precheck")) return false;

  if (!verifyVisatonXAxisDominance()) {
    setOutputEnabled(false);
    return false;
  }

  const float target = localTargetAmp[CONTROL_AXIS];
  const float lower = target * (1.0f - amplitudeTolerancePct / 100.0f);
  const float upper = target * (1.0f + amplitudeTolerancePct / 100.0f);
  const float innerLower =
      target * (1.0f - INNER_TOLERANCE_PCT / 100.0f);
  const float innerUpper =
      target * (1.0f + INNER_TOLERANCE_PCT / 100.0f);

  Serial.println();
  Serial.println("====================================================================");
  Serial.println("FROZEN FAST ROBUST AMPLITUDE MATCHER — TOOL OFF");
  Serial.printf("Frequency: %.6f Hz\n", operatingFrequencyHz);
  Serial.printf("Target single-ADXL X: %.5f m/s^2 | final band %.5f to %.5f\n",
                target, lower, upper);
  Serial.printf("Inner goal: %.5f to %.5f | DAC resolution %.2f\n",
                innerLower, innerUpper, DAC_RESOLUTION);
  Serial.println("Each DAC point is now switched OFF after measurement to prevent amplifier protection and ADXL reset.");
  Serial.println("If an exact match is physically unreachable, the strongest stable X-dominant DAC is accepted for a phase-proof sweep.");
  Serial.println("====================================================================");

  const float probeDac[] = {
      1, 3, 5, 7, 9, 12, 17, 22, 27, 32, 38, 45, 55, 70, 90, 110
  };
  const uint8_t probeCount = sizeof(probeDac) / sizeof(probeDac[0]);

  bool haveLow = false;
  bool haveHigh = false;
  float lowDac = 0.0f;
  float lowAmp = 0.0f;
  float highDac = 0.0f;
  float highAmp = 0.0f;
  float candidateDac = NAN;
  StableMeasurement candidateMeasurement = {};

  bool haveBestStable = false;
  float bestStableDac = 0.0f;
  StableMeasurement bestStable = {};
  float largestStableX = 0.0f;

  uint32_t startMs = millis();
  uint8_t measurementNumber = 0;

  for (uint8_t i = 0; i < probeCount; ++i) {
    ++measurementNumber;
    char label[32];
    snprintf(label, sizeof(label), "SEARCH %u", measurementNumber);

    StableMeasurement point = measureActuatorPoint(probeDac[i], label);

    if (measurementHardwareFault) {
      setOutputEnabled(false);
      if (haveBestStable) {
        Serial.println("MATCHER STOPPED AT THE LAST SAFE RESPONSE AFTER A SENSOR/OUTPUT FAULT.");
        Serial.println("The program will attempt a stable partial-authority fallback instead of discarding all progress.");
        break;
      }
      Serial.println("MATCH ABORTED: no safe stable actuator point existed before the repeated fault.");
      return false;
    }

    if (!point.valid) continue;

    const float amp = point.amplitude[CONTROL_AXIS];
    const bool xDominant = actuatorMeasurementXAxisDominant(point, 0.95f);

    if (xDominant && (!haveBestStable || amp > bestStable.amplitude[0])) {
      haveBestStable = true;
      bestStableDac = probeDac[i];
      bestStable = point;
    }

    // Detect amplifier protection or electrical collapse: response falls sharply
    // even though DAC increased. Do not continue driving harder.
    if (largestStableX > 0.0f &&
        amp < RESPONSE_COLLAPSE_RATIO * largestStableX &&
        probeDac[i] > bestStableDac) {
      Serial.printf("RESPONSE COLLAPSE DETECTED at DAC %.2f: X fell from %.5f to %.5f.\n",
                    probeDac[i], largestStableX, amp);
      Serial.println("Search stopped before further heating/protection. Last stable point will be considered.");
      break;
    }
    if (amp > largestStableX) largestStableX = amp;

    if (amp < target) {
      haveLow = true;
      lowDac = probeDac[i];
      lowAmp = amp;
    } else {
      haveHigh = true;
      highDac = probeDac[i];
      highAmp = amp;
    }

    if (amp >= innerLower && amp <= innerUpper) {
      candidateDac = probeDac[i];
      candidateMeasurement = point;
      break;
    }

    if (haveLow && haveHigh) break;

    if (amp >= lower && amp <= upper) {
      candidateDac = probeDac[i];
      candidateMeasurement = point;
      break;
    }
  }

  if (!isfinite(candidateDac) &&
      haveLow && haveHigh && highAmp > lowAmp) {
    candidateDac =
        lowDac + (target - lowAmp) *
        (highDac - lowDac) / (highAmp - lowAmp);
    candidateDac = quantizeDac(candidateDac);

    ++measurementNumber;
    char label[32];
    snprintf(label, sizeof(label), "INTERPOLATE %u", measurementNumber);
    candidateMeasurement = measureActuatorPoint(candidateDac, label);

    if (measurementHardwareFault) {
      candidateDac = NAN;
    }
  }

  StableMeasurement accepted = {};
  bool confirmed = false;

  if (isfinite(candidateDac)) {
    confirmed = confirmFixedDac(
        candidateDac, accepted,
        "FINAL TWO-OF-THREE EXACT-MATCH CONFIRMATION");

    if (!confirmed && !measurementHardwareFault) {
      const float measured =
          candidateMeasurement.amplitude[CONTROL_AXIS];

      if (candidateMeasurement.valid && measured > 0.01f) {
        float trimDac = candidateDac * target / measured;
        float delta = trimDac - candidateDac;

        if (delta > 2.0f) delta = 2.0f;
        if (delta < -2.0f) delta = -2.0f;

        trimDac = quantizeDac(candidateDac + delta);

        if (fabsf(trimDac - candidateDac) >= 0.24f) {
          Serial.printf("\nONE BOUNDED TRIM: %.2f -> %.2f\n",
                        candidateDac, trimDac);
          candidateDac = trimDac;
          confirmed = confirmFixedDac(
              candidateDac, accepted,
              "TRIMMED TWO-OF-THREE EXACT-MATCH CONFIRMATION");
        }
      }
    }

    if (confirmed && !actuatorMeasurementXAxisDominant(accepted, 0.95f)) {
      Serial.println("EXACT-MATCH POINT REJECTED: Visaton response was no longer X-dominant at the accepted DAC.");
      confirmed = false;
    }

    if (confirmed) {
      exactAmplitudeMatch = true;
    }
  }

  // Safe fallback: exact matching may be physically impossible because the
  // actuator has less X authority than the tool. A smaller opposing vector can
  // still demonstrate attenuation. Do not block the actual phase experiment.
  if (!confirmed) {
    setOutputEnabled(false);

    if (!haveBestStable) {
      Serial.println("MATCH FAILED: no stable X-dominant actuator point was available.");
      Serial.println("Correct the physical mounting/power path before phase sweeping.");
      return false;
    }

    const float authorityRatio =
        bestStable.amplitude[0] / fmaxf(target, 1e-6f);

    Serial.println();
    Serial.println("EXACT X-AMPLITUDE MATCH WAS NOT PHYSICALLY REACHED.");
    Serial.printf("Strongest safe X-dominant point: DAC %.2f | X %.5f | authority %.2f%% of tool X\n",
                  bestStableDac, bestStable.amplitude[0],
                  100.0f * authorityRatio);

    if (authorityRatio < MIN_PARTIAL_AUTHORITY_RATIO) {
      Serial.printf("FALLBACK REJECTED: actuator X authority is below %.1f%% of tool X.\n",
                    100.0f * MIN_PARTIAL_AUTHORITY_RATIO);
      Serial.println("A phase sweep would be buried in measurement variation.");
      return false;
    }

    if (!confirmStableDac(
            bestStableDac, accepted,
            "PARTIAL-AUTHORITY TWO-OF-THREE CONFIRMATION")) {
      Serial.println("FALLBACK FAILED: the strongest safe DAC was not repeatable.");
      return false;
    }

    candidateDac = bestStableDac;
    confirmed = true;
    exactAmplitudeMatch = false;
  }

  setOutputEnabled(false);

  if (!confirmed) {
    Serial.println("MATCH FAILED: no repeatable actuator setting was acquired.");
    return false;
  }

  matchedDac = candidateDac;
  matchedControlAmp = accepted.amplitude[CONTROL_AXIS];
  amplitudeMatched = true;
  phaseSweepCompleted = false;

  const float errorPct =
      100.0f * (matchedControlAmp - target) /
      fmaxf(target, 1e-6f);
  const float authorityPct =
      100.0f * matchedControlAmp /
      fmaxf(target, 1e-6f);

  Serial.println();
  Serial.println("====================================================================");
  if (exactAmplitudeMatch) {
    Serial.println("EXACT AMPLITUDE MATCH ACQUIRED AND STORED");
  } else {
    Serial.println("SAFE PARTIAL-AUTHORITY AMPLITUDE STORED FOR PHASE PROOF");
  }
  Serial.printf("Final DAC: %.2f\n", matchedDac);
  Serial.printf("Validated single-ADXL X: %.5f m/s^2 | error %+.2f%% | authority %.2f%%\n",
                matchedControlAmp, errorPct, authorityPct);
  Serial.printf("Actuator X/Y/Z/TOTAL: %.5f / %.5f / %.5f / %.5f m/s^2\n",
                accepted.amplitude[0], accepted.amplitude[1],
                accepted.amplitude[2], accepted.total);
  Serial.printf("Matcher time: %.3f s\n",
                (millis() - startMs) / 1000.0f);

  if (!exactAmplitudeMatch) {
    Serial.println("The sweep can still show attenuation, but the maximum possible X reduction is limited by actuator authority.");
  }
  if (accepted.total >
      3.0f * fmaxf(localTargetTotal, 0.05f)) {
    Serial.println("WARNING: actuator total vibration is more than 3x the stored tool-only total.");
  }

  Serial.println("Visaton is OFF. Turn the tool ON and enter w.");
  Serial.println("====================================================================");
  return true;
}

// =============================================================================
// One-time stored phase anchor and local baseline (NO live tracking)
// =============================================================================
static IQBlockResult capturePhaseLocalBlock(float &referenceAmp,
                                            float &referencePhaseDeg) {
  IQBlockResult local = {};
  uint32_t nextSampleUs = micros();
  uint16_t impossibleSamples = 0;
  uint16_t longestIdenticalRun = 0;
  uint16_t identicalRun = 0;
  RawAccel previous = {};

  lastIqRawFault = false;

  for (uint16_t i = 0; i < PHASE_BLOCK_N; ++i) {
    nextSampleUs += SAMPLE_PERIOD_US;
    waitUntilMicros(nextSampleUs);
    phaseWords[i] = ncoPhaseAcc;
    const RawAccel sample = readRawXYZ(ADXL_CS);

    const bool allZero =
        sample.x == 0 && sample.y == 0 && sample.z == 0;
    const bool allMinusOne =
        sample.x == -1 && sample.y == -1 && sample.z == -1;
    if (allZero || allMinusOne) ++impossibleSamples;

    if (i > 0 && sample.x == previous.x &&
        sample.y == previous.y && sample.z == previous.z) {
      ++identicalRun;
      if (identicalRun > longestIdenticalRun) {
        longestIdenticalRun = identicalRun;
      }
    } else {
      identicalRun = 0;
    }
    previous = sample;

    phaseRefRaw[i] = axisRaw(sample, referenceAxis);
    phaseLocalRaw[0][i] = sample.x;
    phaseLocalRaw[1][i] = sample.y;
    phaseLocalRaw[2][i] = sample.z;
  }

  if (impossibleSamples > PHASE_BLOCK_N / 20 ||
      longestIdenticalRun > PHASE_BLOCK_N / 2) {
    lastIqRawFault = true;
  }

  projectSignalAgainstNco(phaseRefRaw, phaseWords, PHASE_BLOCK_N,
                          referenceAmp, referencePhaseDeg);
  for (uint8_t axis = 0; axis < 3; ++axis) {
    projectSignalAgainstNco(phaseLocalRaw[axis], phaseWords, PHASE_BLOCK_N,
                            local.amplitude[axis], local.phaseDeg[axis]);
  }
  local.total = sqrtf(local.amplitude[0] * local.amplitude[0] +
                      local.amplitude[1] * local.amplitude[1] +
                      local.amplitude[2] * local.amplitude[2]);
  return local;
}

static bool fitReferencePhaseTrend(const float *phaseDeg, uint8_t count,
                                   float blockSeconds, float &phaseAtEndDeg,
                                   float &dfHz, float &rmseDeg) {
  if (count < 5) return false;
  float unwrapped[BASELINE_BLOCKS];
  float times[BASELINE_BLOCKS];
  unwrapped[0] = phaseDeg[0] * PI / 180.0f;
  times[0] = 0.5f * blockSeconds;
  for (uint8_t i = 1; i < count; ++i) {
    float phase = phaseDeg[i] * PI / 180.0f;
    while (phase - unwrapped[i - 1] > PI) phase -= TWO_PI;
    while (phase - unwrapped[i - 1] < -PI) phase += TWO_PI;
    unwrapped[i] = phase;
    times[i] = (i + 0.5f) * blockSeconds;
  }

  double sumT = 0.0, sumP = 0.0;
  for (uint8_t i = 0; i < count; ++i) {
    sumT += times[i];
    sumP += unwrapped[i];
  }
  const double meanT = sumT / count;
  const double meanP = sumP / count;
  double numerator = 0.0, denominator = 0.0;
  for (uint8_t i = 0; i < count; ++i) {
    const double dt = times[i] - meanT;
    numerator += dt * (unwrapped[i] - meanP);
    denominator += dt * dt;
  }
  if (denominator <= 0.0) return false;
  const double slope = numerator / denominator;
  const double intercept = meanP - slope * meanT;
  double ss = 0.0;
  for (uint8_t i = 0; i < count; ++i) {
    const double error = unwrapped[i] - (intercept + slope * times[i]);
    ss += error * error;
  }

  const double endTime = count * blockSeconds;
  phaseAtEndDeg = wrap360((float)((intercept + slope * endTime) * 180.0 / PI));
  dfHz = (float)(slope / TWO_PI);
  rmseDeg = (float)(sqrt(ss / count) * 180.0 / PI);
  return isfinite(phaseAtEndDeg) && isfinite(dfHz) && isfinite(rmseDeg);
}

static PhaseBaseline acquirePhaseBaseline(uint8_t blocks, bool printProgress) {
  PhaseBaseline result = {};
  float refAmp[BASELINE_BLOCKS];
  float refPhase[BASELINE_BLOCKS];
  float localValues[3][BASELINE_BLOCKS];
  float totalValues[BASELINE_BLOCKS];

  if (blocks > BASELINE_BLOCKS) blocks = BASELINE_BLOCKS;
  uint8_t validCount = 0;
  uint8_t rawFaultCount = 0;
  for (uint8_t block = 0; block < blocks; ++block) {
    float referenceAmp = 0.0f;
    float referencePhase = 0.0f;
    IQBlockResult local = capturePhaseLocalBlock(referenceAmp, referencePhase);
    if (lastIqRawFault) {
      ++rawFaultCount;
      continue;
    }
    if (isfinite(referenceAmp) && referenceAmp >= MIN_TOOL_REFERENCE_AMP) {
      refAmp[validCount] = referenceAmp;
      refPhase[validCount] = referencePhase;
      for (uint8_t axis = 0; axis < 3; ++axis) {
        localValues[axis][validCount] = local.amplitude[axis];
      }
      totalValues[validCount] = local.total;
      ++validCount;
    }
    if (printProgress && ((block + 1) % 5 == 0 || block + 1 == blocks)) {
      Serial.printf("  baseline blocks %u/%u | valid %u\n",
                    block + 1, blocks, validCount);
    }
  }

  if (rawFaultCount > blocks / 4) return result;
  if (validCount < blocks / 2 || validCount < 5) return result;

  result.referenceAmplitude = medianOf(refAmp, validCount);
  for (uint8_t axis = 0; axis < 3; ++axis) {
    result.localAmp[axis] = medianOf(localValues[axis], validCount);
  }
  result.localTotal = medianOf(totalValues, validCount);
  result.localControlMadPct = madPercent(localValues[CONTROL_AXIS], validCount,
                                         result.localAmp[CONTROL_AXIS]);
  result.localControlCvPct = cvPercent(localValues[CONTROL_AXIS], validCount);

  const float blockSeconds = (float)PHASE_BLOCK_N / SAMPLE_RATE_HZ;
  result.valid = fitReferencePhaseTrend(refPhase, validCount, blockSeconds,
                                        result.referencePhaseEndDeg,
                                        result.residualDfHz,
                                        result.referencePhaseRmseDeg);
  return result;
}

static bool acquireOneTimePhaseAnchorAndBaseline(PhaseBaseline &baseline) {
  setOutputEnabled(false);
  delay(OUTPUT_POINT_COOLDOWN_MS);
  if (!ensureADXLReady("before initial phase anchor")) return false;
  setMatcherPhaseZero();

  Serial.println();
  Serial.println("INITIAL SINGLE-ADXL QUADRATURE RESYNCHRONIZATION — VISATON OFF");

  PhaseBaseline pass1 = acquirePhaseBaseline(12, false);
  if (!pass1.valid || pass1.referenceAmplitude < MIN_TOOL_REFERENCE_AMP) {
    Serial.println("PHASE ANCHOR FAILED: single-ADXL reference is missing or invalid.");
    return false;
  }

  float correction1 = constrain(pass1.residualDfHz, -0.50f, 0.50f);
  Serial.printf("Resync pass 1 | ADXL %s amp %.5f | df %+.6f Hz | RMSE %.2f deg\n",
                axisName(referenceAxis), pass1.referenceAmplitude,
                pass1.residualDfHz, pass1.referencePhaseRmseDeg);
  if (fabsf(correction1) > 0.002f && pass1.referencePhaseRmseDeg <= 35.0f) {
    operatingFrequencyHz += correction1;
    applyNcoFrequency();
    Serial.printf("Resync correction 1 applied: %.6f Hz\n", operatingFrequencyHz);
    delay(100);
  }

  PhaseBaseline pass2 = acquirePhaseBaseline(12, false);
  if (!pass2.valid || pass2.referenceAmplitude < MIN_TOOL_REFERENCE_AMP) {
    Serial.println("PHASE ANCHOR FAILED DURING RESYNC PASS 2.");
    return false;
  }

  float correction2 = constrain(pass2.residualDfHz, -0.25f, 0.25f);
  Serial.printf("Resync pass 2 | ADXL %s amp %.5f | df %+.6f Hz | RMSE %.2f deg\n",
                axisName(referenceAxis), pass2.referenceAmplitude,
                pass2.residualDfHz, pass2.referencePhaseRmseDeg);
  if (fabsf(correction2) > 0.002f && pass2.referencePhaseRmseDeg <= 35.0f) {
    operatingFrequencyHz += correction2;
    applyNcoFrequency();
    Serial.printf("Resync correction 2 applied: %.6f Hz\n", operatingFrequencyHz);
    delay(100);
  }

  baseline = acquirePhaseBaseline(BASELINE_BLOCKS, true);
  if (!baseline.valid || baseline.referenceAmplitude < MIN_TOOL_REFERENCE_AMP) {
    Serial.println("PHASE ANCHOR FAILED DURING FINAL BASELINE.");
    return false;
  }

  // One extra bounded correction is allowed because every later point is also
  // independently re-anchored with the Visaton OFF.
  if (fabsf(baseline.residualDfHz) > 0.03f &&
      baseline.referencePhaseRmseDeg <= 30.0f) {
    const float correction3 = constrain(baseline.residualDfHz, -0.20f, 0.20f);
    operatingFrequencyHz += correction3;
    applyNcoFrequency();
    Serial.printf("Final bounded frequency correction applied: %.6f Hz\n",
                  operatingFrequencyHz);
    delay(100);
    baseline = acquirePhaseBaseline(BASELINE_BLOCKS, true);
    if (!baseline.valid || baseline.referenceAmplitude < MIN_TOOL_REFERENCE_AMP) {
      Serial.println("PHASE ANCHOR FAILED AFTER FINAL FREQUENCY CORRECTION.");
      return false;
    }
  }

  storedToolPhaseDeg = baseline.referencePhaseEndDeg;
  Serial.printf("INITIAL STORED PHASE ANCHOR: %.2f deg relative to ESP32 NCO\n",
                storedToolPhaseDeg);
  Serial.printf("Residual df after initial resync: %+.6f Hz | phase RMSE %.2f deg\n",
                baseline.residualDfHz, baseline.referencePhaseRmseDeg);
  Serial.printf("TOOL-ONLY LOCAL BASELINE X/Y/Z/T: %.5f / %.5f / %.5f / %.5f m/s^2\n",
                baseline.localAmp[0], baseline.localAmp[1],
                baseline.localAmp[2], baseline.localTotal);
  Serial.printf("Baseline X MAD %.2f%% | CV %.2f%%\n",
                baseline.localControlMadPct, baseline.localControlCvPct);

  if (fabsf(baseline.residualDfHz) > MAX_POINT_RESIDUAL_DF_HZ) {
    Serial.println("SWEEP STOPPED: tool frequency is changing too quickly even for stepwise synchronization.");
    return false;
  }
  if (fabsf(baseline.residualDfHz) > 0.03f) {
    Serial.println("NOTE: residual drift remains, but each phase point will be re-anchored separately.");
  }
  return true;
}

// Reacquire tool phase with the Visaton OFF immediately before one measured
// phase point. This is stepwise synchronization, not continuous live tracking.
static bool relockPhaseForPoint(float relativePhaseDeg, const char *label) {
  setOutputEnabled(false);
  delay(OUTPUT_POINT_COOLDOWN_MS);
  if (!ensureADXLReady("before phase-point anchor")) {
    Serial.printf("%s %.1f deg | ADXL RECOVERY FAILED BEFORE ANCHOR\n",
                  label, wrap360(relativePhaseDeg));
    return false;
  }
  setMatcherPhaseZero();

  PhaseBaseline first = acquirePhaseBaseline(POINT_ANCHOR_BLOCKS, false);
  if (!first.valid || first.referenceAmplitude < MIN_TOOL_REFERENCE_AMP) {
    Serial.printf("%s %.1f deg | ANCHOR FAILED: weak/invalid single-ADXL reference\n",
                  label, wrap360(relativePhaseDeg));
    return false;
  }

  if (fabsf(first.residualDfHz) > 0.003f &&
      first.referencePhaseRmseDeg <= 30.0f) {
    const float correction = constrain(first.residualDfHz, -0.30f, 0.30f);
    operatingFrequencyHz += correction;
    applyNcoFrequency();
    delay(70);
  }

  PhaseBaseline finalAnchor = acquirePhaseBaseline(POINT_ANCHOR_BLOCKS, false);
  if (!finalAnchor.valid || finalAnchor.referenceAmplitude < MIN_TOOL_REFERENCE_AMP) {
    Serial.printf("%s %.1f deg | FINAL ANCHOR FAILED\n",
                  label, wrap360(relativePhaseDeg));
    return false;
  }

  // A second small correction is permitted if the first correction did not
  // fully settle. Then take one final short anchor.
  if (fabsf(finalAnchor.residualDfHz) > 0.05f &&
      finalAnchor.referencePhaseRmseDeg <= 25.0f) {
    const float correction = constrain(finalAnchor.residualDfHz, -0.15f, 0.15f);
    operatingFrequencyHz += correction;
    applyNcoFrequency();
    delay(70);
    finalAnchor = acquirePhaseBaseline(POINT_ANCHOR_BLOCKS, false);
  }

  if (!finalAnchor.valid ||
      finalAnchor.referenceAmplitude < MIN_TOOL_REFERENCE_AMP ||
      finalAnchor.referencePhaseRmseDeg > 35.0f ||
      fabsf(finalAnchor.residualDfHz) > MAX_POINT_RESIDUAL_DF_HZ) {
    Serial.printf("%s %.1f deg | ANCHOR REJECTED | amp %.4f | df %+.5f | RMSE %.2f\n",
                  label, wrap360(relativePhaseDeg),
                  finalAnchor.referenceAmplitude, finalAnchor.residualDfHz,
                  finalAnchor.referencePhaseRmseDeg);
    return false;
  }

  storedToolPhaseDeg = finalAnchor.referencePhaseEndDeg;
  setOpenLoopRelativePhase(relativePhaseDeg);
  Serial.printf("%s %.1f deg | RE-LOCKED | f %.6f Hz | ref %.4f | df %+.5f | RMSE %.2f\n",
                label, wrap360(relativePhaseDeg), operatingFrequencyHz,
                finalAnchor.referenceAmplitude, finalAnchor.residualDfHz,
                finalAnchor.referencePhaseRmseDeg);
  return true;
}

// =============================================================================
// Local measurement during stored-phase sweep (ADXL only)
// =============================================================================
static StableMeasurement measureLocalPhasePoint(uint8_t blocks) {
  StableMeasurement result = {};
  float axisValues[3][PROOF_BLOCKS];
  float totalValues[PROOF_BLOCKS];
  float controlPhase[PROOF_BLOCKS];
  bool captureFault = false;

  if (blocks > PROOF_BLOCKS) blocks = PROOF_BLOCKS;

  for (uint8_t block = 0; block < blocks; ++block) {
    IQBlockResult iq = captureLocalIqBlock(PHASE_BLOCK_N);
    if (lastIqRawFault) captureFault = true;
    for (uint8_t axis = 0; axis < 3; ++axis) {
      axisValues[axis][block] = iq.amplitude[axis];
    }
    totalValues[block] = iq.total;
    controlPhase[block] = iq.phaseDeg[CONTROL_AXIS];
  }

  result.count = blocks;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    result.amplitude[axis] =
        medianOf(axisValues[axis], blocks);
  }
  result.total = medianOf(totalValues, blocks);
  result.controlMadPct =
      madPercent(axisValues[CONTROL_AXIS], blocks,
                 result.amplitude[CONTROL_AXIS]);
  result.controlCvPct =
      cvPercent(axisValues[CONTROL_AXIS], blocks);
  result.controlCoherencePct =
      circularCoherencePct(controlPhase, blocks);

  result.valid =
      !captureFault &&
      isfinite(result.amplitude[0]) &&
      isfinite(result.amplitude[1]) &&
      isfinite(result.amplitude[2]) &&
      isfinite(result.total) &&
      isfinite(result.controlMadPct) &&
      isfinite(result.controlCvPct);

  return result;
}

static void printPhaseMeasurement(const char *stage, float phaseDeg,
                                  const StableMeasurement &m,
                                  float baselineX) {
  const float reduction = 100.0f * (baselineX - m.amplitude[0]) /
                          fmaxf(baselineX, 1e-6f);
  Serial.printf("%s %6.1f deg | X %.5f | Y/Z/T %.5f / %.5f / %.5f | X reduction %+.2f%% | MAD %.2f%% | CV %.2f%%\n",
                stage, wrap360(phaseDeg), m.amplitude[0], m.amplitude[1],
                m.amplitude[2], m.total, reduction,
                m.controlMadPct, m.controlCvPct);
}

static int bestSweepIndex(const SweepPoint *points, uint8_t count) {
  int best = -1;
  float minimum = INFINITY;
  for (uint8_t i = 0; i < count; ++i) {
    if (points[i].measurement.valid &&
        points[i].measurement.amplitude[CONTROL_AXIS] < minimum) {
      minimum = points[i].measurement.amplitude[CONTROL_AXIS];
      best = i;
    }
  }
  return best;
}

static int bestTotalIndex(const SweepPoint *points, uint8_t count) {
  int best = -1;
  float minimum = INFINITY;
  for (uint8_t i = 0; i < count; ++i) {
    if (points[i].measurement.valid && points[i].measurement.total < minimum) {
      minimum = points[i].measurement.total;
      best = i;
    }
  }
  return best;
}

static StableMeasurement measureStoredPhase(float phaseDeg, uint8_t blocks,
                                             const char *stage,
                                             float baselineX) {
  StableMeasurement result = {};

  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    setOutputEnabled(false);
    delay(OUTPUT_POINT_COOLDOWN_MS + (attempt ? 350 : 0));

    if (!relockPhaseForPoint(phaseDeg, stage)) {
      if (attempt == 0 && recoverADXL("after failed phase anchor")) {
        continue;
      }
      result.valid = false;
      return result;
    }

    setOutputEnabled(true);
    delay(PHASE_SETTLE_MS);
    result = measureLocalPhasePoint(blocks);
    setOutputEnabled(false);
    delay(120);

    const bool sensorHealthy = adxlRegisterHealthCheck(false);
    if (result.valid && sensorHealthy) {
      printPhaseMeasurement(stage, phaseDeg, result, baselineX);
      return result;
    }

    if (!sensorHealthy) adxlRegisterHealthCheck(true);
    Serial.printf("%s %.1f deg | PHASE POINT RETRY %u/2 AFTER INVALID CAPTURE/SPI RESET\n",
                  stage, wrap360(phaseDeg), attempt + 1);
    if (!recoverADXL("after invalid phase measurement")) {
      result.valid = false;
      return result;
    }
  }

  result.valid = false;
  printPhaseMeasurement(stage, phaseDeg, result, baselineX);
  return result;
}

static bool runStoredPhaseSweep() {
  if (!profileStored) {
    Serial.println("NO STORED PROFILE. Tool ON, Visaton OFF: enter b.");
    return false;
  }
  if (!amplitudeMatched) {
    Serial.println("NO STORED MATCHED DAC. Tool OFF: enter m.");
    return false;
  }

  holdActive = false;
  setOutputEnabled(false);
  applyNcoAmplitude(matchedDac);

  Serial.println();
  Serial.println("====================================================================");
  Serial.println("STEP-SYNCHRONIZED PHASE SWEEP — TOOL ON");
  Serial.println("The single ADXL is sampled with Visaton OFF before every phase point.");
  Serial.println("The same ADXL X axis scores the sweep; Y/Z/total are always reported.");
  Serial.println("Each measured phase is fixed; no continuous live tracking is used.");
  Serial.println("====================================================================");

  PhaseBaseline preBaseline;
  if (!acquireOneTimePhaseAnchorAndBaseline(preBaseline)) {
    setOutputEnabled(false);
    return false;
  }

  const float baselineX = preBaseline.localAmp[0];
  if (baselineX < 0.05f) {
    Serial.println("SWEEP STOPPED: current single-ADXL X baseline is below 0.05 m/s^2.");
    return false;
  }

  // Compare current tool-only X with the profile target used by the matcher.
  const float targetDriftPct = 100.0f * fabsf(baselineX - localTargetAmp[0]) /
                               fmaxf(localTargetAmp[0], 1e-6f);
  Serial.printf("Current baseline vs stored matcher target drift: %.2f%%\n",
                targetDriftPct);
  if (targetDriftPct > 50.0f) {
    Serial.println("SWEEP STOPPED: current tool-only X differs by more than 50% from the stored profile.");
    Serial.println("The tool condition or mounting changed too much. Repeat b and m once.");
    return false;
  }
  if (targetDriftPct > 25.0f) {
    Serial.println("WARNING: current tool-only X changed by more than 25%, but the sweep will continue.");
    Serial.println("Results will be judged only against the fresh pre/post baselines measured inside w.");
  }

  Serial.println();
  Serial.println("1) COARSE SWEEP: 0 to 330 deg in 30-deg steps");
  SweepPoint coarse[12];
  for (uint8_t i = 0; i < 12; ++i) {
    coarse[i].phaseDeg = 30.0f * i;
    coarse[i].measurement = measureStoredPhase(coarse[i].phaseDeg,
                                                COARSE_BLOCKS,
                                                "COARSE", baselineX);
  }

  const int coarseBest = bestSweepIndex(coarse, 12);
  const int coarseTotalBest = bestTotalIndex(coarse, 12);
  if (coarseBest < 0) {
    setOutputEnabled(false);
    Serial.println("SWEEP FAILED: no valid coarse measurement.");
    return false;
  }
  Serial.printf("BEST COARSE X PHASE: %.1f deg | X %.5f m/s^2\n",
                coarse[coarseBest].phaseDeg,
                coarse[coarseBest].measurement.amplitude[0]);
  if (coarseTotalBest >= 0) {
    Serial.printf("BEST COARSE TOTAL PHASE: %.1f deg | total %.5f m/s^2\n",
                  coarse[coarseTotalBest].phaseDeg,
                  coarse[coarseTotalBest].measurement.total);
  }

  Serial.println();
  Serial.println("2) FINE SWEEP: best coarse +/-30 deg in 10-deg steps");
  SweepPoint fine[7];
  for (uint8_t i = 0; i < 7; ++i) {
    fine[i].phaseDeg = wrap360(coarse[coarseBest].phaseDeg - 30.0f + 10.0f * i);
    fine[i].measurement = measureStoredPhase(fine[i].phaseDeg,
                                              FINE_BLOCKS,
                                              "FINE  ", baselineX);
  }

  const int fineBest = bestSweepIndex(fine, 7);
  const int fineTotalBest = bestTotalIndex(fine, 7);
  if (fineBest < 0) {
    setOutputEnabled(false);
    Serial.println("SWEEP FAILED: no valid fine measurement.");
    return false;
  }
  bestPhaseDeg = fine[fineBest].phaseDeg;

  Serial.println();
  Serial.println("3) BEST-PHASE CONFIRMATION");
  StableMeasurement proof = measureStoredPhase(bestPhaseDeg, PROOF_BLOCKS,
                                                "BEST  ", baselineX);

  Serial.println();
  Serial.println("4) POST-SWEEP TOOL-ONLY BASELINE");
  setOutputEnabled(false);
  PhaseBaseline postBaseline = acquirePhaseBaseline(BASELINE_BLOCKS, false);

  float baselineUsed[3];
  float baselineTotal = preBaseline.localTotal;
  for (uint8_t axis = 0; axis < 3; ++axis) baselineUsed[axis] = preBaseline.localAmp[axis];
  float prePostXDriftPct = NAN;
  if (postBaseline.valid) {
    for (uint8_t axis = 0; axis < 3; ++axis) {
      baselineUsed[axis] = 0.5f * (preBaseline.localAmp[axis] +
                                   postBaseline.localAmp[axis]);
    }
    baselineTotal = 0.5f * (preBaseline.localTotal + postBaseline.localTotal);
    prePostXDriftPct = 100.0f * fabsf(postBaseline.localAmp[0] -
                                      preBaseline.localAmp[0]) /
                       fmaxf(preBaseline.localAmp[0], 1e-6f);
    Serial.printf("POST BASELINE X/Y/Z/T: %.5f / %.5f / %.5f / %.5f\n",
                  postBaseline.localAmp[0], postBaseline.localAmp[1],
                  postBaseline.localAmp[2], postBaseline.localTotal);
  } else {
    Serial.println("Post baseline unavailable; pre-baseline alone is used.");
  }

  const float xReductionPct = 100.0f *
      (baselineUsed[0] - proof.amplitude[0]) / fmaxf(baselineUsed[0], 1e-6f);
  const float totalReductionPct = 100.0f *
      (baselineTotal - proof.total) / fmaxf(baselineTotal, 1e-6f);

  phaseSweepCompleted = true;

  Serial.println();
  Serial.println("====================================================================");
  Serial.println("PHASE SWEEP RESULT");
  Serial.printf("Best X phase: %.1f deg\n", bestPhaseDeg);
  Serial.printf("Baseline used X/Y/Z/T: %.5f / %.5f / %.5f / %.5f\n",
                baselineUsed[0], baselineUsed[1], baselineUsed[2], baselineTotal);
  Serial.printf("Best proof X/Y/Z/T: %.5f / %.5f / %.5f / %.5f\n",
                proof.amplitude[0], proof.amplitude[1],
                proof.amplitude[2], proof.total);
  Serial.printf("X-AXIS REDUCTION: %+.2f%%\n", xReductionPct);
  Serial.printf("TOTAL-VECTOR REDUCTION: %+.2f%%\n", totalReductionPct);
  if (isfinite(prePostXDriftPct)) {
    Serial.printf("Pre/post tool-only X drift: %.2f%%\n", prePostXDriftPct);
  }
  if (fineTotalBest >= 0) {
    Serial.printf("Fine-sweep minimum-total phase: %.1f deg | total %.5f\n",
                  fine[fineTotalBest].phaseDeg,
                  fine[fineTotalBest].measurement.total);
  }

  if (xReductionPct < MIN_HOLD_START_REDUCTION_PCT) {
    Serial.println("NO VERIFIED X ATTENUATION. Fixed hold will NOT start.");
    Serial.println("Visaton is OFF. Do not call the least-bad phase a damping result.");
    setOutputEnabled(false);
    holdActive = false;
    Serial.println("====================================================================");
    return false;
  }

  // Start one final synchronized hold. After this one anchor, the actuator
  // phase remains fixed open-loop as requested; ADXL X/Y/Z/total are reported
  // once per second without continuous tracking or periodic phase correction.
  for (uint8_t axis = 0; axis < 3; ++axis) holdBaselineAmp[axis] = baselineUsed[axis];
  holdBaselineTotal = baselineTotal;
  applyNcoAmplitude(matchedDac);
  if (!relockPhaseForPoint(bestPhaseDeg, "HOLD START")) {
    Serial.println("HOLD COULD NOT START: final phase re-lock failed.");
    setOutputEnabled(false);
    holdActive = false;
    return false;
  }
  setOutputEnabled(true);
  holdActive = true;
  lastHoldCheckMs = millis();
  consecutiveBadHoldChecks = 0;
  holdCheckNumber = 0;

  Serial.println("VERIFIED X ATTENUATION: FIXED OPEN-LOOP BEST-PHASE HOLD STARTED.");
  Serial.println("The final phase anchor is now frozen. ADXL X/Y/Z/total are reported once per second.");
  Serial.println("No live tracking and no periodic phase correction are used during this hold. Enter s to stop.");
  Serial.println("====================================================================");
  return true;
}

// =============================================================================
// Fixed open-loop hold service: report one local measurement per second
// =============================================================================
static void serviceFixedHold() {
  if (!holdActive) return;
  if (millis() - lastHoldCheckMs < HOLD_INTERVAL_MS) return;
  lastHoldCheckMs = millis();

  // Output remains continuously ON at the best stored phase. This is the
  // deliberately simple open-loop hold requested by the user.
  StableMeasurement check = measureLocalPhasePoint(HOLD_BLOCKS);
  ++holdCheckNumber;

  if (!check.valid || !isfinite(check.amplitude[0])) {
    Serial.println("HOLD STOPPED: invalid raw ADXL measurement.");
    setOutputEnabled(false);
    holdActive = false;
    recoverADXL("after invalid fixed-hold measurement");
    return;
  }

  const float xReduction = 100.0f *
      (holdBaselineAmp[0] - check.amplitude[0]) /
      fmaxf(holdBaselineAmp[0], 1e-6f);
  const float totalReduction = 100.0f *
      (holdBaselineTotal - check.total) /
      fmaxf(holdBaselineTotal, 1e-6f);

  Serial.printf("HOLD %lu | fixed phase %.1f deg | X %.5f (%+.2f%%) | Y %.5f | Z %.5f | TOTAL %.5f (%+.2f%%) | MAD %.2f%% | CV %.2f%%\n",
                (unsigned long)holdCheckNumber, bestPhaseDeg,
                check.amplitude[0], xReduction,
                check.amplitude[1], check.amplitude[2],
                check.total, totalReduction,
                check.controlMadPct, check.controlCvPct);

  if (xReduction <= 0.0f) ++consecutiveBadHoldChecks;
  else consecutiveBadHoldChecks = 0;

  const bool excessiveX = check.amplitude[0] > 2.0f * holdBaselineAmp[0];
  const bool excessiveTotal = check.total > 5.0f * fmaxf(holdBaselineTotal, 0.05f);

  if (excessiveX || excessiveTotal) {
    Serial.println("SAFETY STOP: fixed phase strongly increased local vibration.");
    setOutputEnabled(false);
    holdActive = false;
    return;
  }

  if (consecutiveBadHoldChecks >= 3) {
    Serial.println("HOLD STOPPED: three consecutive one-second checks showed no X attenuation.");
    Serial.println("This indicates open-loop phase drift or loss of cancellation.");
    setOutputEnabled(false);
    holdActive = false;
  }
}

// =============================================================================
// Profile capture command
// =============================================================================
static bool captureAndStoreProfile() {
  holdActive = false;
  setOutputEnabled(false);
  profileStored = false;
  amplitudeMatched = false;
  exactAmplitudeMatch = false;
  actuatorXAxisVerified = false;
  measurementHardwareFault = false;
  measurementSensorFault = false;
  measurementOutputDropout = false;
  phaseSweepCompleted = false;
  matchedDac = 0.0f;

  Serial.println();
  Serial.println("====================================================================");
  Serial.println("SINGLE-ADXL TOOL PROFILE CAPTURE — TOOL ON, VISATON OFF");
  Serial.println("ONE ADXL: frozen 3-axis FFT + two-pass quadrature refinement");
  Serial.println("SAME ADXL: X/Y/Z amplitude; X is stored as matcher target");
  Serial.printf("For this 270-Hz tool test, the final profile must lie between %.0f and %.0f Hz.\n",
                EXPECTED_TOOL_MIN_HZ, EXPECTED_TOOL_MAX_HZ);
  Serial.println("The program automatically recovers/reinitializes the ADXL once if SPI was disturbed.");
  Serial.println("====================================================================");

  for (uint8_t attempt = 1; attempt <= 2; ++attempt) {
    if (!ensureADXLReady("before profile capture")) {
      Serial.println("PROFILE FAILED: ADXL could not be recovered.");
      return false;
    }

    Serial.printf("\nPROFILE ATTEMPT %u/2\n", attempt);
    float fftCenter = 0.0f;

    const bool fftOkay = runFrozenFft(fftCenter);
    const bool healthAfterFft = adxlRegisterHealthCheck(false);

    if (!fftOkay || !healthAfterFft) {
      if (!healthAfterFft) adxlRegisterHealthCheck(true);
      Serial.println("Profile attempt rejected after FFT; recovering the ADXL and retrying once.");
      recoverADXL("after invalid FFT profile");
      continue;
    }

    const bool quadOkay = runFrozenQuadrature(fftCenter);
    const bool frequencyPlausible =
        operatingFrequencyHz >= EXPECTED_TOOL_MIN_HZ &&
        operatingFrequencyHz <= EXPECTED_TOOL_MAX_HZ;
    const bool healthAfterQuad = adxlRegisterHealthCheck(false);

    if (!quadOkay || !frequencyPlausible || !healthAfterQuad) {
      if (!healthAfterQuad) adxlRegisterHealthCheck(true);
      if (!frequencyPlausible) {
        Serial.printf("PROFILE REJECTED: final %.6f Hz is outside the expected %.0f-%.0f Hz tool band.\n",
                      operatingFrequencyHz, EXPECTED_TOOL_MIN_HZ,
                      EXPECTED_TOOL_MAX_HZ);
        Serial.println("This prevents a corrupted/reset sensor from being accepted as a false 7-Hz profile.");
      }
      if (attempt < 2) {
        recoverADXL("after invalid quadrature profile");
        continue;
      }
      Serial.println("PROFILE FAILED AFTER TWO ATTEMPTS.");
      Serial.println("Make sure the tool is ON and the ADXL is rigidly connected.");
      return false;
    }

    profileStored = true;
    Serial.println();
    Serial.println("====================================================================");
    Serial.println("PROFILE STORED SUCCESSFULLY");
    Serial.printf("Frozen operating frequency: %.6f Hz\n",
                  operatingFrequencyHz);
    Serial.printf("Stored phase-reference axis: %s\n",
                  axisName(referenceAxis));
    Serial.printf("Single-ADXL X matcher target: %.5f m/s^2 peak\n",
                  localTargetAmp[0]);
    Serial.println("Now turn the TOOL OFF and enter m.");
    Serial.println("====================================================================");
    return true;
  }

  return false;
}

// =============================================================================
// Serial interface
// =============================================================================
static void printHelp() {
  Serial.println();
  Serial.println("====================================================================");
  Serial.println("KK-EXOSKELETON SINGLE-ADXL PHASE SWEEP V11.2 RECOVERY + SAFE FALLBACK");
  Serial.println("====================================================================");
  Serial.println("b   TOOL ON, VISATON OFF: frozen FFT + quadrature + local target");
  Serial.println("x   TOOL OFF: low-DAC Visaton X-axis dominance check only");
  Serial.println("m   TOOL OFF: X-axis precheck + frozen robust single-ADXL X amplitude matcher");
  Serial.println("w   TOOL ON: stepwise re-anchored sweep, proof, periodic hold");
  Serial.println("s   stop Visaton/fixed hold; preserve stored results");
  Serial.println("p   print state");
  Serial.println("h   print help");
  Serial.println();
  Serial.println("BENCH TEST: mount the ONE ADXL rigidly at the tool/Visaton cancellation point; align its X axis with the Visaton force direction.");
  Serial.println("EXACT ORDER: TOOL ON b -> TOOL OFF m -> TOOL ON w -> s");
  Serial.println("No continuous live tracking: the same ADXL re-locks only while Visaton is briefly OFF.");
  Serial.println("====================================================================");
}

static void printState() {
  Serial.println();
  Serial.println("================ CURRENT STATE ================");
  Serial.printf("Sensor: %s\n", sensorsReady ? "READY" : "NOT READY");
  Serial.printf("Profile stored: %s\n", profileStored ? "YES" : "NO");
  if (profileStored) {
    Serial.printf("Frequency: %.6f Hz | phase-reference axis %s\n",
                  operatingFrequencyHz, axisName(referenceAxis));
    Serial.printf("single-ADXL target X/Y/Z/T: %.5f / %.5f / %.5f / %.5f\n",
                  localTargetAmp[0], localTargetAmp[1],
                  localTargetAmp[2], localTargetTotal);
  }
  Serial.printf("Visaton X-axis verified: %s\n", actuatorXAxisVerified ? "YES" : "NO");
  Serial.printf("Amplitude setting stored: %s\n", amplitudeMatched ? "YES" : "NO");
  if (amplitudeMatched) {
    Serial.printf("Matched DAC %.2f | X %.5f | mode %s\n",
                  matchedDac, matchedControlAmp,
                  exactAmplitudeMatch ? "EXACT MATCH" : "SAFE PARTIAL AUTHORITY");
  }
  Serial.printf("Phase sweep completed: %s\n", phaseSweepCompleted ? "YES" : "NO");
  if (phaseSweepCompleted) Serial.printf("Best phase: %.1f deg\n", bestPhaseDeg);
  Serial.printf("Visaton: %s | fixed hold: %s\n",
                ncoOutputEnabled ? "ON" : "OFF",
                holdActive ? "ON" : "OFF");
  Serial.println("===============================================");
}

static void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  const char command = (char)tolower(line.charAt(0));

  if (command == 'h') {
    printHelp();
  } else if (command == 'p') {
    printState();
  } else if (command == 's') {
    holdActive = false;
    setOutputEnabled(false);
    Serial.println("VISATON OFF. Stored profile, matched DAC and best phase remain in RAM.");
  } else if (command == 'b') {
    captureAndStoreProfile();
  } else if (command == 'x') {
    verifyVisatonXAxisDominance();
  } else if (command == 'm') {
    runAmplitudeMatcher();
  } else if (command == 'w') {
    runStoredPhaseSweep();
  } else {
    Serial.println("Unknown command. Enter h.");
  }
}

// =============================================================================
// Arduino setup and loop
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1200);

  pinMode(ADXL_CS, OUTPUT);
  digitalWrite(ADXL_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  pinMode(DAC_PIN, OUTPUT);
  dacWrite(DAC_PIN, 128);

  for (uint16_t i = 0; i < 256; ++i) {
    sineTable[i] = (int8_t)lroundf(127.0f * sinf(TWO_PI * i / 256.0f));
  }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  dacTimer = timerBegin(1000000UL); // 1 MHz: 1 tick = 1 us
  if (dacTimer != nullptr) {
    timerAttachInterrupt(dacTimer, &onDacTimer);
    timerAlarm(dacTimer, DAC_TIMER_US, true, 0);
  }
#else
  dacTimer = timerBegin(0, 80, true);
  if (dacTimer != nullptr) {
    timerAttachInterrupt(dacTimer, &onDacTimer, true);
    timerAlarmWrite(dacTimer, DAC_TIMER_US, true);
    timerAlarmEnable(dacTimer);
  }
#endif

  if (dacTimer == nullptr) {
    Serial.println("FATAL: DAC hardware timer could not be created.");
    while (true) delay(1000);
  }

  Serial.printf("ESP32 Arduino core: %d.%d.%d\n",
                ESP_ARDUINO_VERSION_MAJOR,
                ESP_ARDUINO_VERSION_MINOR,
                ESP_ARDUINO_VERSION_PATCH);
  Serial.println("DAC TIMER READY: 10 kHz NCO update.");

  Serial.println();
  Serial.println("====================================================================");
  Serial.println("KK-EXOSKELETON SINGLE-ADXL PHASE SWEEP V11.2 RECOVERY + SAFE FALLBACK");
  Serial.println("ONE SENSOR | FROZEN FFT + QUADRATURE + MATCHER + PHASE SWEEP");
  Serial.println("====================================================================");

  sensorsReady = initializeADXL(ADXL_CS, "ADXL SINGLE GPIO17");

  if (sensorsReady) Serial.println("SINGLE ADXL345 SENSOR READY.");
  else Serial.println("STOP: ADXL345 check failed.");

  printHelp();
  printState();
}

void loop() {
  if (Serial.available()) {
    const String line = Serial.readStringUntil('\n');
    processCommand(line);
  }

  if (holdActive) serviceFixedHold();
  else delay(5);
}
