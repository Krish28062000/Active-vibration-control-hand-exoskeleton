#include <Arduino.h>
#include <SPI.h>
#include <arduinoFFT.h>
#include <math.h>

/*
  ============================================================================
  KK-EXOSKELETON — FROZEN DUAL-ADXL TRIAXIAL FFT + QUADRATURE ANALYSER V1
  ESP32 + 2 x ADXL345 over shared SPI
  ============================================================================

  SENSOR ROLES
    ADXL1, CS GPIO5  = TOOL / REFERENCE
    ADXL2, CS GPIO17 = HAND / BACK OF HAND

  PURPOSE
    1) Discover the mechanically relevant narrowband tool-frequency candidates
       WITHOUT a compiled-in tool frequency.
    2) Print each sensor axis's independent DOMINANT PEAK.
    3) Select/lock ONE common control frequency f0.
    4) Print X/Y/Z AMPLITUDE AT f0 for BOTH sensors using exact-frequency
       synchronous quadrature.
    5) Export the local ADXL2 X/Y/Z amplitudes and triaxial objective J so the
       later X/Y/Z amplitude matcher can consume the result directly.

  IMPORTANT
    - Independent X/Y/Z FFT peaks are DIAGNOSTIC ONLY.
    - The cancellation system will use ONE frequency f0.
    - ADXL1 and ADXL2 do NOT need aligned axes for this analyser because f0 is
      scalar frequency and ADXL2 is evaluated in its own local coordinate frame.
    - Shared SPI is paired acquisition, NOT hardware-synchronous phase between
      the two ADXL345 internal sample clocks. Do not interpret ADXL1 phase minus
      ADXL2 phase as laboratory-grade transfer phase.

  VALIDATED MEASUREMENT FOUNDATION RETAINED
    ADXL345 full resolution, +/-16 g
    Nominal ODR                  = 1600 Hz
    ESP32 sample-pair rate       = 1600 pairs/s
    FFT samples                  = 1024
    FFT frames averaged          = 3
    FFT resolution               = 1.5625 Hz/bin
    FFT window                   = Hamming
    FFT DC removal               = enabled
    Default discovery range      = 20..750 Hz (runtime configurable)
    Quadrature record            = 10 x 256 = 2560 uninterrupted sample pairs

  NO TOOL FREQUENCY IS HARD-CODED.

  COMMANDS
    a                 = full discovery + FFT + candidate validation + auto-lock
                        only when one candidate is clearly superior
    k N               = lock candidate N from the last discovery (N = 1..5)
    q                 = new exact-frequency quadrature measurement at locked f0
    b MIN MAX         = set runtime FFT discovery band in Hz, e.g. b 20 750
    p                 = print current settings / lock status
    x                 = clear the current f0 lock
    h                 = help

  SERIAL MONITOR
    115200 baud, Newline or Both NL & CR
  ============================================================================
*/

// ============================================================================
// HARDWARE
// ============================================================================
static const uint8_t PIN_CS_ADXL1 = 5;
static const uint8_t PIN_CS_ADXL2 = 17;
static const uint8_t PIN_SPI_SCK  = 18;
static const uint8_t PIN_SPI_MISO = 19;
static const uint8_t PIN_SPI_MOSI = 23;
static const uint8_t PIN_DAC_SAFE = 25;  // held at midscale; Visaton is not driven here

SPISettings adxlSpiSettings(5000000, MSBFIRST, SPI_MODE3);

// ADXL345 registers.
static const uint8_t REG_DEVID       = 0x00;
static const uint8_t REG_BW_RATE     = 0x2C;
static const uint8_t REG_POWER_CTL   = 0x2D;
static const uint8_t REG_DATA_FORMAT = 0x31;
static const uint8_t REG_DATAX0      = 0x32;
static const uint8_t SPI_READ_BIT    = 0x80;
static const uint8_t SPI_MB_BIT      = 0x40;

// ============================================================================
// VALIDATED SIGNAL SETTINGS
// ============================================================================
static const double SAMPLE_RATE_HZ = 1600.0;
static const uint32_t SAMPLE_PERIOD_US = 625;
static const double ADXL_G_PER_LSB = 0.0039;
static const double GRAVITY_MS2 = 9.80665;
static const double ADXL_MS2_PER_LSB = ADXL_G_PER_LSB * GRAVITY_MS2;

static const uint16_t FFT_N = 1024;
static const uint16_t HALF_BINS = FFT_N / 2;
static const uint8_t FFT_FRAMES = 3;
static const double FFT_RESOLUTION_HZ = SAMPLE_RATE_HZ / (double)FFT_N;
static const double HAMMING_COHERENT_GAIN = 0.54;

// Runtime-configurable discovery band. These are measurement-band defaults,
// NOT a tool-frequency assumption.
double searchMinHz = 20.0;
double searchMaxHz = 750.0;

static const uint8_t TOP_CANDIDATES = 5;
static const uint8_t PEAK_EXCLUSION_BINS = 3;
static const double MIN_FFT_SNR_DB = 8.0;
static const double MIN_SIGNAL_AMPLITUDE_MS2 = 0.03;

// Continuous quadrature settings retained from the frozen architecture.
static const uint16_t QUAD_BLOCK_N = 256;
static const uint8_t QUAD_BLOCKS = 10;
static const uint16_t QUAD_N = QUAD_BLOCK_N * QUAD_BLOCKS;
static const double QUAD_MAX_CORRECTION_HZ = FFT_RESOLUTION_HZ;
static const double QUAD_MAX_PHASE_RMSE_DEG = 35.0;
static const double QUAD_MIN_SYNC = 0.20;

// Conservative automatic-lock requirement. If two credible candidates coexist,
// the program deliberately refuses to guess the user's mechanical objective.
static const double AUTO_LOCK_SCORE_RATIO = 1.35;
static const double AUTO_LOCK_MIN_WEIGHTED_SYNC = 0.50;
static const uint8_t AUTO_LOCK_MIN_SUPPORT_AXES = 1;

// Acquisition integrity gates.
static const uint32_t LATE_SAMPLE_WARNING_US = 10;
static const int16_t RAW_CLIP_LIMIT = 32000;
static const uint16_t MAX_ZERO_TRIPLETS_PER_RECORD = 20;
static const uint8_t MAX_FRAME_RETRIES = 2;

static const double TWO_PI_D = 2.0 * PI;

// ============================================================================
// DATA TYPES
// ============================================================================
struct RawAccel {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct AcquisitionTiming {
  uint32_t elapsedUs;
  double meanReadSeparationUs;
  uint32_t maxReadSeparationUs;
  uint32_t maxScheduleLatenessUs;
  uint32_t lateSampleCount;
  uint16_t zeroTriplets;
  uint16_t clippedSamples;
  bool valid;
};

struct AxisFFTResult {
  bool valid;
  uint16_t bin;
  double frequencyHz;
  double amplitudeMs2;
  double snrDb;
};

struct SensorFFTResult {
  AxisFFTResult axis[3];
};

struct AxisQuadResult {
  bool valid;
  double referenceHz;
  double correctionHz;
  double refinedHz;
  double amplitudeMs2;
  double phaseDeg;
  double syncRatio;
  double phaseFitRmseDeg;
};

struct SensorAtF0 {
  AxisQuadResult axis[3];
  double vectorAmplitudeMs2;
  double J;
};

struct Candidate {
  bool fftValid;
  bool quadValid;
  uint8_t fftRank;
  uint16_t bin;
  double fftSeedHz;
  double fftVectorAmplitudeMs2;
  double fftSnrDb;

  double refinedHz;
  double toolVectorAmplitudeMs2;
  double handVectorAmplitudeMs2;
  double toolWeightedSync;
  double toolWeightedPhaseRmseDeg;
  uint8_t supportAxes;
  double qualityScore;

  SensorAtF0 toolAtCandidate;
  SensorAtF0 handAtCandidate;
};

// ============================================================================
// MEMORY
// ============================================================================
// Same raw buffers are reused: first 1024 entries during FFT frames, then the
// full 2560 entries during the uninterrupted quadrature record.
int16_t rawToolX[QUAD_N];
int16_t rawToolY[QUAD_N];
int16_t rawToolZ[QUAD_N];
int16_t rawHandX[QUAD_N];
int16_t rawHandY[QUAD_N];
int16_t rawHandZ[QUAD_N];

// FFT scratch.
double fftReal[FFT_N];
double fftImag[FFT_N];
ArduinoFFT<double> FFT(fftReal, fftImag, FFT_N, SAMPLE_RATE_HZ);

// Three-frame accumulated spectra.
double toolSpecX[HALF_BINS];
double toolSpecY[HALF_BINS];
double toolSpecZ[HALF_BINS];
double handSpecX[HALF_BINS];
double handSpecY[HALF_BINS];
double handSpecZ[HALF_BINS];
double toolVectorSpectrum[HALF_BINS];
double handVectorSpectrum[HALF_BINS];

Candidate candidates[TOP_CANDIDATES];
uint8_t candidateCount = 0;
bool discoveryAvailable = false;

bool f0Locked = false;
double lockedF0Hz = 0.0;
uint8_t lockedCandidateNumber = 0;
SensorFFTResult lastToolFFT;
SensorFFTResult lastHandFFT;
SensorAtF0 lastToolAtF0;
SensorAtF0 lastHandAtF0;

// ============================================================================
// HELPERS
// ============================================================================
double clampDouble(double value, double lo, double hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

double wrapDeg(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

const char *axisName(uint8_t axis) {
  if (axis == 0) return "X";
  if (axis == 1) return "Y";
  return "Z";
}

bool rawTripletZero(const RawAccel &a) {
  return a.x == 0 && a.y == 0 && a.z == 0;
}

bool rawTripletClipped(const RawAccel &a) {
  return abs((int)a.x) >= RAW_CLIP_LIMIT ||
         abs((int)a.y) >= RAW_CLIP_LIMIT ||
         abs((int)a.z) >= RAW_CLIP_LIMIT;
}

// ============================================================================
// ADXL345 SPI
// ============================================================================
void deselectSensors() {
  digitalWrite(PIN_CS_ADXL1, HIGH);
  digitalWrite(PIN_CS_ADXL2, HIGH);
}

void writeRegister(uint8_t csPin, uint8_t reg, uint8_t value) {
  SPI.beginTransaction(adxlSpiSettings);
  deselectSensors();
  digitalWrite(csPin, LOW);
  SPI.transfer(reg & 0x3F);
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(uint8_t csPin, uint8_t reg) {
  SPI.beginTransaction(adxlSpiSettings);
  deselectSensors();
  digitalWrite(csPin, LOW);
  SPI.transfer(SPI_READ_BIT | (reg & 0x3F));
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return value;
}

RawAccel readRawXYZ(uint8_t csPin) {
  RawAccel result = {};

  SPI.beginTransaction(adxlSpiSettings);
  deselectSensors();
  digitalWrite(csPin, LOW);
  SPI.transfer(SPI_READ_BIT | SPI_MB_BIT | (REG_DATAX0 & 0x3F));

  uint8_t x0 = SPI.transfer(0x00);
  uint8_t x1 = SPI.transfer(0x00);
  uint8_t y0 = SPI.transfer(0x00);
  uint8_t y1 = SPI.transfer(0x00);
  uint8_t z0 = SPI.transfer(0x00);
  uint8_t z1 = SPI.transfer(0x00);

  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  result.x = (int16_t)(((uint16_t)x1 << 8) | x0);
  result.y = (int16_t)(((uint16_t)y1 << 8) | y0);
  result.z = (int16_t)(((uint16_t)z1 << 8) | z0);
  return result;
}

bool initializeSensor(uint8_t csPin, const char *name) {
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  writeRegister(csPin, REG_POWER_CTL, 0x00);
  delay(10);
  writeRegister(csPin, REG_DATA_FORMAT, 0x0B); // FULL_RES, +/-16 g, 4-wire SPI
  writeRegister(csPin, REG_BW_RATE, 0x0E);     // nominal 1600 Hz ODR
  writeRegister(csPin, REG_POWER_CTL, 0x08);   // measurement mode
  delay(25);

  uint8_t id = readRegister(csPin, REG_DEVID);
  uint8_t fmt = readRegister(csPin, REG_DATA_FORMAT);
  uint8_t bw = readRegister(csPin, REG_BW_RATE);
  uint8_t pwr = readRegister(csPin, REG_POWER_CTL);

  Serial.print(name);
  Serial.print(" | DEVID 0x"); Serial.print(id, HEX);
  Serial.print(" | FORMAT 0x"); Serial.print(fmt, HEX);
  Serial.print(" | RATE 0x"); Serial.print(bw, HEX);
  Serial.print(" | POWER 0x"); Serial.println(pwr, HEX);

  return id == 0xE5 && fmt == 0x0B && bw == 0x0E && pwr == 0x08;
}

bool verifySensor(uint8_t csPin) {
  return readRegister(csPin, REG_DEVID) == 0xE5 &&
         readRegister(csPin, REG_DATA_FORMAT) == 0x0B &&
         readRegister(csPin, REG_BW_RATE) == 0x0E &&
         readRegister(csPin, REG_POWER_CTL) == 0x08;
}

bool verifyBothSensorsOrRecover() {
  bool toolOk = verifySensor(PIN_CS_ADXL1);
  bool handOk = verifySensor(PIN_CS_ADXL2);

  if (toolOk && handOk) return true;

  Serial.println("ADXL configuration check failed. Reinitializing both sensors once...");
  bool a = initializeSensor(PIN_CS_ADXL1, "ADXL1 TOOL");
  bool b = initializeSensor(PIN_CS_ADXL2, "ADXL2 HAND");
  return a && b;
}

// ============================================================================
// FFT CORE
// ============================================================================
void clearSpectrum(double spectrum[]) {
  for (uint16_t i = 0; i < HALF_BINS; i++) spectrum[i] = 0.0;
}

void clearAllSpectra() {
  clearSpectrum(toolSpecX);
  clearSpectrum(toolSpecY);
  clearSpectrum(toolSpecZ);
  clearSpectrum(handSpecX);
  clearSpectrum(handSpecY);
  clearSpectrum(handSpecZ);
  clearSpectrum(toolVectorSpectrum);
  clearSpectrum(handVectorSpectrum);
}

void accumulateSpectrum(const int16_t source[], double destination[]) {
  double mean = 0.0;
  for (uint16_t i = 0; i < FFT_N; i++) {
    mean += (double)source[i] * ADXL_MS2_PER_LSB;
  }
  mean /= (double)FFT_N;

  for (uint16_t i = 0; i < FFT_N; i++) {
    fftReal[i] = (double)source[i] * ADXL_MS2_PER_LSB - mean;
    fftImag[i] = 0.0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  for (uint16_t bin = 1; bin < HALF_BINS; bin++) {
    destination[bin] += fftReal[bin];
  }
}

uint16_t minSearchBin() {
  uint16_t b = (uint16_t)ceil(searchMinHz / FFT_RESOLUTION_HZ);
  if (b < 1) b = 1;
  return b;
}

uint16_t maxSearchBin() {
  uint16_t b = (uint16_t)floor(searchMaxHz / FFT_RESOLUTION_HZ);
  if (b >= HALF_BINS) b = HALF_BINS - 1;
  return b;
}

double avgMag(const double spectrum[], uint16_t bin) {
  return spectrum[bin] / (double)FFT_FRAMES;
}

double fftMagnitudeToPeakAmplitude(double magnitude) {
  return (2.0 * magnitude) / ((double)FFT_N * HAMMING_COHERENT_GAIN);
}

double parabolicOffset(const double spectrum[], uint16_t bin) {
  if (bin < 1 || bin + 1 >= HALF_BINS) return 0.0;
  double a = avgMag(spectrum, bin - 1);
  double b = avgMag(spectrum, bin);
  double c = avgMag(spectrum, bin + 1);
  double den = a - 2.0 * b + c;
  if (fabs(den) < 1e-18) return 0.0;
  double delta = 0.5 * (a - c) / den;
  return clampDouble(delta, -0.5, 0.5);
}

double parabolicPeakMagnitude(const double spectrum[], uint16_t bin, double delta) {
  double a = avgMag(spectrum, bin - 1);
  double b = avgMag(spectrum, bin);
  double c = avgMag(spectrum, bin + 1);
  return b - 0.25 * (a - c) * delta;
}

double calculateSpectrumSnrDb(const double spectrum[], uint16_t peakBin) {
  uint16_t lo = minSearchBin();
  uint16_t hi = maxSearchBin();
  double noiseSum = 0.0;
  uint16_t noiseCount = 0;

  for (uint16_t bin = lo; bin <= hi; bin++) {
    if (abs((int)bin - (int)peakBin) <= PEAK_EXCLUSION_BINS) continue;
    noiseSum += avgMag(spectrum, bin);
    noiseCount++;
  }

  double peak = avgMag(spectrum, peakBin);
  if (peak <= 0.0 || noiseCount == 0) return 0.0;
  double noise = noiseSum / (double)noiseCount;
  if (noise <= 0.0) return 0.0;
  return 20.0 * log10(peak / noise);
}

AxisFFTResult analyzeAxisFFT(const double spectrum[]) {
  AxisFFTResult result = {};
  uint16_t lo = minSearchBin();
  uint16_t hi = maxSearchBin();
  uint16_t bestBin = lo;
  double bestMag = 0.0;

  for (uint16_t bin = lo; bin <= hi; bin++) {
    double m = avgMag(spectrum, bin);
    if (m > bestMag) {
      bestMag = m;
      bestBin = bin;
    }
  }

  double delta = parabolicOffset(spectrum, bestBin);
  double peakMag = parabolicPeakMagnitude(spectrum, bestBin, delta);
  result.bin = bestBin;
  result.frequencyHz = ((double)bestBin + delta) * FFT_RESOLUTION_HZ;
  result.amplitudeMs2 = fftMagnitudeToPeakAmplitude(peakMag);
  result.snrDb = calculateSpectrumSnrDb(spectrum, bestBin);
  result.valid = result.amplitudeMs2 >= MIN_SIGNAL_AMPLITUDE_MS2 &&
                 result.snrDb >= MIN_FFT_SNR_DB;
  return result;
}

bool isLocalMaximum(const double spectrum[], uint16_t bin, uint16_t lo, uint16_t hi) {
  if (bin <= lo || bin >= hi) return false;
  return spectrum[bin] > spectrum[bin - 1] &&
         spectrum[bin] >= spectrum[bin + 1];
}

void buildVectorSpectrum(const double sx[], const double sy[], const double sz[], double out[]) {
  for (uint16_t bin = 0; bin < HALF_BINS; bin++) {
    double x = avgMag(sx, bin);
    double y = avgMag(sy, bin);
    double z = avgMag(sz, bin);
    out[bin] = sqrt(x * x + y * y + z * z);
  }
}

// vectorSpectrum already contains the FRAME-AVERAGED vector magnitude, so SNR
// is calculated directly without another /FFT_FRAMES.
double calculateVectorSnrDb(const double vectorSpectrum[], uint16_t peakBin) {
  uint16_t lo = minSearchBin();
  uint16_t hi = maxSearchBin();
  double noiseSum = 0.0;
  uint16_t noiseCount = 0;

  for (uint16_t bin = lo; bin <= hi; bin++) {
    if (abs((int)bin - (int)peakBin) <= PEAK_EXCLUSION_BINS) continue;
    noiseSum += vectorSpectrum[bin];
    noiseCount++;
  }

  double peak = vectorSpectrum[peakBin];
  if (peak <= 0.0 || noiseCount == 0) return 0.0;
  double noise = noiseSum / (double)noiseCount;
  if (noise <= 0.0) return 0.0;
  return 20.0 * log10(peak / noise);
}

AcquisitionTiming acquirePairedRecord(uint16_t sampleCount) {
  AcquisitionTiming timing = {};
  uint32_t nextSampleUs = micros();
  uint32_t startUs = nextSampleUs;
  uint64_t separationSumUs = 0;

  for (uint16_t i = 0; i < sampleCount; i++) {
    while ((int32_t)(micros() - nextSampleUs) < 0) {}

    uint32_t actualStartUs = micros();
    uint32_t latenessUs = actualStartUs - nextSampleUs;
    nextSampleUs += SAMPLE_PERIOD_US;

    if (latenessUs > timing.maxScheduleLatenessUs) {
      timing.maxScheduleLatenessUs = latenessUs;
    }
    if (latenessUs > LATE_SAMPLE_WARNING_US) timing.lateSampleCount++;

    // Fixed read order. Cross-sensor phase is NOT used.
    uint32_t toolReadUs = micros();
    RawAccel tool = readRawXYZ(PIN_CS_ADXL1);
    uint32_t handReadUs = micros();
    RawAccel hand = readRawXYZ(PIN_CS_ADXL2);

    uint32_t sep = handReadUs - toolReadUs;
    separationSumUs += sep;
    if (sep > timing.maxReadSeparationUs) timing.maxReadSeparationUs = sep;

    rawToolX[i] = tool.x;
    rawToolY[i] = tool.y;
    rawToolZ[i] = tool.z;
    rawHandX[i] = hand.x;
    rawHandY[i] = hand.y;
    rawHandZ[i] = hand.z;

    if (rawTripletZero(tool)) timing.zeroTriplets++;
    if (rawTripletZero(hand)) timing.zeroTriplets++;
    if (rawTripletClipped(tool)) timing.clippedSamples++;
    if (rawTripletClipped(hand)) timing.clippedSamples++;
  }

  timing.elapsedUs = micros() - startUs;
  timing.meanReadSeparationUs = (double)separationSumUs / (double)sampleCount;
  timing.valid = timing.zeroTriplets <= MAX_ZERO_TRIPLETS_PER_RECORD &&
                 timing.clippedSamples == 0;
  return timing;
}

bool acquireThreeFftFrames() {
  clearAllSpectra();
  if (!verifyBothSensorsOrRecover()) return false;

  for (uint8_t frame = 0; frame < FFT_FRAMES; frame++) {
    bool frameAccepted = false;

    for (uint8_t attempt = 0; attempt <= MAX_FRAME_RETRIES && !frameAccepted; attempt++) {
      AcquisitionTiming t = acquirePairedRecord(FFT_N);

      if (!t.valid) {
        Serial.print("FFT frame "); Serial.print(frame + 1);
        Serial.print(" integrity reject | zero triplets "); Serial.print(t.zeroTriplets);
        Serial.print(" | clipped samples "); Serial.println(t.clippedSamples);
        if (!verifyBothSensorsOrRecover()) return false;
        continue;
      }

      accumulateSpectrum(rawToolX, toolSpecX);
      accumulateSpectrum(rawToolY, toolSpecY);
      accumulateSpectrum(rawToolZ, toolSpecZ);
      accumulateSpectrum(rawHandX, handSpecX);
      accumulateSpectrum(rawHandY, handSpecY);
      accumulateSpectrum(rawHandZ, handSpecZ);

      Serial.print("Paired FFT frame "); Serial.print(frame + 1);
      Serial.print("/3 accepted | elapsed ");
      Serial.print((double)t.elapsedUs / 1000.0, 2);
      Serial.print(" ms | mean tool->hand read separation ");
      Serial.print(t.meanReadSeparationUs, 2);
      Serial.print(" us | late samples ");
      Serial.println(t.lateSampleCount);

      frameAccepted = true;
    }

    if (!frameAccepted) return false;
  }

  buildVectorSpectrum(toolSpecX, toolSpecY, toolSpecZ, toolVectorSpectrum);
  buildVectorSpectrum(handSpecX, handSpecY, handSpecZ, handVectorSpectrum);
  return true;
}

void analyzeIndependentFftResults() {
  lastToolFFT.axis[0] = analyzeAxisFFT(toolSpecX);
  lastToolFFT.axis[1] = analyzeAxisFFT(toolSpecY);
  lastToolFFT.axis[2] = analyzeAxisFFT(toolSpecZ);

  lastHandFFT.axis[0] = analyzeAxisFFT(handSpecX);
  lastHandFFT.axis[1] = analyzeAxisFFT(handSpecY);
  lastHandFFT.axis[2] = analyzeAxisFFT(handSpecZ);
}

void findToolCandidates() {
  candidateCount = 0;
  for (uint8_t i = 0; i < TOP_CANDIDATES; i++) candidates[i] = {};

  uint16_t lo = minSearchBin();
  uint16_t hi = maxSearchBin();
  bool excluded[HALF_BINS];
  for (uint16_t i = 0; i < HALF_BINS; i++) excluded[i] = false;

  for (uint8_t rank = 0; rank < TOP_CANDIDATES; rank++) {
    uint16_t bestBin = 0;
    double best = 0.0;

    for (uint16_t bin = lo; bin <= hi; bin++) {
      if (excluded[bin]) continue;
      if (!isLocalMaximum(toolVectorSpectrum, bin, lo, hi)) continue;
      if (toolVectorSpectrum[bin] > best) {
        best = toolVectorSpectrum[bin];
        bestBin = bin;
      }
    }

    if (bestBin == 0 || best <= 0.0) break;

    Candidate &c = candidates[candidateCount];
    c.fftRank = candidateCount + 1;
    c.bin = bestBin;

    // Parabolic interpolation uses the vector spectrum directly.
    double a = toolVectorSpectrum[bestBin - 1];
    double b = toolVectorSpectrum[bestBin];
    double d = toolVectorSpectrum[bestBin + 1];
    double den = a - 2.0 * b + d;
    double delta = 0.0;
    if (fabs(den) > 1e-18) delta = clampDouble(0.5 * (a - d) / den, -0.5, 0.5);
    double peakVectorMag = b - 0.25 * (a - d) * delta;

    c.fftSeedHz = ((double)bestBin + delta) * FFT_RESOLUTION_HZ;
    c.fftVectorAmplitudeMs2 = fftMagnitudeToPeakAmplitude(peakVectorMag);
    c.fftSnrDb = calculateVectorSnrDb(toolVectorSpectrum, bestBin);
    c.fftValid = c.fftVectorAmplitudeMs2 >= MIN_SIGNAL_AMPLITUDE_MS2 &&
                 c.fftSnrDb >= MIN_FFT_SNR_DB;
    candidateCount++;

    int start = (int)bestBin - PEAK_EXCLUSION_BINS;
    int end = (int)bestBin + PEAK_EXCLUSION_BINS;
    if (start < (int)lo) start = lo;
    if (end > (int)hi) end = hi;
    for (int bin = start; bin <= end; bin++) excluded[bin] = true;
  }
}

// ============================================================================
// EXACT-FREQUENCY QUADRATURE
// ============================================================================
const int16_t *axisBuffer(const int16_t *x, const int16_t *y, const int16_t *z, uint8_t axis) {
  if (axis == 0) return x;
  if (axis == 1) return y;
  return z;
}

AxisQuadResult analyzeAxisQuadrature(const int16_t samples[], double referenceHz) {
  AxisQuadResult result = {};
  result.referenceHz = referenceHz;
  result.refinedHz = referenceHz;

  if (referenceHz <= 0.0 || referenceHz >= SAMPLE_RATE_HZ * 0.5) return result;

  double blockPhase[QUAD_BLOCKS];
  double blockAmplitude[QUAD_BLOCKS];

  for (uint8_t block = 0; block < QUAD_BLOCKS; block++) {
    uint32_t offset = (uint32_t)block * QUAD_BLOCK_N;
    double mean = 0.0;
    for (uint16_t n = 0; n < QUAD_BLOCK_N; n++) {
      mean += (double)samples[offset + n];
    }
    mean /= (double)QUAD_BLOCK_N;

    double I = 0.0;
    double Q = 0.0;
    double windowSum = 0.0;

    for (uint16_t n = 0; n < QUAD_BLOCK_N; n++) {
      uint32_t globalIndex = offset + n;
      double w = 0.5 - 0.5 * cos(TWO_PI_D * (double)n / (double)(QUAD_BLOCK_N - 1));
      double theta = TWO_PI_D * referenceHz * (double)globalIndex / SAMPLE_RATE_HZ;
      double v = ((double)samples[globalIndex] - mean) * ADXL_MS2_PER_LSB * w;
      I += v * cos(theta);
      Q += v * sin(theta);
      windowSum += w;
    }

    blockPhase[block] = atan2(-Q, I);
    blockAmplitude[block] = (windowSum > 0.0)
      ? 2.0 * sqrt(I * I + Q * Q) / windowSum
      : 0.0;
  }

  // Unwrap block phase and fit phase slope -> residual frequency error.
  double unwrapped[QUAD_BLOCKS];
  unwrapped[0] = blockPhase[0];
  for (uint8_t block = 1; block < QUAD_BLOCKS; block++) {
    double d = blockPhase[block] - blockPhase[block - 1];
    while (d > PI) d -= TWO_PI_D;
    while (d < -PI) d += TWO_PI_D;
    unwrapped[block] = unwrapped[block - 1] + d;
  }

  double blockDuration = (double)QUAD_BLOCK_N / SAMPLE_RATE_HZ;
  double meanT = 0.0;
  double meanP = 0.0;
  double meanA = 0.0;

  for (uint8_t block = 0; block < QUAD_BLOCKS; block++) {
    double t = ((double)block + 0.5) * blockDuration;
    meanT += t;
    meanP += unwrapped[block];
    meanA += blockAmplitude[block];
  }
  meanT /= (double)QUAD_BLOCKS;
  meanP /= (double)QUAD_BLOCKS;
  meanA /= (double)QUAD_BLOCKS;

  double cov = 0.0;
  double var = 0.0;
  for (uint8_t block = 0; block < QUAD_BLOCKS; block++) {
    double t = ((double)block + 0.5) * blockDuration;
    cov += (t - meanT) * (unwrapped[block] - meanP);
    var += (t - meanT) * (t - meanT);
  }
  double phaseSlopeRadPerS = (var > 0.0) ? cov / var : 0.0;
  result.correctionHz = phaseSlopeRadPerS / TWO_PI_D;
  result.refinedHz = referenceHz + result.correctionHz;
  result.amplitudeMs2 = meanA;

  double rss = 0.0;
  for (uint8_t block = 0; block < QUAD_BLOCKS; block++) {
    double t = ((double)block + 0.5) * blockDuration;
    double fit = meanP + phaseSlopeRadPerS * (t - meanT);
    double r = unwrapped[block] - fit;
    rss += r * r;
  }
  result.phaseFitRmseDeg = sqrt(rss / (double)QUAD_BLOCKS) * 180.0 / PI;

  // Full-record exact-frequency phasor and synchronous fitted-component ratio.
  double recordMean = 0.0;
  for (uint16_t i = 0; i < QUAD_N; i++) recordMean += (double)samples[i];
  recordMean /= (double)QUAD_N;

  double I = 0.0;
  double Q = 0.0;
  double sumSq = 0.0;
  double windowSum = 0.0;

  for (uint16_t i = 0; i < QUAD_N; i++) {
    double centered = ((double)samples[i] - recordMean) * ADXL_MS2_PER_LSB;
    sumSq += centered * centered;

    double w = 0.5 - 0.5 * cos(TWO_PI_D * (double)i / (double)(QUAD_N - 1));
    double theta = TWO_PI_D * referenceHz * (double)i / SAMPLE_RATE_HZ;
    I += centered * w * cos(theta);
    Q += centered * w * sin(theta);
    windowSum += w;
  }

  double fullAmp = (windowSum > 0.0) ? 2.0 * sqrt(I * I + Q * Q) / windowSum : 0.0;
  result.phaseDeg = wrapDeg(atan2(-Q, I) * 180.0 / PI);

  double totalRms = (QUAD_N > 0) ? sqrt(sumSq / (double)QUAD_N) : 0.0;
  double synchronousRms = fullAmp / sqrt(2.0);
  result.syncRatio = (totalRms > 1e-12) ? synchronousRms / totalRms : 0.0;
  result.syncRatio = clampDouble(result.syncRatio, 0.0, 1.0);

  result.valid = result.amplitudeMs2 >= MIN_SIGNAL_AMPLITUDE_MS2 &&
                 fabs(result.correctionHz) <= QUAD_MAX_CORRECTION_HZ &&
                 result.phaseFitRmseDeg <= QUAD_MAX_PHASE_RMSE_DEG &&
                 result.syncRatio >= QUAD_MIN_SYNC;
  return result;
}

SensorAtF0 analyzeSensorAtFrequency(
  const int16_t x[], const int16_t y[], const int16_t z[], double fHz
) {
  SensorAtF0 result = {};
  result.axis[0] = analyzeAxisQuadrature(x, fHz);
  result.axis[1] = analyzeAxisQuadrature(y, fHz);
  result.axis[2] = analyzeAxisQuadrature(z, fHz);

  double ax = result.axis[0].amplitudeMs2;
  double ay = result.axis[1].amplitudeMs2;
  double az = result.axis[2].amplitudeMs2;
  result.J = ax * ax + ay * ay + az * az;
  result.vectorAmplitudeMs2 = sqrt(result.J);
  return result;
}

double weightedCorrection(const SensorAtF0 &sensor) {
  double sumW = 0.0;
  double sum = 0.0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    const AxisQuadResult &q = sensor.axis[axis];
    if (q.amplitudeMs2 < MIN_SIGNAL_AMPLITUDE_MS2) continue;
    if (q.phaseFitRmseDeg > QUAD_MAX_PHASE_RMSE_DEG) continue;
    if (fabs(q.correctionHz) > QUAD_MAX_CORRECTION_HZ) continue;
    double w = q.amplitudeMs2 * q.amplitudeMs2 * fmax(q.syncRatio, 0.10);
    sumW += w;
    sum += w * q.correctionHz;
  }
  return (sumW > 0.0) ? sum / sumW : 0.0;
}

double weightedSync(const SensorAtF0 &sensor) {
  double sumW = 0.0;
  double sum = 0.0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    const AxisQuadResult &q = sensor.axis[axis];
    double w = q.amplitudeMs2 * q.amplitudeMs2;
    sumW += w;
    sum += w * q.syncRatio;
  }
  return (sumW > 0.0) ? sum / sumW : 0.0;
}

double weightedPhaseRmse(const SensorAtF0 &sensor) {
  double sumW = 0.0;
  double sum = 0.0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    const AxisQuadResult &q = sensor.axis[axis];
    double w = q.amplitudeMs2 * q.amplitudeMs2;
    sumW += w;
    sum += w * q.phaseFitRmseDeg;
  }
  return (sumW > 0.0) ? sum / sumW : 999.0;
}

uint8_t countSupportAxes(const SensorAtF0 &sensor) {
  uint8_t count = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    const AxisQuadResult &q = sensor.axis[axis];
    if (q.amplitudeMs2 >= MIN_SIGNAL_AMPLITUDE_MS2 &&
        q.syncRatio >= 0.35 &&
        q.phaseFitRmseDeg <= QUAD_MAX_PHASE_RMSE_DEG) {
      count++;
    }
  }
  return count;
}

void validateCandidate(Candidate &c) {
  if (!c.fftValid) {
    c.quadValid = false;
    return;
  }

  // Pass 1: all TOOL axes at the triaxial FFT seed.
  SensorAtF0 pass1 = analyzeSensorAtFrequency(
    rawToolX, rawToolY, rawToolZ, c.fftSeedHz
  );
  double df1 = clampDouble(
    weightedCorrection(pass1), -QUAD_MAX_CORRECTION_HZ, QUAD_MAX_CORRECTION_HZ
  );
  double f1 = c.fftSeedHz + df1;

  // Pass 2 on the SAME uninterrupted record. This is a numerical refinement,
  // not a new acquisition and not live frequency tracking.
  SensorAtF0 pass2 = analyzeSensorAtFrequency(
    rawToolX, rawToolY, rawToolZ, f1
  );
  double df2 = clampDouble(
    weightedCorrection(pass2), -QUAD_MAX_CORRECTION_HZ, QUAD_MAX_CORRECTION_HZ
  );

  // Prevent two-pass refinement from escaping more than one FFT bin from seed.
  double totalDf = clampDouble(df1 + df2, -QUAD_MAX_CORRECTION_HZ, QUAD_MAX_CORRECTION_HZ);
  c.refinedHz = c.fftSeedHz + totalDf;

  c.toolAtCandidate = analyzeSensorAtFrequency(
    rawToolX, rawToolY, rawToolZ, c.refinedHz
  );
  c.handAtCandidate = analyzeSensorAtFrequency(
    rawHandX, rawHandY, rawHandZ, c.refinedHz
  );

  c.toolVectorAmplitudeMs2 = c.toolAtCandidate.vectorAmplitudeMs2;
  c.handVectorAmplitudeMs2 = c.handAtCandidate.vectorAmplitudeMs2;
  c.toolWeightedSync = weightedSync(c.toolAtCandidate);
  c.toolWeightedPhaseRmseDeg = weightedPhaseRmse(c.toolAtCandidate);
  c.supportAxes = countSupportAxes(c.toolAtCandidate);

  c.quadValid = c.toolVectorAmplitudeMs2 >= MIN_SIGNAL_AMPLITUDE_MS2 &&
                c.toolWeightedSync >= QUAD_MIN_SYNC &&
                c.toolWeightedPhaseRmseDeg <= QUAD_MAX_PHASE_RMSE_DEG &&
                c.supportAxes >= 1;

  // Quality-weighted periodicity score. It is deliberately NOT just amplitude.
  // It rewards a strong, synchronous, phase-stable, multi-axis tool component.
  double syncFactor = 0.5 + 0.5 * clampDouble(c.toolWeightedSync, 0.0, 1.0);
  double supportFactor = 0.7 + 0.15 * (double)c.supportAxes; // 0.85..1.15
  double stabilityFactor = 1.0 / (1.0 + c.toolWeightedPhaseRmseDeg / 30.0);
  double snrFactor = clampDouble((c.fftSnrDb - 3.0) / 17.0, 0.35, 1.20);

  c.qualityScore = c.quadValid
    ? c.toolVectorAmplitudeMs2 * syncFactor * supportFactor * stabilityFactor * snrFactor
    : 0.0;
}

bool acquireQuadratureRecord() {
  if (!verifyBothSensorsOrRecover()) return false;
  Serial.println("Acquiring uninterrupted 2560-sample paired quadrature record...");
  AcquisitionTiming t = acquirePairedRecord(QUAD_N);
  Serial.print("Quadrature acquisition: ");
  Serial.print((double)t.elapsedUs / 1000.0, 2);
  Serial.print(" ms | mean tool->hand read separation ");
  Serial.print(t.meanReadSeparationUs, 2);
  Serial.print(" us | late samples "); Serial.print(t.lateSampleCount);
  Serial.print(" | zero triplets "); Serial.print(t.zeroTriplets);
  Serial.print(" | clipped samples "); Serial.println(t.clippedSamples);

  if (!t.valid) {
    Serial.println("QUADRATURE RECORD REJECTED: acquisition integrity gate failed.");
    return false;
  }
  return true;
}

// ============================================================================
// PRINTING
// ============================================================================
void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  a           full FFT + quadrature discovery");
  Serial.println("  k N         lock candidate N from last discovery, e.g. k 2");
  Serial.println("  q           new exact-frequency measurement at locked f0");
  Serial.println("  b MIN MAX   set discovery band at runtime, e.g. b 20 750");
  Serial.println("  p           print settings / lock status");
  Serial.println("  x           clear f0 lock");
  Serial.println("  h           help");
  Serial.println();
}

void printSettings() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("CURRENT ANALYSER SETTINGS");
  Serial.println("============================================================");
  Serial.print("Sampling:               "); Serial.print(SAMPLE_RATE_HZ, 1); Serial.println(" Hz");
  Serial.print("FFT:                    "); Serial.print(FFT_N); Serial.print(" samples x "); Serial.print(FFT_FRAMES); Serial.println(" frames");
  Serial.print("FFT resolution:         "); Serial.print(FFT_RESOLUTION_HZ, 5); Serial.println(" Hz/bin");
  Serial.print("Runtime search band:    "); Serial.print(searchMinHz, 2); Serial.print(" .. "); Serial.print(searchMaxHz, 2); Serial.println(" Hz");
  Serial.print("Quadrature record:      "); Serial.print(QUAD_N); Serial.println(" paired samples");
  Serial.print("f0 lock:                ");
  if (f0Locked) {
    Serial.print("LOCKED at "); Serial.print(lockedF0Hz, 5); Serial.println(" Hz");
  } else {
    Serial.println("NOT LOCKED");
  }
  Serial.println("No compiled-in tool frequency is used.");
  Serial.println("============================================================");
}

void printIndependentPeaks(const char *title, const SensorFFTResult &r) {
  Serial.println();
  Serial.println(title);
  Serial.println("AXIS   DOMINANT PEAK       FFT AMP (peak)     SNR       STATUS");
  Serial.println("---------------------------------------------------------------");
  for (uint8_t axis = 0; axis < 3; axis++) {
    const AxisFFTResult &a = r.axis[axis];
    Serial.print(axisName(axis)); Serial.print("      ");
    Serial.print(a.frequencyHz, 3); Serial.print(" Hz        ");
    Serial.print(a.amplitudeMs2, 5); Serial.print(" m/s2      ");
    Serial.print(a.snrDb, 1); Serial.print(" dB     ");
    Serial.println(a.valid ? "VALID" : "LOW CONF");
  }
}

void printCandidateTable() {
  Serial.println();
  Serial.println("ADXL1 TOOL — TRIAXIAL CANDIDATES");
  Serial.println("N  FFT_seed_Hz  FFT_vecAmp  FFT_SNR  Refined_Hz  Tool_vecAmp  Tool_sync  RMSEdeg  Axes  Hand_vecAmp  Score");
  Serial.println("-----------------------------------------------------------------------------------------------------------");
  for (uint8_t i = 0; i < candidateCount; i++) {
    Candidate &c = candidates[i];
    Serial.print(i + 1); Serial.print("  ");
    Serial.print(c.fftSeedHz, 3); Serial.print("       ");
    Serial.print(c.fftVectorAmplitudeMs2, 4); Serial.print("      ");
    Serial.print(c.fftSnrDb, 1); Serial.print("     ");
    Serial.print(c.refinedHz, 4); Serial.print("      ");
    Serial.print(c.toolVectorAmplitudeMs2, 4); Serial.print("       ");
    Serial.print(c.toolWeightedSync, 3); Serial.print("      ");
    Serial.print(c.toolWeightedPhaseRmseDeg, 2); Serial.print("     ");
    Serial.print(c.supportAxes); Serial.print("     ");
    Serial.print(c.handVectorAmplitudeMs2, 4); Serial.print("       ");
    Serial.print(c.qualityScore, 4);
    if (!c.quadValid) Serial.print("  LOW_CONF");
    Serial.println();
  }
}

void printSensorAtF0(const char *title, const SensorFFTResult &fft, const SensorAtF0 &q, double f0) {
  Serial.println();
  Serial.println(title);
  Serial.print("LOCKED f0 = "); Serial.print(f0, 5); Serial.println(" Hz");
  Serial.println("AXIS   DOMINANT PEAK       AMPLITUDE @ f0      PHASE@f0    SyncRatio   STATUS");
  Serial.println("----------------------------------------------------------------------------");
  for (uint8_t axis = 0; axis < 3; axis++) {
    Serial.print(axisName(axis)); Serial.print("      ");
    Serial.print(fft.axis[axis].frequencyHz, 3); Serial.print(" Hz        ");
    Serial.print(q.axis[axis].amplitudeMs2, 5); Serial.print(" m/s2      ");
    Serial.print(q.axis[axis].phaseDeg, 1); Serial.print(" deg      ");
    Serial.print(q.axis[axis].syncRatio, 3); Serial.print("       ");
    Serial.println(q.axis[axis].valid ? "VALID" : "LOW CONF");
  }
  Serial.print("Vector amplitude @ f0 = "); Serial.print(q.vectorAmplitudeMs2, 5); Serial.println(" m/s2");
  Serial.print("J @ f0 = |X|^2+|Y|^2+|Z|^2 = "); Serial.print(q.J, 6); Serial.println(" (m/s2)^2");
}

void printMatcherExport() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("AMPLITUDE-MATCHER EXPORT — LOCAL ADXL2 / BACK OF HAND");
  Serial.println("============================================================");
  Serial.print("f0_Hz = "); Serial.println(lockedF0Hz, 6);
  Serial.print("X_target_at_f0 = "); Serial.print(lastHandAtF0.axis[0].amplitudeMs2, 6); Serial.println(" m/s2 peak");
  Serial.print("Y_target_at_f0 = "); Serial.print(lastHandAtF0.axis[1].amplitudeMs2, 6); Serial.println(" m/s2 peak");
  Serial.print("Z_target_at_f0 = "); Serial.print(lastHandAtF0.axis[2].amplitudeMs2, 6); Serial.println(" m/s2 peak");
  Serial.print("J_at_f0 = "); Serial.println(lastHandAtF0.J, 6);
  Serial.println();
  Serial.println("CSV_EXPORT,f0_Hz,Hand_X_at_f0,Hand_Y_at_f0,Hand_Z_at_f0,J_at_f0");
  Serial.print("CSV_EXPORT,"); Serial.print(lockedF0Hz, 6); Serial.print(",");
  Serial.print(lastHandAtF0.axis[0].amplitudeMs2, 6); Serial.print(",");
  Serial.print(lastHandAtF0.axis[1].amplitudeMs2, 6); Serial.print(",");
  Serial.print(lastHandAtF0.axis[2].amplitudeMs2, 6); Serial.print(",");
  Serial.println(lastHandAtF0.J, 6);
  Serial.println("============================================================");
}

void printLockedResult() {
  printSensorAtF0("ADXL1 TOOL / REFERENCE", lastToolFFT, lastToolAtF0, lockedF0Hz);
  printSensorAtF0("ADXL2 BACK OF HAND / LOCAL ERROR SENSOR", lastHandFFT, lastHandAtF0, lockedF0Hz);
  printMatcherExport();

  Serial.println();
  Serial.println("INTERPRETATION RULE:");
  Serial.println("DOMINANT PEAK answers: strongest component on that local axis.");
  Serial.println("AMPLITUDE @ f0 answers: amplitude of the selected tool component on that axis.");
  Serial.println("The phase sweep must use f0, not an unrelated local dominant peak.");
  Serial.println("Do not subtract ADXL1 and ADXL2 phases as if the sensors were hardware-synchronous.");
}

// ============================================================================
// LOCK / DISCOVERY WORKFLOW
// ============================================================================
int bestCandidateIndexByScore() {
  int best = -1;
  double bestScore = 0.0;
  for (uint8_t i = 0; i < candidateCount; i++) {
    if (candidates[i].quadValid && candidates[i].qualityScore > bestScore) {
      bestScore = candidates[i].qualityScore;
      best = i;
    }
  }
  return best;
}

int secondBestCandidateIndexByScore(int best) {
  int second = -1;
  double secondScore = 0.0;
  for (uint8_t i = 0; i < candidateCount; i++) {
    if ((int)i == best) continue;
    if (candidates[i].quadValid && candidates[i].qualityScore > secondScore) {
      secondScore = candidates[i].qualityScore;
      second = i;
    }
  }
  return second;
}

void applyCandidateLock(uint8_t candidateIndex) {
  if (candidateIndex >= candidateCount || !candidates[candidateIndex].quadValid) {
    Serial.println("LOCK FAILED: selected candidate is not valid.");
    return;
  }

  Candidate &c = candidates[candidateIndex];
  lockedF0Hz = c.refinedHz;
  f0Locked = true;
  lockedCandidateNumber = candidateIndex + 1;
  lastToolAtF0 = c.toolAtCandidate;
  lastHandAtF0 = c.handAtCandidate;

  Serial.println();
  Serial.println("============================================================");
  Serial.print("FREQUENCY LOCKED FROM CANDIDATE #"); Serial.println(lockedCandidateNumber);
  Serial.print("LOCKED f0 = "); Serial.print(lockedF0Hz, 6); Serial.println(" Hz");
  Serial.println("============================================================");
  printLockedResult();
}

void runDiscovery() {
  f0Locked = false;
  lockedF0Hz = 0.0;
  lockedCandidateNumber = 0;
  discoveryAvailable = false;

  Serial.println();
  Serial.println("================================================================================");
  Serial.println("FULL DUAL-ADXL TOOL/HAND FFT + QUADRATURE DISCOVERY");
  Serial.println("Tool ON, Visaton OFF. Hold the mechanical configuration steady.");
  Serial.println("================================================================================");
  Serial.print("Runtime search band: "); Serial.print(searchMinHz, 2); Serial.print(" .. "); Serial.print(searchMaxHz, 2); Serial.println(" Hz");
  Serial.println("No expected tool frequency is compiled into this program.");

  if (!acquireThreeFftFrames()) {
    Serial.println("DISCOVERY FAILED during FFT acquisition.");
    return;
  }

  analyzeIndependentFftResults();
  findToolCandidates();

  printIndependentPeaks("ADXL1 TOOL — INDEPENDENT DOMINANT PEAKS", lastToolFFT);
  printIndependentPeaks("ADXL2 HAND — INDEPENDENT DOMINANT PEAKS", lastHandFFT);

  if (candidateCount == 0) {
    Serial.println("NO TRIAXIAL TOOL CANDIDATES FOUND in the current runtime search band.");
    return;
  }

  Serial.println();
  Serial.println("FFT candidate extraction complete. Now validating all candidates on ONE uninterrupted quadrature record.");
  if (!acquireQuadratureRecord()) return;

  for (uint8_t i = 0; i < candidateCount; i++) validateCandidate(candidates[i]);
  discoveryAvailable = true;
  printCandidateTable();

  int best = bestCandidateIndexByScore();
  if (best < 0) {
    Serial.println("NO CANDIDATE PASSED quadrature quality validation.");
    return;
  }

  int second = secondBestCandidateIndexByScore(best);
  bool clearWinner = false;
  if (second < 0) {
    clearWinner = true;
  } else if (candidates[second].qualityScore > 0.0) {
    double ratio = candidates[best].qualityScore / candidates[second].qualityScore;
    clearWinner = ratio >= AUTO_LOCK_SCORE_RATIO;
  }

  clearWinner = clearWinner &&
                candidates[best].toolWeightedSync >= AUTO_LOCK_MIN_WEIGHTED_SYNC &&
                candidates[best].supportAxes >= AUTO_LOCK_MIN_SUPPORT_AXES;

  Serial.println();
  Serial.print("QUALITY-SCORE RECOMMENDATION: candidate #"); Serial.println(best + 1);
  Serial.print("Recommended refined frequency: "); Serial.print(candidates[best].refinedHz, 6); Serial.println(" Hz");

  if (clearWinner) {
    Serial.println("AUTO SELECTION CONFIDENCE: CLEAR WINNER");
    Serial.println("Locking automatically. This is a quality-weighted periodicity decision, not a largest-axis-peak decision.");
    applyCandidateLock(best);
  } else {
    Serial.println("AUTO SELECTION CONFIDENCE: AMBIGUOUS");
    Serial.println("More than one credible periodic tool component exists. The program WILL NOT guess which mechanical component you intend to cancel.");
    Serial.println("Inspect the candidate frequencies and enter k N to lock the mechanically relevant candidate.");
    Serial.println("Example: k 2");
  }
}

void runExactAtLockedF0() {
  if (!f0Locked) {
    Serial.println("No f0 is locked. Run a first, then use automatic lock or k N.");
    return;
  }

  Serial.println();
  Serial.println("============================================================");
  Serial.println("FRESH EXACT-FREQUENCY CHECK AT LOCKED f0");
  Serial.print("f0 = "); Serial.print(lockedF0Hz, 6); Serial.println(" Hz");
  Serial.println("No blind FFT is rerun in this q command.");
  Serial.println("============================================================");

  if (!acquireQuadratureRecord()) return;
  lastToolAtF0 = analyzeSensorAtFrequency(rawToolX, rawToolY, rawToolZ, lockedF0Hz);
  lastHandAtF0 = analyzeSensorAtFrequency(rawHandX, rawHandY, rawHandZ, lockedF0Hz);
  printLockedResult();
}

void setSearchBandFromCommand(const String &command) {
  double lo = 0.0;
  double hi = 0.0;
  int parsed = sscanf(command.c_str(), "b %lf %lf", &lo, &hi);
  if (parsed != 2) parsed = sscanf(command.c_str(), "B %lf %lf", &lo, &hi);

  if (parsed != 2) {
    Serial.println("Use: b MIN MAX   Example: b 20 750");
    return;
  }

  double nyquistGuard = SAMPLE_RATE_HZ * 0.5 - FFT_RESOLUTION_HZ;
  if (lo < FFT_RESOLUTION_HZ || hi <= lo || hi > nyquistGuard) {
    Serial.print("Invalid band. Valid range is approximately ");
    Serial.print(FFT_RESOLUTION_HZ, 2); Serial.print(" .. "); Serial.print(nyquistGuard, 2); Serial.println(" Hz.");
    return;
  }

  searchMinHz = lo;
  searchMaxHz = hi;
  f0Locked = false;
  discoveryAvailable = false;
  Serial.print("Runtime discovery band changed to "); Serial.print(searchMinHz, 2); Serial.print(" .. "); Serial.print(searchMaxHz, 2); Serial.println(" Hz.");
  Serial.println("Previous discovery/lock cleared. Run a again.");
}

void lockCandidateFromCommand(const String &command) {
  int n = 0;
  int parsed = sscanf(command.c_str(), "k %d", &n);
  if (parsed != 1) parsed = sscanf(command.c_str(), "K %d", &n);

  if (!discoveryAvailable) {
    Serial.println("No candidate table is available. Run a first.");
    return;
  }
  if (parsed != 1 || n < 1 || n > candidateCount) {
    Serial.print("Use k N where N is 1.."); Serial.println(candidateCount);
    return;
  }

  applyCandidateLock((uint8_t)(n - 1));
}

void processCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  if (command == "a" || command == "A") {
    runDiscovery();
    return;
  }
  if (command == "q" || command == "Q") {
    runExactAtLockedF0();
    return;
  }
  if (command == "p" || command == "P") {
    printSettings();
    return;
  }
  if (command == "x" || command == "X") {
    f0Locked = false;
    lockedF0Hz = 0.0;
    lockedCandidateNumber = 0;
    Serial.println("f0 lock cleared. Candidate table remains available until the next discovery.");
    return;
  }
  if (command == "h" || command == "H") {
    printHelp();
    return;
  }
  if (command[0] == 'b' || command[0] == 'B') {
    setSearchBandFromCommand(command);
    return;
  }
  if (command[0] == 'k' || command[0] == 'K') {
    lockCandidateFromCommand(command);
    return;
  }

  Serial.println("Unknown command.");
  printHelp();
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(200);
  delay(1200);

  pinMode(PIN_CS_ADXL1, OUTPUT);
  pinMode(PIN_CS_ADXL2, OUTPUT);
  deselectSensors();

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  // This standalone analyser never commands the Visaton.
  pinMode(PIN_DAC_SAFE, OUTPUT);
  dacWrite(PIN_DAC_SAFE, 128);

  Serial.println();
  Serial.println("================================================================================");
  Serial.println("KK DUAL-ADXL FROZEN TRIAXIAL FFT + QUADRATURE ANALYSER V1");
  Serial.println("ADXL1 GPIO5 = TOOL / reference");
  Serial.println("ADXL2 GPIO17 = BACK OF HAND / local error location");
  Serial.println("NO TOOL FREQUENCY IS HARD-CODED");
  Serial.println("================================================================================");

  bool toolOk = initializeSensor(PIN_CS_ADXL1, "ADXL1 TOOL");
  bool handOk = initializeSensor(PIN_CS_ADXL2, "ADXL2 HAND");

  if (!toolOk || !handOk) {
    Serial.println("STARTUP FAILED. Check both ADXL345 SPI connections.");
    while (true) {
      dacWrite(PIN_DAC_SAFE, 128);
      delay(1000);
    }
  }

  printSettings();
  printHelp();
  Serial.println("For the current tool test: Tool ON, Visaton OFF, then enter a.");
}

void loop() {
  dacWrite(PIN_DAC_SAFE, 128); // keep actuator command neutral in this analyser

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  delay(2);
}
