/*
  ============================================================
  KK-EXOSKELETON — TEST 02 V2
  DUAL ADXL345 STATIONARY NOISE-FLOOR CHARACTERIZATION
  DATA_READY-SYNCHRONIZED ACQUISITION
  ============================================================

  WHY V2
  ------
  This version does NOT assume that new ADXL345 data arrives exactly
  on the ESP32's 625 us software schedule. Instead, each sensor is
  sampled from its own ADXL345 DATA_READY event.

  This avoids counting repeated register values as new samples and
  allows the effective sample rate to be measured from the actual
  DATA_READY timing.

  HARDWARE — UNCHANGED
  --------------------
  Shared SPI:
    SCK  GPIO18
    MOSI GPIO23
    MISO GPIO19
    ADXL1 CS GPIO5   = tool/reference sensor
    ADXL2 CS GPIO17  = hand/error sensor

  Actuator path:
    DAC GPIO25 remains at 128 with no waveform.
    Amplifier OFF.
    Visaton unplugged/not driven.

  ADXL345 configuration:
    DATA_FORMAT 0x0B = full resolution, +/-16 g, 4-wire SPI
    BW_RATE     0x0E = nominal 1600 Hz ODR
    POWER_CTL   0x08 = measurement mode

  IMPORTANT
  ---------
  - No target result/noise level is hardcoded.
  - f 268 only selects which frequency is PROBED.
  - n 10 only selects how many valid repeated records are collected.
  - Raw LSB values are preserved.
  - m/s^2 values use nominal 0.0039 g/LSB and are explicitly labelled
    nominal; final calibrated rescaling can be done later.
  - No candidate static calibration constants are applied.

  ACQUISITION METHOD
  ------------------
  Each sensor is acquired independently inside each campaign record:
    ADXL1: N fresh DATA_READY samples
    ADXL2: N fresh DATA_READY samples

  This is preferable for stationary sensor-noise characterization:
  each spectrum contains genuinely fresh samples from that sensor's
  own ODR clock. Simultaneous dual-sensor timing is NOT required for
  a stationary noise-floor test.

  Each record:
    N = 4096 fresh samples/sensor
    nominal duration ~2.56 s/sensor
    actual effective Fs is measured from DATA_READY timestamps

  COMMANDS
  --------
    o       one diagnostic record
    t       full campaign
    n 10    number of valid campaign records
    f 268   exact-frequency noise probe
    v       verify registers
    p       print settings
    c       print last campaign summary
    x       clear summary
    h       help
  ============================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// ---------------- Hardware ----------------
static constexpr int PIN_SCK  = 18;
static constexpr int PIN_MOSI = 23;
static constexpr int PIN_MISO = 19;
static constexpr int PIN_CS1  = 5;
static constexpr int PIN_CS2  = 17;
static constexpr int PIN_DAC  = 25;

SPISettings adxlSPI(5000000, MSBFIRST, SPI_MODE3);

// ---------------- ADXL345 registers ----------------
static constexpr uint8_t REG_DEVID       = 0x00;
static constexpr uint8_t REG_BW_RATE     = 0x2C;
static constexpr uint8_t REG_POWER_CTL   = 0x2D;
static constexpr uint8_t REG_INT_SOURCE  = 0x30;
static constexpr uint8_t REG_DATA_FORMAT = 0x31;
static constexpr uint8_t REG_DATAX0      = 0x32;

static constexpr uint8_t EXPECT_DEVID       = 0xE5;
static constexpr uint8_t EXPECT_DATA_FORMAT = 0x0B;
static constexpr uint8_t EXPECT_BW_RATE     = 0x0E;
static constexpr uint8_t EXPECT_POWER_CTL   = 0x08;

static constexpr uint8_t MASK_DATA_READY = 0x80;

// ---------------- Test configuration ----------------
static constexpr float NOMINAL_ODR_HZ = 1600.0f;
static constexpr int N = 4096;
static constexpr float G0 = 9.80665f;
static constexpr float NOMINAL_G_PER_LSB = 0.0039f;
static constexpr float NOMINAL_MPS2_PER_LSB = NOMINAL_G_PER_LSB * G0;

static constexpr float FFT_SEARCH_MIN_HZ = 5.0f;
static constexpr float FFT_SEARCH_MAX_HZ = 700.0f;

static constexpr int CLIP_RAW_LIMIT = 4090;
static constexpr uint32_t DATA_READY_TIMEOUT_US = 5000; // > several nominal sample periods
static constexpr int MAX_CAMPAIGN_RECORDS = 30;
static constexpr int MAX_EXTRA_ATTEMPTS = 8;

static int campaignTargetRecords = 10;
static float targetFreqHz = 268.0f;

// ---------------- Sensor metadata ----------------
struct SensorCfg {
  const char* name;
  const char* role;
  int cs;
};

SensorCfg sensors[2] = {
  {"ADXL1", "TOOL_REFERENCE", PIN_CS1},
  {"ADXL2", "HAND_ERROR", PIN_CS2}
};

// Only one sensor record is stored at a time.
static int16_t rawAxis[3][N];
static uint32_t sampleTimeUs[N];

static float fftReal[N];
static float fftImag[N];

// ---------------- Metrics ----------------
struct AxisMetrics {
  double meanLSB;
  double sdLSB;
  double p2pLSB;
  double targetAmpLSB;
  double fftPeakHz;
  double fftPeakAmpLSB;
};

struct SensorRecordMetrics {
  AxisMetrics axis[3];

  uint32_t clipCount;
  uint32_t dataReadyTimeouts;
  uint32_t minIntervalUs;
  uint32_t maxIntervalUs;
  double meanIntervalUs;
  double sdIntervalUs;
  double effectiveFsHz;

  bool rawChanged;
  bool regBefore;
  bool regAfter;
  bool valid;
};

struct StatAccum {
  int n;
  double sum;
  double sumSq;
  double maxVal;

  void clear() {
    n = 0;
    sum = 0.0;
    sumSq = 0.0;
    maxVal = 0.0;
  }

  void add(double x) {
    n++;
    sum += x;
    sumSq += x * x;
    if (n == 1 || x > maxVal) maxVal = x;
  }

  double mean() const {
    return n ? sum / n : NAN;
  }

  double sd() const {
    if (n < 2) return 0.0;
    double m = mean();
    double v = (sumSq - n * m * m) / (n - 1);
    if (v < 0.0) v = 0.0;
    return sqrt(v);
  }
};

struct CampaignStats {
  StatAccum broadbandSd[2][3];
  StatAccum p2p[2][3];
  StatAccum targetAmp[2][3];
  StatAccum fftPeakAmp[2][3];
  StatAccum effectiveFs[2];
  StatAccum intervalSdUs[2];

  int validRecords;
  int attemptedRecords;

  void clear() {
    validRecords = 0;
    attemptedRecords = 0;

    for (int s = 0; s < 2; s++) {
      effectiveFs[s].clear();
      intervalSdUs[s].clear();

      for (int a = 0; a < 3; a++) {
        broadbandSd[s][a].clear();
        p2p[s][a].clear();
        targetAmp[s][a].clear();
        fftPeakAmp[s][a].clear();
      }
    }
  }
};

CampaignStats campaign;

// ---------------- SPI helpers ----------------
static inline void csLow(int cs) {
  digitalWrite(cs, LOW);
}

static inline void csHigh(int cs) {
  digitalWrite(cs, HIGH);
}

uint8_t adxlReadReg(int cs, uint8_t reg) {
  SPI.beginTransaction(adxlSPI);
  csLow(cs);
  SPI.transfer(reg | 0x80);
  uint8_t v = SPI.transfer(0x00);
  csHigh(cs);
  SPI.endTransaction();
  return v;
}

void adxlWriteReg(int cs, uint8_t reg, uint8_t value) {
  SPI.beginTransaction(adxlSPI);
  csLow(cs);
  SPI.transfer(reg & 0x3F);
  SPI.transfer(value);
  csHigh(cs);
  SPI.endTransaction();
}

void adxlReadXYZ(int cs, int16_t &x, int16_t &y, int16_t &z) {
  uint8_t b[6];

  SPI.beginTransaction(adxlSPI);
  csLow(cs);
  SPI.transfer(REG_DATAX0 | 0xC0); // READ + MULTIBYTE
  for (int i = 0; i < 6; i++) b[i] = SPI.transfer(0x00);
  csHigh(cs);
  SPI.endTransaction();

  x = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
  y = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
  z = (int16_t)(((uint16_t)b[5] << 8) | b[4]);
}

bool adxlDataReady(int cs) {
  return (adxlReadReg(cs, REG_INT_SOURCE) & MASK_DATA_READY) != 0;
}

void initializeADXL(int cs) {
  adxlWriteReg(cs, REG_POWER_CTL, 0x00);
  delay(5);

  adxlWriteReg(cs, REG_DATA_FORMAT, EXPECT_DATA_FORMAT);
  adxlWriteReg(cs, REG_BW_RATE, EXPECT_BW_RATE);
  adxlWriteReg(cs, REG_POWER_CTL, EXPECT_POWER_CTL);

  delay(20);

  // Read once to clear any stale DATA_READY state.
  int16_t x, y, z;
  if (adxlDataReady(cs)) {
    adxlReadXYZ(cs, x, y, z);
  }
}

bool verifyOneSensor(int idx, bool printLine = true) {
  int cs = sensors[idx].cs;

  uint8_t devid = adxlReadReg(cs, REG_DEVID);
  uint8_t fmt = adxlReadReg(cs, REG_DATA_FORMAT);
  uint8_t bw = adxlReadReg(cs, REG_BW_RATE);
  uint8_t pwr = adxlReadReg(cs, REG_POWER_CTL);

  bool ok =
      devid == EXPECT_DEVID &&
      fmt == EXPECT_DATA_FORMAT &&
      bw == EXPECT_BW_RATE &&
      pwr == EXPECT_POWER_CTL;

  if (printLine) {
    Serial.print("REGISTER_CHECK,");
    Serial.print(sensors[idx].name);
    Serial.print(",DEVID=0x"); Serial.print(devid, HEX);
    Serial.print(",DATA_FORMAT=0x"); Serial.print(fmt, HEX);
    Serial.print(",BW_RATE=0x"); Serial.print(bw, HEX);
    Serial.print(",POWER_CTL=0x"); Serial.print(pwr, HEX);
    Serial.print(",STATUS="); Serial.println(ok ? "PASS" : "FAIL");
  }

  return ok;
}

bool ensureSensorHealthy(int idx) {
  if (verifyOneSensor(idx, false)) return true;

  Serial.print("REGISTER_RECOVERY,sensor=");
  Serial.print(sensors[idx].name);
  Serial.println(",action=REINITIALIZE");

  initializeADXL(sensors[idx].cs);
  return verifyOneSensor(idx, true);
}

// ---------------- FFT ----------------
void fftInPlace(float *real, float *imag, int n) {
  int j = 0;

  for (int i = 1; i < n; i++) {
    int bit = n >> 1;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j ^= bit;

    if (i < j) {
      float tr = real[i]; real[i] = real[j]; real[j] = tr;
      float ti = imag[i]; imag[i] = imag[j]; imag[j] = ti;
    }
  }

  for (int len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * PI / len;
    float wc = cosf(ang);
    float ws = sinf(ang);

    for (int i = 0; i < n; i += len) {
      float c = 1.0f;
      float s = 0.0f;

      for (int k = 0; k < len / 2; k++) {
        int u = i + k;
        int v = i + k + len / 2;

        float vr = real[v] * c - imag[v] * s;
        float vi = real[v] * s + imag[v] * c;

        float ur = real[u];
        float ui = imag[u];

        real[u] = ur + vr;
        imag[u] = ui + vi;
        real[v] = ur - vr;
        imag[v] = ui - vi;

        float nc = c * wc - s * ws;
        float ns = c * ws + s * wc;
        c = nc;
        s = ns;
      }
    }
  }
}

// ---------------- Axis analysis ----------------
AxisMetrics analyzeAxis(const int16_t *x, double effectiveFsHz) {
  AxisMetrics m{};

  double sum = 0.0;
  double minv = x[0];
  double maxv = x[0];

  for (int i = 0; i < N; i++) {
    double v = x[i];
    sum += v;
    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
  }

  m.meanLSB = sum / N;
  m.p2pLSB = maxv - minv;

  double ss = 0.0;
  for (int i = 0; i < N; i++) {
    double d = (double)x[i] - m.meanLSB;
    ss += d * d;
  }
  m.sdLSB = sqrt(ss / (N - 1));

  // Hann weighted exact-frequency amplitude.
  double sumW = 0.0;
  double sumC = 0.0;
  double sumS = 0.0;
  double omega = 2.0 * PI * targetFreqHz / effectiveFsHz;

  for (int i = 0; i < N; i++) {
    double w = 0.5 - 0.5 * cos(2.0 * PI * i / (N - 1));
    double d = ((double)x[i] - m.meanLSB) * w;
    double ph = omega * i;

    sumC += d * cos(ph);
    sumS += d * sin(ph);
    sumW += w;
  }

  m.targetAmpLSB = 2.0 * sqrt(sumC * sumC + sumS * sumS) / sumW;

  // FFT using measured effective Fs.
  for (int i = 0; i < N; i++) {
    float w = 0.5f - 0.5f * cosf(2.0f * PI * i / (N - 1));
    fftReal[i] = ((float)x[i] - (float)m.meanLSB) * w;
    fftImag[i] = 0.0f;
  }

  fftInPlace(fftReal, fftImag, N);

  double df = effectiveFsHz / N;
  int kMin = (int)ceil(FFT_SEARCH_MIN_HZ / df);
  int kMax = (int)floor(FFT_SEARCH_MAX_HZ / df);

  if (kMin < 1) kMin = 1;
  if (kMax > N / 2 - 1) kMax = N / 2 - 1;

  double bestAmp = -1.0;
  int bestBin = kMin;

  for (int k = kMin; k <= kMax; k++) {
    double mag = sqrt((double)fftReal[k] * fftReal[k] +
                      (double)fftImag[k] * fftImag[k]);

    double amp = 2.0 * mag / sumW;

    if (amp > bestAmp) {
      bestAmp = amp;
      bestBin = k;
    }
  }

  m.fftPeakHz = bestBin * df;
  m.fftPeakAmpLSB = bestAmp;

  return m;
}

// ---------------- DATA_READY acquisition ----------------
bool waitForFreshSample(int cs, uint32_t &readyTimeUs) {
  uint32_t start = micros();

  while (true) {
    if (adxlDataReady(cs)) {
      readyTimeUs = micros();
      return true;
    }

    if ((uint32_t)(micros() - start) > DATA_READY_TIMEOUT_US) {
      return false;
    }
  }
}

bool acquireSensorRecord(int sensorIdx,
                         int recordNo,
                         SensorRecordMetrics &m) {
  memset(&m, 0, sizeof(m));
  m.minIntervalUs = UINT32_MAX;

  int cs = sensors[sensorIdx].cs;

  m.regBefore = ensureSensorHealthy(sensorIdx);
  if (!m.regBefore) {
    m.valid = false;
    return false;
  }

  Serial.print("SENSOR_RECORD_START,record=");
  Serial.print(recordNo);
  Serial.print(",sensor=");
  Serial.print(sensors[sensorIdx].name);
  Serial.print(",role=");
  Serial.print(sensors[sensorIdx].role);
  Serial.print(",N=");
  Serial.print(N);
  Serial.print(",ODR_setting_Hz=");
  Serial.print(NOMINAL_ODR_HZ, 3);
  Serial.print(",acquisition=DATA_READY_SYNCHRONIZED");
  Serial.println();

  // Quiet settling interval
  delay(750);

  // Clear stale ready state if needed
  int16_t dumpX, dumpY, dumpZ;
  if (adxlDataReady(cs)) {
    adxlReadXYZ(cs, dumpX, dumpY, dumpZ);
  }

  bool havePrev = false;
  int16_t prev[3] = {0,0,0};

  for (int i = 0; i < N; i++) {
    uint32_t tReady = 0;

    if (!waitForFreshSample(cs, tReady)) {
      m.dataReadyTimeouts++;
      Serial.print("DATA_READY_TIMEOUT,sensor=");
      Serial.print(sensors[sensorIdx].name);
      Serial.print(",sample=");
      Serial.println(i);
      m.valid = false;
      return false;
    }

    int16_t x, y, z;
    adxlReadXYZ(cs, x, y, z); // this clears the DATA_READY condition

    sampleTimeUs[i] = tReady;
    rawAxis[0][i] = x;
    rawAxis[1][i] = y;
    rawAxis[2][i] = z;

    int16_t cur[3] = {x,y,z};

    for (int a = 0; a < 3; a++) {
      if (abs((int)cur[a]) >= CLIP_RAW_LIMIT) m.clipCount++;

      if (havePrev && cur[a] != prev[a]) {
        m.rawChanged = true;
      }

      prev[a] = cur[a];
    }

    havePrev = true;
  }

  // Timing statistics from DATA_READY timestamps.
  double sumDt = 0.0;
  double sumDt2 = 0.0;

  for (int i = 1; i < N; i++) {
    uint32_t dt = sampleTimeUs[i] - sampleTimeUs[i - 1];

    if (dt < m.minIntervalUs) m.minIntervalUs = dt;
    if (dt > m.maxIntervalUs) m.maxIntervalUs = dt;

    sumDt += dt;
    sumDt2 += (double)dt * dt;
  }

  int dtN = N - 1;
  m.meanIntervalUs = sumDt / dtN;

  double varDt =
      (sumDt2 - dtN * m.meanIntervalUs * m.meanIntervalUs) /
      (dtN - 1);

  if (varDt < 0.0) varDt = 0.0;
  m.sdIntervalUs = sqrt(varDt);

  double elapsedUs = (double)(sampleTimeUs[N - 1] - sampleTimeUs[0]);
  m.effectiveFsHz = (N - 1) * 1000000.0 / elapsedUs;

  m.regAfter = verifyOneSensor(sensorIdx, false);

  for (int a = 0; a < 3; a++) {
    m.axis[a] = analyzeAxis(rawAxis[a], m.effectiveFsHz);
  }

  m.valid =
      m.regBefore &&
      m.regAfter &&
      m.rawChanged &&
      m.clipCount == 0 &&
      m.dataReadyTimeouts == 0 &&
      m.effectiveFsHz > 1000.0 &&
      m.effectiveFsHz < 2200.0;

  Serial.print("SENSOR_RECORD_VALIDITY,record=");
  Serial.print(recordNo);
  Serial.print(",sensor=");
  Serial.print(sensors[sensorIdx].name);
  Serial.print(",clipCount=");
  Serial.print(m.clipCount);
  Serial.print(",dataReadyTimeouts=");
  Serial.print(m.dataReadyTimeouts);
  Serial.print(",rawChanged=");
  Serial.print(m.rawChanged ? 1 : 0);
  Serial.print(",regBefore=");
  Serial.print(m.regBefore ? 1 : 0);
  Serial.print(",regAfter=");
  Serial.print(m.regAfter ? 1 : 0);
  Serial.print(",meanInterval_us=");
  Serial.print(m.meanIntervalUs, 4);
  Serial.print(",sdInterval_us=");
  Serial.print(m.sdIntervalUs, 4);
  Serial.print(",minInterval_us=");
  Serial.print(m.minIntervalUs);
  Serial.print(",maxInterval_us=");
  Serial.print(m.maxIntervalUs);
  Serial.print(",effectiveFs_Hz=");
  Serial.print(m.effectiveFsHz, 6);
  Serial.print(",STATUS=");
  Serial.println(m.valid ? "VALID" : "REJECTED");

  double conv = NOMINAL_MPS2_PER_LSB;

  for (int a = 0; a < 3; a++) {
    AxisMetrics &x = m.axis[a];

    const char* axis =
        a == 0 ? "X" :
        a == 1 ? "Y" : "Z";

    Serial.print("NOISE_AXIS");
    Serial.print(",record="); Serial.print(recordNo);
    Serial.print(",sensor="); Serial.print(sensors[sensorIdx].name);
    Serial.print(",role="); Serial.print(sensors[sensorIdx].role);
    Serial.print(",axis="); Serial.print(axis);

    Serial.print(",mean_LSB="); Serial.print(x.meanLSB, 6);
    Serial.print(",mean_mps2_nominal="); Serial.print(x.meanLSB * conv, 8);

    Serial.print(",sd_LSB="); Serial.print(x.sdLSB, 6);
    Serial.print(",sd_mps2_nominal="); Serial.print(x.sdLSB * conv, 10);

    Serial.print(",p2p_LSB="); Serial.print(x.p2pLSB, 3);
    Serial.print(",p2p_mps2_nominal="); Serial.print(x.p2pLSB * conv, 10);

    Serial.print(",targetHz="); Serial.print(targetFreqHz, 4);
    Serial.print(",targetAmp_LSB="); Serial.print(x.targetAmpLSB, 8);
    Serial.print(",targetAmp_mps2_nominal="); Serial.print(x.targetAmpLSB * conv, 12);

    Serial.print(",fftPeakHz="); Serial.print(x.fftPeakHz, 5);
    Serial.print(",fftPeakAmp_LSB="); Serial.print(x.fftPeakAmpLSB, 8);
    Serial.print(",fftPeakAmp_mps2_nominal="); Serial.print(x.fftPeakAmpLSB * conv, 12);

    Serial.println();
  }

  return m.valid;
}

// ---------------- Campaign ----------------
void addSensorRecordToCampaign(int sensorIdx,
                               const SensorRecordMetrics &m) {
  campaign.effectiveFs[sensorIdx].add(m.effectiveFsHz);
  campaign.intervalSdUs[sensorIdx].add(m.sdIntervalUs);

  for (int a = 0; a < 3; a++) {
    campaign.broadbandSd[sensorIdx][a].add(
        m.axis[a].sdLSB * NOMINAL_MPS2_PER_LSB);

    campaign.p2p[sensorIdx][a].add(
        m.axis[a].p2pLSB * NOMINAL_MPS2_PER_LSB);

    campaign.targetAmp[sensorIdx][a].add(
        m.axis[a].targetAmpLSB * NOMINAL_MPS2_PER_LSB);

    campaign.fftPeakAmp[sensorIdx][a].add(
        m.axis[a].fftPeakAmpLSB * NOMINAL_MPS2_PER_LSB);
  }
}

void printCampaignSummary() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("TEST 02 V2 STATIONARY NOISE-FLOOR SUMMARY");
  Serial.println("============================================================");

  Serial.print("CAMPAIGN_STATUS,validRecords=");
  Serial.print(campaign.validRecords);
  Serial.print(",attemptedRecords=");
  Serial.print(campaign.attemptedRecords);
  Serial.print(",recordsTarget=");
  Serial.print(campaignTargetRecords);
  Serial.print(",targetHz=");
  Serial.print(targetFreqHz, 4);
  Serial.print(",N=");
  Serial.println(N);

  for (int s = 0; s < 2; s++) {
    Serial.print("SAMPLING_SUMMARY,sensor=");
    Serial.print(sensors[s].name);
    Serial.print(",n=");
    Serial.print(campaign.effectiveFs[s].n);
    Serial.print(",meanEffectiveFs_Hz=");
    Serial.print(campaign.effectiveFs[s].mean(), 8);
    Serial.print(",sdEffectiveFs_Hz=");
    Serial.print(campaign.effectiveFs[s].sd(), 8);
    Serial.print(",meanIntervalJitterSD_us=");
    Serial.print(campaign.intervalSdUs[s].mean(), 8);
    Serial.println();

    for (int a = 0; a < 3; a++) {
      const char* axis =
          a == 0 ? "X" :
          a == 1 ? "Y" : "Z";

      StatAccum &br = campaign.broadbandSd[s][a];
      StatAccum &pp = campaign.p2p[s][a];
      StatAccum &ta = campaign.targetAmp[s][a];
      StatAccum &fp = campaign.fftPeakAmp[s][a];

      Serial.print("NOISE_SUMMARY");
      Serial.print(",sensor="); Serial.print(sensors[s].name);
      Serial.print(",role="); Serial.print(sensors[s].role);
      Serial.print(",axis="); Serial.print(axis);
      Serial.print(",n="); Serial.print(br.n);

      Serial.print(",meanBroadbandSD_mps2_nominal=");
      Serial.print(br.mean(), 12);
      Serial.print(",sdBroadbandSD_mps2_nominal=");
      Serial.print(br.sd(), 12);
      Serial.print(",maxBroadbandSD_mps2_nominal=");
      Serial.print(br.maxVal, 12);

      Serial.print(",meanP2P_mps2_nominal=");
      Serial.print(pp.mean(), 12);
      Serial.print(",maxP2P_mps2_nominal=");
      Serial.print(pp.maxVal, 12);

      Serial.print(",targetHz=");
      Serial.print(targetFreqHz, 4);
      Serial.print(",meanTargetAmp_mps2_nominal=");
      Serial.print(ta.mean(), 12);
      Serial.print(",sdTargetAmp_mps2_nominal=");
      Serial.print(ta.sd(), 12);
      Serial.print(",maxTargetAmp_mps2_nominal=");
      Serial.print(ta.maxVal, 12);

      Serial.print(",meanFFTpeakAmp_mps2_nominal=");
      Serial.print(fp.mean(), 12);
      Serial.print(",maxFFTpeakAmp_mps2_nominal=");
      Serial.print(fp.maxVal, 12);

      Serial.println();
    }
  }

  Serial.println("SUMMARY_NOTE,all_results_computed_from_live_ADXL_samples=1");
  Serial.println("SUMMARY_NOTE,target_frequency_is_probe_only_not_expected_result=1");
  Serial.println("SUMMARY_NOTE,nominal_0.0039g_per_LSB_conversion_only=1");
  Serial.println("SUMMARY_NOTE,raw_LSB_preserved=1");
  Serial.println("SUMMARY_NOTE,static_candidate_calibration_applied=0");
  Serial.println("============================================================");
}

bool acquireCompleteRecord(int recordNo) {
  SensorRecordMetrics m1;
  SensorRecordMetrics m2;

  // Preserve actuator wiring, but no intentional drive.
  dacWrite(PIN_DAC, 128);

  Serial.println();
  Serial.print("RECORD_START,record=");
  Serial.print(recordNo);
  Serial.print(",targetHz=");
  Serial.print(targetFreqHz, 4);
  Serial.println(",condition=STATIONARY");

  bool ok1 = acquireSensorRecord(0, recordNo, m1);
  delay(400);
  bool ok2 = acquireSensorRecord(1, recordNo, m2);

  bool valid = ok1 && ok2;

  Serial.print("DUAL_RECORD_VALIDITY,record=");
  Serial.print(recordNo);
  Serial.print(",ADXL1=");
  Serial.print(ok1 ? "VALID" : "REJECTED");
  Serial.print(",ADXL2=");
  Serial.print(ok2 ? "VALID" : "REJECTED");
  Serial.print(",STATUS=");
  Serial.println(valid ? "VALID" : "REJECTED");

  if (valid) {
    addSensorRecordToCampaign(0, m1);
    addSensorRecordToCampaign(1, m2);
  }

  return valid;
}

void runOneDiagnostic() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("TEST 02 V2 — ONE DIAGNOSTIC RECORD");
  Serial.println("Do not touch the assembly.");
  Serial.println("============================================================");

  bool ok = acquireCompleteRecord(1);

  Serial.print("ONE_RECORD_COMPLETE,STATUS=");
  Serial.println(ok ? "VALID" : "REJECTED");
}

void runCampaign() {
  campaign.clear();

  Serial.println();
  Serial.println("============================================================");
  Serial.println("TEST 02 V2 — STATIONARY NOISE-FLOOR CAMPAIGN");
  Serial.println("Tool OFF. Amplifier OFF. Visaton unplugged/not driven.");
  Serial.println("Leave the assembly completely untouched.");
  Serial.println("============================================================");

  int maxAttempts = campaignTargetRecords + MAX_EXTRA_ATTEMPTS;

  while (campaign.validRecords < campaignTargetRecords &&
         campaign.attemptedRecords < maxAttempts) {

    campaign.attemptedRecords++;

    int recordNo = campaign.validRecords + 1;

    bool ok = acquireCompleteRecord(recordNo);

    if (ok) {
      campaign.validRecords++;

      Serial.print("CAMPAIGN_PROGRESS,validRecords=");
      Serial.print(campaign.validRecords);
      Serial.print(",targetRecords=");
      Serial.println(campaignTargetRecords);
    } else {
      Serial.println("CAMPAIGN_PROGRESS,rejectedRecord=1,retry=1");
    }

    delay(750);
  }

  Serial.print("CAMPAIGN_COMPLETE,STATUS=");
  Serial.print(
      campaign.validRecords == campaignTargetRecords ?
      "PASS_DATA_COLLECTION" : "INCOMPLETE");

  Serial.print(",validRecords=");
  Serial.print(campaign.validRecords);
  Serial.print(",targetRecords=");
  Serial.println(campaignTargetRecords);

  printCampaignSummary();
}

// ---------------- UI ----------------
void printSettings() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("TEST 02 V2 CONFIGURATION");
  Serial.println("============================================================");

  Serial.println("ADXL1=TOOL_REFERENCE,CS=GPIO5");
  Serial.println("ADXL2=HAND_ERROR,CS=GPIO17");
  Serial.println("SPI_SCK=GPIO18,SPI_MOSI=GPIO23,SPI_MISO=GPIO19");
  Serial.println("SPI_CLOCK_HZ=5000000,SPI_MODE=3");
  Serial.println("DATA_FORMAT=0x0B,BW_RATE=0x0E,POWER_CTL=0x08");
  Serial.println("ACQUISITION=ADXL_DATA_READY_SYNCHRONIZED");
  Serial.println("DAC_GPIO25=128,NO_WAVEFORM=1");

  Serial.print("nominalODR_Hz=");
  Serial.println(NOMINAL_ODR_HZ, 3);

  Serial.print("N=");
  Serial.println(N);

  Serial.print("nominalRecordDuration_s=");
  Serial.println(N / NOMINAL_ODR_HZ, 6);

  Serial.print("targetFrequency_Hz=");
  Serial.println(targetFreqHz, 4);

  Serial.print("campaignValidRecords=");
  Serial.println(campaignTargetRecords);

  Serial.print("nominalScale_gPerLSB=");
  Serial.println(NOMINAL_G_PER_LSB, 7);

  Serial.print("nominalScale_mps2PerLSB=");
  Serial.println(NOMINAL_MPS2_PER_LSB, 10);

  Serial.println("candidate_static_calibration_applied=0");
  Serial.println("raw_LSB_preserved=1");
  Serial.println("============================================================");
}

void printHelp() {
  Serial.println();
  Serial.println("================ TEST 02 V2 COMMANDS =============");
  Serial.println("o        = one diagnostic stationary-noise record");
  Serial.println("t        = full stationary-noise campaign");
  Serial.println("n 10     = set number of valid campaign records (1..30)");
  Serial.println("f 268    = set exact-frequency probe (5..700 Hz)");
  Serial.println("v        = verify both ADXL345 registers");
  Serial.println("p        = print settings");
  Serial.println("c        = print current/last campaign summary");
  Serial.println("x        = clear campaign summary");
  Serial.println("h        = help");
  Serial.println("===================================================");
}

void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;

  if (cmd == "h") {
    printHelp();
    return;
  }

  if (cmd == "p") {
    printSettings();
    return;
  }

  if (cmd == "v") {
    verifyOneSensor(0, true);
    verifyOneSensor(1, true);
    return;
  }

  if (cmd == "o") {
    runOneDiagnostic();
    return;
  }

  if (cmd == "t") {
    runCampaign();
    return;
  }

  if (cmd == "c") {
    printCampaignSummary();
    return;
  }

  if (cmd == "x") {
    campaign.clear();
    Serial.println("CAMPAIGN_RESULTS_CLEARED,STATUS=OK");
    return;
  }

  if (cmd.startsWith("n ")) {
    int n = cmd.substring(2).toInt();

    if (n < 1 || n > MAX_CAMPAIGN_RECORDS) {
      Serial.println("COMMAND_ERROR,n_must_be_1_to_30");
      return;
    }

    campaignTargetRecords = n;

    Serial.print("SETTING_UPDATED,campaignValidRecords=");
    Serial.println(campaignTargetRecords);
    return;
  }

  if (cmd.startsWith("f ")) {
    float f = cmd.substring(2).toFloat();

    if (f < 5.0f || f > 700.0f) {
      Serial.println("COMMAND_ERROR,target_frequency_must_be_5_to_700_Hz");
      return;
    }

    targetFreqHz = f;

    Serial.print("SETTING_UPDATED,targetFrequency_Hz=");
    Serial.println(targetFreqHz, 4);
    return;
  }

  Serial.print("COMMAND_ERROR,unknown_command=");
  Serial.print(cmd);
  Serial.println(",type_h_for_help=1");
}

// ---------------- Setup / loop ----------------
void setup() {
  pinMode(PIN_CS1, OUTPUT);
  pinMode(PIN_CS2, OUTPUT);

  csHigh(PIN_CS1);
  csHigh(PIN_CS2);

  Serial.begin(115200);
  delay(1200);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  dacWrite(PIN_DAC, 128);

  initializeADXL(PIN_CS1);
  initializeADXL(PIN_CS2);

  campaign.clear();

  Serial.println();
  Serial.println("============================================================");
  Serial.println("KK TEST 02 V2 — DUAL ADXL345 STATIONARY NOISE FLOOR");
  Serial.println("============================================================");
  Serial.println("RESULT_HARDCODING=NONE");
  Serial.println("ACQUISITION=DATA_READY_SYNCHRONIZED");
  Serial.println("CONDITION,tool=OFF,amplifier=OFF,visaton=UNPLUGGED_OR_NOT_DRIVEN");
  Serial.println("CONDITION,assembly=RIGID_AND_UNTOUCHED");
  Serial.println("CONDITION,hardware_architecture=UNCHANGED");

  bool ok1 = verifyOneSensor(0, true);
  bool ok2 = verifyOneSensor(1, true);

  Serial.print("STARTUP_STATUS=");
  Serial.println(ok1 && ok2 ? "PASS" : "FAIL");

  printSettings();
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}
