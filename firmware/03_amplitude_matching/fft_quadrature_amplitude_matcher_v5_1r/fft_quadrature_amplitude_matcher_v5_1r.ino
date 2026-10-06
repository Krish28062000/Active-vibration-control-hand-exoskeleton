/*
  ============================================================================
  KK-EXOSKELETON - INTEGRATED NARROWBAND FFT/QUADRATURE + V5.1R MATCHER V1.3A DRIFT-TOLERANT ASCII-SAFE
  ============================================================================

  THIS FILE INTEGRATES THE TWO FROZEN ALGORITHMS:
    A) Frozen dual-ADXL FFT + exact-frequency quadrature analyser
    B) Frozen Universal XYZ Adaptive Amplitude Matcher V5.1R

  FINAL EXPERIMENTAL WORKFLOW
    1) Tool ON, Visaton OFF -> enter: a
       - hard-coded 240..320 Hz TOOL search on ADXL1
       - automatic quality-weighted f0 selection only if there is a clear winner
       - exact-frequency X/Y/Z amplitudes printed for ADXL1 and ADXL2 at f0
       - if the narrowband result is ambiguous, the run aborts and must be repeated

    2) Choose the ADXL2 error-sensor axis to attenuate -> enter: x, y, or z
       - frequency is ALWAYS the common tool frequency f0 from ADXL1
       - target amplitude is the selected ADXL2-axis amplitude measured at that f0

    3) Switch TOOL OFF without moving the setup -> enter: c
       - V5.1R performs its frozen adaptive amplitude-matching procedure
       - after servo lock, the FINAL DAC is frozen
       - the NCO keeps running continuously at f0
       - NO amplitude servo runs after this point

    4) Visaton remains at the fixed matched command until: s
       - this fixed-DAC state is the correct hand-off for the later phase sweep
       - during phase cancellation the selected ADXL2 signal will intentionally fall,
         so a live amplitude servo must NOT be allowed to increase the actuator drive

  FINAL COMMANDS
    a       acquire tool/hand baseline and auto-select f0 in 240..320 Hz
    x/y/z   choose ADXL2 error-sensor axis after a successful baseline
    c       operator confirms TOOL OFF; run V5.1R matcher and freeze final DAC
    g       diagnostic local triaxial FFT while Visaton is running
    p       settings/status
    s       immediate Visaton stop and reset integrated workflow
    ?       help

  IMPORTANT
    - The legacy V5.1R comments below are retained because this file preserves its
      validated control core. Their old manual f/t/r/v/m/k command list is superseded
      by the integrated command list above.
    - ADXL1 and ADXL2 cross-sensor phase is NOT used as a transfer phase.
    - ADXL345 raw clipping guard is standardized at +/-4000 counts.
  ============================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <arduinoFFT.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

/*
  ============================================================================
  KK-EXOSKELETON - UNIVERSAL XYZ ADAPTIVE AMPLITUDE MATCHER V5.1R ROBUST LOCAL-REBRACKET - ARDUINO 1.8.x COMPILE FIX
  ESP32 Arduino Core 3.x + ADXL345 SPI + TPA3116D2 + Visaton EX 45 S
  ============================================================================

  DESIGN GOAL
    Reliably acquire and regulate one selected exact-frequency acceleration
    component (X, Y or Z) even when the physical DAC -> acceleration mapping
    drifts between runs or slowly changes during a run.

  FINAL ARCHITECTURE
    1) phase-continuous 10 kHz hardware-timer NCO;
    2) exact-command-frequency NCO-referenced X/Y/Z I/Q measurement;
    3) robust same-DAC repeated-point validation;
    4) hybrid sign-bracketed root finder (secant/regula-falsi + bisection);
    5) robust local dA/dDAC slope memory;
    6) slow, conservative PI amplitude servo around the continuously running NCO;
    7) automatic transient confirmation and bounded reacquisition;
    8) closed-loop amplitude-hold statistics with quantization-aware regulation;
    9) final triaxial vector FFT frequency verification;
   10) V5.1 candidate confirmation + state-aware plant-change guard;
   11) verified trial/revert DAC corrections;
   12) conservative warm-start from the last proven same-condition solution;
   13) bounded local re-bracketing after a moved bracket, before any full restart.

  IMPORTANT CONTROL PRINCIPLE
    DAC is NOT the controlled physical quantity.
    The selected exact-frequency acceleration is controlled.
    Therefore DAC is allowed to move slowly during closed-loop regulation.

  IMPORTANT PHASE PRINCIPLE
    Frequency/phase generation never restarts while matching or tracking.
    Amplitude table changes occur only at NCO phase wrap (near sine zero),
    avoiding a mid-cycle amplitude discontinuity.

  IMPORTANT MEASUREMENT PRINCIPLE
    The same canonical estimator is used throughout acquisition and regulation:
      3 windows x 512 samples/window at 1600 samples/s.
    Timing-bad windows are discarded individually; clean windows remain usable.
    Amplitude validity is determined from the exact-frequency amplitude estimator,
    repeatability, sensor integrity and stability. Phase/Sync diagnostics do not gate
    amplitude acquisition or amplitude-hold PASS.

  IMPORTANT MEMORY PRINCIPLE
    All large FFT/sample buffers are global/static. No large search/history arrays
    are placed on the ESP32 loop-task stack. Closed-loop statistics use running
    accumulators instead of large local arrays. This directly avoids the stack
    corruption that produced the previous "Double exception" Guru Meditation.

  HARDWARE
    ADXL1 CS GPIO5   : forced HIGH, not read
    ADXL2 CS GPIO17  : rigidly mounted to Visaton/final actuator plate
    SPI SCK GPIO18
    SPI MISO GPIO19
    SPI MOSI GPIO23
    ADXL SPI: 5 MHz, MSBFIRST, SPI_MODE3 (preserved from working V4)
    ESP32 DAC GPIO25 -> TPA3116D2 -> Visaton EX 45 S

  SERIAL COMMANDS (115200 baud, Newline)
    f 268       set frequency [20...350 Hz]
    t 1.20      set selected-axis target peak acceleration [m/s^2]
    x / y / z   select control axis
    h 8         set closed-loop hold duration [5...30 s]
    n 5         set number of independent full runs [1...8]

    r           one COMPLETE run:
                acquire -> servo lock -> closed-loop hold -> FFT -> stop
    v           N independent COMPLETE runs

    m           acquire + servo lock only; leave actuator running
    k 30        track current acquired target for 5...600 s
    q 15        fixed-DAC 3-record diagnostic at DAC 15
    g           final local triaxial FFT at current running command

    p           settings/status
    s           emergency stop
    ?           help

  RECOMMENDED FIRST VALIDATION
    f 272
    t 1.20
    z
    h 8
    n 1
    r

  Then, without moving the setup:
    n 5
    v
  ============================================================================
*/

// ============================================================================
// DATA TYPES
// ============================================================================

enum class ControlAxis : uint8_t { X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2 };

enum class AcquireStatus : uint8_t {
  OK,
  QUANTIZATION_LIMITED,
  TARGET_BELOW_RANGE,
  TARGET_ABOVE_RANGE,
  NONSTATIONARY,
  HARDWARE_FAULT,
  SAFETY_STOP,
  ABORTED
};

enum class ServoStatus : uint8_t {
  LOCKED,
  COMPLETED,
  NEEDS_REACQUIRE,
  AUTHORITY_LIMIT,
  HARDWARE_FAULT,
  SAFETY_STOP,
  ABORTED
};

enum class PhaseClass : uint8_t { STRONG, MODERATE, WEAK };

struct RawAcceleration {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct AxisSyncResult {
  double amplitudeMs2;
  double phaseDeg;
  double syncRatio;
};

struct WindowResult {
  bool valid;
  bool communicationFault;
  bool rawClipped;
  bool timingFault;
  uint16_t zeroTriplets;
  uint16_t longestStagnantRun;
  uint32_t lateSamples;
  uint32_t maxLatenessUs;
  AxisSyncResult axis[3];
  double totalAmplitudeMs2;
};

struct ControlRecord {
  bool measured;
  bool amplitudeValid;
  bool phaseReadyStrong;
  bool communicationFault;
  bool rawClipped;
  bool safetyExceeded;
  bool timingRejected;
  uint8_t validWindows;
  uint8_t inlierCount;
  bool windowValid[3];
  double windowSelectedMs2[3];
  double dac;
  uint32_t timestampMs;

  double selectedMedianMs2;
  double selectedMeanMs2;
  double axisMeanMs2[3];
  double axisSync[3];
  double axisPhaseDeg[3];
  double selectedPhaseSdDeg;
  double totalMeanMs2;
  double madPercent;
  double cvPercent;
};

struct StablePoint {
  bool valid;
  bool stationary;
  bool phaseReadyStrong;
  bool safetyExceeded;
  bool hardwareFault;
  double dac;
  double amplitudeMs2;
  double pairDriftPercent;
  uint8_t recordsUsed;
  uint32_t timestampMs;
  double axisMeanMs2[3];
  double axisSync[3];
  double axisPhaseDeg[3];
  double totalMeanMs2;
  double selectedPhaseSdDeg;
};

struct AcquisitionResult {
  AcquireStatus status;
  bool success;
  StablePoint candidate;
  StablePoint low;
  StablePoint high;
  uint16_t recordsMeasured;
  uint16_t qualityRejects;
  uint8_t restarts;
  uint32_t elapsedMs;
};

struct PlantBaseline {
  bool valid;
  double dac;
  double selectedMs2;
  double selectedCvPercent;
  double axisMeanMs2[3];
  double totalMeanMs2;
};

struct WarmStartMemory {
  bool valid;
  ControlAxis axis;
  double frequencyHz;
  double targetMs2;
  double dac;
};

struct ServoState {
  double integralErrorMs2;
  int8_t persistentSign;
  uint8_t persistenceCount;
  uint8_t invalidCount;
  uint8_t unproductiveCorrections;

  // V5.1: a small same-DAC plant fingerprint established only from
  // confirmed stable records. It is a state detector, not a calibration map.
  PlantBaseline plantBaseline;

  // V5.1: every DAC correction is a bounded trial. Two post-step records
  // verify improvement before the new DAC is accepted and its slope is learned.
  bool trialPending;
  double trialOldDac;
  double trialOldAmp;
  double trialOldErrorPct;
  PlantBaseline trialOldBaseline;
  uint8_t trialValidCount;
  double trialAmp[2];
  double trialAxisSum[3];
  double trialTotalSum;
  double trialMaxCvPercent;

  bool havePrevious;
  ControlRecord previous;
};

struct RunningStats {
  uint16_t n;
  double mean;
  double m2;
  double minValue;
  double maxValue;
};

struct HoldResult {
  bool completed;
  bool amplitudePass;
  bool phaseSweepReady;
  ServoStatus status;
  uint16_t validRecords;
  uint16_t tightInBandRecords;
  uint16_t finalInBandRecords;
  uint16_t corrections;
  uint16_t reacquireEvents;

  double meanAmplitudeMs2;
  double sdAmplitudeMs2;
  double cvPercent;
  double meanErrorPercent;
  double tightInBandPercent;
  double finalInBandPercent;
  double rmsErrorPercent;
  double maxAbsErrorPercent;

  double meanSyncRatio;
  double phaseSpanDeg;
  double minAmplitudeMs2;
  double maxAmplitudeMs2;
  double minDac;
  double maxDac;
  double meanDac;
  PhaseClass phaseClass;
};

struct VectorFftResult {
  bool valid;
  double frequencyHz;
  double xAmplitudeMs2;
  double yAmplitudeMs2;
  double zAmplitudeMs2;
  double totalAmplitudeMs2;
  double snrDb;
  char dominantAxis;
};

struct CompleteRunResult {
  bool completed;
  bool amplitudePass;
  bool cancellationReady;
  bool matchLockAchieved;
  uint32_t matchLockTimeMs;
  uint32_t totalRunTimeMs;
  double finalDac;
  AcquisitionResult acquisition;
  HoldResult hold;
  VectorFftResult fft;
};


// ============================================================================
// INTEGRATED NARROWBAND DISCOVERY TYPES
// ============================================================================

struct DiscoveryTiming {
  uint32_t elapsedUs;
  double meanReadSeparationUs;
  uint32_t maxReadSeparationUs;
  uint32_t maxScheduleLatenessUs;
  uint32_t lateSampleCount;
  uint16_t zeroTriplets;
  uint16_t clippedSamples;
  bool valid;
};

struct DiscoveryAxisFFTResult {
  bool valid;
  uint16_t bin;
  double frequencyHz;
  double amplitudeMs2;
  double snrDb;
};

struct DiscoverySensorFFTResult {
  DiscoveryAxisFFTResult axis[3];
};

struct DiscoveryAxisQuadResult {
  bool valid;
  double referenceHz;
  double correctionHz;
  double refinedHz;
  double amplitudeMs2;
  double phaseDeg;

  // Whole-record synchronous ratio is retained as a DIAGNOSTIC only.  A
  // hand-held tool can drift in frequency/phase over the 1.6 s record, so
  // this quantity is deliberately NOT the primary acceptance gate in V1.3.
  double syncRatio;

  // Drift-tolerant synchronization: synchronous ratio is calculated inside
  // each 256-sample (~160 ms) block and the median is used for quality gating.
  double localMedianSyncRatio;
  uint8_t localGoodBlocks;

  double phaseFitRmseDeg;
};

struct DiscoverySensorAtF0 {
  DiscoveryAxisQuadResult axis[3];
  double vectorAmplitudeMs2;
  double J;
};

struct DiscoveryCandidate {
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

  // Long-record sync is retained for reporting; local sync is the principal
  // drift-tolerant quality metric used by the V1.3 discovery gate.
  double toolWeightedSync;
  double toolWeightedLocalSync;
  double toolWeightedPhaseRmseDeg;
  uint8_t supportAxes;

  // Independent FFT peak consensus around the candidate.  This prevents a
  // low-sync but clearly repeated X/Y/hand frequency family from being called
  // "ambiguous", while still rejecting isolated/spurious candidates.
  uint8_t toolFrequencyConsensusAxes;
  uint8_t handFrequencyConsensusAxes;

  double qualityScore;
  DiscoverySensorAtF0 toolAtCandidate;
  DiscoverySensorAtF0 handAtCandidate;
};

enum class IntegratedWorkflowState : uint8_t {
  NEED_BASELINE,
  AWAIT_AXIS,
  AWAIT_TOOL_OFF,
  MATCHING,
  FIXED_MATCH_RUNNING
};

// ============================================================================
// HARDWARE
// ============================================================================

static const uint8_t PIN_CS_ADXL1 = 5;
static const uint8_t PIN_CS_ADXL2 = 17;
static const uint8_t PIN_SPI_SCK = 18;
static const uint8_t PIN_SPI_MISO = 19;
static const uint8_t PIN_SPI_MOSI = 23;
static const uint8_t PIN_VISATON_DAC = 25;

static const uint8_t REG_DEVID = 0x00;
static const uint8_t REG_BW_RATE = 0x2C;
static const uint8_t REG_POWER_CTL = 0x2D;
static const uint8_t REG_DATA_FORMAT = 0x31;
static const uint8_t REG_DATAX0 = 0x32;
static const uint8_t SPI_READ_BIT = 0x80;
static const uint8_t SPI_MB_BIT = 0x40;
static const uint8_t ADXL_DEVID_OK = 0xE5;

SPISettings adxlSpiSettings(5000000, MSBFIRST, SPI_MODE3);

// ============================================================================
// SENSOR / SAMPLE SETTINGS
// ============================================================================

static const double SAMPLE_RATE_HZ = 1600.0;
static const uint32_t SAMPLE_PERIOD_US = 625;

// Allow small scheduler/SPI jitter. A whole window is rejected only if more
// than a few samples exceed this threshold.
static const uint32_t LATE_SAMPLE_WARNING_US = 25;
static const uint8_t MAX_LATE_SAMPLES_PER_WINDOW = 4;

static const double ADXL_G_PER_LSB = 0.0039;
static const double GRAVITY_MS2 = 9.80665;
static const double ADXL_MS2_PER_LSB = ADXL_G_PER_LSB * GRAVITY_MS2;

// ADXL345 full-resolution +/-16 g is ~ +/-4100 counts, NOT +/-32768 counts.
// 4000 is therefore a useful conservative clipping warning threshold.
static const int16_t ADXL_RAW_CLIP_LIMIT = 4000;
static const uint16_t MAX_ZERO_TRIPLETS = 5;
static const uint16_t MAX_STAGNANT_RUN = 20;

// Software guardrails only; not certified safety limits.
static const double AXIS_SOFTWARE_GUARD_MS2 = 25.0;
static const double TOTAL_SOFTWARE_GUARD_MS2 = 30.0;

// ============================================================================
// NCO / COMMAND SETTINGS
// ============================================================================

static const double COMMAND_MIN_HZ = 20.0;
static const double COMMAND_MAX_HZ = 350.0;
static const double DAC_MIN = 3.0;
static const double DAC_MAX = 29.0;

// Keep the experimentally characterized command quantum for V3.
// 0.125 can be tested later as an optional experiment; it is not assumed here.
static const double DAC_QUANTUM = 0.25;

static const uint32_t DAC_UPDATE_RATE_HZ = 10000;
static const uint32_t TIMER_BASE_FREQUENCY_HZ = 1000000;
static const uint64_t TIMER_ALARM_TICKS = 100; // 1 MHz / 100 = 10 kHz

hw_timer_t *visatonTimer = nullptr;
volatile bool visatonRunning = false;
volatile uint32_t visatonPhaseAccumulator = 0;
volatile uint32_t visatonPhaseIncrement = 0;

// Double-buffered tables: amplitude switches only at phase wrap.
uint8_t visatonSineTable[2][256];
volatile uint8_t activeSineTable = 0;
volatile uint8_t pendingSineTable = 1;
volatile bool amplitudeTablePending = false;

double currentDacAmplitude = 0.0;
double currentCommandFrequencyHz = 0.0;

// ============================================================================
// CANONICAL EXACT-FREQUENCY ESTIMATOR
// ============================================================================

static const uint16_t CONTROL_SAMPLES = 512;
static const uint8_t CONTROL_WINDOWS = 3;
static const uint8_t CONTROL_MIN_INLIERS = 2;
static const uint16_t WINDOW_GAP_MS = 20;

static const double WINDOW_OUTLIER_PERCENT = 15.0;
static const double RECORD_MAX_MAD_PERCENT = 10.0;
static const double RECORD_MAX_CV_PERCENT = 8.0;
static const double MIN_REPORT_AMPLITUDE_MS2 = 0.03;

// Phase readiness is separate from amplitude validity.
static const double PHASE_STRONG_MIN_SYNC = 0.80;
static const double PHASE_STRONG_MAX_RECORD_SD_DEG = 8.0;
static const double PHASE_MODERATE_MIN_SYNC = 0.65;

// ============================================================================
// SETTLING / STABLE-POINT SETTINGS
// ============================================================================

static const uint16_t SETTLE_BASE_MS = 400;
static const uint16_t SETTLE_PER_DAC_MS = 45;
static const uint16_t SETTLE_MAX_MS = 1400;
static const double SETTLE_MIN_CYCLES = 25.0;

// Up to 3 canonical records at the SAME DAC. The closest pair is accepted only
// if they agree sufficiently, allowing one transient record to be ignored.
static const uint8_t STABLE_POINT_MAX_RECORDS = 3;
static const double STABLE_PAIR_MAX_DRIFT_PERCENT = 6.0;

// ============================================================================
// HYBRID ACQUISITION SETTINGS
// ============================================================================

static const double COARSE_DAC_STEP = 2.0;
// V5 two-band logic:
//   PREFERRED: the target we actively try to reach during acquisition.
//   ACCEPTED : a safe fallback that is retained while one bounded interpolation
//              refinement is attempted. The fallback is restored if refinement
//              does not produce a genuinely better measured point.
static const double PREFERRED_BAND_PERCENT = 2.5;
static const double ACCEPTED_BAND_PERCENT = 5.0;
static const double REFINEMENT_IMPROVEMENT_EPS_PERCENT = 0.10;
static const uint8_t MAX_REFINE_ITERATIONS = 12;
static const uint8_t MAX_ACQUISITION_RESTARTS = 2;
static const uint32_t BRACKET_MAX_AGE_MS = 18000;
static const double SECANT_GUARD_FRACTION = 0.20;

// V5.1R local re-bracket recovery. If a previously valid bracket moves because
// the plant changes, do NOT immediately restart the whole DAC 3...29 scan.
// Re-measure near the old root and move only in the direction required to
// recover a fresh sign crossing. A full V5.1 restart remains the fallback.
static const double LOCAL_REBRACKET_STEP_DAC = 0.50;
static const double LOCAL_REBRACKET_RADIUS_DAC = 3.00;
static const uint8_t LOCAL_REBRACKET_MAX_STEPS = 6;

// Common-path / transient detection.
static const double SAME_DAC_ABRUPT_PERCENT = 20.0; // retained as a hard fallback guard
static const double COMMON_PATH_COLLAPSE_PERCENT = 35.0;

// V5.1 final-candidate confirmation. Keep this deliberately small: three
// canonical records, accepting at least two clean ones.
static const uint8_t CANDIDATE_CONFIRM_RECORDS = 3;
static const uint8_t CANDIDATE_CONFIRM_MIN_VALID = 2;
static const double CANDIDATE_CONFIRM_MAX_CV_PERCENT = 5.0;
static const double CANDIDATE_CONFIRM_MAX_SPREAD_PERCENT = 10.0;

// V5.1 adaptive same-DAC plant-change guard. Normal stable data in the new
// validation sets is usually ~1-3% variable; deliberate posture changes caused
// persistent ~10-17% shifts. The threshold therefore adapts to the confirmed
// baseline CV but is never made hypersensitive.
static const double PLANT_CHANGE_MIN_PERCENT = 8.0;
static const double PLANT_CHANGE_MAX_PERCENT = 15.0;
static const double PLANT_CHANGE_CV_GAIN = 3.0;
static const double PLANT_CHANGE_CV_OFFSET_PERCENT = 3.0;
static const double PLANT_VECTOR_CHANGE_PERCENT = 12.0;

// V5.1 verified correction: one step is only kept after two clean post-step
// records show a real improvement.
static const uint8_t TRIAL_VERIFY_RECORDS = 2;
static const double TRIAL_REQUIRED_IMPROVEMENT_PERCENT = 0.25;

// Warm-start is only a search-location hint. It is never trusted without a
// fresh same-DAC measurement and is used only for nearly identical conditions.
static const double WARM_START_MAX_FREQUENCY_DELTA_HZ = 1.0;
static const double WARM_START_MAX_TARGET_DELTA_PERCENT = 3.0;

// ============================================================================
// SLOW SERVO SETTINGS
// ============================================================================

static const double SERVO_DEADBAND_PERCENT = ACCEPTED_BAND_PERCENT; // hold stable accepted solutions
static const double SERVO_LOCK_BAND_PERCENT = ACCEPTED_BAND_PERCENT;
static const uint8_t SERVO_LOCK_CONSECUTIVE_RECORDS = 2;
static const uint8_t SERVO_ACQUIRE_MAX_RECORDS = 16;
static const uint8_t SERVO_ERROR_PERSISTENCE_RECORDS = 3; // modest errors must persist before correction
static const uint8_t SERVO_SEVERE_PERSISTENCE_RECORDS = 2;
static const double SERVO_QUANTIZATION_GUARD_MARGIN_PERCENT = 0.50;

// Conservative PI in amplitude space; divided by local slope to obtain DAC step.
static const double SERVO_KP = 0.45;
static const double SERVO_KI = 0.04;
static const double INTEGRAL_LIMIT_TARGET_FRACTION = 0.50;

static const double SERVO_NORMAL_MAX_STEP_DAC = 0.25;
static const double SERVO_SEVERE_MAX_STEP_DAC = 0.50;
static const double SERVO_SEVERE_ERROR_PERCENT = 10.0;

static const double MIN_VALID_SLOPE = 0.005;  // m/s^2 per DAC
static const double MAX_VALID_SLOPE = 10.0;
static const uint8_t MAX_SLOPE_HISTORY = 5;
static const uint8_t MAX_CONSECUTIVE_INVALID_SERVO_RECORDS = 3;
static const uint8_t MAX_UNPRODUCTIVE_CORRECTIONS = 2;
static const uint8_t MAX_RUNTIME_REACQUIRES = 2;

// ============================================================================
// CLOSED-LOOP HOLD / THESIS ACCEPTANCE
// ============================================================================

static const double HOLD_TIGHT_BAND_PERCENT = PREFERRED_BAND_PERCENT; // preferred reported band
static const double HOLD_FINAL_BAND_PERCENT = ACCEPTED_BAND_PERCENT; // primary accepted/pass band
static const double HOLD_MAX_CV_PERCENT = 5.0;
static const double HOLD_MAX_RMS_ERROR_PERCENT = 5.0; // primary time-domain tracking quality
// +/-5% in-band percentage is reported, not used as a binary gate.
static const uint8_t HOLD_MIN_VALID_RECORDS = 3;

// Stronger readiness gate for the next phase-sweep experiment. This does not
// alter the existing amplitude PASS definition.
static const double PHASE_READY_MIN_IN_BAND_PERCENT = 75.0;
static const double PHASE_READY_MAX_CV_PERCENT = 3.0;
static const double PHASE_READY_MAX_RMS_ERROR_PERCENT = 4.0;
static const uint16_t PHASE_READY_MAX_CORRECTIONS = 1;

// Phase classification from the closed-loop hold.
static const double HOLD_STRONG_SYNC = 0.80;
static const double HOLD_STRONG_PHASE_SPAN_DEG = 5.0;
static const double HOLD_MODERATE_SYNC = 0.65;
static const double HOLD_MODERATE_PHASE_SPAN_DEG = 10.0;

// ============================================================================
// FFT SETTINGS
// ============================================================================

static const uint16_t FFT_SAMPLES = 1024;
static const uint16_t HALF_BINS = FFT_SAMPLES / 2;
static const uint8_t FFT_FRAMES = 3;
static const double HAMMING_COHERENT_GAIN = 0.54;
static const double LOCAL_SEARCH_HALF_WIDTH_HZ = 8.0;
static const double FFT_MIN_AMPLITUDE_MS2 = 0.03;
static const double FFT_MIN_SNR_DB = 3.0;
static const uint8_t FFT_PEAK_EXCLUSION_BINS = 3;

// ============================================================================
// GLOBAL / STATIC MEMORY - LARGE BUFFERS NEVER LIVE ON TASK STACK
// ============================================================================

int16_t rawX[FFT_SAMPLES];
int16_t rawY[FFT_SAMPLES];
int16_t rawZ[FFT_SAMPLES];
uint32_t phaseReference[FFT_SAMPLES];

double fftReal[FFT_SAMPLES];
double fftImag[FFT_SAMPLES];
double spectrumX[HALF_BINS];
double spectrumY[HALF_BINS];
double spectrumZ[HALF_BINS];
ArduinoFFT<double> FFT(fftReal, fftImag, FFT_SAMPLES, SAMPLE_RATE_HZ);

double slopeHistory[MAX_SLOPE_HISTORY];
uint8_t slopeCount = 0;

// User settings/state.
double testFrequencyHz = 268.0;
double targetAmplitudeMs2 = 1.00;
ControlAxis controlAxis = ControlAxis::Z_AXIS;
uint8_t holdSeconds = 8;
uint8_t independentRuns = 5;

bool automaticTestRunning = false;
bool emergencyStopRequested = false;
bool targetAcquired = false;
StablePoint lockedPoint = {};
StablePoint lastBracketLow = {};
StablePoint lastBracketHigh = {};

// V5.1 state. Warm memory is deliberately tiny: one last proven operating
// point, not a DAC->amplitude calibration curve.
WarmStartMemory warmStartMemory = {};
PlantBaseline lockedPlantBaseline = {};

// Fixed serial command buffer avoids String heap fragmentation.
static const uint8_t COMMAND_BUFFER_SIZE = 96;
char commandBuffer[COMMAND_BUFFER_SIZE];
uint8_t commandLength = 0;

// Automatic tests run inside a blocking command. Keep a separate tiny line
// parser for emergency-stop input so an arbitrary character 's' embedded in
// unrelated serial text cannot stop the actuator. The intended command is a
// standalone line: s + Newline (or S + Newline).
static const uint8_t EMERGENCY_BUFFER_SIZE = 12;
char emergencyCommandBuffer[EMERGENCY_BUFFER_SIZE];
uint8_t emergencyCommandLength = 0;

// ============================================================================
// INTEGRATED NARROWBAND DISCOVERY SETTINGS / STATIC MEMORY
// ============================================================================

static const double DISC_SEARCH_MIN_HZ = 240.0;
static const double DISC_SEARCH_MAX_HZ = 320.0;
static const uint8_t DISC_TOP_CANDIDATES = 5;
static const uint8_t DISC_PEAK_EXCLUSION_BINS = 3;
static const double DISC_MIN_FFT_SNR_DB = 8.0;
static const double DISC_MIN_SIGNAL_AMPLITUDE_MS2 = 0.03;
static const uint16_t DISC_QUAD_BLOCK_N = 256;
static const uint8_t DISC_QUAD_BLOCKS = 10;
static const uint16_t DISC_QUAD_N = DISC_QUAD_BLOCK_N * DISC_QUAD_BLOCKS;
static const double DISC_FFT_RESOLUTION_HZ = SAMPLE_RATE_HZ / static_cast<double>(FFT_SAMPLES);
static const double DISC_QUAD_MAX_CORRECTION_HZ = DISC_FFT_RESOLUTION_HZ;
static const double DISC_QUAD_MAX_PHASE_RMSE_DEG = 35.0;

// --------------------------------------------------------------------------
// V1.3A DRIFT-TOLERANT ASCII-SAFE QUALITY GATES
// --------------------------------------------------------------------------
// The old V1.2 auto-lock required whole-record weighted SyncRatio >= 0.50.
// Real hand-held records repeatedly showed one obvious 268..273 Hz family but
// whole-record ratios around 0.3..0.4 because the tool frequency/phase wanders
// during the ~1.6 s quadrature record.  V1.3 does NOT simply lower that old
// threshold.  Instead it separates local periodicity from long-record drift:
//   * local block sync = periodicity over each ~160 ms block
//   * phase-fit RMSE    = orderly long-record phase/frequency evolution
//   * FFT consensus     = independent axes/sensors agreeing on the same family
//   * candidate ratio   = protection against genuinely competing frequencies
static const double DISC_LOCAL_GOOD_BLOCK_SYNC = 0.20;
static const double DISC_LOCAL_AXIS_MIN_MEDIAN_SYNC = 0.25;
static const double DISC_SUPPORT_AXIS_MIN_LOCAL_SYNC = 0.30;
static const uint8_t DISC_LOCAL_MIN_GOOD_BLOCKS = 5;       // >= 50% of 10 blocks
static const double DISC_AUTO_LOCK_MIN_WEIGHTED_LOCAL_SYNC = 0.30;
static const double DISC_AUTO_LOCK_SCORE_RATIO = 1.35;
static const uint8_t DISC_AUTO_LOCK_MIN_SUPPORT_AXES = 1;

// FFT interpolation is finer than a raw bin, but a tolerance of two raw FFT
// bins is intentionally used so natural short-term tool drift does not destroy
// cross-axis consensus.  It is still far narrower than the 240..320 Hz band.
static const double DISC_FREQ_CONSENSUS_TOL_HZ = 2.0 * DISC_FFT_RESOLUTION_HZ;
static const uint8_t DISC_MIN_TOOL_FREQ_CONSENSUS_AXES = 1;
static const uint8_t DISC_MIN_TOTAL_FREQ_CONSENSUS_AXES = 3;
static const uint32_t DISC_LATE_SAMPLE_WARNING_US = 10;
static const uint16_t DISC_MAX_ZERO_TRIPLETS_PER_RECORD = 20;
static const uint8_t DISC_MAX_FRAME_RETRIES = 2;

// First 1024 entries are reused for FFT frames; all 2560 entries are reused for
// the uninterrupted quadrature record. Large arrays remain global/static.
int16_t discToolX[DISC_QUAD_N];
int16_t discToolY[DISC_QUAD_N];
int16_t discToolZ[DISC_QUAD_N];
int16_t discHandX[DISC_QUAD_N];
int16_t discHandY[DISC_QUAD_N];
int16_t discHandZ[DISC_QUAD_N];

// MEMORY REUSE (V1.3): Stage A discovery and Stage B matcher never need
// their tool spectra at the same time. Reuse the matcher's three spectrum
// buffers instead of reserving another 12,288 bytes of DRAM.
#define discToolSpecX spectrumX
#define discToolSpecY spectrumY
#define discToolSpecZ spectrumZ

// ADXL2/hand discovery spectra must coexist with the tool spectra while
// Stage A is being analysed, so these remain dedicated.
double discHandSpecX[HALF_BINS];
double discHandSpecY[HALF_BINS];
double discHandSpecZ[HALF_BINS];

// MEMORY REUSE (V1.3): after the three FFT frames are accumulated, fftReal
// is no longer needed as FFT work memory until a later, separate stage. Use
// its first HALF_BINS entries as the tool vector spectrum. This saves another
// 4,096 bytes. A hand vector spectrum was never consumed by the algorithm,
// so the unused 4,096-byte buffer has been removed entirely.
#define discToolVectorSpectrum fftReal

DiscoveryCandidate discoveryCandidates[DISC_TOP_CANDIDATES];
uint8_t discoveryCandidateCount = 0;
DiscoverySensorFFTResult discoveryToolFFT = {};
DiscoverySensorFFTResult discoveryHandFFT = {};
DiscoverySensorAtF0 baselineToolAtF0 = {};
DiscoverySensorAtF0 baselineHandAtF0 = {};

double baselineF0Hz = 0.0;
bool baselineValid = false;
bool baselineAxisSelected = false;
IntegratedWorkflowState integratedWorkflowState = IntegratedWorkflowState::NEED_BASELINE;
double frozenMatchDac = 0.0;
double frozenMatchMeasuredMs2 = 0.0;

// ============================================================================
// BASIC HELPERS
// ============================================================================

double clampDouble(double value, double low, double high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

double quantizeDac(double value) {
  value = clampDouble(value, DAC_MIN, DAC_MAX);
  value = round(value / DAC_QUANTUM) * DAC_QUANTUM;
  return clampDouble(value, DAC_MIN, DAC_MAX);
}

double wrapDegrees(double value) {
  while (value >= 180.0) value -= 360.0;
  while (value < -180.0) value += 360.0;
  return value;
}

double percentDifference(double a, double b) {
  double denom = 0.5 * (fabs(a) + fabs(b));
  if (denom < 1.0e-12) return 999.0;
  return 100.0 * fabs(a - b) / denom;
}

double targetErrorPercent(double measured) {
  if (targetAmplitudeMs2 < 1.0e-12) return 999.0;
  return 100.0 * (measured - targetAmplitudeMs2) / targetAmplitudeMs2;
}

bool inTargetBand(double measured, double percent) {
  double f = percent / 100.0;
  return measured >= targetAmplitudeMs2 * (1.0 - f) &&
         measured <= targetAmplitudeMs2 * (1.0 + f);
}

uint8_t axisIndex() {
  return static_cast<uint8_t>(controlAxis);
}

char axisChar() {
  if (controlAxis == ControlAxis::X_AXIS) return 'X';
  if (controlAxis == ControlAxis::Y_AXIS) return 'Y';
  return 'Z';
}

char dominantAxis(double x, double y, double z) {
  if (x >= y && x >= z) return 'X';
  if (y >= x && y >= z) return 'Y';
  return 'Z';
}

void sortSmall(double *values, uint8_t count) {
  for (uint8_t i = 1; i < count; i++) {
    double key = values[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = key;
  }
}

double medianSmall(const double *source, uint8_t count) {
  if (count == 0) return 0.0;
  double work[8];
  if (count > 8) count = 8;
  for (uint8_t i = 0; i < count; i++) work[i] = source[i];
  sortSmall(work, count);
  if (count & 1U) return work[count / 2];
  return 0.5 * (work[count / 2 - 1] + work[count / 2]);
}

double meanSmall(const double *values, uint8_t count) {
  if (count == 0) return 0.0;
  double sum = 0.0;
  for (uint8_t i = 0; i < count; i++) sum += values[i];
  return sum / static_cast<double>(count);
}

double sampleSdSmall(const double *values, uint8_t count, double mean) {
  if (count < 2) return 0.0;
  double ss = 0.0;
  for (uint8_t i = 0; i < count; i++) {
    double d = values[i] - mean;
    ss += d * d;
  }
  return sqrt(ss / static_cast<double>(count - 1));
}

double circularMeanDeg(const double *angles, uint8_t count) {
  if (count == 0) return 0.0;
  double c = 0.0, s = 0.0;
  for (uint8_t i = 0; i < count; i++) {
    double r = angles[i] * PI / 180.0;
    c += cos(r);
    s += sin(r);
  }
  return wrapDegrees(atan2(s, c) * 180.0 / PI);
}

double circularSdDeg(const double *angles, uint8_t count, double meanDeg) {
  if (count < 2) return 0.0;
  double diff[8];
  for (uint8_t i = 0; i < count; i++) diff[i] = wrapDegrees(angles[i] - meanDeg);
  double m = meanSmall(diff, count);
  return sampleSdSmall(diff, count, m);
}

void resetRunningStats(RunningStats &s) {
  s.n = 0;
  s.mean = 0.0;
  s.m2 = 0.0;
  s.minValue = 1.0e99;
  s.maxValue = -1.0e99;
}

void pushRunningStats(RunningStats &s, double value) {
  s.n++;
  double delta = value - s.mean;
  s.mean += delta / static_cast<double>(s.n);
  double delta2 = value - s.mean;
  s.m2 += delta * delta2;
  if (value < s.minValue) s.minValue = value;
  if (value > s.maxValue) s.maxValue = value;
}

double runningSd(const RunningStats &s) {
  if (s.n < 2) return 0.0;
  return sqrt(s.m2 / static_cast<double>(s.n - 1));
}

void clearSlopeHistory() {
  slopeCount = 0;
  for (uint8_t i = 0; i < MAX_SLOPE_HISTORY; i++) slopeHistory[i] = 0.0;
}

void pushSlope(double slope) {
  if (!isfinite(slope) || slope < MIN_VALID_SLOPE || slope > MAX_VALID_SLOPE) return;
  if (slopeCount < MAX_SLOPE_HISTORY) {
    slopeHistory[slopeCount++] = slope;
  } else {
    for (uint8_t i = 1; i < MAX_SLOPE_HISTORY; i++) slopeHistory[i - 1] = slopeHistory[i];
    slopeHistory[MAX_SLOPE_HISTORY - 1] = slope;
  }
}

double medianSlope() {
  if (slopeCount == 0) return 0.0;
  return medianSmall(slopeHistory, slopeCount);
}

void clearTrialState(ServoState &s) {
  s.trialPending = false;
  s.trialValidCount = 0;
  s.trialAmp[0] = s.trialAmp[1] = 0.0;
  for (uint8_t a = 0; a < 3; a++) s.trialAxisSum[a] = 0.0;
  s.trialTotalSum = 0.0;
  s.trialMaxCvPercent = 0.0;
}

bool warmStartUsable() {
  if (!warmStartMemory.valid) return false;
  if (warmStartMemory.axis != controlAxis) return false;
  if (fabs(warmStartMemory.frequencyHz - testFrequencyHz) >
      WARM_START_MAX_FREQUENCY_DELTA_HZ) return false;
  double targetDeltaPct = 100.0 * fabs(warmStartMemory.targetMs2 - targetAmplitudeMs2) /
                          fmax(targetAmplitudeMs2, 1.0e-12);
  return targetDeltaPct <= WARM_START_MAX_TARGET_DELTA_PERCENT;
}

void rememberWarmStart(double dac) {
  warmStartMemory.valid = true;
  warmStartMemory.axis = controlAxis;
  warmStartMemory.frequencyHz = testFrequencyHz;
  warmStartMemory.targetMs2 = targetAmplitudeMs2;
  warmStartMemory.dac = quantizeDac(dac);
}

void setPlantBaselineFromRecords(PlantBaseline &b, const ControlRecord recs[], uint8_t count) {
  b = {};
  if (count == 0) return;
  double amps[3];
  double mean = 0.0;
  for (uint8_t i = 0; i < count; i++) {
    amps[i] = recs[i].selectedMedianMs2;
    mean += amps[i];
    for (uint8_t a = 0; a < 3; a++) b.axisMeanMs2[a] += recs[i].axisMeanMs2[a];
    b.totalMeanMs2 += recs[i].totalMeanMs2;
  }
  mean /= count;
  for (uint8_t a = 0; a < 3; a++) b.axisMeanMs2[a] /= count;
  b.totalMeanMs2 /= count;
  b.valid = true;
  b.dac = recs[count - 1].dac;
  b.selectedMs2 = mean;
  b.selectedCvPercent = count >= 2 && mean > 1.0e-12
      ? 100.0 * sampleSdSmall(amps, count, mean) / mean : 0.0;
}

void setPlantBaselineFromTrialAverages(PlantBaseline &b, const ServoState &s, double dac) {
  b = {};
  if (s.trialValidCount == 0) return;
  b.valid = true;
  b.dac = dac;
  b.selectedMs2 = (s.trialAmp[0] + s.trialAmp[1]) / s.trialValidCount;
  for (uint8_t a = 0; a < 3; a++) b.axisMeanMs2[a] = s.trialAxisSum[a] / s.trialValidCount;
  b.totalMeanMs2 = s.trialTotalSum / s.trialValidCount;
  double drift = s.trialValidCount >= 2 ? percentDifference(s.trialAmp[0], s.trialAmp[1]) : 0.0;
  b.selectedCvPercent = drift / sqrt(2.0); // two-point CV proxy, conservative enough for guarding
}

double plantChangeThresholdPercent(const PlantBaseline &b) {
  double adaptive = PLANT_CHANGE_CV_GAIN * b.selectedCvPercent +
                    PLANT_CHANGE_CV_OFFSET_PERCENT;
  return clampDouble(adaptive, PLANT_CHANGE_MIN_PERCENT, PLANT_CHANGE_MAX_PERCENT);
}

bool plantStateMismatch(const PlantBaseline &b, const ControlRecord &r, bool printDiag) {
  if (!b.valid || !r.amplitudeValid) return false;
  if (fabs(b.dac - r.dac) > 1.0e-9) return false;

  double selectedChange = percentDifference(b.selectedMs2, r.selectedMedianMs2);
  double threshold = plantChangeThresholdPercent(b);
  double totalChange = percentDifference(b.totalMeanMs2, r.totalMeanMs2);
  uint8_t changedAxes = 0;
  for (uint8_t a = 0; a < 3; a++) {
    if (b.axisMeanMs2[a] < 0.03 && r.axisMeanMs2[a] < 0.03) continue;
    if (percentDifference(b.axisMeanMs2[a], r.axisMeanMs2[a]) >=
        PLANT_VECTOR_CHANGE_PERCENT) changedAxes++;
  }

  bool vectorEvidence = totalChange >= PLANT_VECTOR_CHANGE_PERCENT || changedAxes >= 2;
  bool strongSelectedEvidence = selectedChange >= threshold + 4.0;
  bool mismatch = selectedChange >= threshold && (vectorEvidence || strongSelectedEvidence);

  if (printDiag && selectedChange >= threshold) {
    Serial.print("PLANT_STATE_CHECK | selected change "); Serial.print(selectedChange, 2);
    Serial.print(" % | adaptive threshold "); Serial.print(threshold, 2);
    Serial.print(" % | vector change "); Serial.print(totalChange, 2);
    Serial.print(" % | changed axes "); Serial.println(changedAxes);
  }
  return mismatch;
}

const char *phaseClassText(PhaseClass p) {
  if (p == PhaseClass::STRONG) return "STRONG";
  if (p == PhaseClass::MODERATE) return "MODERATE";
  return "WEAK";
}

// ============================================================================
// ADXL345 SPI
// ============================================================================

void writeRegister(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(adxlSpiSettings);
  digitalWrite(PIN_CS_ADXL2, LOW);
  SPI.transfer(reg & 0x3F);
  SPI.transfer(value);
  digitalWrite(PIN_CS_ADXL2, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(uint8_t reg) {
  SPI.beginTransaction(adxlSpiSettings);
  digitalWrite(PIN_CS_ADXL2, LOW);
  SPI.transfer(SPI_READ_BIT | (reg & 0x3F));
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(PIN_CS_ADXL2, HIGH);
  SPI.endTransaction();
  return value;
}

RawAcceleration readRawXYZ() {
  RawAcceleration r = {};
  SPI.beginTransaction(adxlSpiSettings);
  digitalWrite(PIN_CS_ADXL2, LOW);
  SPI.transfer(SPI_READ_BIT | SPI_MB_BIT | (REG_DATAX0 & 0x3F));
  uint8_t x0 = SPI.transfer(0), x1 = SPI.transfer(0);
  uint8_t y0 = SPI.transfer(0), y1 = SPI.transfer(0);
  uint8_t z0 = SPI.transfer(0), z1 = SPI.transfer(0);
  digitalWrite(PIN_CS_ADXL2, HIGH);
  SPI.endTransaction();
  r.x = static_cast<int16_t>((static_cast<uint16_t>(x1) << 8) | x0);
  r.y = static_cast<int16_t>((static_cast<uint16_t>(y1) << 8) | y0);
  r.z = static_cast<int16_t>((static_cast<uint16_t>(z1) << 8) | z0);
  return r;
}

bool verifyAdxl2(bool printDetails) {
  uint8_t devid = readRegister(REG_DEVID);
  uint8_t format = readRegister(REG_DATA_FORMAT);
  uint8_t rate = readRegister(REG_BW_RATE);
  uint8_t power = readRegister(REG_POWER_CTL);

  bool ok = devid == ADXL_DEVID_OK &&
            ((format & 0x0F) == 0x0B) &&
            ((rate & 0x1F) == 0x0E) &&
            ((power & 0x08) != 0);

  if (printDetails || !ok) {
    Serial.print("ADXL2 DEVID=0x"); Serial.print(devid, HEX);
    Serial.print(" DATA_FORMAT=0x"); Serial.print(format, HEX);
    Serial.print(" BW_RATE=0x"); Serial.print(rate, HEX);
    Serial.print(" POWER_CTL=0x"); Serial.println(power, HEX);
  }
  return ok;
}

bool initializeAdxl2() {
  writeRegister(REG_POWER_CTL, 0x00);
  delay(10);
  if (readRegister(REG_DEVID) != ADXL_DEVID_OK) return false;
  writeRegister(REG_DATA_FORMAT, 0x0B); // FULL_RES, +/-16 g, 4-wire SPI
  writeRegister(REG_BW_RATE, 0x0E);     // 1600 Hz ODR
  writeRegister(REG_POWER_CTL, 0x08);   // measurement mode
  delay(30);
  return verifyAdxl2(true);
}


// Generic shared-SPI helpers used only by the integrated dual-ADXL discovery.
// The frozen V5.1R matcher continues to use its original ADXL2-only functions.
void deselectBothAdxl() {
  digitalWrite(PIN_CS_ADXL1, HIGH);
  digitalWrite(PIN_CS_ADXL2, HIGH);
}

void dualWriteRegister(uint8_t csPin, uint8_t reg, uint8_t value) {
  SPI.beginTransaction(adxlSpiSettings);
  deselectBothAdxl();
  digitalWrite(csPin, LOW);
  SPI.transfer(reg & 0x3F);
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

uint8_t dualReadRegister(uint8_t csPin, uint8_t reg) {
  SPI.beginTransaction(adxlSpiSettings);
  deselectBothAdxl();
  digitalWrite(csPin, LOW);
  SPI.transfer(SPI_READ_BIT | (reg & 0x3F));
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return value;
}

RawAcceleration dualReadRawXYZ(uint8_t csPin) {
  RawAcceleration r = {};
  SPI.beginTransaction(adxlSpiSettings);
  deselectBothAdxl();
  digitalWrite(csPin, LOW);
  SPI.transfer(SPI_READ_BIT | SPI_MB_BIT | (REG_DATAX0 & 0x3F));
  uint8_t x0 = SPI.transfer(0x00), x1 = SPI.transfer(0x00);
  uint8_t y0 = SPI.transfer(0x00), y1 = SPI.transfer(0x00);
  uint8_t z0 = SPI.transfer(0x00), z1 = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  r.x = static_cast<int16_t>((static_cast<uint16_t>(x1) << 8) | x0);
  r.y = static_cast<int16_t>((static_cast<uint16_t>(y1) << 8) | y0);
  r.z = static_cast<int16_t>((static_cast<uint16_t>(z1) << 8) | z0);
  return r;
}

bool verifyReferenceAdxl1(bool printDetails) {
  uint8_t devid = dualReadRegister(PIN_CS_ADXL1, REG_DEVID);
  uint8_t format = dualReadRegister(PIN_CS_ADXL1, REG_DATA_FORMAT);
  uint8_t rate = dualReadRegister(PIN_CS_ADXL1, REG_BW_RATE);
  uint8_t power = dualReadRegister(PIN_CS_ADXL1, REG_POWER_CTL);
  bool ok = devid == ADXL_DEVID_OK &&
            ((format & 0x0F) == 0x0B) &&
            ((rate & 0x1F) == 0x0E) &&
            ((power & 0x08) != 0);
  if (printDetails || !ok) {
    Serial.print("ADXL1 DEVID=0x"); Serial.print(devid, HEX);
    Serial.print(" DATA_FORMAT=0x"); Serial.print(format, HEX);
    Serial.print(" BW_RATE=0x"); Serial.print(rate, HEX);
    Serial.print(" POWER_CTL=0x"); Serial.println(power, HEX);
  }
  return ok;
}

bool initializeReferenceAdxl1() {
  dualWriteRegister(PIN_CS_ADXL1, REG_POWER_CTL, 0x00);
  delay(10);
  if (dualReadRegister(PIN_CS_ADXL1, REG_DEVID) != ADXL_DEVID_OK) return false;
  dualWriteRegister(PIN_CS_ADXL1, REG_DATA_FORMAT, 0x0B);
  dualWriteRegister(PIN_CS_ADXL1, REG_BW_RATE, 0x0E);
  dualWriteRegister(PIN_CS_ADXL1, REG_POWER_CTL, 0x08);
  delay(30);
  return verifyReferenceAdxl1(true);
}

bool verifyBothIntegratedSensorsOrRecover() {
  bool a = verifyReferenceAdxl1(false);
  bool b = verifyAdxl2(false);
  if (a && b) return true;
  Serial.println("Dual-ADXL configuration check failed. Reinitializing both sensors once...");
  a = initializeReferenceAdxl1();
  b = initializeAdxl2();
  return a && b;
}

// ============================================================================
// PHASE-CONTINUOUS HARDWARE-TIMER NCO
// ============================================================================

void buildSineTable(uint8_t bank, double dacAmplitude) {
  dacAmplitude = quantizeDac(dacAmplitude);
  for (uint16_t i = 0; i < 256; i++) {
    double theta = 2.0 * PI * static_cast<double>(i) / 256.0;
    int code = 128 + static_cast<int>(lround(dacAmplitude * sin(theta)));
    code = constrain(code, 0, 255);
    visatonSineTable[bank][i] = static_cast<uint8_t>(code);
  }
}

void ARDUINO_ISR_ATTR visatonTimerISR() {
  if (!visatonRunning) return;

  uint32_t previous = visatonPhaseAccumulator;
  uint32_t next = previous + visatonPhaseIncrement;
  visatonPhaseAccumulator = next;

  // Only switch amplitude at phase wrap, near sine zero.
  if (amplitudeTablePending && next < previous) {
    activeSineTable = pendingSineTable;
    amplitudeTablePending = false;
  }

  uint8_t index = static_cast<uint8_t>(next >> 24);
  dacWrite(PIN_VISATON_DAC, visatonSineTable[activeSineTable][index]);
}

bool initializeVisatonTimer() {
  visatonTimer = timerBegin(TIMER_BASE_FREQUENCY_HZ);
  if (visatonTimer == nullptr) return false;
  timerAttachInterrupt(visatonTimer, &visatonTimerISR);
  timerAlarm(visatonTimer, TIMER_ALARM_TICKS, true, 0);
  return true;
}

double actualNcoFrequencyHz() {
  return static_cast<double>(visatonPhaseIncrement) *
         static_cast<double>(DAC_UPDATE_RATE_HZ) / 4294967296.0;
}

bool startVisatonInitial(double frequencyHz, double dacAmplitude) {
  frequencyHz = clampDouble(frequencyHz, COMMAND_MIN_HZ, COMMAND_MAX_HZ);
  dacAmplitude = quantizeDac(dacAmplitude);

  visatonRunning = false;
  amplitudeTablePending = false;
  activeSineTable = 0;
  pendingSineTable = 1;
  buildSineTable(0, dacAmplitude);
  buildSineTable(1, dacAmplitude);

  visatonPhaseAccumulator = 0;
  visatonPhaseIncrement = static_cast<uint32_t>(
      frequencyHz * 4294967296.0 / static_cast<double>(DAC_UPDATE_RATE_HZ));
  currentCommandFrequencyHz = frequencyHz;
  currentDacAmplitude = dacAmplitude;
  dacWrite(PIN_VISATON_DAC, 128);
  visatonRunning = true;
  return true;
}

bool checkEmergencyStop();

bool waitForPendingAmplitude(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (amplitudeTablePending) {
    if (checkEmergencyStop()) return false;
    if (millis() - start > timeoutMs) return false;
    delay(1);
  }
  return true;
}

bool setVisatonAmplitudeContinuous(double dacAmplitude) {
  dacAmplitude = quantizeDac(dacAmplitude);

  if (!visatonRunning || fabs(currentCommandFrequencyHz - testFrequencyHz) > 1.0e-9) {
    return startVisatonInitial(testFrequencyHz, dacAmplitude);
  }

  if (fabs(dacAmplitude - currentDacAmplitude) < 1.0e-9) return true;

  if (!waitForPendingAmplitude(200)) return false;
  uint8_t inactive = static_cast<uint8_t>(1U - activeSineTable);
  buildSineTable(inactive, dacAmplitude);
  pendingSineTable = inactive;
  amplitudeTablePending = true;

  // At 20 Hz one wrap is <=50 ms; 180 ms is generous.
  if (!waitForPendingAmplitude(180)) return false;
  currentDacAmplitude = dacAmplitude;
  return true;
}

void stopVisaton() {
  visatonRunning = false;
  amplitudeTablePending = false;
  visatonPhaseAccumulator = 0;
  visatonPhaseIncrement = 0;
  currentDacAmplitude = 0.0;
  currentCommandFrequencyHz = 0.0;
  dacWrite(PIN_VISATON_DAC, 128);
}

// ============================================================================
// EMERGENCY STOP / SETTLING
// ============================================================================

bool checkEmergencyStop() {
  if (emergencyStopRequested) return true;
  if (!automaticTestRunning) return false;

  bool stopSeen = false;
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;

    if (c == '\n') {
      emergencyCommandBuffer[emergencyCommandLength] = '\0';

      // Trim surrounding spaces/tabs and require the ENTIRE command to be s/S.
      char *p = emergencyCommandBuffer;
      while (*p == ' ' || *p == '\t') p++;
      char *tail = p + strlen(p);
      while (tail > p && (tail[-1] == ' ' || tail[-1] == '\t')) *--tail = '\0';

      if ((p[0] == 's' || p[0] == 'S') && p[1] == '\0') stopSeen = true;
      emergencyCommandLength = 0;
      emergencyCommandBuffer[0] = '\0';
    } else if (emergencyCommandLength + 1 < EMERGENCY_BUFFER_SIZE) {
      emergencyCommandBuffer[emergencyCommandLength++] = c;
    } else {
      // Overflow means this is not a valid emergency line; discard safely.
      emergencyCommandLength = 0;
      emergencyCommandBuffer[0] = '\0';
    }
  }

  if (stopSeen) {
    emergencyStopRequested = true;
    stopVisaton();
    Serial.println("\nEMERGENCY STOP - VISATON OFF.");
    return true;
  }
  return false;
}

bool interruptibleDelay(uint32_t durationMs) {
  uint32_t start = millis();
  while (millis() - start < durationMs) {
    if (checkEmergencyStop()) return false;
    delay(2);
  }
  return true;
}

uint16_t adaptiveSettleMs(double oldDac, double newDac) {
  double commandPart = SETTLE_BASE_MS + SETTLE_PER_DAC_MS * fabs(newDac - oldDac);
  double cyclePart = SETTLE_MIN_CYCLES * 1000.0 / fmax(testFrequencyHz, 1.0);
  return static_cast<uint16_t>(clampDouble(fmax(commandPart, cyclePart),
                                           SETTLE_BASE_MS, SETTLE_MAX_MS));
}

// ============================================================================
// EXACT-COMMAND-FREQUENCY NCO-REFERENCED I/Q
// ============================================================================

void calculateSyncAxis(const int16_t samples[], const uint32_t phaseRef[],
                       uint16_t sampleCount, AxisSyncResult &result) {
  result = {};
  if (sampleCount < 32) return;

  double meanRaw = 0.0;
  for (uint16_t i = 0; i < sampleCount; i++) meanRaw += samples[i];
  meanRaw /= static_cast<double>(sampleCount);

  double I = 0.0;
  double Q = 0.0;
  double windowSum = 0.0;
  double acSq = 0.0;
  const double PHASE_TO_RAD = 2.0 * PI / 4294967296.0;

  for (uint16_t i = 0; i < sampleCount; i++) {
    double value = (static_cast<double>(samples[i]) - meanRaw) * ADXL_MS2_PER_LSB;
    double w = 0.5 - 0.5 * cos(2.0 * PI * static_cast<double>(i) /
                                static_cast<double>(sampleCount - 1));
    double theta = static_cast<double>(phaseRef[i]) * PHASE_TO_RAD;
    I += value * w * cos(theta);
    Q += value * w * sin(theta);
    windowSum += w;
    acSq += value * value;
  }

  if (windowSum <= 0.0) return;

  result.amplitudeMs2 = 2.0 * sqrt(I * I + Q * Q) / windowSum;

  // Reference convention:
  // generated command = sin(theta)
  // measured = A*sin(theta + phi)
  // therefore I ~ sin(phi), Q ~ cos(phi), so phi = atan2(I,Q).
  result.phaseDeg = wrapDegrees(atan2(I, Q) * 180.0 / PI);

  double acRms = sqrt(acSq / static_cast<double>(sampleCount));
  double fittedRms = result.amplitudeMs2 / sqrt(2.0);
  result.syncRatio = acRms > 1.0e-12 ? fittedRms / acRms : 0.0;
  result.syncRatio = clampDouble(result.syncRatio, 0.0, 1.0);
}

WindowResult acquireWindow(uint16_t sampleCount) {
  WindowResult result = {};
  if (sampleCount < 32 || sampleCount > FFT_SAMPLES) return result;

  if (!verifyAdxl2(false)) {
    result.communicationFault = true;
    return result;
  }

  uint32_t nextSampleUs = micros();
  RawAcceleration previous = {};
  bool havePrevious = false;
  uint16_t stagnantRun = 0;

  for (uint16_t i = 0; i < sampleCount; i++) {
    while (static_cast<int32_t>(micros() - nextSampleUs) < 0) {}
    uint32_t actualUs = micros();
    uint32_t lateness = actualUs - nextSampleUs;
    if (lateness > result.maxLatenessUs) result.maxLatenessUs = lateness;
    if (lateness > LATE_SAMPLE_WARNING_US) result.lateSamples++;
    nextSampleUs += SAMPLE_PERIOD_US;

    phaseReference[i] = visatonPhaseAccumulator;
    RawAcceleration raw = readRawXYZ();
    rawX[i] = raw.x;
    rawY[i] = raw.y;
    rawZ[i] = raw.z;

    if (raw.x == 0 && raw.y == 0 && raw.z == 0) result.zeroTriplets++;

    if (abs(static_cast<int>(raw.x)) >= ADXL_RAW_CLIP_LIMIT ||
        abs(static_cast<int>(raw.y)) >= ADXL_RAW_CLIP_LIMIT ||
        abs(static_cast<int>(raw.z)) >= ADXL_RAW_CLIP_LIMIT) {
      result.rawClipped = true;
    }

    if (havePrevious && raw.x == previous.x && raw.y == previous.y && raw.z == previous.z) {
      stagnantRun++;
      if (stagnantRun > result.longestStagnantRun) result.longestStagnantRun = stagnantRun;
    } else {
      stagnantRun = 0;
    }

    previous = raw;
    havePrevious = true;
  }

  if (!verifyAdxl2(false) ||
      result.zeroTriplets > MAX_ZERO_TRIPLETS ||
      result.longestStagnantRun > MAX_STAGNANT_RUN) {
    result.communicationFault = true;
  }

  result.timingFault = result.lateSamples > MAX_LATE_SAMPLES_PER_WINDOW;

  calculateSyncAxis(rawX, phaseReference, sampleCount, result.axis[0]);
  calculateSyncAxis(rawY, phaseReference, sampleCount, result.axis[1]);
  calculateSyncAxis(rawZ, phaseReference, sampleCount, result.axis[2]);

  result.totalAmplitudeMs2 = sqrt(
      result.axis[0].amplitudeMs2 * result.axis[0].amplitudeMs2 +
      result.axis[1].amplitudeMs2 * result.axis[1].amplitudeMs2 +
      result.axis[2].amplitudeMs2 * result.axis[2].amplitudeMs2);

  result.valid = !result.communicationFault && !result.rawClipped && !result.timingFault;
  return result;
}

ControlRecord measureControlRecord(double dac, bool settleIfChanged) {
  ControlRecord result = {};
  result.measured = true;
  result.dac = quantizeDac(dac);

  if (!verifyAdxl2(false)) {
    result.communicationFault = true;
    return result;
  }

  double oldDac = currentDacAmplitude;
  bool commandChanged = !visatonRunning ||
                        fabs(currentCommandFrequencyHz - testFrequencyHz) > 1.0e-9 ||
                        fabs(currentDacAmplitude - result.dac) > 1.0e-9;

  if (commandChanged) {
    if (!setVisatonAmplitudeContinuous(result.dac)) return result;
    if (settleIfChanged && !interruptibleDelay(adaptiveSettleMs(oldDac, result.dac))) return result;
  }

  WindowResult windows[CONTROL_WINDOWS];
  uint8_t validIndex[CONTROL_WINDOWS];
  double selected[CONTROL_WINDOWS];
  uint8_t validCount = 0;

  for (uint8_t w = 0; w < CONTROL_WINDOWS; w++) {
    windows[w] = acquireWindow(CONTROL_SAMPLES);
    result.windowValid[w] = windows[w].valid;
    result.windowSelectedMs2[w] = windows[w].axis[axisIndex()].amplitudeMs2;

    if (windows[w].communicationFault) result.communicationFault = true;
    if (windows[w].rawClipped) result.rawClipped = true;
    if (windows[w].timingFault) result.timingRejected = true;

    for (uint8_t a = 0; a < 3; a++) {
      if (windows[w].axis[a].amplitudeMs2 > AXIS_SOFTWARE_GUARD_MS2) result.safetyExceeded = true;
    }
    if (windows[w].totalAmplitudeMs2 > TOTAL_SOFTWARE_GUARD_MS2) result.safetyExceeded = true;

    if (result.safetyExceeded) {
      stopVisaton();
      return result;
    }

    if (windows[w].valid) {
      validIndex[validCount] = w;
      selected[validCount] = windows[w].axis[axisIndex()].amplitudeMs2;
      validCount++;
    }

    if (w + 1 < CONTROL_WINDOWS && !interruptibleDelay(WINDOW_GAP_MS)) return result;
  }

  result.validWindows = validCount;
  if (validCount < CONTROL_MIN_INLIERS) return result;

  result.selectedMedianMs2 = medianSmall(selected, validCount);
  if (result.selectedMedianMs2 < MIN_REPORT_AMPLITUDE_MS2) return result;

  double deviation[CONTROL_WINDOWS];
  for (uint8_t i = 0; i < validCount; i++) deviation[i] = fabs(selected[i] - result.selectedMedianMs2);
  double mad = medianSmall(deviation, validCount);
  result.madPercent = 100.0 * 1.4826 * mad / fmax(result.selectedMedianMs2, 1.0e-12);

  double acceptedSelected[CONTROL_WINDOWS];
  double phaseByAxis[3][CONTROL_WINDOWS];
  double sumAxis[3] = {0.0, 0.0, 0.0};
  double sumSync[3] = {0.0, 0.0, 0.0};
  double sumTotal = 0.0;
  uint8_t inliers = 0;

  for (uint8_t i = 0; i < validCount; i++) {
    double diffPct = 100.0 * fabs(selected[i] - result.selectedMedianMs2) /
                     fmax(result.selectedMedianMs2, 1.0e-12);
    if (diffPct > WINDOW_OUTLIER_PERCENT) continue;

    uint8_t idx = validIndex[i];
    acceptedSelected[inliers] = selected[i];
    for (uint8_t a = 0; a < 3; a++) {
      sumAxis[a] += windows[idx].axis[a].amplitudeMs2;
      sumSync[a] += windows[idx].axis[a].syncRatio;
      phaseByAxis[a][inliers] = windows[idx].axis[a].phaseDeg;
    }
    sumTotal += windows[idx].totalAmplitudeMs2;
    inliers++;
  }

  result.inlierCount = inliers;
  if (inliers < CONTROL_MIN_INLIERS) return result;

  result.selectedMeanMs2 = meanSmall(acceptedSelected, inliers);
  result.cvPercent = result.selectedMeanMs2 > 1.0e-12
      ? 100.0 * sampleSdSmall(acceptedSelected, inliers, result.selectedMeanMs2) /
        result.selectedMeanMs2
      : 999.0;

  for (uint8_t a = 0; a < 3; a++) {
    result.axisMeanMs2[a] = sumAxis[a] / inliers;
    result.axisSync[a] = sumSync[a] / inliers;
    result.axisPhaseDeg[a] = circularMeanDeg(phaseByAxis[a], inliers);
  }

  result.totalMeanMs2 = sumTotal / inliers;
  uint8_t selectedAxis = axisIndex();
  result.selectedPhaseSdDeg = circularSdDeg(phaseByAxis[selectedAxis], inliers,
                                            result.axisPhaseDeg[selectedAxis]);
  result.timestampMs = millis();

  // SyncRatio intentionally excluded from amplitude validity.
  result.amplitudeValid = !result.communicationFault && !result.rawClipped &&
                          !result.safetyExceeded &&
                          result.selectedMedianMs2 >= MIN_REPORT_AMPLITUDE_MS2 &&
                          result.madPercent <= RECORD_MAX_MAD_PERCENT &&
                          result.cvPercent <= RECORD_MAX_CV_PERCENT;

  result.phaseReadyStrong = result.amplitudeValid &&
                            result.axisSync[selectedAxis] >= PHASE_STRONG_MIN_SYNC &&
                            result.selectedPhaseSdDeg <= PHASE_STRONG_MAX_RECORD_SD_DEG;
  return result;
}

void printControlRecord(const char *label, uint16_t index, const ControlRecord &r) {
  bool haveAggregate = r.inlierCount >= CONTROL_MIN_INLIERS &&
                       r.selectedMedianMs2 >= MIN_REPORT_AMPLITUDE_MS2;

  Serial.print(label); Serial.print(" #"); Serial.print(index);
  Serial.print(" | DAC "); Serial.print(r.dac, 2);
  Serial.print(" | CTRL "); Serial.print(axisChar()); Serial.print(" ");

  if (haveAggregate) {
    Serial.print(r.selectedMedianMs2, 5);
    Serial.print(" | err "); Serial.print(targetErrorPercent(r.selectedMedianMs2), 2); Serial.print(" %");
    Serial.print(" | X/Y/Z ");
    Serial.print(r.axisMeanMs2[0], 5); Serial.print(" / ");
    Serial.print(r.axisMeanMs2[1], 5); Serial.print(" / ");
    Serial.print(r.axisMeanMs2[2], 5);
    Serial.print(" | CV "); Serial.print(r.cvPercent, 2);
    Serial.print(" | MAD "); Serial.print(r.madPercent, 2);
  } else {
    // A rejected/unavailable estimate is NOT a physical zero.
    Serial.print("N/A | err N/A | X/Y/Z N/A / N/A / N/A | CV N/A | MAD N/A");
  }

  Serial.print(" | inliers "); Serial.print(r.inlierCount);

  if (r.safetyExceeded) Serial.println(" | SAFETY_STOP");
  else if (r.communicationFault) Serial.println(" | SPI/STALE_DATA_FAULT");
  else if (r.rawClipped) Serial.println(" | RAW_CLIPPING");
  else if (!r.amplitudeValid) Serial.println(" | AMPLITUDE_QUALITY_REJECT");
  else if (inTargetBand(r.selectedMedianMs2, PREFERRED_BAND_PERCENT)) Serial.println(" | PREFERRED");
  else if (inTargetBand(r.selectedMedianMs2, ACCEPTED_BAND_PERCENT)) Serial.println(" | ACCEPTED");
  else Serial.println(" | VALID");

  // Only print per-window detail when the canonical record was rejected. This
  // preserves V4's 3x512 estimator while making rejection causes observable.
  if (!r.amplitudeValid) {
    Serial.print("  WINDOW_DIAG ");
    for (uint8_t w = 0; w < CONTROL_WINDOWS; w++) {
      Serial.print("W"); Serial.print(w + 1); Serial.print("=");
      if (r.windowValid[w]) Serial.print(r.windowSelectedMs2[w], 5);
      else Serial.print("N/A");
      if (w + 1 < CONTROL_WINDOWS) Serial.print(" | ");
    }
    Serial.println();
  }
}

// ============================================================================
// SAME-DAC STABLE POINT
// ============================================================================

StablePoint aggregateStablePair(const ControlRecord &a, const ControlRecord &b, double driftPct) {
  StablePoint p = {};
  p.valid = true;
  p.stationary = true;
  p.phaseReadyStrong = a.phaseReadyStrong && b.phaseReadyStrong;
  p.dac = a.dac;
  p.amplitudeMs2 = 0.5 * (a.selectedMedianMs2 + b.selectedMedianMs2);
  p.pairDriftPercent = driftPct;
  p.recordsUsed = 2;
  p.timestampMs = b.timestampMs;
  p.totalMeanMs2 = 0.5 * (a.totalMeanMs2 + b.totalMeanMs2);
  p.selectedPhaseSdDeg = fmax(a.selectedPhaseSdDeg, b.selectedPhaseSdDeg);

  for (uint8_t axis = 0; axis < 3; axis++) {
    p.axisMeanMs2[axis] = 0.5 * (a.axisMeanMs2[axis] + b.axisMeanMs2[axis]);
    p.axisSync[axis] = 0.5 * (a.axisSync[axis] + b.axisSync[axis]);
    double angles[2] = {a.axisPhaseDeg[axis], b.axisPhaseDeg[axis]};
    p.axisPhaseDeg[axis] = circularMeanDeg(angles, 2);
  }
  return p;
}

StablePoint measureStablePoint(double dac, const char *label, uint16_t &recordCounter) {
  StablePoint out = {};
  out.dac = quantizeDac(dac);

  ControlRecord validRecords[STABLE_POINT_MAX_RECORDS];
  uint8_t validCount = 0;

  for (uint8_t attempt = 0; attempt < STABLE_POINT_MAX_RECORDS; attempt++) {
    ControlRecord rec = measureControlRecord(out.dac, true);
    recordCounter++;
    printControlRecord(label, recordCounter, rec);

    if (rec.safetyExceeded) {
      out.safetyExceeded = true;
      return out;
    }
    if (rec.communicationFault || rec.rawClipped) {
      out.hardwareFault = true;
      return out;
    }
    if (!rec.amplitudeValid) continue;

    validRecords[validCount++] = rec;

    // As soon as any pair is sufficiently consistent, accept the closest pair.
    if (validCount >= 2) {
      double bestDrift = 1.0e99;
      int bestI = -1, bestJ = -1;
      for (uint8_t i = 0; i < validCount; i++) {
        for (uint8_t j = i + 1; j < validCount; j++) {
          double d = percentDifference(validRecords[i].selectedMedianMs2,
                                       validRecords[j].selectedMedianMs2);
          if (d < bestDrift) {
            bestDrift = d;
            bestI = i;
            bestJ = j;
          }
        }
      }
      if (bestI >= 0 && bestDrift <= STABLE_PAIR_MAX_DRIFT_PERCENT) {
        return aggregateStablePair(validRecords[bestI], validRecords[bestJ], bestDrift);
      }
    }
  }

  out.stationary = false;
  return out;
}

void printStablePoint(const char *label, const StablePoint &p) {
  Serial.print(label);
  Serial.print(" | DAC "); Serial.print(p.dac, 2);
  Serial.print(" | amp "); Serial.print(p.amplitudeMs2, 5);
  Serial.print(" | pair drift "); Serial.print(p.pairDriftPercent, 2); Serial.print(" %");
  Serial.print(" | Sync "); Serial.print(p.axisSync[axisIndex()], 3);
  if (!p.valid) Serial.println(" | INVALID/NONSTATIONARY");
  else Serial.println(" | STABLE");
}

// ============================================================================
// TRANSIENT / TRANSFER-STATE DETECTORS
// ============================================================================

bool commonPathCollapse(const StablePoint &previous, const StablePoint &current) {
  if (!previous.valid || !current.valid) return false;
  if (current.dac <= previous.dac + 1.0e-9) return false;
  if (previous.totalMeanMs2 < 0.10) return false;

  double threshold = 1.0 - COMMON_PATH_COLLAPSE_PERCENT / 100.0;
  if (current.totalMeanMs2 / previous.totalMeanMs2 > threshold) return false;

  uint8_t collapsedAxes = 0;
  for (uint8_t a = 0; a < 3; a++) {
    if (previous.axisMeanMs2[a] < 0.08) continue;
    if (current.axisMeanMs2[a] / previous.axisMeanMs2[a] < threshold) collapsedAxes++;
  }
  return collapsedAxes >= 2;
}

bool sameDacAbruptChange(const ControlRecord &previous, const ControlRecord &current) {
  if (!previous.amplitudeValid || !current.amplitudeValid) return false;
  if (fabs(previous.dac - current.dac) > 1.0e-9) return false;

  if (percentDifference(previous.selectedMedianMs2, current.selectedMedianMs2) <
      SAME_DAC_ABRUPT_PERCENT) return false;

  double totalChange = percentDifference(previous.totalMeanMs2, current.totalMeanMs2);
  uint8_t changedAxes = 0;
  for (uint8_t a = 0; a < 3; a++) {
    if (percentDifference(previous.axisMeanMs2[a], current.axisMeanMs2[a]) >=
        SAME_DAC_ABRUPT_PERCENT) changedAxes++;
  }
  return totalChange >= SAME_DAC_ABRUPT_PERCENT || changedAxes >= 2;
}

// ============================================================================
// HYBRID BRACKETED ROOT ACQUISITION
// ============================================================================

StablePoint chooseCloserPoint(const StablePoint &a, const StablePoint &b) {
  if (!a.valid) return b;
  if (!b.valid) return a;
  double ea = fabs(targetErrorPercent(a.amplitudeMs2));
  double eb = fabs(targetErrorPercent(b.amplitudeMs2));
  if (ea < eb - 1.0e-9) return a;
  if (eb < ea - 1.0e-9) return b;
  if (a.pairDriftPercent < b.pairDriftPercent - 0.1) return a;
  if (b.pairDriftPercent < a.pairDriftPercent - 0.1) return b;
  return a.dac <= b.dac ? a : b;
}

bool refreshBracket(StablePoint &low, StablePoint &high, uint16_t &recordCounter) {
  Serial.println("\nRefreshing bracket endpoints in the CURRENT plant state...");
  StablePoint lowNew = measureStablePoint(low.dac, "BRACKET_LOW_REFRESH", recordCounter);
  if (!lowNew.valid) return false;
  StablePoint highNew = measureStablePoint(high.dac, "BRACKET_HIGH_REFRESH", recordCounter);
  if (!highNew.valid) return false;

  if (!(lowNew.dac < highNew.dac &&
        lowNew.amplitudeMs2 < targetAmplitudeMs2 &&
        highNew.amplitudeMs2 > targetAmplitudeMs2)) {
    Serial.println("Bracket crossing moved during refresh.");
    return false;
  }

  low = lowNew;
  high = highNew;
  return true;
}

// Arduino 1.8.x preprocessor-safe local re-bracket result codes.
// Use uint8_t constants instead of a mid-file custom enum return type because
// the legacy Arduino prototype generator can emit a function prototype before
// seeing a locally-declared enum type.
static const uint8_t LOCAL_RB_FAILED = 0;
static const uint8_t LOCAL_RB_BRACKET_FOUND = 1;
static const uint8_t LOCAL_RB_CANDIDATE_FOUND = 2;
static const uint8_t LOCAL_RB_HARDWARE_FAULT = 3;
static const uint8_t LOCAL_RB_SAFETY_STOP = 4;
static const uint8_t LOCAL_RB_ABORTED = 5;

// Recover locally after a previously valid bracket stops crossing the target.
// The old bracket is used only to choose a starting DAC; every point used for
// the recovered bracket is freshly measured in the CURRENT plant state.
uint8_t tryLocalRebracket(const StablePoint &oldLow,
                                        const StablePoint &oldHigh,
                                        StablePoint &newLow,
                                        StablePoint &newHigh,
                                        StablePoint &candidate,
                                        uint16_t &recordCounter,
                                        uint16_t &qualityRejects) {
  Serial.println("\n--- LOCAL RE-BRACKET RECOVERY ---");
  Serial.println("Previous bracket no longer crosses the target.");
  Serial.println("Trying the previous local solution region before a full DAC scan.");

  if (checkEmergencyStop()) return LOCAL_RB_ABORTED;

  double seedDac = chooseCloserPoint(oldLow, oldHigh).dac;
  double oldSlope = 0.0;
  if (oldLow.valid && oldHigh.valid && oldHigh.dac > oldLow.dac + 1.0e-9) {
    oldSlope = (oldHigh.amplitudeMs2 - oldLow.amplitudeMs2) /
               (oldHigh.dac - oldLow.dac);
    if (isfinite(oldSlope) && oldSlope > MIN_VALID_SLOPE) {
      double root = oldLow.dac +
                    (targetAmplitudeMs2 - oldLow.amplitudeMs2) / oldSlope;
      if (root >= DAC_MIN && root <= DAC_MAX) seedDac = root;
    }
  }
  seedDac = quantizeDac(seedDac);
  if (seedDac < DAC_MIN) seedDac = DAC_MIN;
  if (seedDac > DAC_MAX) seedDac = DAC_MAX;

  Serial.print("LOCAL RE-BRACKET seed DAC: "); Serial.println(seedDac, 2);
  StablePoint seed = measureStablePoint(seedDac, "LOCAL_REBRACKET", recordCounter);
  if (seed.safetyExceeded) return LOCAL_RB_SAFETY_STOP;
  if (!seed.valid) {
    qualityRejects++;
    if (seed.hardwareFault) return LOCAL_RB_HARDWARE_FAULT;
    Serial.println("Local seed was not stably measurable -> use normal full restart.");
    return LOCAL_RB_FAILED;
  }

  candidate = seed;
  if (inTargetBand(seed.amplitudeMs2, PREFERRED_BAND_PERCENT)) {
    Serial.println("LOCAL RE-BRACKET: fresh seed is already in PREFERRED band.");
    return LOCAL_RB_CANDIDATE_FOUND;
  }

  // Move only in the direction that can restore a sign crossing if the local
  // DAC->amplitude slope remains positive. If that local assumption fails, the
  // bounded search simply gives up and the original full V5.1 restart runs.
  const int8_t direction = (seed.amplitudeMs2 < targetAmplitudeMs2) ? +1 : -1;
  StablePoint previous = seed;
  bool haveAcceptedFallback = inTargetBand(seed.amplitudeMs2, ACCEPTED_BAND_PERCENT);

  for (uint8_t step = 1; step <= LOCAL_REBRACKET_MAX_STEPS; step++) {
    if (checkEmergencyStop()) return LOCAL_RB_ABORTED;

    double offset = direction * LOCAL_REBRACKET_STEP_DAC * step;
    if (fabs(offset) > LOCAL_REBRACKET_RADIUS_DAC + 1.0e-9) break;
    double d = quantizeDac(seedDac + offset);
    if (d < DAC_MIN - 1.0e-9 || d > DAC_MAX + 1.0e-9) break;

    StablePoint p = measureStablePoint(d, "LOCAL_REBRACKET", recordCounter);
    if (p.safetyExceeded) return LOCAL_RB_SAFETY_STOP;
    if (!p.valid) {
      qualityRejects++;
      if (p.hardwareFault) return LOCAL_RB_HARDWARE_FAULT;
      continue;
    }

    candidate = chooseCloserPoint(candidate, p);
    if (inTargetBand(p.amplitudeMs2, ACCEPTED_BAND_PERCENT)) {
      haveAcceptedFallback = true;
    }

    if (inTargetBand(p.amplitudeMs2, PREFERRED_BAND_PERCENT)) {
      candidate = p;
      Serial.println("LOCAL RE-BRACKET: PREFERRED target recovered without a full scan.");
      return LOCAL_RB_CANDIDATE_FOUND;
    }

    if (direction > 0 && previous.amplitudeMs2 < targetAmplitudeMs2 &&
                        p.amplitudeMs2 > targetAmplitudeMs2 &&
                        previous.dac < p.dac) {
      newLow = previous;
      newHigh = p;
      Serial.println("LOCAL RE-BRACKET: fresh local sign bracket recovered.");
      return LOCAL_RB_BRACKET_FOUND;
    }
    if (direction < 0 && p.amplitudeMs2 < targetAmplitudeMs2 &&
                        previous.amplitudeMs2 > targetAmplitudeMs2 &&
                        p.dac < previous.dac) {
      newLow = p;
      newHigh = previous;
      Serial.println("LOCAL RE-BRACKET: fresh local sign bracket recovered.");
      return LOCAL_RB_BRACKET_FOUND;
    }

    previous = p;
  }

  if (haveAcceptedFallback && candidate.valid &&
      inTargetBand(candidate.amplitudeMs2, ACCEPTED_BAND_PERCENT)) {
    Serial.print("LOCAL RE-BRACKET: no crossing found, but retained fresh ACCEPTED candidate DAC ");
    Serial.print(candidate.dac, 2);
    Serial.print(" | err ");
    Serial.print(targetErrorPercent(candidate.amplitudeMs2), 2);
    Serial.println(" %.");
    return LOCAL_RB_CANDIDATE_FOUND;
  }

  Serial.println("LOCAL RE-BRACKET failed inside +/-3 DAC -> use normal full restart.");
  return LOCAL_RB_FAILED;
}

AcquisitionResult acquireTargetOnce() {
  AcquisitionResult out = {};
  out.status = AcquireStatus::NONSTATIONARY;

  Serial.println("\n============================================================");
  Serial.println("STAGE 1 - STABLE AUTHORITY SCAN + V5 HYBRID ROOT REFINEMENT");
  Serial.print("Frequency: "); Serial.print(testFrequencyHz, 5); Serial.println(" Hz");
  Serial.print("Axis: "); Serial.println(axisChar());
  Serial.print("Target: "); Serial.print(targetAmplitudeMs2, 5); Serial.println(" m/s^2 peak");
  Serial.print("Preferred band: +/-"); Serial.print(PREFERRED_BAND_PERCENT, 1); Serial.println(" %");
  Serial.print("Accepted band: +/-"); Serial.print(ACCEPTED_BAND_PERCENT, 1); Serial.println(" %");
  Serial.println("Accepted points are retained as safe fallbacks and interpolation is tried.");
  Serial.println("Every model point requires a stable same-DAC pair.");
  Serial.println("Phase/Sync diagnostics do not gate amplitude acquisition.");
  Serial.println("============================================================");

  StablePoint previous = {};
  bool havePrevious = false;
  StablePoint best = {};
  StablePoint low = {};
  StablePoint high = {};
  bool bracketFound = false;
  bool everBelowTarget = false;
  bool everAboveTarget = false;

  // V5.1 warm start: one fresh measurement at the last proven same-condition
  // solution. If it is not already accepted, abandon the hint immediately and
  // execute the unchanged full V5 authority scan.
  if (warmStartUsable()) {
    Serial.print("WARM START - rechecking previous proven DAC ");
    Serial.println(warmStartMemory.dac, 2);
    StablePoint warm = measureStablePoint(warmStartMemory.dac, "WARM_START", out.recordsMeasured);
    if (warm.valid && inTargetBand(warm.amplitudeMs2, ACCEPTED_BAND_PERCENT)) {
      out.success = true;
      out.status = inTargetBand(warm.amplitudeMs2, PREFERRED_BAND_PERCENT)
          ? AcquireStatus::OK : AcquireStatus::QUANTIZATION_LIMITED;
      out.candidate = warm;
      Serial.println("WARM START ACCEPTED after fresh measurement.");
      return out;
    }
    Serial.println("WARM START not valid for the current plant state -> full scan.");
  }

  for (double d = DAC_MIN; d <= DAC_MAX + 1.0e-9; d += COARSE_DAC_STEP) {
    if (checkEmergencyStop()) {
      out.status = AcquireStatus::ABORTED;
      return out;
    }

    StablePoint p = measureStablePoint(d, "COARSE", out.recordsMeasured);
    if (p.safetyExceeded) {
      out.status = AcquireStatus::SAFETY_STOP;
      return out;
    }
    if (!p.valid) {
      out.qualityRejects++;
      if (p.hardwareFault) {
        out.status = AcquireStatus::HARDWARE_FAULT;
        return out;
      }
      continue;
    }

    if (havePrevious && commonPathCollapse(previous, p)) {
      Serial.println("COMMON-PATH COLLAPSE SUSPECTED - confirming same DAC before model update.");
      StablePoint confirm = measureStablePoint(p.dac, "COLLAPSE_CONFIRM", out.recordsMeasured);
      if (!confirm.valid) {
        out.status = confirm.hardwareFault ? AcquireStatus::HARDWARE_FAULT
                                           : AcquireStatus::NONSTATIONARY;
        return out;
      }
      if (commonPathCollapse(previous, confirm)) {
        Serial.println("Persistent common-path state change confirmed.");
        out.status = AcquireStatus::NONSTATIONARY;
        return out;
      }
      Serial.println("Transient collapse rejected; confirmation accepted.");
      p = confirm;
    }

    if (p.amplitudeMs2 < targetAmplitudeMs2) everBelowTarget = true;
    if (p.amplitudeMs2 > targetAmplitudeMs2) everAboveTarget = true;

    // Preserve useful local slope information even when a coarse point itself
    // lands directly in the preferred band. V5 could otherwise enter Stage 2
    // with a good target but no slope memory.
    if (havePrevious && p.dac > previous.dac) {
      double localSlope = (p.amplitudeMs2 - previous.amplitudeMs2) /
                          (p.dac - previous.dac);
      pushSlope(localSlope);
    }

    StablePoint oldBest = best;
    best = chooseCloserPoint(best, p);
    bool becameBest = !oldBest.valid || fabs(targetErrorPercent(best.amplitudeMs2)) <
                                      fabs(targetErrorPercent(oldBest.amplitudeMs2)) - 1.0e-9;

    if (inTargetBand(p.amplitudeMs2, PREFERRED_BAND_PERCENT)) {
      out.success = true;
      out.status = AcquireStatus::OK;
      out.candidate = p;
      if (havePrevious && previous.dac < p.dac) {
        if (previous.amplitudeMs2 < targetAmplitudeMs2) out.low = previous;
        if (p.amplitudeMs2 > targetAmplitudeMs2) out.high = p;
      }
      return out;
    }

    if (becameBest && inTargetBand(best.amplitudeMs2, ACCEPTED_BAND_PERCENT)) {
      Serial.print("ACCEPTED FALLBACK SAVED: DAC "); Serial.print(best.dac, 2);
      Serial.print(" | amp "); Serial.print(best.amplitudeMs2, 5);
      Serial.print(" | err "); Serial.print(targetErrorPercent(best.amplitudeMs2), 2);
      Serial.println(" % - continuing to obtain a bracket for interpolation.");
    }

    // If the minimum DAC is already above target, no lower interpolation point
    // exists. Accept it only if it is inside the accepted band.
    if (!havePrevious && p.amplitudeMs2 > targetAmplitudeMs2) {
      out.candidate = p;
      if (inTargetBand(p.amplitudeMs2, ACCEPTED_BAND_PERCENT)) {
        out.success = true;
        out.status = AcquireStatus::QUANTIZATION_LIMITED;
      } else if (fabs(p.dac - DAC_MIN) < 1.0e-9) {
        // Only the actual minimum command can prove a true below-range target.
        out.status = AcquireStatus::TARGET_BELOW_RANGE;
      } else {
        Serial.println("Lower DAC points were not stably qualified; range cannot be proven.");
        out.status = AcquireStatus::NONSTATIONARY;
      }
      return out;
    }

    if (havePrevious && previous.amplitudeMs2 < targetAmplitudeMs2 &&
                        p.amplitudeMs2 > targetAmplitudeMs2 &&
                        previous.dac < p.dac) {
      low = previous;
      high = p;
      bracketFound = true;
      break;
    }

    previous = p;
    havePrevious = true;
  }

  if (!bracketFound) {
    out.candidate = best;
    if (best.valid && inTargetBand(best.amplitudeMs2, ACCEPTED_BAND_PERCENT)) {
      out.success = true;
      out.status = AcquireStatus::QUANTIZATION_LIMITED;
    } else if (everBelowTarget && everAboveTarget) {
      // The target was crossed somewhere but no stable increasing bracket
      // survived. Do not falsely call this an authority-limit failure.
      Serial.println("Target crossing observed but response was non-monotonic/nonstationary.");
      out.status = AcquireStatus::NONSTATIONARY;
    } else if (everBelowTarget && !everAboveTarget) {
      out.status = AcquireStatus::TARGET_ABOVE_RANGE;
    } else {
      out.status = AcquireStatus::NONSTATIONARY;
    }
    return out;
  }

  Serial.println("\n--- RAW SIGN BRACKET FOUND ---");
  printStablePoint("LOW", low);
  printStablePoint("HIGH", high);

  {
    StablePoint oldLow = low;
    StablePoint oldHigh = high;
    if (!refreshBracket(low, high, out.recordsMeasured)) {
      StablePoint localCandidate = {};
      uint8_t lr = tryLocalRebracket(oldLow, oldHigh, low, high,
                                                   localCandidate, out.recordsMeasured,
                                                   out.qualityRejects);
      if (lr == LOCAL_RB_CANDIDATE_FOUND) {
        out.candidate = localCandidate;
        out.success = true;
        out.status = inTargetBand(localCandidate.amplitudeMs2, PREFERRED_BAND_PERCENT)
            ? AcquireStatus::OK : AcquireStatus::QUANTIZATION_LIMITED;
        return out;
      }
      if (lr == LOCAL_RB_HARDWARE_FAULT) {
        out.status = AcquireStatus::HARDWARE_FAULT;
        return out;
      }
      if (lr == LOCAL_RB_SAFETY_STOP) {
        out.status = AcquireStatus::SAFETY_STOP;
        return out;
      }
      if (lr == LOCAL_RB_ABORTED) {
        out.status = AcquireStatus::ABORTED;
        return out;
      }
      if (lr != LOCAL_RB_BRACKET_FOUND) {
        out.status = AcquireStatus::NONSTATIONARY;
        return out;
      }
    }
  }

  out.low = low;
  out.high = high;
  best = chooseCloserPoint(low, high); // current-state fallback after refresh
  pushSlope((high.amplitudeMs2 - low.amplitudeMs2) / (high.dac - low.dac));

  for (uint8_t iteration = 0; iteration < MAX_REFINE_ITERATIONS; iteration++) {
    if (checkEmergencyStop()) {
      out.status = AcquireStatus::ABORTED;
      return out;
    }

    if (inTargetBand(best.amplitudeMs2, PREFERRED_BAND_PERCENT)) {
      out.success = true;
      out.status = AcquireStatus::OK;
      out.candidate = best;
      out.low = low;
      out.high = high;
      return out;
    }

    bool acceptedFallback = inTargetBand(best.amplitudeMs2, ACCEPTED_BAND_PERCENT);
    if (acceptedFallback) {
      Serial.print("ACCEPTED FALLBACK ACTIVE: DAC "); Serial.print(best.dac, 2);
      Serial.print(" | err "); Serial.print(targetErrorPercent(best.amplitudeMs2), 2);
      Serial.println(" % - attempting bounded interpolation toward PREFERRED.");
    }

    double width = high.dac - low.dac;
    if (width <= DAC_QUANTUM + 1.0e-9) {
      out.candidate = best;
      out.low = low;
      out.high = high;
      out.success = best.valid && inTargetBand(best.amplitudeMs2, ACCEPTED_BAND_PERCENT);
      out.status = out.success ? AcquireStatus::QUANTIZATION_LIMITED
                               : AcquireStatus::NONSTATIONARY;
      return out;
    }

    uint32_t now = millis();
    if ((now - low.timestampMs) > BRACKET_MAX_AGE_MS ||
        (now - high.timestampMs) > BRACKET_MAX_AGE_MS) {
      StablePoint oldLow = low;
      StablePoint oldHigh = high;
      if (!refreshBracket(low, high, out.recordsMeasured)) {
        StablePoint localCandidate = {};
        uint8_t lr = tryLocalRebracket(oldLow, oldHigh, low, high,
                                                     localCandidate, out.recordsMeasured,
                                                     out.qualityRejects);
        if (lr == LOCAL_RB_CANDIDATE_FOUND) {
          out.candidate = localCandidate;
          out.success = true;
          out.status = inTargetBand(localCandidate.amplitudeMs2, PREFERRED_BAND_PERCENT)
              ? AcquireStatus::OK : AcquireStatus::QUANTIZATION_LIMITED;
          return out;
        }
        if (lr == LOCAL_RB_HARDWARE_FAULT) {
          out.status = AcquireStatus::HARDWARE_FAULT;
          return out;
        }
        if (lr == LOCAL_RB_SAFETY_STOP) {
          out.status = AcquireStatus::SAFETY_STOP;
          return out;
        }
        if (lr == LOCAL_RB_ABORTED) {
          out.status = AcquireStatus::ABORTED;
          return out;
        }
        if (lr != LOCAL_RB_BRACKET_FOUND) {
          out.status = AcquireStatus::NONSTATIONARY;
          return out;
        }
      }
      // A bracket refresh represents the current plant state. Do not prefer a
      // stale old fallback over freshly remeasured endpoints.
      best = chooseCloserPoint(low, high);
      acceptedFallback = inTargetBand(best.amplitudeMs2, ACCEPTED_BAND_PERCENT);
    }

    double slope = (high.amplitudeMs2 - low.amplitudeMs2) / (high.dac - low.dac);
    double predicted = 0.5 * (low.dac + high.dac); // safe bisection default
    const char *refineMode = "BISECTION";

    if (isfinite(slope) && slope > MIN_VALID_SLOPE) {
      double secant = low.dac + (targetAmplitudeMs2 - low.amplitudeMs2) / slope;

      if (acceptedFallback) {
        // V5 requirement: once we already have an accepted fallback, explicitly
        // try the interpolated/root estimate rather than rejecting it merely for
        // lying near a bracket endpoint. Quantization/bounds still protect it.
        if (secant > low.dac && secant < high.dac) {
          predicted = secant;
          refineMode = "ACCEPTED_INTERPOLATION";
        }
      } else {
        double guard = SECANT_GUARD_FRACTION * width;
        if (secant > low.dac + guard && secant < high.dac - guard) {
          predicted = secant;
          refineMode = "SECANT";
        }
      }
    }

    double testDac = quantizeDac(predicted);
    if (testDac <= low.dac + 1.0e-9) testDac = quantizeDac(low.dac + DAC_QUANTUM);
    if (testDac >= high.dac - 1.0e-9) testDac = quantizeDac(high.dac - DAC_QUANTUM);

    Serial.print("\nREFINE "); Serial.print(iteration + 1);
    Serial.print(" | mode "); Serial.print(refineMode);
    Serial.print(" | bracket "); Serial.print(low.dac, 2); Serial.print("(");
    Serial.print(low.amplitudeMs2, 4); Serial.print(") -> ");
    Serial.print(high.dac, 2); Serial.print("("); Serial.print(high.amplitudeMs2, 4);
    Serial.print(") | slope "); Serial.print(slope, 6);
    Serial.print(" | predicted "); Serial.print(predicted, 3);
    Serial.print(" | test DAC "); Serial.println(testDac, 2);

    StablePoint savedBest = best;
    double savedBestAbsErr = fabs(targetErrorPercent(savedBest.amplitudeMs2));

    StablePoint p = measureStablePoint(testDac, "REFINE", out.recordsMeasured);
    if (!p.valid) {
      out.qualityRejects++;
      if (p.hardwareFault) {
        out.status = AcquireStatus::HARDWARE_FAULT;
        return out;
      }
      p = measureStablePoint(testDac, "REFINE_RETRY", out.recordsMeasured);
      if (!p.valid) {
        out.status = p.hardwareFault ? AcquireStatus::HARDWARE_FAULT
                                     : AcquireStatus::NONSTATIONARY;
        return out;
      }
    }

    double pAbsErr = fabs(targetErrorPercent(p.amplitudeMs2));
    if (pAbsErr < savedBestAbsErr - 1.0e-9) best = p;

    if (p.amplitudeMs2 < targetAmplitudeMs2) low = p;
    else high = p;

    if (high.dac > low.dac) {
      double newSlope = (high.amplitudeMs2 - low.amplitudeMs2) / (high.dac - low.dac);
      pushSlope(newSlope);
    }
    out.low = low;
    out.high = high;

    if (inTargetBand(p.amplitudeMs2, PREFERRED_BAND_PERCENT)) {
      best = p;
      Serial.println("REFINEMENT RESULT: PREFERRED band reached; keeping measured refinement.");
      continue; // top of loop performs clean success return
    }

    if (acceptedFallback) {
      if (pAbsErr < savedBestAbsErr - REFINEMENT_IMPROVEMENT_EPS_PERCENT) {
        Serial.print("REFINEMENT RESULT: improved accepted fallback by ");
        Serial.print(savedBestAbsErr - pAbsErr, 2); Serial.println(" percentage points; continuing.");
      } else {
        best = savedBest;
        Serial.println("REFINEMENT RESULT: no meaningful measured improvement.");
        Serial.print("Restoring accepted fallback DAC "); Serial.print(best.dac, 2);
        Serial.print(" | err "); Serial.print(targetErrorPercent(best.amplitudeMs2), 2);
        Serial.println(" % and ending acquisition without hunting.");
        out.candidate = best;
        out.success = true;
        out.status = AcquireStatus::QUANTIZATION_LIMITED;
        return out;
      }
    }
  }

  out.candidate = best;
  out.success = best.valid && inTargetBand(best.amplitudeMs2, ACCEPTED_BAND_PERCENT);
  out.status = out.success ? AcquireStatus::QUANTIZATION_LIMITED : AcquireStatus::NONSTATIONARY;
  return out;
}

AcquisitionResult acquireTargetWithRestart() {
  clearSlopeHistory();
  AcquisitionResult last = {};
  uint32_t acquisitionStartMs = millis();

  for (uint8_t restart = 0; restart <= MAX_ACQUISITION_RESTARTS; restart++) {
    if (restart > 0) {
      Serial.println("\n============================================================");
      Serial.print("ACQUISITION RESTART "); Serial.println(restart);
      Serial.println("Local re-bracket failed or no stable bracket survived; performing full scan.");
      Serial.println("============================================================");
      if (!interruptibleDelay(500)) break;
    }

    last = acquireTargetOnce();
    last.restarts = restart;
    last.elapsedMs = millis() - acquisitionStartMs;
    if (last.success) return last;
    if (last.status != AcquireStatus::NONSTATIONARY) return last;
    if (checkEmergencyStop()) return last;
  }
  last.elapsedMs = millis() - acquisitionStartMs;
  return last;
}

const char *acquireStatusText(AcquireStatus s) {
  switch (s) {
    case AcquireStatus::OK: return "PREFERRED_TARGET_ACQUIRED";
    case AcquireStatus::QUANTIZATION_LIMITED: return "ACCEPTED_QUANTIZATION_LIMITED";
    case AcquireStatus::TARGET_BELOW_RANGE: return "TARGET_BELOW_DAC_RANGE";
    case AcquireStatus::TARGET_ABOVE_RANGE: return "TARGET_ABOVE_DAC_RANGE";
    case AcquireStatus::NONSTATIONARY: return "NONSTATIONARY";
    case AcquireStatus::HARDWARE_FAULT: return "HARDWARE_FAULT";
    case AcquireStatus::SAFETY_STOP: return "SAFETY_STOP";
    default: return "ABORTED";
  }
}

void printAcquisitionSummary(const AcquisitionResult &r) {
  Serial.println("\n============================================================");
  Serial.println("ACQUISITION SUMMARY");
  Serial.print("Status: "); Serial.println(acquireStatusText(r.status));
  Serial.print("Success: "); Serial.println(r.success ? "YES" : "NO");
  Serial.print("Canonical records measured: "); Serial.println(r.recordsMeasured);
  Serial.print("Quality rejects: "); Serial.println(r.qualityRejects);
  Serial.print("Restarts: "); Serial.println(r.restarts);
  Serial.print("Acquisition time: "); Serial.print(r.elapsedMs / 1000.0, 3); Serial.println(" s");
  if (r.candidate.valid) {
    Serial.print("Candidate DAC: "); Serial.println(r.candidate.dac, 2);
    Serial.print("Candidate amplitude: "); Serial.print(r.candidate.amplitudeMs2, 5);
    Serial.print(" | error "); Serial.print(targetErrorPercent(r.candidate.amplitudeMs2), 2); Serial.println(" %");

  }
  Serial.println("============================================================");
}

void acceptAcquisition(const AcquisitionResult &r) {
  targetAcquired = r.success && r.candidate.valid;
  if (!targetAcquired) return;

  lockedPoint = r.candidate;
  lastBracketLow = r.low;
  lastBracketHigh = r.high;

  if (r.low.valid && r.high.valid && r.high.dac > r.low.dac) {
    pushSlope((r.high.amplitudeMs2 - r.low.amplitudeMs2) /
              (r.high.dac - r.low.dac));
  }
}

// ============================================================================
// V5.1 FINAL CANDIDATE CONFIRMATION
// ============================================================================

bool confirmCandidateForServo(AcquisitionResult &acq, ServoState &state) {
  Serial.println("\n============================================================");
  Serial.println("V5.1R CANDIDATE FIXED-DAC CONFIRMATION");
  Serial.println("Three fresh records; no DAC movement while stability is checked.");
  Serial.println("============================================================");

  ControlRecord valid[CANDIDATE_CONFIRM_RECORDS];
  uint8_t count = 0;
  for (uint8_t i = 0; i < CANDIDATE_CONFIRM_RECORDS; i++) {
    ControlRecord r = measureControlRecord(currentDacAmplitude, false);
    printControlRecord("CANDIDATE_CONFIRM", i + 1, r);
    if (r.safetyExceeded || r.communicationFault || r.rawClipped) return false;
    if (r.amplitudeValid && count < CANDIDATE_CONFIRM_RECORDS) valid[count++] = r;
  }

  if (count < CANDIDATE_CONFIRM_MIN_VALID) {
    Serial.println("CANDIDATE_NONSTATIONARY - too few valid confirmation records.");
    return false;
  }

  double amps[3];
  double mean = 0.0;
  double minAmp = 1.0e99, maxAmp = -1.0e99;
  for (uint8_t i = 0; i < count; i++) {
    amps[i] = valid[i].selectedMedianMs2;
    mean += amps[i];
    minAmp = fmin(minAmp, amps[i]);
    maxAmp = fmax(maxAmp, amps[i]);
  }
  mean /= count;
  double cv = count >= 2 ? 100.0 * sampleSdSmall(amps, count, mean) / fmax(mean, 1.0e-12) : 0.0;
  double spread = percentDifference(minAmp, maxAmp);

  Serial.print("Candidate confirmation mean: "); Serial.print(mean, 5);
  Serial.print(" | error "); Serial.print(targetErrorPercent(mean), 2);
  Serial.print(" % | inter-record CV "); Serial.print(cv, 2);
  Serial.print(" % | spread "); Serial.print(spread, 2); Serial.println(" %");

  if (!inTargetBand(mean, ACCEPTED_BAND_PERCENT) ||
      cv > CANDIDATE_CONFIRM_MAX_CV_PERCENT ||
      spread > CANDIDATE_CONFIRM_MAX_SPREAD_PERCENT) {
    Serial.println("CANDIDATE_NONSTATIONARY - do not let PI chase this state.");
    return false;
  }

  setPlantBaselineFromRecords(state.plantBaseline, valid, count);
  lockedPlantBaseline = state.plantBaseline;
  lockedPoint.dac = currentDacAmplitude;
  lockedPoint.amplitudeMs2 = mean;
  lockedPoint.valid = true;
  lockedPoint.stationary = true;
  for (uint8_t a = 0; a < 3; a++) lockedPoint.axisMeanMs2[a] = state.plantBaseline.axisMeanMs2[a];
  lockedPoint.totalMeanMs2 = state.plantBaseline.totalMeanMs2;
  acq.candidate = lockedPoint;

  Serial.println("CANDIDATE CONFIRMED STABLE.");
  return true;
}

// ============================================================================
// SLOW ADAPTIVE PI SERVO
// ============================================================================

void resetServoState(ServoState &s) {
  memset(&s, 0, sizeof(s));
  clearTrialState(s);
}

ServoStatus validateAndPrepareServoRecord(ControlRecord &rec, ServoState &state) {
  if (rec.safetyExceeded) return ServoStatus::SAFETY_STOP;

  if (!rec.amplitudeValid) {
    state.invalidCount++;
    Serial.print("SERVO INVALID #"); Serial.print(state.invalidCount);
    Serial.println(" - no DAC correction; remeasure.");
    if (state.invalidCount >= MAX_CONSECUTIVE_INVALID_SERVO_RECORDS) {
      return rec.communicationFault ? ServoStatus::HARDWARE_FAULT
                                    : ServoStatus::NEEDS_REACQUIRE;
    }
    return ServoStatus::COMPLETED;
  }
  state.invalidCount = 0;

  // V5.1 verified trial logic. A new DAC is not accepted on one record.
  if (state.trialPending) {
    if (fabs(rec.dac - currentDacAmplitude) > 1.0e-9) return ServoStatus::NEEDS_REACQUIRE;

    if (state.trialValidCount < TRIAL_VERIFY_RECORDS) {
      uint8_t i = state.trialValidCount;
      state.trialAmp[i] = rec.selectedMedianMs2;
      for (uint8_t a = 0; a < 3; a++) state.trialAxisSum[a] += rec.axisMeanMs2[a];
      state.trialTotalSum += rec.totalMeanMs2;
      state.trialMaxCvPercent = fmax(state.trialMaxCvPercent, rec.cvPercent);
      state.trialValidCount++;
    }

    if (state.trialValidCount < TRIAL_VERIFY_RECORDS) {
      Serial.println("TRIAL_VERIFY - first post-step record stored; waiting for confirmation.");
      return ServoStatus::COMPLETED;
    }

    double trialAmp = 0.5 * (state.trialAmp[0] + state.trialAmp[1]);
    double trialErr = fabs(targetErrorPercent(trialAmp));
    double oldErr = fabs(state.trialOldErrorPct);
    double trialDrift = percentDifference(state.trialAmp[0], state.trialAmp[1]);
    bool improved = trialErr <= oldErr - TRIAL_REQUIRED_IMPROVEMENT_PERCENT;
    bool stableTrial = trialDrift <= STABLE_PAIR_MAX_DRIFT_PERCENT &&
                       state.trialMaxCvPercent <= RECORD_MAX_CV_PERCENT;

    if (improved && stableTrial) {
      double learned = (trialAmp - state.trialOldAmp) /
                       (currentDacAmplitude - state.trialOldDac);
      if (learned >= MIN_VALID_SLOPE && learned <= MAX_VALID_SLOPE) {
        pushSlope(learned);
        Serial.print("LOCAL_SLOPE_ACCEPTED,dA_dDAC="); Serial.println(learned, 6);
      } else {
        Serial.print("LOCAL_SLOPE_REJECTED,dA_dDAC="); Serial.println(learned, 6);
      }

      setPlantBaselineFromTrialAverages(state.plantBaseline, state, currentDacAmplitude);
      lockedPlantBaseline = state.plantBaseline;
      state.unproductiveCorrections = 0;
      rec.selectedMedianMs2 = trialAmp;
      for (uint8_t a = 0; a < 3; a++) rec.axisMeanMs2[a] = state.plantBaseline.axisMeanMs2[a];
      rec.totalMeanMs2 = state.plantBaseline.totalMeanMs2;
      Serial.print("TRIAL_ACCEPTED | verified error "); Serial.print(trialErr, 2);
      Serial.print(" % vs old "); Serial.print(oldErr, 2); Serial.println(" %");
      clearTrialState(state);
    } else {
      double badDac = currentDacAmplitude;
      double restoreDac = state.trialOldDac;
      PlantBaseline restoreBaseline = state.trialOldBaseline;
      Serial.print("TRIAL_REJECTED | verified error "); Serial.print(trialErr, 2);
      Serial.print(" % vs old "); Serial.print(oldErr, 2);
      Serial.print(" % | reverting DAC "); Serial.print(badDac, 2);
      Serial.print(" -> "); Serial.println(restoreDac, 2);

      if (!setVisatonAmplitudeContinuous(restoreDac)) return ServoStatus::ABORTED;
      if (!interruptibleDelay(adaptiveSettleMs(badDac, restoreDac))) return ServoStatus::ABORTED;
      state.plantBaseline = restoreBaseline;
      lockedPlantBaseline = restoreBaseline;
      state.unproductiveCorrections++;
      clearTrialState(state);
      state.havePrevious = false;
      state.persistentSign = 0;
      state.persistenceCount = 0;
      state.integralErrorMs2 *= 0.5;

      if (state.unproductiveCorrections >= MAX_UNPRODUCTIVE_CORRECTIONS) {
        Serial.println("Repeated verified corrections failed -> REACQUIRE.");
        return ServoStatus::NEEDS_REACQUIRE;
      }
      return ServoStatus::COMPLETED;
    }
  }

  // V5.1 baseline-aware plant-state guard. If the same DAC produces a response
  // inconsistent with the confirmed XYZ fingerprint, confirm once more without
  // changing DAC. Persistent mismatch means the plant moved: stop PI and reacquire.
  if (plantStateMismatch(state.plantBaseline, rec, true)) {
    Serial.println("PLANT_CHANGE_SUSPECTED - confirming at the same DAC.");
    ControlRecord confirm = measureControlRecord(currentDacAmplitude, false);
    printControlRecord("PLANT_CONFIRM", 1, confirm);
    if (!confirm.amplitudeValid) return ServoStatus::NEEDS_REACQUIRE;
    if (plantStateMismatch(state.plantBaseline, confirm, true)) {
      Serial.println("PLANT_CHANGE_CONFIRMED -> stop PI and REACQUIRE.");
      return ServoStatus::NEEDS_REACQUIRE;
    }
    Serial.println("Plant-change suspicion rejected as a transient.");
    rec = confirm;
  }

  // Retain V5's hard 20% same-DAC guard as a second independent fallback.
  if (state.havePrevious && sameDacAbruptChange(state.previous, rec)) {
    Serial.println("ABRUPT SAME-DAC CHANGE - confirming without changing DAC.");
    ControlRecord confirm = measureControlRecord(currentDacAmplitude, false);
    if (!confirm.amplitudeValid) return ServoStatus::NEEDS_REACQUIRE;
    if (sameDacAbruptChange(state.previous, confirm)) {
      Serial.println("Persistent transfer-state change confirmed -> REACQUIRE.");
      return ServoStatus::NEEDS_REACQUIRE;
    }
    rec = confirm;
  }

  return ServoStatus::LOCKED;
}

ServoStatus maybeApplyServoCorrection(const ControlRecord &rec, ServoState &state,
                                      uint16_t &correctionCounter, const char *context) {
  double errorPct = targetErrorPercent(rec.selectedMedianMs2);
  double errorMs2 = targetAmplitudeMs2 - rec.selectedMedianMs2;

  if (fabs(errorPct) <= SERVO_DEADBAND_PERCENT) {
    state.persistentSign = 0;
    state.persistenceCount = 0;
    state.integralErrorMs2 *= 0.5; // gently unwind integral near target
    state.previous = rec;
    state.havePrevious = true;
    return ServoStatus::LOCKED;
  }

  int8_t sign = errorMs2 > 0.0 ? +1 : -1; // low amp -> increase DAC
  if (sign == state.persistentSign) state.persistenceCount++;
  else {
    state.persistentSign = sign;
    state.persistenceCount = 1;
  }

  // Accumulate a small bounded integral only for persistent out-of-band error.
  state.integralErrorMs2 += errorMs2;
  double iLimit = INTEGRAL_LIMIT_TARGET_FRACTION * targetAmplitudeMs2;
  state.integralErrorMs2 = clampDouble(state.integralErrorMs2, -iLimit, iLimit);

  uint8_t requiredPersistence = fabs(errorPct) >= SERVO_SEVERE_ERROR_PERCENT
      ? SERVO_SEVERE_PERSISTENCE_RECORDS
      : SERVO_ERROR_PERSISTENCE_RECORDS;

  if (state.persistenceCount < requiredPersistence) {
    state.previous = rec;
    state.havePrevious = true;
    return ServoStatus::LOCKED;
  }

  double slope = medianSlope();
  double maxStep = fabs(errorPct) >= SERVO_SEVERE_ERROR_PERCENT
      ? SERVO_SEVERE_MAX_STEP_DAC : SERVO_NORMAL_MAX_STEP_DAC;

  double correction = 0.0;
  const char *action = "SAFE_DIRECTION_STEP";

  if (slope >= MIN_VALID_SLOPE && slope <= MAX_VALID_SLOPE) {
    double amplitudeCommand = SERVO_KP * errorMs2 + SERVO_KI * state.integralErrorMs2;
    correction = amplitudeCommand / slope;
    correction = clampDouble(correction, -maxStep, maxStep);
    action = "ADAPTIVE_PI_SLOPE";
  } else {
    correction = static_cast<double>(sign) * SERVO_NORMAL_MAX_STEP_DAC;
  }

  double nextDac = quantizeDac(currentDacAmplitude + correction);
  if (fabs(nextDac - currentDacAmplitude) < 1.0e-9) {
    nextDac = quantizeDac(currentDacAmplitude + static_cast<double>(sign) * DAC_QUANTUM);
  }

  if (fabs(nextDac - currentDacAmplitude) < 1.0e-9) {
    Serial.println("SERVO reached DAC boundary; no further authority in requested direction.");
    return ServoStatus::AUTHORITY_LIMIT;
  }

  // QUANTIZATION-AWARE GUARD:
  // A common DAC quantum can correspond to very different acceleration changes
  // on X, Y and Z because the local dA/dDAC slope is axis/frequency dependent.
  // If the learned slope predicts that the smallest realizable move will make the
  // absolute amplitude error no better, do not create a limit cycle.
  if (slope >= MIN_VALID_SLOPE && slope <= MAX_VALID_SLOPE) {
    double predictedAmp = rec.selectedMedianMs2 + slope * (nextDac - currentDacAmplitude);
    double predictedErrPct = targetErrorPercent(predictedAmp);
    if (fabs(predictedErrPct) >= fabs(errorPct) - SERVO_QUANTIZATION_GUARD_MARGIN_PERCENT) {
      Serial.print(context);
      Serial.print(" QUANTIZATION_GUARD - hold DAC "); Serial.print(currentDacAmplitude, 2);
      Serial.print(" | current err "); Serial.print(errorPct, 2);
      Serial.print(" % | candidate "); Serial.print(nextDac, 2);
      Serial.print(" predicted err "); Serial.print(predictedErrPct, 2); Serial.println(" %");
      state.persistentSign = 0;
      state.persistenceCount = 0;
      state.integralErrorMs2 *= 0.5;
      state.previous = rec;
      state.havePrevious = true;
      return ServoStatus::LOCKED;
    }
  } else if (fabs(errorPct) < SERVO_SEVERE_ERROR_PERCENT) {
    // Do not make a blind quantized step for a modest error before a usable local
    // slope exists. Wait for more evidence; severe errors may still use safe direction.
    Serial.print(context);
    Serial.println(" NO_SLOPE_GUARD - modest error retained; no blind DAC step.");
    state.persistentSign = 0;
    state.persistenceCount = 0;
    state.integralErrorMs2 *= 0.5;
    state.previous = rec;
    state.havePrevious = true;
    return ServoStatus::LOCKED;
  }

  double oldDac = currentDacAmplitude;

  // V5.1: mark this as a trial. The next two valid records decide whether the
  // step is kept or reverted; slope learning also waits for that verification.
  state.trialPending = true;
  state.trialOldDac = oldDac;
  state.trialOldAmp = rec.selectedMedianMs2;
  state.trialOldErrorPct = errorPct;
  state.trialOldBaseline = state.plantBaseline;
  state.trialValidCount = 0;
  state.trialAmp[0] = state.trialAmp[1] = 0.0;
  for (uint8_t a = 0; a < 3; a++) state.trialAxisSum[a] = 0.0;
  state.trialTotalSum = 0.0;
  state.trialMaxCvPercent = 0.0;

  if (!setVisatonAmplitudeContinuous(nextDac)) {
    clearTrialState(state);
    return ServoStatus::ABORTED;
  }
  correctionCounter++;

  Serial.print(context); Serial.print(" DAC correction ");
  Serial.print(oldDac, 2); Serial.print(" -> "); Serial.print(nextDac, 2);
  Serial.print(" | action TRIAL_"); Serial.print(action);
  Serial.print(" | slope "); Serial.println(slope, 6);

  if (!interruptibleDelay(adaptiveSettleMs(oldDac, nextDac))) return ServoStatus::ABORTED;

  // Post-change record must not be compared as if it were same-DAC continuity.
  state.havePrevious = false;
  state.persistentSign = 0;
  state.persistenceCount = 0;
  return ServoStatus::LOCKED;
}

ServoStatus runServoAcquire(ServoState &state) {
  Serial.println("\n============================================================");
  Serial.println("STAGE 2 - SLOW AMPLITUDE SERVO LOCK");
  Serial.print("Target lock band: +/-"); Serial.print(SERVO_LOCK_BAND_PERCENT, 1); Serial.println(" %");
  Serial.print("Deadband: +/-"); Serial.print(SERVO_DEADBAND_PERCENT, 1); Serial.println(" %");
  Serial.println("Two consecutive lock-band records are required.");
  Serial.println("============================================================");

  uint8_t lockCount = 0;
  uint16_t corrections = 0;

  for (uint8_t recordIndex = 1; recordIndex <= SERVO_ACQUIRE_MAX_RECORDS; recordIndex++) {
    if (checkEmergencyStop()) return ServoStatus::ABORTED;

    ControlRecord rec = measureControlRecord(currentDacAmplitude, false);
    printControlRecord("SERVO_ACQUIRE", recordIndex, rec);

    ServoStatus prep = validateAndPrepareServoRecord(rec, state);
    if (prep == ServoStatus::COMPLETED) continue; // invalid sample; retry
    if (prep != ServoStatus::LOCKED) return prep;

    if (inTargetBand(rec.selectedMedianMs2, SERVO_LOCK_BAND_PERCENT)) lockCount++;
    else lockCount = 0;

    if (lockCount >= SERVO_LOCK_CONSECUTIVE_RECORDS) {
      lockedPoint.dac = rec.dac;
      lockedPoint.amplitudeMs2 = rec.selectedMedianMs2;
      lockedPoint.valid = true;
      lockedPoint.stationary = true;
      Serial.println("SERVO LOCK ACQUIRED.");
      return ServoStatus::LOCKED;
    }

    ServoStatus action = maybeApplyServoCorrection(rec, state, corrections, "SERVO_ACQUIRE");
    if (action != ServoStatus::LOCKED) return action;
  }

  Serial.println("Servo did not achieve two consecutive lock-band records.");
  return ServoStatus::NEEDS_REACQUIRE;
}

// ============================================================================
// CLOSED-LOOP HOLD WITH RUNNING STATISTICS
// ============================================================================

HoldResult runClosedLoopHold(uint8_t seconds, ServoState &state) {
  HoldResult out = {};
  out.status = ServoStatus::COMPLETED;

  // Start the hold with a clean PI/persistence state. Keep the learned global
  // dA/dDAC slope history, but do not let acquisition-stage integral or error
  // persistence trigger an immediate hold correction.
  state.integralErrorMs2 = 0.0;
  state.persistentSign = 0;
  state.persistenceCount = 0;
  state.invalidCount = 0;
  state.unproductiveCorrections = 0;
  clearTrialState(state);
  state.havePrevious = false;

  Serial.println("\n============================================================");
  Serial.println("STAGE 3 - CLOSED-LOOP AMPLITUDE HOLD");
  Serial.print("Duration: "); Serial.print(seconds); Serial.println(" s");
  Serial.print("Primary pass band: +/-"); Serial.print(HOLD_FINAL_BAND_PERCENT, 1); Serial.println(" %");
  Serial.print("Tight reported band: +/-"); Serial.print(HOLD_TIGHT_BAND_PERCENT, 1); Serial.println(" %");
  Serial.println("DAC may move slowly; NCO phase remains continuous.");
  Serial.println("CSV_HEADER,HOLD,t_s,DAC,Axis,Target,Measured,Error_pct,X,Y,Z,CV_pct,MAD_pct,CorrectionCount");
  Serial.println("============================================================");

  RunningStats ampStats, syncStats, dacStats;
  resetRunningStats(ampStats);
  resetRunningStats(syncStats);
  resetRunningStats(dacStats);
  double errorSquareSum = 0.0;
  double maxAbsErrorPercent = 0.0;

  bool havePhase = false;
  double previousWrappedPhase = 0.0;
  double unwrappedPhase = 0.0;
  double minUnwrappedPhase = 0.0;
  double maxUnwrappedPhase = 0.0;

  uint32_t startMs = millis();
  uint16_t recordIndex = 0;

  while (millis() - startMs < static_cast<uint32_t>(seconds) * 1000UL) {
    if (checkEmergencyStop()) {
      out.status = ServoStatus::ABORTED;
      return out;
    }

    ControlRecord rec = measureControlRecord(currentDacAmplitude, false);
    recordIndex++;

    ServoStatus prep = validateAndPrepareServoRecord(rec, state);
    if (prep == ServoStatus::COMPLETED) continue;
    if (prep != ServoStatus::LOCKED) {
      out.status = prep;
      return out;
    }

    double errPct = targetErrorPercent(rec.selectedMedianMs2);
    pushRunningStats(ampStats, rec.selectedMedianMs2);
    pushRunningStats(syncStats, rec.axisSync[axisIndex()]); // diagnostic only
    pushRunningStats(dacStats, rec.dac);
    errorSquareSum += errPct * errPct;
    if (fabs(errPct) > maxAbsErrorPercent) maxAbsErrorPercent = fabs(errPct);

    if (inTargetBand(rec.selectedMedianMs2, HOLD_TIGHT_BAND_PERCENT)) out.tightInBandRecords++;
    if (inTargetBand(rec.selectedMedianMs2, HOLD_FINAL_BAND_PERCENT)) out.finalInBandRecords++;

    double phase = rec.axisPhaseDeg[axisIndex()];
    if (!havePhase) {
      havePhase = true;
      previousWrappedPhase = phase;
      unwrappedPhase = phase;
      minUnwrappedPhase = phase;
      maxUnwrappedPhase = phase;
    } else {
      double delta = wrapDegrees(phase - previousWrappedPhase);
      unwrappedPhase += delta;
      previousWrappedPhase = phase;
      if (unwrappedPhase < minUnwrappedPhase) minUnwrappedPhase = unwrappedPhase;
      if (unwrappedPhase > maxUnwrappedPhase) maxUnwrappedPhase = unwrappedPhase;
    }

    Serial.print("HOLD,");
    Serial.print((millis() - startMs) / 1000.0, 2); Serial.print(",");
    Serial.print(rec.dac, 2); Serial.print(",");
    Serial.print(axisChar()); Serial.print(",");
    Serial.print(targetAmplitudeMs2, 5); Serial.print(",");
    Serial.print(rec.selectedMedianMs2, 5); Serial.print(",");
    Serial.print(errPct, 2); Serial.print(",");
    Serial.print(rec.axisMeanMs2[0], 5); Serial.print(",");
    Serial.print(rec.axisMeanMs2[1], 5); Serial.print(",");
    Serial.print(rec.axisMeanMs2[2], 5); Serial.print(",");
    Serial.print(rec.cvPercent, 2); Serial.print(",");
    Serial.print(rec.madPercent, 2); Serial.print(",");
    Serial.println(out.corrections);

    ServoStatus action = maybeApplyServoCorrection(rec, state, out.corrections, "HOLD");
    if (action != ServoStatus::LOCKED) {
      out.status = action;
      return out;
    }
  }

  out.completed = true;
  out.validRecords = ampStats.n;
  if (ampStats.n == 0) {
    out.status = ServoStatus::NEEDS_REACQUIRE;
    return out;
  }

  out.meanAmplitudeMs2 = ampStats.mean;
  out.sdAmplitudeMs2 = runningSd(ampStats);
  out.cvPercent = 100.0 * out.sdAmplitudeMs2 / fmax(out.meanAmplitudeMs2, 1.0e-12);
  out.meanErrorPercent = targetErrorPercent(out.meanAmplitudeMs2);
  out.tightInBandPercent = 100.0 * static_cast<double>(out.tightInBandRecords) /
                           static_cast<double>(out.validRecords);
  out.finalInBandPercent = 100.0 * static_cast<double>(out.finalInBandRecords) /
                           static_cast<double>(out.validRecords);
  out.rmsErrorPercent = sqrt(errorSquareSum / static_cast<double>(out.validRecords));
  out.maxAbsErrorPercent = maxAbsErrorPercent;
  out.meanSyncRatio = syncStats.mean; // diagnostic only; never gates amplitude PASS
  out.phaseSpanDeg = havePhase ? maxUnwrappedPhase - minUnwrappedPhase : 999.0;
  out.minAmplitudeMs2 = ampStats.minValue;
  out.maxAmplitudeMs2 = ampStats.maxValue;
  out.minDac = dacStats.minValue;
  out.maxDac = dacStats.maxValue;
  out.meanDac = dacStats.mean;

  if (out.meanSyncRatio >= HOLD_STRONG_SYNC && out.phaseSpanDeg <= HOLD_STRONG_PHASE_SPAN_DEG) {
    out.phaseClass = PhaseClass::STRONG;
  } else if (out.meanSyncRatio >= HOLD_MODERATE_SYNC &&
             out.phaseSpanDeg <= HOLD_MODERATE_PHASE_SPAN_DEG) {
    out.phaseClass = PhaseClass::MODERATE;
  } else {
    out.phaseClass = PhaseClass::WEAK;
  }

  out.amplitudePass = out.validRecords >= HOLD_MIN_VALID_RECORDS &&
                      inTargetBand(out.meanAmplitudeMs2, HOLD_FINAL_BAND_PERCENT) &&
                      out.cvPercent <= HOLD_MAX_CV_PERCENT &&
                      out.rmsErrorPercent <= HOLD_MAX_RMS_ERROR_PERCENT;

  out.phaseSweepReady = out.amplitudePass &&
                        out.finalInBandPercent >= PHASE_READY_MIN_IN_BAND_PERCENT &&
                        out.cvPercent <= PHASE_READY_MAX_CV_PERCENT &&
                        out.rmsErrorPercent <= PHASE_READY_MAX_RMS_ERROR_PERCENT &&
                        out.corrections <= PHASE_READY_MAX_CORRECTIONS;

  Serial.println("\n---------------- CLOSED-LOOP HOLD SUMMARY ----------------");
  Serial.print("Valid records: "); Serial.println(out.validRecords);
  Serial.print("Mean amplitude: "); Serial.print(out.meanAmplitudeMs2, 5);
  Serial.print(" m/s^2 | mean error "); Serial.print(out.meanErrorPercent, 2); Serial.println(" %");
  Serial.print("CV: "); Serial.print(out.cvPercent, 2); Serial.println(" %");
  Serial.print("Tight (+/-2.5%) in-band: "); Serial.print(out.tightInBandPercent, 1); Serial.println(" %");
  Serial.print("Primary (+/-5%) in-band [reported]: "); Serial.print(out.finalInBandPercent, 1); Serial.println(" %");
  Serial.print("Hold RMS error: "); Serial.print(out.rmsErrorPercent, 2); Serial.println(" %");
  Serial.print("Maximum absolute hold error: "); Serial.print(out.maxAbsErrorPercent, 2); Serial.println(" %");
  Serial.print("Amplitude min/max: "); Serial.print(out.minAmplitudeMs2, 5);
  Serial.print(" / "); Serial.println(out.maxAmplitudeMs2, 5);
  Serial.print("DAC mean/min/max: "); Serial.print(out.meanDac, 3); Serial.print(" / ");
  Serial.print(out.minDac, 2); Serial.print(" / "); Serial.println(out.maxDac, 2);
  Serial.print("Servo corrections: "); Serial.println(out.corrections);
  Serial.print("Amplitude regulation: "); Serial.println(out.amplitudePass ? "PASS" : "FAIL");
  Serial.print("Phase-sweep readiness: "); Serial.println(out.phaseSweepReady ? "READY" : "NOT READY");
  Serial.println("-----------------------------------------------------------");
  return out;
}

// ============================================================================
// VECTOR FFT VERIFICATION
// ============================================================================

void clearSpectrum(double spectrum[]) {
  for (uint16_t i = 0; i < HALF_BINS; i++) spectrum[i] = 0.0;
}

bool acquireFftFrame() {
  if (!verifyAdxl2(false)) return false;

  uint32_t nextSampleUs = micros();
  RawAcceleration previous = {};
  bool havePrevious = false;
  uint16_t stagnantRun = 0;
  uint16_t longestStagnantRun = 0;
  uint16_t zeroTriplets = 0;
  bool clipped = false;
  uint32_t lateSamples = 0;

  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    while (static_cast<int32_t>(micros() - nextSampleUs) < 0) {}
    uint32_t actualUs = micros();
    if (actualUs - nextSampleUs > LATE_SAMPLE_WARNING_US) lateSamples++;
    nextSampleUs += SAMPLE_PERIOD_US;

    RawAcceleration raw = readRawXYZ();
    rawX[i] = raw.x;
    rawY[i] = raw.y;
    rawZ[i] = raw.z;

    if (raw.x == 0 && raw.y == 0 && raw.z == 0) zeroTriplets++;
    if (abs(static_cast<int>(raw.x)) >= ADXL_RAW_CLIP_LIMIT ||
        abs(static_cast<int>(raw.y)) >= ADXL_RAW_CLIP_LIMIT ||
        abs(static_cast<int>(raw.z)) >= ADXL_RAW_CLIP_LIMIT) clipped = true;

    if (havePrevious && raw.x == previous.x && raw.y == previous.y && raw.z == previous.z) {
      stagnantRun++;
      if (stagnantRun > longestStagnantRun) longestStagnantRun = stagnantRun;
    } else {
      stagnantRun = 0;
    }
    previous = raw;
    havePrevious = true;
  }

  return verifyAdxl2(false) && !clipped &&
         lateSamples <= MAX_LATE_SAMPLES_PER_WINDOW &&
         zeroTriplets <= MAX_ZERO_TRIPLETS &&
         longestStagnantRun <= MAX_STAGNANT_RUN;
}

void accumulateSpectrum(const int16_t source[], double destination[]) {
  double meanRaw = 0.0;
  for (uint16_t i = 0; i < FFT_SAMPLES; i++) meanRaw += source[i];
  meanRaw /= static_cast<double>(FFT_SAMPLES);

  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    fftReal[i] = (static_cast<double>(source[i]) - meanRaw) * ADXL_MS2_PER_LSB;
    fftImag[i] = 0.0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  for (uint16_t bin = 1; bin < HALF_BINS; bin++) destination[bin] += fftReal[bin];
}

double averageMagnitude(const double spectrum[], uint16_t bin) {
  return spectrum[bin] / static_cast<double>(FFT_FRAMES);
}

double magnitudeToAmplitude(double magnitude) {
  return 2.0 * magnitude /
         (static_cast<double>(FFT_SAMPLES) * HAMMING_COHERENT_GAIN);
}

VectorFftResult analyzeVectorBand(double minHz, double maxHz) {
  VectorFftResult out = {};

  uint16_t lo = static_cast<uint16_t>(ceil(minHz * FFT_SAMPLES / SAMPLE_RATE_HZ));
  uint16_t hi = static_cast<uint16_t>(floor(maxHz * FFT_SAMPLES / SAMPLE_RATE_HZ));
  if (lo < 1) lo = 1;
  if (hi >= HALF_BINS) hi = HALF_BINS - 1;
  if (hi <= lo) return out;

  double bestCombined = -1.0;
  uint16_t bestBin = lo;

  for (uint16_t bin = lo; bin <= hi; bin++) {
    double x = averageMagnitude(spectrumX, bin);
    double y = averageMagnitude(spectrumY, bin);
    double z = averageMagnitude(spectrumZ, bin);
    double combined = sqrt(x*x + y*y + z*z);
    if (combined > bestCombined) {
      bestCombined = combined;
      bestBin = bin;
    }
  }

  double delta = 0.0;
  if (bestBin > lo && bestBin < hi) {
    double m0, m1, m2;
    {
      uint16_t b = bestBin - 1;
      double x = averageMagnitude(spectrumX, b);
      double y = averageMagnitude(spectrumY, b);
      double z = averageMagnitude(spectrumZ, b);
      m0 = sqrt(x*x + y*y + z*z);
    }
    {
      uint16_t b = bestBin;
      double x = averageMagnitude(spectrumX, b);
      double y = averageMagnitude(spectrumY, b);
      double z = averageMagnitude(spectrumZ, b);
      m1 = sqrt(x*x + y*y + z*z);
    }
    {
      uint16_t b = bestBin + 1;
      double x = averageMagnitude(spectrumX, b);
      double y = averageMagnitude(spectrumY, b);
      double z = averageMagnitude(spectrumZ, b);
      m2 = sqrt(x*x + y*y + z*z);
    }
    double denom = m0 - 2.0*m1 + m2;
    if (fabs(denom) > 1.0e-12) {
      delta = clampDouble(0.5 * (m0 - m2) / denom, -0.5, 0.5);
    }
  }

  out.frequencyHz = (static_cast<double>(bestBin) + delta) * SAMPLE_RATE_HZ /
                    static_cast<double>(FFT_SAMPLES);
  out.xAmplitudeMs2 = magnitudeToAmplitude(averageMagnitude(spectrumX, bestBin));
  out.yAmplitudeMs2 = magnitudeToAmplitude(averageMagnitude(spectrumY, bestBin));
  out.zAmplitudeMs2 = magnitudeToAmplitude(averageMagnitude(spectrumZ, bestBin));
  out.totalAmplitudeMs2 = sqrt(out.xAmplitudeMs2*out.xAmplitudeMs2 +
                               out.yAmplitudeMs2*out.yAmplitudeMs2 +
                               out.zAmplitudeMs2*out.zAmplitudeMs2);
  out.dominantAxis = dominantAxis(out.xAmplitudeMs2, out.yAmplitudeMs2, out.zAmplitudeMs2);

  double noiseSum = 0.0;
  uint16_t noiseCount = 0;
  for (uint16_t bin = lo; bin <= hi; bin++) {
    if (abs(static_cast<int>(bin) - static_cast<int>(bestBin)) <= FFT_PEAK_EXCLUSION_BINS) continue;
    double x = averageMagnitude(spectrumX, bin);
    double y = averageMagnitude(spectrumY, bin);
    double z = averageMagnitude(spectrumZ, bin);
    noiseSum += sqrt(x*x + y*y + z*z);
    noiseCount++;
  }

  if (bestCombined > 0.0 && noiseCount > 0 && noiseSum > 0.0) {
    out.snrDb = 20.0 * log10(bestCombined / (noiseSum / noiseCount));
  }

  out.valid = out.totalAmplitudeMs2 >= FFT_MIN_AMPLITUDE_MS2 &&
              out.snrDb >= FFT_MIN_SNR_DB;
  return out;
}

bool runLocalVectorFft(VectorFftResult &out) {
  clearSpectrum(spectrumX);
  clearSpectrum(spectrumY);
  clearSpectrum(spectrumZ);

  for (uint8_t frame = 0; frame < FFT_FRAMES; frame++) {
    if (!acquireFftFrame()) return false;
    accumulateSpectrum(rawX, spectrumX);
    accumulateSpectrum(rawY, spectrumY);
    accumulateSpectrum(rawZ, spectrumZ);
  }

  out = analyzeVectorBand(testFrequencyHz - LOCAL_SEARCH_HALF_WIDTH_HZ,
                          testFrequencyHz + LOCAL_SEARCH_HALF_WIDTH_HZ);
  return out.valid;
}

void printFftResult(const VectorFftResult &r) {
  Serial.println("\n---------------- VECTOR FFT VERIFICATION ----------------");
  if (!r.valid) {
    Serial.println("FFT verification: FAIL QUALITY");
    return;
  }
  Serial.print("Command frequency: "); Serial.print(testFrequencyHz, 5); Serial.println(" Hz");
  Serial.print("Actual NCO frequency: "); Serial.print(actualNcoFrequencyHz(), 7); Serial.println(" Hz");
  Serial.print("Vector local FFT: "); Serial.print(r.frequencyHz, 5);
  Serial.print(" Hz | error "); Serial.print(r.frequencyHz - testFrequencyHz, 5); Serial.println(" Hz");
  Serial.print("X/Y/Z FFT amp: "); Serial.print(r.xAmplitudeMs2, 5); Serial.print(" / ");
  Serial.print(r.yAmplitudeMs2, 5); Serial.print(" / "); Serial.println(r.zAmplitudeMs2, 5);
  Serial.print("Total: "); Serial.print(r.totalAmplitudeMs2, 5);
  Serial.print(" | SNR "); Serial.print(r.snrDb, 2);
  Serial.print(" dB | dominant "); Serial.println(r.dominantAxis);
  Serial.println("---------------------------------------------------------");
}


bool acquireAndServoLock(AcquisitionResult &acq, ServoState &servo);

// ============================================================================
// INTEGRATED HARD-CODED 240..320 Hz DUAL-ADXL FFT + QUADRATURE DISCOVERY
// ============================================================================

const char *discAxisName(uint8_t axis) {
  if (axis == 0) return "X";
  if (axis == 1) return "Y";
  return "Z";
}

bool discRawTripletZero(const RawAcceleration &a) {
  return a.x == 0 && a.y == 0 && a.z == 0;
}

bool discRawTripletClipped(const RawAcceleration &a) {
  return abs(static_cast<int>(a.x)) >= ADXL_RAW_CLIP_LIMIT ||
         abs(static_cast<int>(a.y)) >= ADXL_RAW_CLIP_LIMIT ||
         abs(static_cast<int>(a.z)) >= ADXL_RAW_CLIP_LIMIT;
}

uint16_t discMinSearchBin() {
  uint16_t b = static_cast<uint16_t>(ceil(DISC_SEARCH_MIN_HZ / DISC_FFT_RESOLUTION_HZ));
  if (b < 1) b = 1;
  return b;
}

uint16_t discMaxSearchBin() {
  uint16_t b = static_cast<uint16_t>(floor(DISC_SEARCH_MAX_HZ / DISC_FFT_RESOLUTION_HZ));
  if (b >= HALF_BINS) b = HALF_BINS - 1;
  return b;
}

void discClearSpectrum(double spectrum[]) {
  for (uint16_t i = 0; i < HALF_BINS; i++) spectrum[i] = 0.0;
}

void discClearAllSpectra() {
  discClearSpectrum(discToolSpecX);
  discClearSpectrum(discToolSpecY);
  discClearSpectrum(discToolSpecZ);
  discClearSpectrum(discHandSpecX);
  discClearSpectrum(discHandSpecY);
  discClearSpectrum(discHandSpecZ);
  discClearSpectrum(discToolVectorSpectrum);
}

void discAccumulateSpectrum(const int16_t source[], double destination[]) {
  double mean = 0.0;
  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    mean += static_cast<double>(source[i]) * ADXL_MS2_PER_LSB;
  }
  mean /= static_cast<double>(FFT_SAMPLES);

  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    fftReal[i] = static_cast<double>(source[i]) * ADXL_MS2_PER_LSB - mean;
    fftImag[i] = 0.0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  for (uint16_t bin = 1; bin < HALF_BINS; bin++) destination[bin] += fftReal[bin];
}

double discAvgMag(const double spectrum[], uint16_t bin) {
  return spectrum[bin] / static_cast<double>(FFT_FRAMES);
}

double discFftMagnitudeToPeakAmplitude(double magnitude) {
  return (2.0 * magnitude) /
         (static_cast<double>(FFT_SAMPLES) * HAMMING_COHERENT_GAIN);
}

double discParabolicOffset(const double spectrum[], uint16_t bin) {
  if (bin < 1 || bin + 1 >= HALF_BINS) return 0.0;
  double a = discAvgMag(spectrum, bin - 1);
  double b = discAvgMag(spectrum, bin);
  double c = discAvgMag(spectrum, bin + 1);
  double den = a - 2.0 * b + c;
  if (fabs(den) < 1.0e-18) return 0.0;
  return clampDouble(0.5 * (a - c) / den, -0.5, 0.5);
}

double discParabolicPeakMagnitude(const double spectrum[], uint16_t bin, double delta) {
  double a = discAvgMag(spectrum, bin - 1);
  double b = discAvgMag(spectrum, bin);
  double c = discAvgMag(spectrum, bin + 1);
  return b - 0.25 * (a - c) * delta;
}

double discCalculateSpectrumSnrDb(const double spectrum[], uint16_t peakBin) {
  uint16_t lo = discMinSearchBin();
  uint16_t hi = discMaxSearchBin();
  double noiseSum = 0.0;
  uint16_t noiseCount = 0;
  for (uint16_t bin = lo; bin <= hi; bin++) {
    if (abs(static_cast<int>(bin) - static_cast<int>(peakBin)) <= DISC_PEAK_EXCLUSION_BINS) continue;
    noiseSum += discAvgMag(spectrum, bin);
    noiseCount++;
  }
  double peak = discAvgMag(spectrum, peakBin);
  if (peak <= 0.0 || noiseCount == 0) return 0.0;
  double noise = noiseSum / static_cast<double>(noiseCount);
  if (noise <= 0.0) return 0.0;
  return 20.0 * log10(peak / noise);
}

DiscoveryAxisFFTResult discAnalyzeAxisFFT(const double spectrum[]) {
  DiscoveryAxisFFTResult result = {};
  uint16_t lo = discMinSearchBin();
  uint16_t hi = discMaxSearchBin();
  uint16_t bestBin = lo;
  double bestMag = 0.0;
  for (uint16_t bin = lo; bin <= hi; bin++) {
    double m = discAvgMag(spectrum, bin);
    if (m > bestMag) {
      bestMag = m;
      bestBin = bin;
    }
  }
  double delta = discParabolicOffset(spectrum, bestBin);
  double peakMag = discParabolicPeakMagnitude(spectrum, bestBin, delta);
  result.bin = bestBin;
  result.frequencyHz = (static_cast<double>(bestBin) + delta) * DISC_FFT_RESOLUTION_HZ;
  result.amplitudeMs2 = discFftMagnitudeToPeakAmplitude(peakMag);
  result.snrDb = discCalculateSpectrumSnrDb(spectrum, bestBin);
  result.valid = result.amplitudeMs2 >= DISC_MIN_SIGNAL_AMPLITUDE_MS2 &&
                 result.snrDb >= DISC_MIN_FFT_SNR_DB;
  return result;
}

bool discIsLocalMaximum(const double spectrum[], uint16_t bin, uint16_t lo, uint16_t hi) {
  if (bin <= lo || bin >= hi) return false;
  return spectrum[bin] > spectrum[bin - 1] && spectrum[bin] >= spectrum[bin + 1];
}

void discBuildVectorSpectrum(const double sx[], const double sy[], const double sz[], double out[]) {
  for (uint16_t bin = 0; bin < HALF_BINS; bin++) {
    double x = discAvgMag(sx, bin);
    double y = discAvgMag(sy, bin);
    double z = discAvgMag(sz, bin);
    out[bin] = sqrt(x * x + y * y + z * z);
  }
}

double discCalculateVectorSnrDb(const double vectorSpectrum[], uint16_t peakBin) {
  uint16_t lo = discMinSearchBin();
  uint16_t hi = discMaxSearchBin();
  double noiseSum = 0.0;
  uint16_t noiseCount = 0;
  for (uint16_t bin = lo; bin <= hi; bin++) {
    if (abs(static_cast<int>(bin) - static_cast<int>(peakBin)) <= DISC_PEAK_EXCLUSION_BINS) continue;
    noiseSum += vectorSpectrum[bin];
    noiseCount++;
  }
  double peak = vectorSpectrum[peakBin];
  if (peak <= 0.0 || noiseCount == 0) return 0.0;
  double noise = noiseSum / static_cast<double>(noiseCount);
  if (noise <= 0.0) return 0.0;
  return 20.0 * log10(peak / noise);
}

DiscoveryTiming discAcquirePairedRecord(uint16_t sampleCount) {
  DiscoveryTiming timing = {};
  uint32_t nextSampleUs = micros();
  uint32_t startUs = nextSampleUs;
  uint64_t separationSumUs = 0;

  for (uint16_t i = 0; i < sampleCount; i++) {
    while (static_cast<int32_t>(micros() - nextSampleUs) < 0) {}
    uint32_t actualStartUs = micros();
    uint32_t latenessUs = actualStartUs - nextSampleUs;
    nextSampleUs += SAMPLE_PERIOD_US;
    if (latenessUs > timing.maxScheduleLatenessUs) timing.maxScheduleLatenessUs = latenessUs;
    if (latenessUs > DISC_LATE_SAMPLE_WARNING_US) timing.lateSampleCount++;

    uint32_t toolReadUs = micros();
    RawAcceleration tool = dualReadRawXYZ(PIN_CS_ADXL1);
    uint32_t handReadUs = micros();
    RawAcceleration hand = dualReadRawXYZ(PIN_CS_ADXL2);
    uint32_t sep = handReadUs - toolReadUs;
    separationSumUs += sep;
    if (sep > timing.maxReadSeparationUs) timing.maxReadSeparationUs = sep;

    discToolX[i] = tool.x; discToolY[i] = tool.y; discToolZ[i] = tool.z;
    discHandX[i] = hand.x; discHandY[i] = hand.y; discHandZ[i] = hand.z;

    if (discRawTripletZero(tool)) timing.zeroTriplets++;
    if (discRawTripletZero(hand)) timing.zeroTriplets++;
    if (discRawTripletClipped(tool)) timing.clippedSamples++;
    if (discRawTripletClipped(hand)) timing.clippedSamples++;
  }

  timing.elapsedUs = micros() - startUs;
  timing.meanReadSeparationUs = static_cast<double>(separationSumUs) /
                                static_cast<double>(sampleCount);
  timing.valid = timing.zeroTriplets <= DISC_MAX_ZERO_TRIPLETS_PER_RECORD &&
                 timing.clippedSamples == 0;
  return timing;
}

bool discAcquireThreeFftFrames() {
  discClearAllSpectra();
  if (!verifyBothIntegratedSensorsOrRecover()) return false;

  for (uint8_t frame = 0; frame < FFT_FRAMES; frame++) {
    bool accepted = false;
    for (uint8_t attempt = 0; attempt <= DISC_MAX_FRAME_RETRIES && !accepted; attempt++) {
      DiscoveryTiming t = discAcquirePairedRecord(FFT_SAMPLES);
      if (!t.valid) {
        Serial.print("FFT frame "); Serial.print(frame + 1);
        Serial.print(" integrity reject | zero triplets "); Serial.print(t.zeroTriplets);
        Serial.print(" | clipped samples "); Serial.println(t.clippedSamples);
        if (!verifyBothIntegratedSensorsOrRecover()) return false;
        continue;
      }

      discAccumulateSpectrum(discToolX, discToolSpecX);
      discAccumulateSpectrum(discToolY, discToolSpecY);
      discAccumulateSpectrum(discToolZ, discToolSpecZ);
      discAccumulateSpectrum(discHandX, discHandSpecX);
      discAccumulateSpectrum(discHandY, discHandSpecY);
      discAccumulateSpectrum(discHandZ, discHandSpecZ);

      Serial.print("Paired narrowband FFT frame "); Serial.print(frame + 1);
      Serial.print("/3 accepted | elapsed "); Serial.print(t.elapsedUs / 1000.0, 2);
      Serial.print(" ms | mean tool->hand read separation ");
      Serial.print(t.meanReadSeparationUs, 2);
      Serial.print(" us | late samples "); Serial.println(t.lateSampleCount);
      accepted = true;
    }
    if (!accepted) return false;
  }

  discBuildVectorSpectrum(discToolSpecX, discToolSpecY, discToolSpecZ, discToolVectorSpectrum);
  return true;
}

void discAnalyzeIndependentFftResults() {
  discoveryToolFFT.axis[0] = discAnalyzeAxisFFT(discToolSpecX);
  discoveryToolFFT.axis[1] = discAnalyzeAxisFFT(discToolSpecY);
  discoveryToolFFT.axis[2] = discAnalyzeAxisFFT(discToolSpecZ);
  discoveryHandFFT.axis[0] = discAnalyzeAxisFFT(discHandSpecX);
  discoveryHandFFT.axis[1] = discAnalyzeAxisFFT(discHandSpecY);
  discoveryHandFFT.axis[2] = discAnalyzeAxisFFT(discHandSpecZ);
}

void discFindToolCandidates() {
  discoveryCandidateCount = 0;
  for (uint8_t i = 0; i < DISC_TOP_CANDIDATES; i++) discoveryCandidates[i] = {};

  uint16_t lo = discMinSearchBin();
  uint16_t hi = discMaxSearchBin();
  bool excluded[HALF_BINS];
  for (uint16_t i = 0; i < HALF_BINS; i++) excluded[i] = false;

  for (uint8_t rank = 0; rank < DISC_TOP_CANDIDATES; rank++) {
    uint16_t bestBin = 0;
    double best = 0.0;
    for (uint16_t bin = lo; bin <= hi; bin++) {
      if (excluded[bin]) continue;
      if (!discIsLocalMaximum(discToolVectorSpectrum, bin, lo, hi)) continue;
      if (discToolVectorSpectrum[bin] > best) {
        best = discToolVectorSpectrum[bin];
        bestBin = bin;
      }
    }
    if (bestBin == 0 || best <= 0.0) break;

    DiscoveryCandidate &c = discoveryCandidates[discoveryCandidateCount];
    c.fftRank = discoveryCandidateCount + 1;
    c.bin = bestBin;
    double a = discToolVectorSpectrum[bestBin - 1];
    double b = discToolVectorSpectrum[bestBin];
    double d = discToolVectorSpectrum[bestBin + 1];
    double den = a - 2.0 * b + d;
    double delta = 0.0;
    if (fabs(den) > 1.0e-18) delta = clampDouble(0.5 * (a - d) / den, -0.5, 0.5);
    double peakVectorMag = b - 0.25 * (a - d) * delta;
    c.fftSeedHz = (static_cast<double>(bestBin) + delta) * DISC_FFT_RESOLUTION_HZ;
    c.fftVectorAmplitudeMs2 = discFftMagnitudeToPeakAmplitude(peakVectorMag);
    c.fftSnrDb = discCalculateVectorSnrDb(discToolVectorSpectrum, bestBin);
    c.fftValid = c.fftVectorAmplitudeMs2 >= DISC_MIN_SIGNAL_AMPLITUDE_MS2 &&
                 c.fftSnrDb >= DISC_MIN_FFT_SNR_DB;
    discoveryCandidateCount++;

    int start = static_cast<int>(bestBin) - DISC_PEAK_EXCLUSION_BINS;
    int end = static_cast<int>(bestBin) + DISC_PEAK_EXCLUSION_BINS;
    if (start < static_cast<int>(lo)) start = lo;
    if (end > static_cast<int>(hi)) end = hi;
    for (int bin = start; bin <= end; bin++) excluded[bin] = true;
  }
}

DiscoveryAxisQuadResult discAnalyzeAxisQuadrature(const int16_t samples[], double referenceHz) {
  DiscoveryAxisQuadResult result = {};
  result.referenceHz = referenceHz;
  result.refinedHz = referenceHz;
  if (referenceHz <= 0.0 || referenceHz >= SAMPLE_RATE_HZ * 0.5) return result;

  double blockPhase[DISC_QUAD_BLOCKS];
  double blockAmplitude[DISC_QUAD_BLOCKS];
  double blockSync[DISC_QUAD_BLOCKS];
  const double KK_TWO_PI = 2.0 * PI;

  // ------------------------------------------------------------------------
  // Short-block quadrature analysis.
  // Each 256-sample block is only ~160 ms long at the measured acquisition
  // rate.  A slowly wandering hand-held tool can therefore remain strongly
  // periodic locally even when a single 1.6 s reference sinusoid loses sync.
  // ------------------------------------------------------------------------
  for (uint8_t block = 0; block < DISC_QUAD_BLOCKS; block++) {
    uint32_t offset = static_cast<uint32_t>(block) * DISC_QUAD_BLOCK_N;
    double mean = 0.0;
    for (uint16_t n = 0; n < DISC_QUAD_BLOCK_N; n++) {
      mean += static_cast<double>(samples[offset + n]);
    }
    mean /= static_cast<double>(DISC_QUAD_BLOCK_N);

    double I = 0.0, Q = 0.0, windowSum = 0.0, blockSumSq = 0.0;
    for (uint16_t n = 0; n < DISC_QUAD_BLOCK_N; n++) {
      uint32_t globalIndex = offset + n;
      double centered = (static_cast<double>(samples[globalIndex]) - mean) * ADXL_MS2_PER_LSB;
      blockSumSq += centered * centered;

      double w = 0.5 - 0.5 * cos(KK_TWO_PI * static_cast<double>(n) /
                                 static_cast<double>(DISC_QUAD_BLOCK_N - 1));
      double theta = KK_TWO_PI * referenceHz * static_cast<double>(globalIndex) / SAMPLE_RATE_HZ;
      double v = centered * w;
      I += v * cos(theta);
      Q += v * sin(theta);
      windowSum += w;
    }

    blockPhase[block] = atan2(-Q, I);
    blockAmplitude[block] = windowSum > 0.0 ? 2.0 * sqrt(I * I + Q * Q) / windowSum : 0.0;

    double blockTotalRms = sqrt(blockSumSq / static_cast<double>(DISC_QUAD_BLOCK_N));
    double blockSyncRms = blockAmplitude[block] / sqrt(2.0);
    blockSync[block] = blockTotalRms > 1.0e-12 ? blockSyncRms / blockTotalRms : 0.0;
    blockSync[block] = clampDouble(blockSync[block], 0.0, 1.0);
  }

  // Median local synchronization is robust to one or two disturbed blocks.
  double sortedSync[DISC_QUAD_BLOCKS];
  for (uint8_t i = 0; i < DISC_QUAD_BLOCKS; i++) sortedSync[i] = blockSync[i];
  for (uint8_t i = 1; i < DISC_QUAD_BLOCKS; i++) {
    double key = sortedSync[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && sortedSync[j] > key) {
      sortedSync[j + 1] = sortedSync[j];
      j--;
    }
    sortedSync[j + 1] = key;
  }
  if ((DISC_QUAD_BLOCKS & 1U) != 0U) {
    result.localMedianSyncRatio = sortedSync[DISC_QUAD_BLOCKS / 2];
  } else {
    result.localMedianSyncRatio = 0.5 * (sortedSync[DISC_QUAD_BLOCKS / 2 - 1] +
                                         sortedSync[DISC_QUAD_BLOCKS / 2]);
  }
  result.localGoodBlocks = 0;
  for (uint8_t i = 0; i < DISC_QUAD_BLOCKS; i++) {
    if (blockSync[i] >= DISC_LOCAL_GOOD_BLOCK_SYNC) result.localGoodBlocks++;
  }

  // ------------------------------------------------------------------------
  // Long-record phase progression.  This remains the stability test: a real
  // candidate may drift slowly, but the unwrapped block phase should still be
  // well described by a near-linear trend.  Large/random residuals are rejected.
  // ------------------------------------------------------------------------
  double unwrapped[DISC_QUAD_BLOCKS];
  unwrapped[0] = blockPhase[0];
  for (uint8_t block = 1; block < DISC_QUAD_BLOCKS; block++) {
    double d = blockPhase[block] - blockPhase[block - 1];
    while (d > PI) d -= KK_TWO_PI;
    while (d < -PI) d += KK_TWO_PI;
    unwrapped[block] = unwrapped[block - 1] + d;
  }

  double blockDuration = static_cast<double>(DISC_QUAD_BLOCK_N) / SAMPLE_RATE_HZ;
  double meanT = 0.0, meanP = 0.0, meanA = 0.0;
  for (uint8_t block = 0; block < DISC_QUAD_BLOCKS; block++) {
    double t = (static_cast<double>(block) + 0.5) * blockDuration;
    meanT += t;
    meanP += unwrapped[block];
    meanA += blockAmplitude[block];
  }
  meanT /= static_cast<double>(DISC_QUAD_BLOCKS);
  meanP /= static_cast<double>(DISC_QUAD_BLOCKS);
  meanA /= static_cast<double>(DISC_QUAD_BLOCKS);

  double cov = 0.0, var = 0.0;
  for (uint8_t block = 0; block < DISC_QUAD_BLOCKS; block++) {
    double t = (static_cast<double>(block) + 0.5) * blockDuration;
    cov += (t - meanT) * (unwrapped[block] - meanP);
    var += (t - meanT) * (t - meanT);
  }
  double phaseSlope = var > 0.0 ? cov / var : 0.0;
  result.correctionHz = phaseSlope / KK_TWO_PI;
  result.refinedHz = referenceHz + result.correctionHz;
  result.amplitudeMs2 = meanA;

  double rss = 0.0;
  for (uint8_t block = 0; block < DISC_QUAD_BLOCKS; block++) {
    double t = (static_cast<double>(block) + 0.5) * blockDuration;
    double fit = meanP + phaseSlope * (t - meanT);
    double r = unwrapped[block] - fit;
    rss += r * r;
  }
  result.phaseFitRmseDeg = sqrt(rss / static_cast<double>(DISC_QUAD_BLOCKS)) * 180.0 / PI;

  // ------------------------------------------------------------------------
  // Whole-record synchronous ratio is retained for reporting only.  It is
  // intentionally NOT used as the V1.3 primary gate because it is sensitive
  // to frequency/phase wander across the full ~1.6 s record.
  // ------------------------------------------------------------------------
  double recordMean = 0.0;
  for (uint16_t i = 0; i < DISC_QUAD_N; i++) recordMean += static_cast<double>(samples[i]);
  recordMean /= static_cast<double>(DISC_QUAD_N);

  double I = 0.0, Q = 0.0, sumSq = 0.0, windowSum = 0.0;
  for (uint16_t i = 0; i < DISC_QUAD_N; i++) {
    double centered = (static_cast<double>(samples[i]) - recordMean) * ADXL_MS2_PER_LSB;
    sumSq += centered * centered;
    double w = 0.5 - 0.5 * cos(KK_TWO_PI * static_cast<double>(i) /
                               static_cast<double>(DISC_QUAD_N - 1));
    double theta = KK_TWO_PI * referenceHz * static_cast<double>(i) / SAMPLE_RATE_HZ;
    I += centered * w * cos(theta);
    Q += centered * w * sin(theta);
    windowSum += w;
  }
  double fullAmp = windowSum > 0.0 ? 2.0 * sqrt(I * I + Q * Q) / windowSum : 0.0;
  result.phaseDeg = wrapDegrees(atan2(-Q, I) * 180.0 / PI);
  double totalRms = sqrt(sumSq / static_cast<double>(DISC_QUAD_N));
  double synchronousRms = fullAmp / sqrt(2.0);
  result.syncRatio = totalRms > 1.0e-12 ? synchronousRms / totalRms : 0.0;
  result.syncRatio = clampDouble(result.syncRatio, 0.0, 1.0);

  result.valid = result.amplitudeMs2 >= DISC_MIN_SIGNAL_AMPLITUDE_MS2 &&
                 fabs(result.correctionHz) <= DISC_QUAD_MAX_CORRECTION_HZ &&
                 result.phaseFitRmseDeg <= DISC_QUAD_MAX_PHASE_RMSE_DEG &&
                 result.localMedianSyncRatio >= DISC_LOCAL_AXIS_MIN_MEDIAN_SYNC &&
                 result.localGoodBlocks >= DISC_LOCAL_MIN_GOOD_BLOCKS;
  return result;
}

DiscoverySensorAtF0 discAnalyzeSensorAtFrequency(const int16_t x[], const int16_t y[],
                                                 const int16_t z[], double fHz) {
  DiscoverySensorAtF0 result = {};
  result.axis[0] = discAnalyzeAxisQuadrature(x, fHz);
  result.axis[1] = discAnalyzeAxisQuadrature(y, fHz);
  result.axis[2] = discAnalyzeAxisQuadrature(z, fHz);
  double ax = result.axis[0].amplitudeMs2;
  double ay = result.axis[1].amplitudeMs2;
  double az = result.axis[2].amplitudeMs2;
  result.J = ax * ax + ay * ay + az * az;
  result.vectorAmplitudeMs2 = sqrt(result.J);
  return result;
}

double discWeightedCorrection(const DiscoverySensorAtF0 &sensor) {
  double sumW = 0.0, sum = 0.0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    const DiscoveryAxisQuadResult &q = sensor.axis[axis];
    if (q.amplitudeMs2 < DISC_MIN_SIGNAL_AMPLITUDE_MS2) continue;
    if (q.phaseFitRmseDeg > DISC_QUAD_MAX_PHASE_RMSE_DEG) continue;
    if (fabs(q.correctionHz) > DISC_QUAD_MAX_CORRECTION_HZ) continue;
    double w = q.amplitudeMs2 * q.amplitudeMs2 * fmax(q.localMedianSyncRatio, 0.10);
    sumW += w;
    sum += w * q.correctionHz;
  }
  return sumW > 0.0 ? sum / sumW : 0.0;
}

double discWeightedSync(const DiscoverySensorAtF0 &sensor) {
  double sumW = 0.0, sum = 0.0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    double w = sensor.axis[axis].amplitudeMs2 * sensor.axis[axis].amplitudeMs2;
    sumW += w;
    sum += w * sensor.axis[axis].syncRatio;
  }
  return sumW > 0.0 ? sum / sumW : 0.0;
}

double discWeightedLocalSync(const DiscoverySensorAtF0 &sensor) {
  double sumW = 0.0, sum = 0.0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    double w = sensor.axis[axis].amplitudeMs2 * sensor.axis[axis].amplitudeMs2;
    sumW += w;
    sum += w * sensor.axis[axis].localMedianSyncRatio;
  }
  return sumW > 0.0 ? sum / sumW : 0.0;
}

double discWeightedPhaseRmse(const DiscoverySensorAtF0 &sensor) {
  double sumW = 0.0, sum = 0.0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    double w = sensor.axis[axis].amplitudeMs2 * sensor.axis[axis].amplitudeMs2;
    sumW += w;
    sum += w * sensor.axis[axis].phaseFitRmseDeg;
  }
  return sumW > 0.0 ? sum / sumW : 999.0;
}

uint8_t discCountSupportAxes(const DiscoverySensorAtF0 &sensor) {
  uint8_t count = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    const DiscoveryAxisQuadResult &q = sensor.axis[axis];
    if (q.amplitudeMs2 >= DISC_MIN_SIGNAL_AMPLITUDE_MS2 &&
        q.localMedianSyncRatio >= DISC_SUPPORT_AXIS_MIN_LOCAL_SYNC &&
        q.localGoodBlocks >= DISC_LOCAL_MIN_GOOD_BLOCKS &&
        q.phaseFitRmseDeg <= DISC_QUAD_MAX_PHASE_RMSE_DEG &&
        fabs(q.correctionHz) <= DISC_QUAD_MAX_CORRECTION_HZ) {
      count++;
    }
  }
  return count;
}

bool discAxisFftSupportsFrequency(const DiscoveryAxisFFTResult &a, double fHz) {
  return a.valid && fabs(a.frequencyHz - fHz) <= DISC_FREQ_CONSENSUS_TOL_HZ;
}

uint8_t discCountFftConsensusAxes(const DiscoverySensorFFTResult &sensor, double fHz) {
  uint8_t count = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    if (discAxisFftSupportsFrequency(sensor.axis[axis], fHz)) count++;
  }
  return count;
}

void discValidateCandidate(DiscoveryCandidate &c) {
  if (!c.fftValid) {
    c.quadValid = false;
    return;
  }

  DiscoverySensorAtF0 pass1 = discAnalyzeSensorAtFrequency(discToolX, discToolY, discToolZ, c.fftSeedHz);
  double df1 = clampDouble(discWeightedCorrection(pass1),
                           -DISC_QUAD_MAX_CORRECTION_HZ, DISC_QUAD_MAX_CORRECTION_HZ);
  double f1 = c.fftSeedHz + df1;
  DiscoverySensorAtF0 pass2 = discAnalyzeSensorAtFrequency(discToolX, discToolY, discToolZ, f1);
  double df2 = clampDouble(discWeightedCorrection(pass2),
                           -DISC_QUAD_MAX_CORRECTION_HZ, DISC_QUAD_MAX_CORRECTION_HZ);
  double totalDf = clampDouble(df1 + df2,
                               -DISC_QUAD_MAX_CORRECTION_HZ, DISC_QUAD_MAX_CORRECTION_HZ);
  c.refinedHz = c.fftSeedHz + totalDf;

  c.toolAtCandidate = discAnalyzeSensorAtFrequency(discToolX, discToolY, discToolZ, c.refinedHz);
  c.handAtCandidate = discAnalyzeSensorAtFrequency(discHandX, discHandY, discHandZ, c.refinedHz);
  c.toolVectorAmplitudeMs2 = c.toolAtCandidate.vectorAmplitudeMs2;
  c.handVectorAmplitudeMs2 = c.handAtCandidate.vectorAmplitudeMs2;
  c.toolWeightedSync = discWeightedSync(c.toolAtCandidate);                 // diagnostic
  c.toolWeightedLocalSync = discWeightedLocalSync(c.toolAtCandidate);       // V1.3 gate
  c.toolWeightedPhaseRmseDeg = discWeightedPhaseRmse(c.toolAtCandidate);
  c.supportAxes = discCountSupportAxes(c.toolAtCandidate);
  c.toolFrequencyConsensusAxes = discCountFftConsensusAxes(discoveryToolFFT, c.refinedHz);
  c.handFrequencyConsensusAxes = discCountFftConsensusAxes(discoveryHandFFT, c.refinedHz);
  uint8_t totalConsensus = c.toolFrequencyConsensusAxes + c.handFrequencyConsensusAxes;

  c.quadValid = c.toolVectorAmplitudeMs2 >= DISC_MIN_SIGNAL_AMPLITUDE_MS2 &&
                c.toolWeightedLocalSync >= DISC_LOCAL_AXIS_MIN_MEDIAN_SYNC &&
                c.toolWeightedPhaseRmseDeg <= DISC_QUAD_MAX_PHASE_RMSE_DEG &&
                c.supportAxes >= 1 &&
                c.toolFrequencyConsensusAxes >= DISC_MIN_TOOL_FREQ_CONSENSUS_AXES &&
                totalConsensus >= DISC_MIN_TOTAL_FREQ_CONSENSUS_AXES;

  double syncFactor = 0.5 + 0.5 * clampDouble(c.toolWeightedLocalSync, 0.0, 1.0);
  double supportFactor = 0.7 + 0.15 * static_cast<double>(c.supportAxes);
  double consensusFactor = 0.70 + 0.10 * static_cast<double>(totalConsensus);
  double stabilityFactor = 1.0 / (1.0 + c.toolWeightedPhaseRmseDeg / 30.0);
  double snrFactor = clampDouble((c.fftSnrDb - 3.0) / 17.0, 0.35, 1.20);
  c.qualityScore = c.quadValid
      ? c.toolVectorAmplitudeMs2 * syncFactor * supportFactor * consensusFactor *
        stabilityFactor * snrFactor
      : 0.0;
}

bool discAcquireQuadratureRecord() {
  if (!verifyBothIntegratedSensorsOrRecover()) return false;
  Serial.print("Acquiring uninterrupted "); Serial.print(DISC_QUAD_N); Serial.println("-sample paired quadrature record...");
  DiscoveryTiming t = discAcquirePairedRecord(DISC_QUAD_N);
  Serial.print("Quadrature acquisition: "); Serial.print(t.elapsedUs / 1000.0, 2);
  Serial.print(" ms | mean tool->hand read separation "); Serial.print(t.meanReadSeparationUs, 2);
  Serial.print(" us | late samples "); Serial.print(t.lateSampleCount);
  Serial.print(" | zero triplets "); Serial.print(t.zeroTriplets);
  Serial.print(" | clipped samples "); Serial.println(t.clippedSamples);
  if (!t.valid) {
    Serial.println("QUADRATURE RECORD REJECTED: acquisition integrity gate failed.");
    return false;
  }
  return true;
}

void discPrintIndependentPeaks(const char *title, const DiscoverySensorFFTResult &r) {
  Serial.println();
  Serial.println(title);
  Serial.println("AXIS   DOMINANT PEAK       FFT AMP (peak)     SNR       STATUS");
  Serial.println("---------------------------------------------------------------");
  for (uint8_t axis = 0; axis < 3; axis++) {
    const DiscoveryAxisFFTResult &a = r.axis[axis];
    Serial.print(discAxisName(axis)); Serial.print("      ");
    Serial.print(a.frequencyHz, 3); Serial.print(" Hz        ");
    Serial.print(a.amplitudeMs2, 5); Serial.print(" m/s2      ");
    Serial.print(a.snrDb, 1); Serial.print(" dB     ");
    Serial.println(a.valid ? "VALID" : "LOW CONF");
  }
}

void discPrintCandidateTable() {
  Serial.println();
  Serial.println("ADXL1 TOOL - HARD-CODED 240..320 Hz TRIAXIAL CANDIDATES");
  Serial.println("N  FFTseed  SNR   Refined   ToolAmp  FullSync  LocalSync  RMSE  Sup  FconsT/H  HandAmp  Score");
  Serial.println("------------------------------------------------------------------------------------------------");
  for (uint8_t i = 0; i < discoveryCandidateCount; i++) {
    DiscoveryCandidate &c = discoveryCandidates[i];
    Serial.print(i + 1); Serial.print("  ");
    Serial.print(c.fftSeedHz, 3); Serial.print("  ");
    Serial.print(c.fftSnrDb, 1); Serial.print("  ");
    Serial.print(c.refinedHz, 4); Serial.print("  ");
    Serial.print(c.toolVectorAmplitudeMs2, 4); Serial.print("   ");
    Serial.print(c.toolWeightedSync, 3); Serial.print("     ");
    Serial.print(c.toolWeightedLocalSync, 3); Serial.print("      ");
    Serial.print(c.toolWeightedPhaseRmseDeg, 1); Serial.print("   ");
    Serial.print(c.supportAxes); Serial.print("    ");
    Serial.print(c.toolFrequencyConsensusAxes); Serial.print("/");
    Serial.print(c.handFrequencyConsensusAxes); Serial.print("      ");
    Serial.print(c.handVectorAmplitudeMs2, 4); Serial.print("   ");
    Serial.print(c.qualityScore, 4);
    if (!c.quadValid) Serial.print("  LOW_CONF");
    Serial.println();
  }
  Serial.println("FullSync = old whole-record diagnostic; LocalSync = median of 10 short-block sync ratios.");
  Serial.println("FconsT/H = number of independent ADXL1/ADXL2 FFT axis peaks agreeing with the candidate.");
}

void discPrintSensorAtF0(const char *title, const DiscoverySensorFFTResult &fft,
                         const DiscoverySensorAtF0 &q, double f0) {
  Serial.println();
  Serial.println(title);
  Serial.print("COMMON CONTROL f0 = "); Serial.print(f0, 6); Serial.println(" Hz");
  Serial.println("AXIS   LOCAL DOM PEAK      AMPLITUDE @ f0      PHASE@f0   FullSync  LocalSync  GoodBlk  STATUS");
  Serial.println("------------------------------------------------------------------------------------------------");
  for (uint8_t axis = 0; axis < 3; axis++) {
    Serial.print(discAxisName(axis)); Serial.print("      ");
    Serial.print(fft.axis[axis].frequencyHz, 3); Serial.print(" Hz        ");
    Serial.print(q.axis[axis].amplitudeMs2, 5); Serial.print(" m/s2      ");
    Serial.print(q.axis[axis].phaseDeg, 1); Serial.print(" deg     ");
    Serial.print(q.axis[axis].syncRatio, 3); Serial.print("     ");
    Serial.print(q.axis[axis].localMedianSyncRatio, 3); Serial.print("      ");
    Serial.print(q.axis[axis].localGoodBlocks); Serial.print("/10     ");
    Serial.println(q.axis[axis].valid ? "VALID" : "LOW CONF");
  }
  Serial.print("Vector amplitude @ f0 = "); Serial.print(q.vectorAmplitudeMs2, 5); Serial.println(" m/s2");
  Serial.print("J @ f0 = "); Serial.print(q.J, 6); Serial.println(" (m/s2)^2");
}

int discBestCandidateIndexByScore() {
  int best = -1;
  double bestScore = 0.0;
  for (uint8_t i = 0; i < discoveryCandidateCount; i++) {
    if (discoveryCandidates[i].quadValid && discoveryCandidates[i].qualityScore > bestScore) {
      bestScore = discoveryCandidates[i].qualityScore;
      best = i;
    }
  }
  return best;
}

int discSecondBestCandidateIndexByScore(int best) {
  int second = -1;
  double secondScore = 0.0;
  for (uint8_t i = 0; i < discoveryCandidateCount; i++) {
    if (static_cast<int>(i) == best) continue;
    if (discoveryCandidates[i].quadValid && discoveryCandidates[i].qualityScore > secondScore) {
      secondScore = discoveryCandidates[i].qualityScore;
      second = i;
    }
  }
  return second;
}

void applyIntegratedBaseline(uint8_t candidateIndex) {
  DiscoveryCandidate &c = discoveryCandidates[candidateIndex];
  baselineF0Hz = c.refinedHz;
  baselineToolAtF0 = c.toolAtCandidate;
  baselineHandAtF0 = c.handAtCandidate;
  baselineValid = true;
  baselineAxisSelected = false;
  integratedWorkflowState = IntegratedWorkflowState::AWAIT_AXIS;

  Serial.println();
  Serial.println("====================================================================");
  Serial.println("NARROWBAND TOOL FREQUENCY ACCEPTED");
  Serial.print("f0 = "); Serial.print(baselineF0Hz, 6); Serial.println(" Hz");
  Serial.println("This f0 is common to the later amplitude matcher and phase sweep.");
  Serial.println("====================================================================");
  discPrintSensorAtF0("ADXL1 TOOL / REFERENCE", discoveryToolFFT, baselineToolAtF0, baselineF0Hz);
  discPrintSensorAtF0("ADXL2 BACK-OF-HAND / ERROR SENSOR", discoveryHandFFT, baselineHandAtF0, baselineF0Hz);

  Serial.println();
  Serial.println("CSV_BASELINE,f0_Hz,ADXL1_X,ADXL1_Y,ADXL1_Z,ADXL2_X,ADXL2_Y,ADXL2_Z");
  Serial.print("CSV_BASELINE,"); Serial.print(baselineF0Hz, 6); Serial.print(",");
  Serial.print(baselineToolAtF0.axis[0].amplitudeMs2, 6); Serial.print(",");
  Serial.print(baselineToolAtF0.axis[1].amplitudeMs2, 6); Serial.print(",");
  Serial.print(baselineToolAtF0.axis[2].amplitudeMs2, 6); Serial.print(",");
  Serial.print(baselineHandAtF0.axis[0].amplitudeMs2, 6); Serial.print(",");
  Serial.print(baselineHandAtF0.axis[1].amplitudeMs2, 6); Serial.print(",");
  Serial.println(baselineHandAtF0.axis[2].amplitudeMs2, 6);

  Serial.println();
  Serial.println("Choose the ADXL2 error-sensor axis to attenuate: x / y / z");
  Serial.println("The matcher will use the selected ADXL2 amplitude AT THIS SAME f0.");
}

void runIntegratedDiscovery() {
  stopVisaton();
  targetAcquired = false;
  clearSlopeHistory();
  baselineValid = false;
  baselineAxisSelected = false;
  baselineF0Hz = 0.0;
  frozenMatchDac = 0.0;
  frozenMatchMeasuredMs2 = 0.0;
  integratedWorkflowState = IntegratedWorkflowState::NEED_BASELINE;

  Serial.println();
  Serial.println("================================================================================");
  Serial.println("INTEGRATED STAGE A - TOOL/HAND NARROWBAND FFT + QUADRATURE BASELINE");
  Serial.println("TOOL ON. VISATON OFF. Keep grip/mounting/knob state unchanged.");
  Serial.print("Hard-coded search band: "); Serial.print(DISC_SEARCH_MIN_HZ, 1);
  Serial.print(" .. "); Serial.print(DISC_SEARCH_MAX_HZ, 1); Serial.println(" Hz");
  Serial.println("================================================================================");

  if (!discAcquireThreeFftFrames()) {
    Serial.println("BASELINE FAILED during FFT acquisition.");
    return;
  }

  discAnalyzeIndependentFftResults();
  discFindToolCandidates();
  discPrintIndependentPeaks("ADXL1 TOOL - INDEPENDENT NARROWBAND DOMINANT PEAKS", discoveryToolFFT);
  discPrintIndependentPeaks("ADXL2 HAND - INDEPENDENT NARROWBAND DOMINANT PEAKS", discoveryHandFFT);

  if (discoveryCandidateCount == 0) {
    Serial.println("NO TOOL CANDIDATE FOUND in 240..320 Hz. Baseline rejected.");
    return;
  }

  Serial.println();
  Serial.println("Validating tool candidates on ONE uninterrupted quadrature record...");
  if (!discAcquireQuadratureRecord()) return;
  for (uint8_t i = 0; i < discoveryCandidateCount; i++) discValidateCandidate(discoveryCandidates[i]);
  discPrintCandidateTable();

  int best = discBestCandidateIndexByScore();
  if (best < 0) {
    Serial.println("NO CANDIDATE PASSED quadrature quality validation. Repeat a under steady tool operation.");
    return;
  }

  int second = discSecondBestCandidateIndexByScore(best);
  bool uniqueWinner = true;
  double runnerUpRatio = 999.0;
  if (second >= 0 && discoveryCandidates[second].qualityScore > 0.0) {
    runnerUpRatio = discoveryCandidates[best].qualityScore /
                    discoveryCandidates[second].qualityScore;
    uniqueWinner = runnerUpRatio >= DISC_AUTO_LOCK_SCORE_RATIO;
  }

  DiscoveryCandidate &winner = discoveryCandidates[best];
  uint8_t totalConsensus = winner.toolFrequencyConsensusAxes + winner.handFrequencyConsensusAxes;
  bool qualityFloor =
      winner.toolWeightedLocalSync >= DISC_AUTO_LOCK_MIN_WEIGHTED_LOCAL_SYNC &&
      winner.toolWeightedPhaseRmseDeg <= DISC_QUAD_MAX_PHASE_RMSE_DEG &&
      winner.supportAxes >= DISC_AUTO_LOCK_MIN_SUPPORT_AXES &&
      winner.toolFrequencyConsensusAxes >= DISC_MIN_TOOL_FREQ_CONSENSUS_AXES &&
      totalConsensus >= DISC_MIN_TOTAL_FREQ_CONSENSUS_AXES;

  Serial.println();
  Serial.print("Best quality-weighted candidate: #"); Serial.print(best + 1);
  Serial.print(" at "); Serial.print(winner.refinedHz, 6); Serial.println(" Hz");
  Serial.print("Quality summary: FullSync="); Serial.print(winner.toolWeightedSync, 3);
  Serial.print(" (diagnostic), LocalSync="); Serial.print(winner.toolWeightedLocalSync, 3);
  Serial.print(", RMSE="); Serial.print(winner.toolWeightedPhaseRmseDeg, 2);
  Serial.print(" deg, support axes="); Serial.print(winner.supportAxes);
  Serial.print(", FFT consensus tool/hand="); Serial.print(winner.toolFrequencyConsensusAxes);
  Serial.print("/"); Serial.println(winner.handFrequencyConsensusAxes);

  if (!uniqueWinner) {
    Serial.println("MULTIPLE CREDIBLE FREQUENCIES - CONTROLLER BASELINE ABORTED.");
    Serial.print("Top/runner-up score ratio = "); Serial.print(runnerUpRatio, 3);
    Serial.print(" < required "); Serial.println(DISC_AUTO_LOCK_SCORE_RATIO, 2);
    Serial.println("This is genuine candidate ambiguity. Keep tool operation steady and repeat a.");
    return;
  }

  if (!qualityFloor) {
    Serial.println("UNIQUE FREQUENCY CANDIDATE FOUND, BUT QUALITY FLOOR NOT MET - BASELINE ABORTED.");
    if (winner.toolWeightedLocalSync < DISC_AUTO_LOCK_MIN_WEIGHTED_LOCAL_SYNC) {
      Serial.print("Local weighted sync "); Serial.print(winner.toolWeightedLocalSync, 3);
      Serial.print(" < "); Serial.println(DISC_AUTO_LOCK_MIN_WEIGHTED_LOCAL_SYNC, 2);
    }
    if (winner.supportAxes < DISC_AUTO_LOCK_MIN_SUPPORT_AXES) {
      Serial.println("No ADXL1 axis met the drift-tolerant local-sync/support gate.");
    }
    if (winner.toolFrequencyConsensusAxes < DISC_MIN_TOOL_FREQ_CONSENSUS_AXES ||
        totalConsensus < DISC_MIN_TOTAL_FREQ_CONSENSUS_AXES) {
      Serial.print("FFT frequency consensus insufficient: tool/hand = ");
      Serial.print(winner.toolFrequencyConsensusAxes); Serial.print("/");
      Serial.println(winner.handFrequencyConsensusAxes);
    }
    if (winner.toolWeightedPhaseRmseDeg > DISC_QUAD_MAX_PHASE_RMSE_DEG) {
      Serial.print("Phase-fit RMSE "); Serial.print(winner.toolWeightedPhaseRmseDeg, 2);
      Serial.print(" deg > "); Serial.print(DISC_QUAD_MAX_PHASE_RMSE_DEG, 1); Serial.println(" deg");
    }
    Serial.println("This is NOT candidate ambiguity; it is a signal-quality/stability rejection.");
    return;
  }

  Serial.println("DRIFT-TOLERANT FREQUENCY QUALITY PASS.");
  Serial.println("Whole-record SyncRatio is retained only as a diagnostic; local sync + phase stability + FFT consensus passed.");

  Serial.println("CSV_DISCOVERY_QUALITY,f0_Hz,fullSync,localSync,phaseRMSE_deg,supportAxes,toolConsensus,handConsensus,score");
  Serial.print("CSV_DISCOVERY_QUALITY,"); Serial.print(winner.refinedHz, 6); Serial.print(",");
  Serial.print(winner.toolWeightedSync, 4); Serial.print(",");
  Serial.print(winner.toolWeightedLocalSync, 4); Serial.print(",");
  Serial.print(winner.toolWeightedPhaseRmseDeg, 3); Serial.print(",");
  Serial.print(winner.supportAxes); Serial.print(",");
  Serial.print(winner.toolFrequencyConsensusAxes); Serial.print(",");
  Serial.print(winner.handFrequencyConsensusAxes); Serial.print(",");
  Serial.println(winner.qualityScore, 5);

  applyIntegratedBaseline(static_cast<uint8_t>(best));
}

void selectIntegratedAxis(char c) {
  if (!baselineValid || integratedWorkflowState != IntegratedWorkflowState::AWAIT_AXIS) {
    Serial.println("No valid baseline awaiting axis selection. TOOL ON, VISATON OFF, then run a first.");
    return;
  }

  uint8_t idx = c == 'x' ? 0 : (c == 'y' ? 1 : 2);
  const DiscoveryAxisQuadResult &q = baselineHandAtF0.axis[idx];
  if (!q.valid || q.amplitudeMs2 < MIN_REPORT_AMPLITUDE_MS2) {
    Serial.print("ADXL2 "); Serial.print(static_cast<char>(toupper(c)));
    Serial.println(" @ f0 is LOW CONFIDENCE. Choose another valid axis or repeat baseline a.");
    return;
  }
  if (q.amplitudeMs2 > AXIS_SOFTWARE_GUARD_MS2 * 0.80) {
    Serial.println("Selected baseline amplitude is outside the matcher's allowed target range.");
    return;
  }

  controlAxis = c == 'x' ? ControlAxis::X_AXIS :
                c == 'y' ? ControlAxis::Y_AXIS : ControlAxis::Z_AXIS;
  testFrequencyHz = baselineF0Hz;
  targetAmplitudeMs2 = q.amplitudeMs2;
  targetAcquired = false;
  clearSlopeHistory();
  baselineAxisSelected = true;
  integratedWorkflowState = IntegratedWorkflowState::AWAIT_TOOL_OFF;

  Serial.println();
  Serial.println("====================================================================");
  Serial.println("ERROR AXIS SELECTED");
  Serial.print("Control axis: ADXL2 "); Serial.println(axisChar());
  Serial.print("Common tool frequency f0: "); Serial.print(testFrequencyHz, 6); Serial.println(" Hz");
  Serial.print("Target amplitude at f0: "); Serial.print(targetAmplitudeMs2, 6); Serial.println(" m/s^2 peak");
  Serial.println("====================================================================");
  Serial.println("NOW SWITCH TOOL OFF. DO NOT MOVE THE SENSORS / HAND / VISATON ASSEMBLY.");
  Serial.println("When the tool is fully OFF, enter c to start V5.1R amplitude matching.");
}

void runIntegratedAmplitudeMatch() {
  if (!baselineValid || !baselineAxisSelected ||
      integratedWorkflowState != IntegratedWorkflowState::AWAIT_TOOL_OFF) {
    Serial.println("Not ready for matching. Required order: a -> x/y/z -> TOOL OFF -> c");
    return;
  }

  Serial.println();
  Serial.println("================================================================================");
  Serial.println("INTEGRATED STAGE B - V5.1R ADAPTIVE AMPLITUDE MATCH");
  Serial.println("Operator confirmation received: TOOL OFF.");
  Serial.print("Frequency: "); Serial.print(testFrequencyHz, 6); Serial.println(" Hz");
  Serial.print("Selected ADXL2 axis: "); Serial.println(axisChar());
  Serial.print("Target: "); Serial.print(targetAmplitudeMs2, 6); Serial.println(" m/s^2 peak");
  Serial.println("After lock the final DAC will be FROZEN; no live servo will remain active.");
  Serial.println("================================================================================");

  integratedWorkflowState = IntegratedWorkflowState::MATCHING;
  automaticTestRunning = true;
  emergencyStopRequested = false;
  targetAcquired = false;
  stopVisaton();
  clearSlopeHistory();

  AcquisitionResult acq = {};
  ServoState servo = {};
  uint32_t startMs = millis();
  bool success = acquireAndServoLock(acq, servo);
  uint32_t elapsedMs = millis() - startMs;

  if (success && !emergencyStopRequested && visatonRunning) {
    // runServoAcquire has already required two consecutive lock-band records.
    // From this instant onward there are NO calls to maybeApplyServoCorrection.
    // The hardware-timer NCO simply keeps running at the fixed final DAC.
    frozenMatchDac = currentDacAmplitude;
    frozenMatchMeasuredMs2 = lockedPoint.amplitudeMs2;
    targetAcquired = true;
    integratedWorkflowState = IntegratedWorkflowState::FIXED_MATCH_RUNNING;

    Serial.println();
    Serial.println("================================================================================");
    Serial.println("AMPLITUDE MATCH SUCCESS - FIXED-DAC PHASE-READY HANDOFF");
    Serial.print("f0: "); Serial.print(testFrequencyHz, 6); Serial.println(" Hz");
    Serial.print("Axis: ADXL2 "); Serial.println(axisChar());
    Serial.print("Stored baseline target: "); Serial.print(targetAmplitudeMs2, 6); Serial.println(" m/s^2 peak");
    Serial.print("Final locked measurement: "); Serial.print(frozenMatchMeasuredMs2, 6); Serial.println(" m/s^2 peak");
    Serial.print("Final frozen DAC amplitude: "); Serial.println(frozenMatchDac, 2);
    Serial.print("Actual NCO frequency: "); Serial.print(actualNcoFrequencyHz(), 7); Serial.println(" Hz");
    Serial.print("Match + servo-lock time: "); Serial.print(elapsedMs / 1000.0, 3); Serial.println(" s");
    Serial.println("AMPLITUDE SERVO IS NOW DISABLED BY WORKFLOW.");
    Serial.println("VISATON NCO CONTINUES AT FIXED FREQUENCY + FIXED DAC UNTIL s.");
    Serial.println("Do NOT run a tracking servo during the later destructive-interference phase sweep.");
    Serial.println("PHASE READY.");
    Serial.println("================================================================================");
    Serial.print("CSV_FIXED_MATCH,");
    Serial.print(testFrequencyHz, 6); Serial.print(",");
    Serial.print(axisChar()); Serial.print(",");
    Serial.print(targetAmplitudeMs2, 6); Serial.print(",");
    Serial.print(frozenMatchMeasuredMs2, 6); Serial.print(",");
    Serial.print(frozenMatchDac, 2); Serial.print(",");
    Serial.println(elapsedMs);
  } else {
    Serial.println("AMPLITUDE MATCH FAILED.");
    if (!emergencyStopRequested) {
      stopVisaton();
      integratedWorkflowState = baselineValid && baselineAxisSelected
          ? IntegratedWorkflowState::AWAIT_TOOL_OFF
          : IntegratedWorkflowState::NEED_BASELINE;
    } else {
      // Emergency stop during the blocking matcher invalidates the experimental
      // sequence. Require a fresh tool baseline before another attempt.
      baselineValid = false;
      baselineAxisSelected = false;
      baselineF0Hz = 0.0;
      targetAcquired = false;
      clearSlopeHistory();
      integratedWorkflowState = IntegratedWorkflowState::NEED_BASELINE;
      Serial.println("Emergency stop invalidated this sequence. Restart with TOOL ON -> a.");
    }
  }

  automaticTestRunning = false;
}

// ============================================================================
// COMPLETE RUN / REACQUISITION LOGIC
// ============================================================================

bool acquireAndServoLock(AcquisitionResult &acq, ServoState &servo) {
  acq = acquireTargetWithRestart();
  printAcquisitionSummary(acq);
  acceptAcquisition(acq);
  if (!targetAcquired) return false;

  if (!setVisatonAmplitudeContinuous(lockedPoint.dac)) return false;
  resetServoState(servo);

  if (!confirmCandidateForServo(acq, servo)) {
    acq.success = false;
    acq.status = AcquireStatus::NONSTATIONARY;
    targetAcquired = false;
    Serial.println("Candidate confirmation failed -> controlled reacquisition.");
    return false;
  }

  ServoStatus lock = runServoAcquire(servo);
  if (lock == ServoStatus::LOCKED) return true;

  if (lock == ServoStatus::NEEDS_REACQUIRE) {
    acq.success = false;
    acq.status = AcquireStatus::NONSTATIONARY;
    targetAcquired = false;
  }
  Serial.print("Servo lock failed with code "); Serial.println(static_cast<int>(lock));
  return false;
}

CompleteRunResult runOneComplete(uint8_t runNumber, bool stopAtEnd) {
  CompleteRunResult result = {};
  uint32_t completeRunStartMs = millis();
  automaticTestRunning = true;
  emergencyStopRequested = false;
  targetAcquired = false;
  stopVisaton();
  clearSlopeHistory();

  Serial.println("\n####################################################################");
  Serial.print("V5.1R COMPLETE RUN "); Serial.println(runNumber);
  Serial.println("####################################################################");
  Serial.print("Frequency: "); Serial.print(testFrequencyHz, 5); Serial.println(" Hz");
  Serial.print("Axis: "); Serial.println(axisChar());
  Serial.print("Target: "); Serial.print(targetAmplitudeMs2, 5); Serial.println(" m/s^2 peak");
  Serial.print("Closed-loop hold: "); Serial.print(holdSeconds); Serial.println(" s");
  Serial.println("Tool OFF. Do not touch the mechanical setup during the run.");

  // Calibration/matching timer starts immediately before the control sequence,
  // excluding only the run-identification banner above. Serial diagnostics emitted
  // during acquisition remain part of the measured real test wall time.
  uint32_t matchSequenceStartMs = millis();

  ServoState servo;
  resetServoState(servo);

  uint8_t runtimeReacquires = 0;

  while (true) {
    if (!acquireAndServoLock(result.acquisition, servo)) {
      if (emergencyStopRequested) break;
      if (result.acquisition.status == AcquireStatus::NONSTATIONARY &&
          runtimeReacquires < MAX_RUNTIME_REACQUIRES) {
        runtimeReacquires++;
        Serial.print("Runtime reacquisition attempt "); Serial.println(runtimeReacquires);
        continue;
      }
      break;
    }

    if (!result.matchLockAchieved) {
      result.matchLockAchieved = true;
      result.matchLockTimeMs = millis() - matchSequenceStartMs;
      Serial.println("\n---------------- MATCH TIMING ----------------");
      Serial.print("Run "); Serial.print(runNumber);
      Serial.print(" Stage-1 acquisition: "); Serial.print(result.acquisition.elapsedMs / 1000.0, 3); Serial.println(" s");
      Serial.print("Run "); Serial.print(runNumber);
      Serial.print(" successful amplitude-match + servo-lock time: ");
      Serial.print(result.matchLockTimeMs / 1000.0, 3); Serial.println(" s");
      Serial.println("CSV_MATCH_TIME,Run,AcqTime_ms,MatchLockTime_ms");
      Serial.print("CSV_MATCH_TIME,"); Serial.print(runNumber); Serial.print(",");
      Serial.print(result.acquisition.elapsedMs); Serial.print(",");
      Serial.println(result.matchLockTimeMs);
      Serial.println("------------------------------------------------");
    }

    HoldResult hold = runClosedLoopHold(holdSeconds, servo);
    hold.reacquireEvents = runtimeReacquires;

    if (hold.status == ServoStatus::NEEDS_REACQUIRE &&
        runtimeReacquires < MAX_RUNTIME_REACQUIRES && !emergencyStopRequested) {
      runtimeReacquires++;
      Serial.print("Hold requested REACQUIRE. Restarting controlled hold. Cycle ");
      Serial.println(runtimeReacquires);
      targetAcquired = false;
      continue;
    }

    result.hold = hold;
    result.completed = hold.completed;
    result.amplitudePass = hold.amplitudePass;
    if (hold.phaseSweepReady && hold.completed) rememberWarmStart(currentDacAmplitude);

    if (visatonRunning) {
      VectorFftResult fft = {};
      if (runLocalVectorFft(fft)) result.fft = fft;
      printFftResult(result.fft);
    }

    result.finalDac = currentDacAmplitude;
    // Amplitude matcher validation ends here. Destructive-interference phase
    // optimization is intentionally handled by the subsequent phase-sweep test.
    result.cancellationReady = false;
    break;
  }

  Serial.println("\n===================== FINAL V5.1R AMPLITUDE VERDICT ===================");
  Serial.print("AMPLITUDE CONTROL: "); Serial.println(result.amplitudePass ? "PASS" : "FAIL");
  Serial.print("AMPLITUDE PHASE-SWEEP READY: "); Serial.println(result.hold.phaseSweepReady ? "YES" : "NO");
  Serial.print("VECTOR FFT: "); Serial.println(result.fft.valid ? "PASS" : "FAIL/NOT EVALUATED");
  Serial.println("PHASE/CANCELLATION VERDICT: DEFERRED TO PHASE-SWEEP EXPERIMENT");
  Serial.println("============================================================");

  result.totalRunTimeMs = millis() - completeRunStartMs;
  Serial.print("TOTAL RUN TIME: "); Serial.print(result.totalRunTimeMs / 1000.0, 3); Serial.println(" s");

  if (stopAtEnd || !result.completed || emergencyStopRequested) stopVisaton();
  automaticTestRunning = false;
  return result;
}

void printRunCsvHeader() {
  Serial.println("CSV_SUMMARY_HEADER,Run,Completed,AmplitudePass,Axis,Freq_Hz,Target_mps2,FinalDAC,MeanAmp,MeanErr_pct,HoldCV_pct,RMS_Error_pct,MaxAbsError_pct,TightInBand_pct,FinalInBand_pct,DACmin,DACmax,Corrections,Reacquires,AcqRecords,AcqRejects,AcqRestarts,AcqTime_ms,MatchLockTime_ms,TotalRunTime_ms,FFT_Hz,FFT_SNR_dB");
}

void printRunCsv(uint8_t run, const CompleteRunResult &r) {
  Serial.print("CSV_SUMMARY,");
  Serial.print(run); Serial.print(",");
  Serial.print(r.completed ? 1 : 0); Serial.print(",");
  Serial.print(r.amplitudePass ? 1 : 0); Serial.print(",");
  Serial.print(axisChar()); Serial.print(",");
  Serial.print(testFrequencyHz, 5); Serial.print(",");
  Serial.print(targetAmplitudeMs2, 5); Serial.print(",");
  Serial.print(r.finalDac, 2); Serial.print(",");
  Serial.print(r.hold.meanAmplitudeMs2, 5); Serial.print(",");
  Serial.print(r.hold.meanErrorPercent, 2); Serial.print(",");
  Serial.print(r.hold.cvPercent, 2); Serial.print(",");
  Serial.print(r.hold.rmsErrorPercent, 2); Serial.print(",");
  Serial.print(r.hold.maxAbsErrorPercent, 2); Serial.print(",");
  Serial.print(r.hold.tightInBandPercent, 1); Serial.print(",");
  Serial.print(r.hold.finalInBandPercent, 1); Serial.print(",");
  Serial.print(r.hold.minDac, 2); Serial.print(",");
  Serial.print(r.hold.maxDac, 2); Serial.print(",");
  Serial.print(r.hold.corrections); Serial.print(",");
  Serial.print(r.hold.reacquireEvents); Serial.print(",");
  Serial.print(r.acquisition.recordsMeasured); Serial.print(",");
  Serial.print(r.acquisition.qualityRejects); Serial.print(",");
  Serial.print(r.acquisition.restarts); Serial.print(",");
  Serial.print(r.acquisition.elapsedMs); Serial.print(",");
  Serial.print(r.matchLockTimeMs); Serial.print(",");
  Serial.print(r.totalRunTimeMs); Serial.print(",");
  Serial.print(r.fft.frequencyHz, 5); Serial.print(",");
  Serial.println(r.fft.snrDb, 2);
}

void runIndependentValidation() {
  Serial.println("\n####################################################################");
  Serial.println("V5.1R INDEPENDENT AMPLITUDE VALIDATION SERIES");
  Serial.println("Each run starts from a fresh actuator/NCO state.");
  Serial.println("####################################################################");
  printRunCsvHeader();

  uint8_t completed = 0;
  uint8_t ampPasses = 0;
  RunningStats matchTimeStats;
  RunningStats acquisitionTimeStats;
  resetRunningStats(matchTimeStats);
  resetRunningStats(acquisitionTimeStats);

  for (uint8_t run = 1; run <= independentRuns; run++) {
    CompleteRunResult r = runOneComplete(run, true);
    printRunCsv(run, r);
    if (r.completed) completed++;
    if (r.amplitudePass) ampPasses++;
    if (r.matchLockAchieved) {
      pushRunningStats(matchTimeStats, r.matchLockTimeMs / 1000.0);
      pushRunningStats(acquisitionTimeStats, r.acquisition.elapsedMs / 1000.0);
    }

    if (emergencyStopRequested) break;
    delay(500);
  }

  Serial.println("\nSERIES SUMMARY");
  Serial.print("Completed: "); Serial.print(completed); Serial.print(" / "); Serial.println(independentRuns);
  Serial.print("Amplitude PASS: "); Serial.print(ampPasses); Serial.print(" / "); Serial.println(independentRuns);
  if (matchTimeStats.n > 0) {
    double matchSd = runningSd(matchTimeStats);
    double matchCv = matchTimeStats.mean > 1.0e-12 ? 100.0 * matchSd / matchTimeStats.mean : 0.0;
    Serial.println("\nCALIBRATION / MATCH-TIME SUMMARY (successful locks only)");
    Serial.print("Successful locks: "); Serial.println(matchTimeStats.n);
    Serial.print("Acquisition time mean: "); Serial.print(acquisitionTimeStats.mean, 3); Serial.println(" s");
    Serial.print("Match+servo-lock mean: "); Serial.print(matchTimeStats.mean, 3); Serial.println(" s");
    Serial.print("Match+servo-lock SD: "); Serial.print(matchSd, 3); Serial.println(" s");
    Serial.print("Match+servo-lock CV: "); Serial.print(matchCv, 2); Serial.println(" %");
    Serial.print("Match+servo-lock min/max: "); Serial.print(matchTimeStats.minValue, 3);
    Serial.print(" / "); Serial.print(matchTimeStats.maxValue, 3); Serial.println(" s");
    Serial.println("CSV_MATCH_SERIES,SuccessfulLocks,AcqMean_s,MatchMean_s,MatchSD_s,MatchCV_pct,MatchMin_s,MatchMax_s");
    Serial.print("CSV_MATCH_SERIES,"); Serial.print(matchTimeStats.n); Serial.print(",");
    Serial.print(acquisitionTimeStats.mean, 3); Serial.print(",");
    Serial.print(matchTimeStats.mean, 3); Serial.print(",");
    Serial.print(matchSd, 3); Serial.print(",");
    Serial.print(matchCv, 2); Serial.print(",");
    Serial.print(matchTimeStats.minValue, 3); Serial.print(",");
    Serial.println(matchTimeStats.maxValue, 3);
  }
}

// ============================================================================
// MANUAL / DIAGNOSTIC COMMANDS
// ============================================================================

void runMatchOnly() {
  automaticTestRunning = true;
  emergencyStopRequested = false;
  targetAcquired = false;
  stopVisaton();
  clearSlopeHistory();

  AcquisitionResult acq;
  ServoState servo;
  uint32_t matchStartMs = millis();
  if (acquireAndServoLock(acq, servo)) {
    uint32_t matchTimeMs = millis() - matchStartMs;
    Serial.println("\nMATCH + SERVO LOCK SUCCESS.");
    Serial.print("Acquisition time: "); Serial.print(acq.elapsedMs / 1000.0, 3); Serial.println(" s");
    Serial.print("Match + servo-lock time: "); Serial.print(matchTimeMs / 1000.0, 3); Serial.println(" s");
    Serial.print("CSV_MATCH_ONLY,"); Serial.print(acq.elapsedMs); Serial.print(","); Serial.println(matchTimeMs);
    Serial.print("Visaton left running at DAC "); Serial.println(currentDacAmplitude, 2);
    Serial.println("Use k 30 to continue TRACK, g for FFT, or s to stop.");
  } else {
    Serial.println("MATCH/SERVO LOCK FAILED.");
    if (!emergencyStopRequested) stopVisaton();
  }
  automaticTestRunning = false;
}

void runTrackCurrent(uint32_t seconds) {
  if (!targetAcquired || !visatonRunning) {
    Serial.println("No acquired/running target. Run m first.");
    return;
  }

  automaticTestRunning = true;
  emergencyStopRequested = false;
  ServoState servo;
  resetServoState(servo);
  servo.plantBaseline = lockedPlantBaseline;

  HoldResult h = runClosedLoopHold(static_cast<uint8_t>(clampDouble(seconds, 5, 30)), servo);
  if (seconds > 30) {
    Serial.println("Note: k durations >30 s are executed in <=30 s blocks to keep summary counters bounded.");
    uint32_t remaining = seconds - 30;
    while (remaining > 0 && h.status == ServoStatus::COMPLETED && !emergencyStopRequested) {
      uint8_t block = static_cast<uint8_t>(remaining > 30 ? 30 : remaining);
      h = runClosedLoopHold(block, servo);
      remaining -= block;
    }
  }

  automaticTestRunning = false;
}

void runDiagnostic(double dac) {
  automaticTestRunning = true;
  emergencyStopRequested = false;

  Serial.println("\n============================================================");
  Serial.println("FIXED-DAC CANONICAL DIAGNOSTIC - NO SEARCH / NO TRACK");
  Serial.print("Frequency "); Serial.print(testFrequencyHz, 5);
  Serial.print(" Hz | DAC "); Serial.println(quantizeDac(dac), 2);
  Serial.println("============================================================");

  double oldDac = currentDacAmplitude;
  if (!setVisatonAmplitudeContinuous(dac)) {
    automaticTestRunning = false;
    return;
  }
  if (!interruptibleDelay(adaptiveSettleMs(oldDac, quantizeDac(dac)))) {
    automaticTestRunning = false;
    return;
  }

  for (uint8_t i = 1; i <= 3; i++) {
    ControlRecord r = measureControlRecord(dac, false);
    printControlRecord("DIAG", i, r);
  }
  automaticTestRunning = false;
}

void printIntegratedSettings() {
  Serial.println();
  Serial.println("--- INTEGRATED SETTINGS / STATUS ---");
  Serial.print("Hard-coded tool search band: "); Serial.print(DISC_SEARCH_MIN_HZ, 1);
  Serial.print(" .. "); Serial.print(DISC_SEARCH_MAX_HZ, 1); Serial.println(" Hz");
  Serial.print("ADXL1 role: TOOL / reference on GPIO"); Serial.println(PIN_CS_ADXL1);
  Serial.print("ADXL2 role: BACK-OF-HAND error sensor on GPIO"); Serial.println(PIN_CS_ADXL2);
  Serial.print("Raw clipping guard: +/-"); Serial.print(ADXL_RAW_CLIP_LIMIT); Serial.println(" counts");
  Serial.print("Baseline valid: "); Serial.println(baselineValid ? "YES" : "NO");
  if (baselineValid) {
    Serial.print("Baseline f0: "); Serial.print(baselineF0Hz, 6); Serial.println(" Hz");
    Serial.print("ADXL2 X/Y/Z @ f0: ");
    Serial.print(baselineHandAtF0.axis[0].amplitudeMs2, 5); Serial.print(" / ");
    Serial.print(baselineHandAtF0.axis[1].amplitudeMs2, 5); Serial.print(" / ");
    Serial.println(baselineHandAtF0.axis[2].amplitudeMs2, 5);
  }
  Serial.print("Axis selected: "); Serial.println(baselineAxisSelected ? "YES" : "NO");
  if (baselineAxisSelected) {
    Serial.print("Control axis: "); Serial.println(axisChar());
    Serial.print("Matcher frequency: "); Serial.print(testFrequencyHz, 6); Serial.println(" Hz");
    Serial.print("Matcher target: "); Serial.print(targetAmplitudeMs2, 6); Serial.println(" m/s^2 peak");
  }
  Serial.print("Visaton running: "); Serial.println(visatonRunning ? "YES" : "NO");
  if (visatonRunning) {
    Serial.print("Current/frozen DAC: "); Serial.println(currentDacAmplitude, 2);
    Serial.print("Actual NCO frequency: "); Serial.print(actualNcoFrequencyHz(), 7); Serial.println(" Hz");
  }
  Serial.print("Workflow state: ");
  switch (integratedWorkflowState) {
    case IntegratedWorkflowState::NEED_BASELINE: Serial.println("NEED_BASELINE"); break;
    case IntegratedWorkflowState::AWAIT_AXIS: Serial.println("AWAIT_AXIS"); break;
    case IntegratedWorkflowState::AWAIT_TOOL_OFF: Serial.println("AWAIT_TOOL_OFF_CONFIRMATION"); break;
    case IntegratedWorkflowState::MATCHING: Serial.println("MATCHING"); break;
    case IntegratedWorkflowState::FIXED_MATCH_RUNNING: Serial.println("FIXED_MATCH_RUNNING / PHASE_READY"); break;
  }
  Serial.println("------------------------------------");
}

void printIntegratedHelp() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("KK INTEGRATED NARROWBAND FFT/QUAD + V5.1R MATCHER V1.3A DRIFT-TOLERANT ASCII-SAFE");
  Serial.println("------------------------------------------------------------");
  Serial.println("a       Tool ON + Visaton OFF: acquire 240..320 Hz baseline");
  Serial.println("x/y/z   choose ADXL2 error-sensor axis after successful a");
  Serial.println("c       after switching TOOL OFF: run V5.1R match and freeze DAC");
  Serial.println("g       diagnostic local triaxial FFT while Visaton is running");
  Serial.println("p       print integrated settings/status");
  Serial.println("s       stop Visaton and reset workflow");
  Serial.println("?       help");
  Serial.println("------------------------------------------------------------");
  Serial.println("Normal order: a -> inspect data -> x/y/z -> TOOL OFF -> c");
  Serial.println("After c succeeds: fixed DAC + fixed f0 continue until s.");
  Serial.println("No b command. No manual candidate lock. No live post-match servo.");
  Serial.println("============================================================");
}

const char *skipSpacesIntegrated(const char *p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

void resetIntegratedWorkflowAfterStop() {
  stopVisaton();
  targetAcquired = false;
  clearSlopeHistory();
  baselineValid = false;
  baselineAxisSelected = false;
  baselineF0Hz = 0.0;
  frozenMatchDac = 0.0;
  frozenMatchMeasuredMs2 = 0.0;
  integratedWorkflowState = IntegratedWorkflowState::NEED_BASELINE;
}

void processCommand(const char *line) {
  line = skipSpacesIntegrated(line);
  if (*line == '\0') return;

  char c = static_cast<char>(tolower(static_cast<unsigned char>(*line)));
  const char *arg = skipSpacesIntegrated(line + 1);

  if (c == '?') {
    printIntegratedHelp();
    return;
  }

  if (c == 's' && *arg == '\0') {
    emergencyStopRequested = true;
    resetIntegratedWorkflowAfterStop();
    Serial.println("VISATON STOPPED. Integrated baseline cleared. Start next cycle with a.");
    return;
  }

  if (automaticTestRunning) {
    Serial.println("Automatic matching active. Only emergency stop s is accepted.");
    return;
  }

  if (c == 'a' && *arg == '\0') {
    emergencyStopRequested = false;
    runIntegratedDiscovery();
    return;
  }

  if ((c == 'x' || c == 'y' || c == 'z') && *arg == '\0') {
    selectIntegratedAxis(c);
    return;
  }

  if (c == 'c' && *arg == '\0') {
    emergencyStopRequested = false;
    runIntegratedAmplitudeMatch();
    return;
  }

  if (c == 'g' && *arg == '\0') {
    if (!visatonRunning) {
      Serial.println("g requires the Visaton NCO to be running. Complete c first.");
      return;
    }
    VectorFftResult fft = {};
    runLocalVectorFft(fft);
    printFftResult(fft);
    return;
  }

  if (c == 'p' && *arg == '\0') {
    printIntegratedSettings();
    return;
  }

  Serial.println("Unknown command. Type ? for help.");
}

// ============================================================================
// SETUP / LOOP - INTEGRATED CONTROLLER
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_CS_ADXL1, OUTPUT);
  pinMode(PIN_CS_ADXL2, OUTPUT);
  digitalWrite(PIN_CS_ADXL1, HIGH);
  digitalWrite(PIN_CS_ADXL2, HIGH);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  pinMode(PIN_VISATON_DAC, OUTPUT);
  dacWrite(PIN_VISATON_DAC, 128);

  Serial.println();
  Serial.println("================================================================================");
  Serial.println("KK INTEGRATED NARROWBAND FFT/QUAD + V5.1R AMPLITUDE MATCHER V1.3A DRIFT-TOLERANT ASCII-SAFE");
  Serial.println("ADXL1 GPIO5  = TOOL / reference");
  Serial.println("ADXL2 GPIO17 = BACK-OF-HAND / error sensor");
  Serial.println("Tool discovery band HARD-CODED to 240..320 Hz");
  Serial.println("5 MHz SPI Mode 3 | 1600 Hz paired acquisition | 10 kHz phase-continuous NCO");
  Serial.println("================================================================================");

  if (!initializeReferenceAdxl1()) {
    Serial.println("FATAL: ADXL1 initialization failed on GPIO5.");
    while (true) {
      dacWrite(PIN_VISATON_DAC, 128);
      delay(1000);
    }
  }

  if (!initializeAdxl2()) {
    Serial.println("FATAL: ADXL2 initialization failed on GPIO17.");
    while (true) {
      dacWrite(PIN_VISATON_DAC, 128);
      delay(1000);
    }
  }

  if (!initializeVisatonTimer()) {
    Serial.println("FATAL: hardware timer initialization failed.");
    while (true) {
      dacWrite(PIN_VISATON_DAC, 128);
      delay(1000);
    }
  }

  clearSlopeHistory();
  integratedWorkflowState = IntegratedWorkflowState::NEED_BASELINE;
  printIntegratedHelp();
  printIntegratedSettings();
  Serial.println("Ready. TOOL ON, VISATON OFF, then enter a.");
}

void loop() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      processCommand(commandBuffer);
      commandLength = 0;
      commandBuffer[0] = '\0';
    } else if (commandLength + 1 < COMMAND_BUFFER_SIZE) {
      commandBuffer[commandLength++] = c;
    }
  }
  delay(2);
}
