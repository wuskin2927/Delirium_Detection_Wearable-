// Heart Rate and SpO2 module
#include <DFRobot_MAX30102.h>
#include <Arduino_BMI270_BMM150.h>
#include <arduinoFFT.h>

extern void ble_send(const String &msg);   // BLE send function from main code

// --- MAX30102 sensor ---
static DFRobot_MAX30102 particleSensor;    // Heart rate / SpO2 sensor object

// --- Sampling parameters ---
#define HR_SAMPLING_FREQUENCY 100  // Sampling rate in Hz
#define HR_SAMPLES 256             // FFT / SpO2 buffer size
const int hr_samplePeriod = 1000 / HR_SAMPLING_FREQUENCY; // Sampling interval (ms)

// --- Buffers ---
static double hr_vReal[HR_SAMPLES];        // IR LED samples (filtered)
static double hr_vImag[HR_SAMPLES];        // FFT imaginary part (unused)
static double hr_redBuffer[HR_SAMPLES];    // Red LED samples
static int hr_sampleCounter = 0;           // Sample index
static unsigned long hr_lastSampleTime = 0; // Last sampling timestamp

// --- LMS filter for motion artifact removal ---
#define HR_LMS_MU 0.0001                    // Learning rate
#define HR_LMS_ORDER 4                      // Filter order
static double hr_w[HR_LMS_ORDER] = {0};     // LMS weights
static double hr_x[HR_LMS_ORDER] = {0};     // Input delay line
static double hr_y_hat = 0;                 // Predicted value
static double hr_e = 0;                     // Error output

// --- Beat detection ---
#define HR_MAX_BEATS 50                     // Max RR interval buffer size
static unsigned long hr_lastBeatTime = 0;   // Timestamp of previous beat
static double hr_rrIntervals[HR_MAX_BEATS]; // RR interval history
static int hr_rrCount = 0;                  // Number of stored RR intervals
static double hr_thresholdIR = 0;           // Adaptive IR threshold

// --- Last values ---
static double hr_lastBPM = 0;               // Last computed BPM
static double hr_lastSpO2 = 0;              // Last SpO2 %
static double hr_lastSDNN = 0;              // HRV SDNN
static double hr_lastRMSSD = 0;             // HRV RMSSD
static double hr_lastPNN50 = 0;             // HRV pNN50

// --- Functions ---
void hr_init() {
  Serial.println("Initializing MAX30102...");

  // Loop until sensor initializes successfully
  while (!particleSensor.begin()) {
    Serial.println("MAX30102 not found! Check wiring.");
    delay(1000);
  }
  Serial.println("MAX30102 initialized!");

  // Configure sensor parameters
  particleSensor.sensorConfiguration(60, SAMPLEAVG_8, MODE_MULTILED,
                                     SAMPLERATE_100, PULSEWIDTH_411, ADCRANGE_16384);

  // Reset counters and buffers
  hr_sampleCounter = 0;
  hr_lastSampleTime = millis();
  hr_rrCount = 0;
  hr_thresholdIR = 0;

  for (int i = 0; i < HR_SAMPLES; i++)
    hr_vReal[i] = hr_vImag[i] = hr_redBuffer[i] = 0;
}

// LMS filter (adaptive motion artifact suppression)
static double hr_lmsFilter(double desired, double reference) {
  // Shift delay line
  for (int i = HR_LMS_ORDER - 1; i > 0; i--) hr_x[i] = hr_x[i - 1];
  hr_x[0] = reference; // new reference = motion value

  // Predicted signal = Σ(w[i] * x[i])
  hr_y_hat = 0;
  for (int i = 0; i < HR_LMS_ORDER; i++) hr_y_hat += hr_w[i] * hr_x[i];

  // Error = IR - predicted motion
  hr_e = desired - hr_y_hat;

  // Update filter weights
  for (int i = 0; i < HR_LMS_ORDER; i++) hr_w[i] += 2 * HR_LMS_MU * hr_e * hr_x[i];

  return hr_e; // filtered IR signal
}

// Beat detection with motion rejection
static bool hr_detectBeat(double sample, double motion) {
  if (motion > 0.2) return false; // Ignore beats if too much motion

  static double prev = 0;         // Previous IR value
  static bool rising = false;     // Track rising edge
  double diff = sample - prev;    // Slope
  hr_thresholdIR = hr_thresholdIR * 0.99 + sample * 0.01; // Adaptive baseline

  if (diff > 0) rising = true; // still rising

  // Peak detection: slope flips + above threshold
  if (rising && diff < 0 && sample > hr_thresholdIR + 20) {
    rising = false;
    prev = sample;
    return true; // beat detected
  }
  prev = sample;
  return false;
}

// Calculate HRV metrics from RR intervals
static void hr_calculateHRV() {
  if (hr_rrCount < 3) return; // not enough data

  int start = hr_rrCount > 20 ? hr_rrCount - 20 : 0; // last 20 beats
  int n = hr_rrCount - start;

  // SDNN (standard deviation of RR)
  double mean = 0;
  for (int i = start; i < hr_rrCount; i++) mean += hr_rrIntervals[i];
  mean /= n;

  double sdnn = 0;
  for (int i = start; i < hr_rrCount; i++) {
    double d = hr_rrIntervals[i] - mean;
    sdnn += d * d;
  }
  sdnn = sqrt(sdnn / n);

  // RMSSD (difference between adjacent intervals)
  double sumSq = 0;
  for (int i = start+1; i < hr_rrCount; i++) {
    double d = hr_rrIntervals[i] - hr_rrIntervals[i - 1];
    sumSq += d * d;
  }
  double rmssd = sqrt(sumSq / (n-1));

  // pNN50 (% of RR differences > 50 ms)
  int count50 = 0;
  for (int i = start+1; i < hr_rrCount; i++) {
    if (fabs(hr_rrIntervals[i] - hr_rrIntervals[i-1]) > 50) count50++;
  }
  double pnn50 = 100.0 * count50 / (n-1);

  hr_lastSDNN = sdnn;
  hr_lastRMSSD = rmssd;
  hr_lastPNN50 = pnn50;
}

// Calculate SpO2 using ratio of ratios
static void hr_calculateSpO2() {
  double redMax = hr_redBuffer[0], redMin = hr_redBuffer[0], redMean = 0;
  double irMax = hr_vReal[0], irMin = hr_vReal[0], irMean = 0;

  // Scan buffer for min, max, and mean
  for (int i = 0; i < HR_SAMPLES; i++) {
    redMean += hr_redBuffer[i];
    irMean += hr_vReal[i];
    if (hr_redBuffer[i] > redMax) redMax = hr_redBuffer[i];
    if (hr_redBuffer[i] < redMin) redMin = hr_redBuffer[i];
    if (hr_vReal[i] > irMax) irMax = hr_vReal[i];
    if (hr_vReal[i] < irMin) irMin = hr_vReal[i];
  }

  redMean /= HR_SAMPLES;
  irMean /= HR_SAMPLES;

  double acRed = redMax - redMin;  // AC component
  double acIR = irMax - irMin;    // AC component

  // Prevent division by zero or invalid data
  if (irMean == 0 || redMean == 0 || acIR == 0) {
    hr_lastSpO2 = 0;
    return;
  }

  // Ratio of ratios
  double R = (acRed / redMean) / (acIR / irMean);

  // Estimate SpO2 (empirical)
  hr_lastSpO2 = 110 - 12 * R;

  // Clamp to valid physiological range
  hr_lastSpO2 = constrain(hr_lastSpO2, 70, 100);
}

// --- Public getters ---
double hr_getLastBPM() { return hr_lastBPM; }
double hr_getSpO2() { return hr_lastSpO2; }
String hr_getHRVString() {
  return "HRV SDNN: " + String(hr_lastSDNN, 2) + " ms, RMSSD: " + String(hr_lastRMSSD, 2) + " ms, pNN50: " + String(hr_lastPNN50, 2) + " %";
}

// --- Main update function ---
void hr_update() {
  // Enforce fixed sampling rate
  if (millis() - hr_lastSampleTime <= hr_samplePeriod) return;
  hr_lastSampleTime = millis();

  // Read MAX30102 samples
  double ir = particleSensor.getIR();
  double red = particleSensor.getRed();

  // Motion from IMU acceleration magnitude
  float ax=0, ay=0, az=0;
  double motion = 0;
  if(IMU.accelerationAvailable()){
    IMU.readAcceleration(ax, ay, az);
    motion = sqrt(ax*ax + ay*ay + az*az);
    motion = fabs(motion - 1.0); // remove gravity
  }

  // Apply LMS filter → motion artifact reduction
  double filteredIR = hr_lmsFilter(ir, motion);

  // Beat detection using filtered IR
  if(hr_detectBeat(filteredIR, motion)){
    unsigned long now = millis();

    // If previous beat exists → compute RR interval
    if(hr_lastBeatTime > 0){
      double rr = now - hr_lastBeatTime;

      // Store RR interval (with circular buffer behavior)
      if(hr_rrCount < HR_MAX_BEATS) hr_rrIntervals[hr_rrCount++] = rr;
      else {
        for(int i=1;i<HR_MAX_BEATS;i++) hr_rrIntervals[i-1]=hr_rrIntervals[i];
        hr_rrIntervals[HR_MAX_BEATS-1]=rr;
      }

      // BPM = 60,000 ms per minute / RR interval
      hr_lastBPM = 60000.0 / rr;
    }
    hr_lastBeatTime = now;

    // Recalculate HRV every 5 beats and send via BLE
    if(hr_rrCount >= 5 && hr_rrCount % 5 == 0){
      hr_calculateHRV();
      ble_send(hr_getHRVString() + "\n");
    }
  }

  // Save samples for SpO2 calculation
  hr_vReal[hr_sampleCounter] = filteredIR;
  hr_redBuffer[hr_sampleCounter] = red;
  hr_vImag[hr_sampleCounter] = 0;

  hr_sampleCounter++;

  // When full buffer collected → compute SpO2
  if(hr_sampleCounter >= HR_SAMPLES){
    hr_calculateSpO2();
    hr_sampleCounter = 0;
  }
}

