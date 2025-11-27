#include <Arduino_BMI270_BMM150.h>

// BLE send function defined elsewhere
extern void ble_send(const String &msg);

// Movement globals
static const float mv_movementThreshold = 0.75;     // Mean movement required to trigger ERRATIC
static const float mv_variabilityThreshold = 0.35;  // Std deviation required to trigger ERRATIC
static const float mv_hypoMeanThreshold = 0.05;     // Very low mean movement for HYPOACTIVITY
static const float mv_hypoVarThreshold = 0.05;      // Very low variability for HYPOACTIVITY
static const unsigned long mv_hypoRequiredTime = 300000; // Time in hypo state before alert (5 min)
static unsigned long mv_hypoStartTime = 0;           // Timestamp when hypo timer started
static bool mv_hypoTimerRunning = false;             // Tracks whether hypo timer is active

// Sampling window parameters
static const int mv_windowSize = 20;                 // Number of samples in window
static const int mv_sampleDelay = 100;               // ms between samples

// Circular buffer for delta movement values
static float mv_movementWindow[mv_windowSize];
static int mv_windowIndex = 0;                       // Current index in window
static bool mv_fullWindow = false;                   // True once buffer has wrapped once
static unsigned long mv_lastSampleTime = 0;          // Timestamp of last sample

static String mv_status = "UNKNOWN";                 // Current movement status

// --- Public interface ---
void movement_init() {
  mv_windowIndex = 0;
  mv_fullWindow = false;
  mv_lastSampleTime = millis();
  for(int i=0;i<mv_windowSize;i++) mv_movementWindow[i]=0;  // Clear movement buffer

  pinMode(LED_BUILTIN, OUTPUT);                      // LED used for alerts
  mv_status="IDLE";                                  // Initial state
  Serial.println("Movement module initialized");
}

String mv_getStatus() { return mv_status; }          // Return current movement state

void runMovement() {
  // Enforce sampling rate
  if(millis()-mv_lastSampleTime < mv_sampleDelay) return;
  mv_lastSampleTime = millis();

  // Previous and current accelerometer readings
  static float lastX=0, lastY=0, lastZ=0;
  float x=0,y=0,z=0;

  // Ensure IMU has new data
  if(IMU.accelerationAvailable()) IMU.readAcceleration(x,y,z);
  else return;

  // Compute delta movement (magnitude of difference)
  float delta = sqrt(pow(x-lastX,2)+pow(y-lastY,2)+pow(z-lastZ,2));

  // Add delta to circular window buffer
  mv_movementWindow[mv_windowIndex] = delta;
  mv_windowIndex = (mv_windowIndex+1)%mv_windowSize;
  if(mv_windowIndex==0) mv_fullWindow=true;          // Buffer is full after wrap

  // Process movement only when buffer is full
  if(mv_fullWindow){
    // Calculate mean movement
    float sum=0;
    for(int i=0;i<mv_windowSize;i++) sum+=mv_movementWindow[i];
    float mean=sum/mv_windowSize;

    // Calculate standard deviation (variability)
    float sqSum=0;
    for(int i=0;i<mv_windowSize;i++) sqSum+=pow(mv_movementWindow[i]-mean,2);
    float stddev = sqrt(sqSum/mv_windowSize);

    // ----- ERRATIC MOVEMENT DETECTION -----
    if(mean>mv_movementThreshold && stddev>mv_variabilityThreshold){
      mv_status="ERRATIC";
      digitalWrite(LED_BUILTIN,HIGH);                 // LED ON for alert
      String alert="ALERT: Erratic Movement Detected!\n";
      Serial.println(alert);
      ble_send(alert);                                // Send BLE alert
    }

    // ----- HYPOACTIVITY DETECTION -----
    else if(mean < mv_hypoMeanThreshold && stddev < mv_hypoVarThreshold) {

      // Start hypoactivity timer if not already running
      if(!mv_hypoTimerRunning) {
        mv_hypoTimerRunning = true;
        mv_hypoStartTime = millis();
        mv_status = "HYPO_TIMER_STARTED";             // Indicate timer started
      }

      unsigned long elapsed = millis() - mv_hypoStartTime;

      // If inactivity persisted long enough → hypoactivity alert
      if(elapsed >= mv_hypoRequiredTime) {
          mv_status = "HYPOACTIVITY";
          digitalWrite(LED_BUILTIN, HIGH);            // LED ON for alert
          String alert = "ALERT: Hypoactivity Detected!\n";
          Serial.println(alert);
          ble_send(alert);
      }
    }

    // ----- NORMAL MOVEMENT -----
    else {
      mv_hypoTimerRunning = false;                    // Reset hypo timer
      mv_status = "NORMAL";                           // Back to normal state
      digitalWrite(LED_BUILTIN, LOW);                 // LED OFF
    }
  }

  // Update previous acceleration values
  lastX=x; lastY=y; lastZ=z;
}
