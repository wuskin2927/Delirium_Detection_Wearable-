#include <Arduino_BMI270_BMM150.h>

extern void ble_send(const String &msg);

// Sleep globals
static const float sl_activityThreshold = 0.15;            // Threshold to classify low vs high activity for sleep detection
static const unsigned long sl_epochDuration = 10000;      // Duration of each activity evaluation window (10 seconds)
static unsigned long sl_lastEpochTime = 0;                // Timestamp of last epoch reset
static float sl_activitySum = 0;                          // Accumulated activity within the epoch
static int sl_sampleCount = 0;                            // Number of samples collected in this epoch
static String sl_status = "UNKNOWN";                      // Current sleep/wake status
static unsigned long sl_lastSleepDisruptionAlert = 0;     // Timestamp of last disruption alert sent
static const unsigned long sl_disruptionCooldown = 15000; // Minimum time between disruption alerts

void sleep_init() {
  sl_lastEpochTime = millis();                            // Initialize epoch timer
  sl_activitySum = 0;                                     // Reset activity accumulator
  sl_sampleCount = 0;                                     // Reset sample counter
  sl_status = "IDLE";                                     // Initial state
  Serial.println("Sleep actigraphy initialized");         // Debug message
}

String sl_getStatus() { return sl_status; }               // Return the current sleep status

void runSleepActigraphy() {
  float ax=0, ay=0, az=0;
  
  // Read accelerometer if available
  if(IMU.accelerationAvailable()){
    IMU.readAcceleration(ax,ay,az);                       // Read acceleration values
    float accMag = sqrt(ax*ax+ay*ay+az*az);               // Compute magnitude of acceleration vector
    float activity = fabs(accMag-1.0);                    // Remove gravity to get movement magnitude
    sl_activitySum += activity;                           // Add to epoch activity
    sl_sampleCount++;                                     // Increment sample count
  }

  unsigned long currentTime = millis();

  // Check if the epoch duration has passed
  if(currentTime-sl_lastEpochTime >= sl_epochDuration){
    float averageActivity=0;

    // Calculate average activity if samples exist
    if(sl_sampleCount>0) averageActivity = sl_activitySum/sl_sampleCount;

    // Classify the epoch as sleep or wake
    if(averageActivity < sl_activityThreshold){
      sl_status="SLEEP";                                  // Low activity = sleep
      Serial.println("Epoch: SLEEP");
    }
    else{
      sl_status="WAKE";                                   // High activity = awake
      Serial.println("Epoch: WAKE");

      // Trigger sleep disruption alert if cooldown period passed
      if(millis()-sl_lastSleepDisruptionAlert > sl_disruptionCooldown){
        String alert="ALERT: Sleep disruption (activity spike)\n";
        Serial.println(alert);
        ble_send(alert);                                   // Send alert over BLE
        sl_lastSleepDisruptionAlert=millis();              // Reset cooldown timer
      }
    }

    // Debug print for average activity in G units
    Serial.print("Avg activity (G's): ");
    Serial.println(averageActivity,3);

    // Reset epoch accumulators for next window
    sl_activitySum=0;
    sl_sampleCount=0;
    sl_lastEpochTime=currentTime;
  }
}
