// Main file with sleep/hypo mode, readable telemetry
// Main file
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

// --- Extern globals and functions ---
// These are defined in other files and imported here
extern BLEService uartService;
extern BLEStringCharacteristic txChar;
extern BLEStringCharacteristic rxChar;

extern void ble_init();                 // Initialize BLE module
extern void ble_send(const String &msg); // Send string over BLE UART

extern void hr_init();                 // Initialize heart-rate subsystem
extern void hr_update();               // Update HR/SpO2 sensor sampling
extern double hr_getLastBPM();         // Return last detected BPM
extern String hr_getHRVString();       // Return HRV information as readable text
extern double hr_getSpO2();            // Return last SpO2 estimate

extern void movement_init();           // Initialize hypoactivity/movement detection
extern void runMovement();             // Run movement processing loop
extern String mv_getStatus();          // Return movement/hypoactivity status text

extern void sleep_init();              // Initialize sleep actigraphy module
extern void runSleepActigraphy();      // Run sleep movement classifier
extern String sl_getStatus();          // Return sleep status text

// --- Mode control ---
// sleepModeActive = true → sleep actigraphy running
// hypoModeActive = true → hypoactivity/movement detection running
bool sleepModeActive = false;
bool hypoModeActive = true;

// --- Telemetry timer ---
// Used to send HR/SpO2/movement/sleep information every few seconds
unsigned long lastTelemetryTime = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial); // Wait for serial monitor to open

  ble_init(); // Start BLE communications

  // Initialize the IMU (BMI270)
  if(!IMU.begin()){
    Serial.println("Failed to initialize IMU!");
    while(1); // Halt if IMU fails
  }
  Serial.println("IMU initialized!");

  // Initialize all sensor modules
  hr_init();
  movement_init();
  sleep_init();

  Serial.println("Type 'sleep' to enable sleep mode, 'awake' to enable hypoactivity mode.");
}

void loop() {
  // --- Handle Serial input for mode switching ---
  if (Serial.available() > 0) {
    // Read text command from serial
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Switch to sleep mode
    if (cmd.equalsIgnoreCase("sleep")) {
      sleepModeActive = true;
      hypoModeActive = false;
      Serial.println("Sleep mode enabled. Hypoactivity mode disabled.");
    }
    // Switch to hypoactivity mode
    else if (cmd.equalsIgnoreCase("awake")) {
      sleepModeActive = false;
      hypoModeActive = true;
      Serial.println("Hypoactivity mode enabled. Sleep mode disabled.");
    }
  }

  // --- Update heart rate module as fast as possible ---
  // This continuously samples the MAX30102 and performs filtering/FFT/etc.
  hr_update();

  // --- Run only the currently selected module ---
  if (sleepModeActive) runSleepActigraphy(); // Sleep movement classification
  if (hypoModeActive) runMovement();         // Hypoactivity/erratic movement detection

  // --- Send telemetry every 5 seconds ---
  if (millis() - lastTelemetryTime >= 5000) {
    lastTelemetryTime = millis();

    // --- Serial output (readable for debugging) ---
    Serial.print("HR: "); 
    Serial.print(hr_getLastBPM(), 1); 
    Serial.println(" BPM");

    

    Serial.print("SpO2: "); 
    Serial.print(hr_getSpO2(), 1); 
    Serial.println("%");

    // Only print sleep mode status when sleep is active
    if (sleepModeActive) 
      Serial.println("Sleep status: " + sl_getStatus());

    Serial.print("\n");

    // --- Build BLE telemetry packet (multi-line string) ---
    String telemetry = "";
    telemetry += "HR: " + String(hr_getLastBPM(), 1) + " BPM\n";
    telemetry += "SpO2: " + String(hr_getSpO2(), 1) + "%\n";
    if (sleepModeActive) 
      telemetry += "Sleep: " + sl_getStatus() + "\n";

    Serial.print("\n");

    // Send telemetry packet over BLE UART characteristic
    ble_send(telemetry);
  }

  delay(10); // Small delay prevents CPU overload and allows BLE background handling
}


