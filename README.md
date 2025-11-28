Wearable detects hospital induced delirium
- Monitors sleep disruptions, can send alerts 
- Monitors hypoactivity and hyperactivity, can send alerts
- Monitors heart rate and SpO2 levels
- Can toggle between detecting activity and
  sleep disruptions to eliminate alerts for
  hypoactivity during sleep
- Outputs data and alerts to another device via bluetooth

<img src="https://github.com/user-attachments/assets/498c9962-c268-4598-ac61-8c4b27d3e921" width="280">
<img src="https://github.com/user-attachments/assets/60322051-1faf-402c-ad6d-b3ffcf89188a" width="280">
<img src="https://github.com/user-attachments/assets/e14d5cb6-344f-4d48-97f4-dd609696ff68" width="280">


Hardware 
- Arduino Nano BLE 33 Rev2 (Built in BLE and IMU)
- MAX30102 Pulse Oximeter

Wiring
- Sensor VIN to Arduino 3V3 output
- Sensor GND to Arduino GND
- Sensor SCL to Arduino A5
- Sensor SDA to Arduino A4

Code Breakdown
- main.ino contains variable and function declarations; prints data to terminals
- HeartRate.ino determines heart rate and SpO2 from pulse oximeter data
- MovementMode.ino monitors activity while patient is awake
- SleepMode.ino monitors sleep disruptions
- BLEControl.ino manages BLE connection
- A_BLEGlobals.ino sets up UART
