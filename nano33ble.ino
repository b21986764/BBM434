#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Wire.h>

float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second) 
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps 
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

long lastTime;
long lastInterval;
BLEService customService("12345678-1234-1234-1234-123456789ABC");
BLEStringCharacteristic accCharacteristic("87654321-4321-4321-4321-210987654321", BLENotify, 20);
BLEStringCharacteristic gyroCharacteristic("87654321-4321-4321-4321-210987654322", BLENotify, 20);
BLEStringCharacteristic magCharacteristic("87654321-4321-4321-4321-210987654323", BLENotify, 20);
BLEStringCharacteristic rollCharacteristic("87654321-4321-4321-4321-210987654324", BLENotify, 20);
BLEStringCharacteristic pitchCharacteristic("87654321-4321-4321-4321-210987654325", BLENotify, 20);
BLEStringCharacteristic yawCharacteristic("87654321-4321-4321-4321-210987654326", BLENotify, 20);
void setup() {

  Serial.begin(9600);
  
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  if (!BLE.begin()) {
        Serial.println("Failed to initialize BLE!");
        while (1);
  }

  calibrateIMU(250, 250);

  lastTime = micros();
    BLE.setLocalName("Nano33BLE");
    BLE.setAdvertisedService(customService);
    customService.addCharacteristic(accCharacteristic);
    customService.addCharacteristic(gyroCharacteristic);
    customService.addCharacteristic(magCharacteristic);
    customService.addCharacteristic(rollCharacteristic);
    customService.addCharacteristic(pitchCharacteristic);
    customService.addCharacteristic(yawCharacteristic);
    BLE.addService(customService);
    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");

}

/* 
  the gyro's x,y,z values drift by a steady amount. if we measure this when arduino is still 
  we can correct the drift when doing real measurements later
*/
void calibrateIMU(int delayMillis, int calibrationMillis) {

  int calibrationCount = 0;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;

}

/** 
 * Read accel and gyro data.   
 * returns true if value is 'new' and false if IMU is returning old cached data
 */

void loop() {
    static unsigned long lastBleSendTime = 0;
    const unsigned long bleSendInterval = 1000; // adjust this interval as needed

    if (BLE.connected()) {
        float  mx, my, mz;

        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
            IMU.readAcceleration(accelX, accelY, accelZ);
            IMU.readGyroscope(gyroX, gyroY, gyroZ);
            IMU.readMagneticField(mx, my, mz);

            long currentTime = micros();
            lastInterval = currentTime - lastTime;
            lastTime = currentTime;

            doCalculations();

            // Check if it's time to send data
            if (millis() - lastBleSendTime >= bleSendInterval) {
                String accData = "a:" + String(accelX, 2) + "," + String(accelY, 2) + "," + String(accelZ, 2);
                String gyroData = "g:" + String(gyroX, 2) + "," + String(gyroY, 2) + "," + String(gyroZ, 2);
                String magData = "m:" + String(mx, 2) + "," + String(my, 2) + "," + String(mz, 2);
                String rollData = "r:" + String(complementaryRoll, 2) ;
                String pitchData = "p:" + String(complementaryPitch, 2) ;
                String yawData = "y:" + String(complementaryYaw, 2);
                
                accCharacteristic.writeValue(accData);
                gyroCharacteristic.writeValue(gyroData);
                magCharacteristic.writeValue(magData);
                rollCharacteristic.writeValue(rollData);
                pitchCharacteristic.writeValue(pitchData);
                yawCharacteristic.writeValue(yawData);
                Serial.println(rollData);
                Serial.println(pitchData);
                Serial.println(yawData);

                lastBleSendTime = millis();
            }
        }
    }
}

/**
 * I'm expecting, over time, the Arduino_LSM6DS3.h will add functions to do most of this,
 * but as of 1.0.0 this was missing.  
 */
void doCalculations() {

  accRoll = atan2(accelY, accelZ) * 180/M_PI;
  accPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180/M_PI;

  float lastFrequency = (float)1000000 / lastInterval;


    gyroRoll = gyroRoll + (gyroX/lastFrequency);
  gyroPitch = gyroPitch + (gyroY/lastFrequency);
  gyroYaw = gyroYaw + (gyroZ/lastFrequency);


  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX)/lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY)/lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ)/lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX)/lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY)/lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ)/lastFrequency);

  complementaryRoll = 0.98*complementaryRoll + 0.02*accRoll;
  complementaryPitch = 0.98*complementaryPitch + 0.02*accPitch;
}