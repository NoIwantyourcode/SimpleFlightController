#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_BMP5xx.h>
#include "ICM45605.h"

Adafruit_BMP5xx bmp;
ICM456xx imu(SPI, 10, 1000000); // SPI, CS pin = 10, 1 MHz (change pin number)

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  SPI.begin();

  // Initialize BMP580
  if (!bmp.begin(BMP5XX_DEFAULT_ADDRESS, &Wire)) {
    Serial.println("BMP580 not found!");
    while (1);
  }

  bmp.setTemperatureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_8X);
  bmp.setPressureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff((bmp5xx_iir_filter_t)BMP5_IIR_FILTER_COEFF_3);

  // Initialize IMU
  if (imu.begin() != 0) {
    Serial.println("ICM-45605 init failed!");
    while (1);
  }
  imu.startAccel(1000, 16);
  imu.startGyro(1000, 2000);

  servo1.attach(3); //change pin number
  servo2.attach(5); //change pin number

  Serial.println("Setup complete.");
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to read BMP580!");
    return;
  }
  float altitude = bmp.readAltitude(1013.25);

  inv_imu_sensor_data_t imuData;
  imu.getDataFromRegisters(imuData);

  float ax = imuData.accel_data[0];
  float ay = imuData.accel_data[1];
  float az = imuData.accel_data[2];

  float roll = atan2(ay, az) * 57.3;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.3;

  int servo1Angle = constrain(90 + roll, 0, 180);
  int servo2Angle = constrain(90 + pitch, 0, 180);

  servo1.write(servo1Angle);
  servo2.write(servo2Angle);

  Serial.print("Alt: "); Serial.print(altitude);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.println(pitch);

  delay(50);
}
