
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP5xx.h>
#include "ICM45605.h"

Adafruit_BMP5xx bmp;
ICM456xx imu(SPI, 10, 1000000); // SPI, CS pin, 1 MHz

void setup() {
  Serial.begin(115200);
  Wire.begin();
  SPI.begin();

  // BMP580 init
  if (!bmp.begin(BMP5XX_DEFAULT_ADDRESS, &Wire)) {
    Serial.println("BMP580 init failed!");
    while (1);
  }
  bmp.setTemperatureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_8X);
  bmp.setPressureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff((bmp5xx_iir_filter_t)BMP5_IIR_FILTER_COEFF_3);

  // ICM-45605 init
  if (imu.begin() != 0) {
    Serial.println("ICM-45605 init failed!");
    while (1);
  }
  imu.startAccel(1000, 16);
  imu.startGyro(1000, 2000);

  Serial.println("Setup complete.");
}

void loop() {
  bmp.performReading();
  float altitude = bmp.readAltitude(1013.25);

  inv_imu_sensor_data_t data;
  imu.getDataFromRegisters(data); // Pass by reference, not pointer

  float ax = data.accel_data[0];
  float ay = data.accel_data[1];
  float az = data.accel_data[2];

  float roll = atan2(ay, az) * 57.3;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.3;

  Serial.print("Alt: "); Serial.print(altitude);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.println(pitch);

  delay(20);
}
