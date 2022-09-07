#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define PWM     9
Adafruit_MPU6050 mpu;

void setup(void) {
  

  
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Pause to open console
  }

  // Check MPU connection
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(PWM, OUTPUT);
  digitalWrite(PWM, HIGH);

  Serial.println("");
  delay(100);
}



void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g.gyro.y);
  Serial.println("");

  if (a.acceleration.x <= 0) {
    digitalWrite(PWM, LOW);
    } else{
    digitalWrite(PWM, HIGH);
  }
  delay(10);
  }
