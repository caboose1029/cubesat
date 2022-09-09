#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <QuickPID.h>   //PID https://github.com/Dlloydev/QuickPID/blob/master/examples/PID_Basic/PID_Basic.ino
Adafruit_MPU6050 mpu;

// Set PWM constants
const uint16_t PWM_FREQUENCY = 20000;
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2;

// define pins for Nidec 24H Motor
#define DIRECTION   7 
#define BRAKE       8
#define PWM         9

// Set variables for acceleration in y and z; input and output; bounds; PID
double accY, accZ;
double thetaIn;
double thetaOut;
const double bounds = 0.5;
double pwm = 400;
int dir = 0;

// Set variables for PID
float Setpoint, Input, Output;
float Kp = 2, Ki = 0.5, Kd = 0.5;
//float Kp=1,Ki=0,Kd=0;
//Specify PID links
QuickPID myPID(&Input, &Output, &Setpoint);


void setup(){
  
  // Set PWM frequency to 20kHz
  TCCR1B = (1 << WGM13) | (1 << CS10);
  ICR1 = PWMVALUE;
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
  setPWM(400);

  // Set pin modes & set initial values
  pinMode(DIRECTION, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(PWM, OUTPUT);
  digitalWrite(DIRECTION, LOW);
  digitalWrite(BRAKE, HIGH);
  analogWrite(PWM, 400);

  // Begin communication with MPU6050
  Serial.begin(1000000);
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

  // PID Setup
  //initialize the variables we're linked to
  Input = thetaIn;
  Setpoint = 0;

  //Set PID output limits to +/- 400
  myPID.SetOutputLimits(-400, 400);

  //apply PID gains
  myPID.SetTunings(Kp, Ki, Kd);

  //turn the PID on
  myPID.SetMode(myPID.Control::automatic);
}

void loop(){
  delay(10);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accZ = a.acceleration.z;
  accY = a.acceleration.y;
  thetaIn = atan2(accY, accZ);
  thetaIn = thetaIn * 57.2958;

  if (thetaIn > -bounds && thetaIn < bounds){
    
    pwm = 400;

  }

  //float delta = map(thetaIn, -45, 45, -45, 0);

  Input = thetaIn;
  myPID.Compute();
  //pwm = map(Output, 0, 400, 400, 0);
  pwm=map(abs(Output),0,400,400,0);
  
  if (Output > 0){
    dir = 0;  
  }
  else{
    dir = 1;
  }

  driveMotor(pwm, dir);
  
  Serial.print("Angle: ");
  Serial.print(thetaIn);
  Serial.print(", PWM: ");
  Serial.print(pwm);
  Serial.print(", Direction:");
  Serial.print(dir);
  Serial.print(", Output");
  Serial.print(Output);
  Serial.println("");
  
}

void driveMotor(float pwm, int dir){
  if(dir == 0){

    digitalWrite(DIRECTION, LOW);
    analogWrite(PWM, pwm);

  }

  else if(dir == 1){

    digitalWrite(DIRECTION, HIGH);
    analogWrite(PWM, pwm);

  }
}

void setPWM(uint16_t dutyCycle) { // dutyCycle is a value between 0-ICR1
    OCR1A = dutyCycle;
}