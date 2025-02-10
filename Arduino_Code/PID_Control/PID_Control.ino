#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <math.h>
using namespace BLA;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//PWM Setup 
#define PWM_freq 5000
#define PWM_resolution 12

//Motor Setup 
//Motor #1 
float pwm_1;     
int duration_1 = 0;
boolean Direction_1;
byte encoder0PinALast_1;
const byte encoder0pinA_1 = 36; //35; // Pin A
const byte encoder0pinB_1 = 37; //14; // Pin B
float omega_1 ; 
#define Motor1_dir_1 33
#define Motor1_dir_2 34
#define PWM_pin_1 35
#define PWM_channel_1 0
//new_encoder_setup 
volatile uint8_t lastStateAB_1 = 0;
volatile uint8_t currentStateAB_1 = 0;
volatile int pulseCount_1 = 0;
volatile int pulseCount_distance_1 = 0;
volatile bool direction1 = true;

//Motor #2       Motor 1 
float pwm_2 ;
int duration_2 = 0;
boolean Direction_2;
byte encoder0PinALast_2;
const byte encoder0pinA_2 = 7;//32; // Pin A
const byte encoder0pinB_2 = 16;//5; // Pin B
float omega_2 ; 
#define Motor2_dir_1 4
#define Motor2_dir_2 5
#define PWM_pin_2 6
#define PWM_channel_2 1
//new_encoder_setup 
volatile uint8_t lastStateAB_2 = 0;
volatile uint8_t currentStateAB_2 = 0;
volatile int pulseCount_2 = 0;
volatile bool direction2 = true;
volatile int pulseCount_distance_2 = 0;


//Motor #3
float pwm_3 ;
int duration_3 = 0;
boolean Direction_3;
byte encoder0PinALast_3;
const byte encoder0pinA_3 = 41;//32; // Pin A
const byte encoder0pinB_3 = 42;//5; // Pin B     motor 2
float omega_3 ; 
#define Motor3_dir_1 38
#define Motor3_dir_2 39 
#define PWM_pin_3 40
#define PWM_channel_3 2
//new_encoder_setup 
volatile uint8_t lastStateAB_3 = 0;
volatile uint8_t currentStateAB_3 = 0;
volatile int pulseCount_3 = 0;
volatile bool direction3 = true;
volatile int pulseCount_distance_3 = 0;

double tiltX = 0;
double tiltY = 0;
double tiltZ = 0;
double Error = 0;
double threshold = 7;


double rollOffset = 0 ; 
double yawOffset = 0;
double pitchOffset = 0;
double pwmOffset = 0.075;

//added line 
double angleX = 0;
double angleY = 0;
double alpha = 1;
unsigned long lastTime = 0;
unsigned long time_interval = 0.02;


// PWM Control
// double Kp = 900, Ki = 0, Kd = 0; //800 3 100
double consKp= 750,  consKi= 0, consKd= 50 ;
double aggKp = 760,  aggKi = 0, aggKd = 50 ; 

double ux, uy;
double setpointX = 0, setpointY = 0;

PID pidX(&angleX, &ux, &setpointX, consKp, consKi, consKd, DIRECT);
PID pidY(&angleY, &uy, &setpointY, consKp, consKi, consKd, DIRECT);

void setup() {
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW); 
  digitalWrite(10,LOW);
  //Motor 1 setup
  pinMode(Motor1_dir_1, OUTPUT);
  pinMode(Motor1_dir_2, OUTPUT);
  digitalWrite(Motor1_dir_1, HIGH);
  digitalWrite(Motor1_dir_2, LOW);
  ledcAttach(PWM_pin_1, PWM_freq, PWM_resolution);

  //Motor 2 setup
  pinMode(Motor2_dir_1, OUTPUT);
  pinMode(Motor2_dir_2, OUTPUT);
  digitalWrite(Motor2_dir_1, HIGH);
  digitalWrite(Motor2_dir_2, LOW);
  ledcAttach(PWM_pin_2, PWM_freq, PWM_resolution);

  //Motor 3 setup 
  pinMode(Motor3_dir_1, OUTPUT);
  pinMode(Motor3_dir_2, OUTPUT);
  digitalWrite(Motor3_dir_1, HIGH);
  digitalWrite(Motor3_dir_2, LOW);
  ledcAttach(PWM_pin_3, PWM_freq, PWM_resolution);

  //PID SETUP
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-15000, 15000);  // Example: for 12-bit PWM
  pidY.SetOutputLimits(-15000, 15000);
  pidX.SetSampleTime(20);
  pidY.SetSampleTime(20);

  Serial.begin(115200);

  //IMU SETUP 
    //IMU SETUP
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

  sensors_event_t orientationData;
  sensors_event_t linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  rollOffset = orientationData.orientation.x;
  pitchOffset = orientationData.orientation.z;
  yawOffset = orientationData.orientation.y;

  lastTime = millis();

}


void loop() {
  unsigned long currentTime = millis();
  double dt = ( currentTime - lastTime)/1000.0;

  if (dt >= time_interval){
    lastTime = currentTime;


  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  
  tiltY = orientationData.orientation.z - pitchOffset;  //theta_x  in deg
  tiltX = -orientationData.orientation.y + yawOffset;   //theta_y  in deg
  tiltZ = orientationData.orientation.x;
  if (tiltZ < 180) {
    tiltZ = -tiltZ;
  } else {
    tiltZ = 360 - tiltZ;
  }

  if (abs(tiltX) <= 0.1) {
    tiltX = 0;
  }
  if (abs(tiltY) <= 0.1) {
    tiltY = 0;
  }

  double gyroX = angVelocityData.gyro.y;
  double gyroY = -angVelocityData.gyro.x;
  
  
  angleX = (1 - alpha) * (angleX + gyroX * dt) + alpha * tiltX;
  angleY = (1 - alpha) * (angleY + gyroY * dt) + alpha * tiltY;



  Error = sq(angleX) + sq(angleY);
  if (Error <= threshold){

    pidX.SetTunings(consKp, consKi, consKd);
    pidY.SetTunings(consKp, consKi, consKd);
  }
  else{

    pidX.SetTunings(aggKp, aggKi, aggKd);
    pidY.SetTunings(aggKp, aggKi, aggKd);
    Serial.println("triggered");

  }


  bool pidXComputed = pidX.Compute();  // returns true if it computed, false otherwise
  bool pidYComputed = pidY.Compute();  // same here

  // if (pidXComputed) {
  //   Serial.println("PID X computed successfully");
  // } else {
  //   Serial.println("PID X not computed");
  // }

  // if (pidYComputed) {
  //   Serial.println("PID Y computed successfully");
  // } else {
  //   Serial.println("PID Y not computed");
  // }



  pwm_1 = float(-(2.0 / 3.0) * uy); 
  pwm_2 = float(0.577 * ux + (1.0 / 3.0) * uy);
  pwm_3 = float(-0.577 * ux + (1.0 / 3.0) * uy);


  // if(pwm_1 < -4095){
  //   pwm_1 = -4095;
  // }
  // else if(pwm_1 > 4095){
  //   pwm_1 = 4095;
  // }
  // if(pwm_2 < -4095){
  //   pwm_2 = -4095;
  // }
  // else if(pwm_2 > 4095){
  //   pwm_2 = 4095;
  // }
  // if(pwm_3 < -4095){
  //   pwm_3 = -4095;
  // }
  // else if(pwm_3 > 4095){
  //   pwm_3 = 4095;
  // }

  if(pwm_1<=0){
    pwm_1 -= pwmOffset*4095;
  }
  else{
    pwm_1 += pwmOffset*4095; 
  }
  if(pwm_2<=0){
    pwm_2 -= pwmOffset*4095;
  }
  else{
    pwm_2 += pwmOffset*4095; 
  }
  if(pwm_3<=0){
    pwm_3 -= pwmOffset*4095;
  }
  else{
    pwm_3 += pwmOffset*4095; 
  }

  pwm_1 = constrain(pwm_1, -4095, 4095);
  pwm_2 = constrain(pwm_2, -4095, 4095);
  pwm_3 = constrain(pwm_3, -4095, 4095);


  writeMotor(pwm_1, Motor1_dir_1, Motor1_dir_2, PWM_channel_1);
  writeMotor(pwm_2, Motor2_dir_1, Motor2_dir_2, PWM_channel_2);
  writeMotor(pwm_3, Motor3_dir_1, Motor3_dir_2, PWM_channel_3);

  
  // Serial.print(setpointX);
  // Serial.print(" ");
  // Serial.print(tiltX);
  // Serial.print(" ");
  // Serial.print(tiltY);
  // Serial.print(" ");
  // Serial.print(tiltZ);
  // Serial.print(" ");
  
  // Serial.print(ux);
  // Serial.print(" ");
  // Serial.print(uy);
  // Serial.print(" ");
  // Serial.print(pwm_1);
  // Serial.print(" ");
  // Serial.print(pwm_2);
  // Serial.print(" ");
  // Serial.println(pwm_3);

  Serial.print(angleX);
  Serial.print(" ");
  Serial.print(angleY);
  Serial.print(" ");
  Serial.print(tiltX);
  Serial.print(" ");
  Serial.print(tiltY);
  Serial.print(" ");
  Serial.println(dt);
  }

}

void writeMotor(double pwm, int dirPin1, int dirPin2, int PWM_channel) {
  if (pwm < 0) {
    // pwm = pwm - 0.4 * 255;
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    //ledcWrite(PWM_channel, -pwm);
    ledcWriteChannel(PWM_channel, -pwm);
  } else {
    // pwm = pwm + 0.4 * 255;
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    //ledcWrite(PWM_channel, pwm);
    ledcWriteChannel(PWM_channel, pwm);
  }

}
