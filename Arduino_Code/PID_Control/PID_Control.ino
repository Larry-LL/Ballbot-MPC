#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <math.h>
using namespace BLA;


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//PWM Setup 
#define PWM_freq 5000
#define PWM_resolution 12

//Motor Setup 

//Motor #1 
double pwm_1;     
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
double pwm_2 ;
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
double pwm_3 ;
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

// PWM Control
double Kp = 250, Ki = 0, Kd = 0;
double ux, uy;
double setpointX = 0, setpointY = 0;

PID pidX(&tiltX, &ux, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&tiltY, &uy, &setpointY, Kp, Ki, Kd, DIRECT);




void setup() {
  
  //Motor 1 setup
  pinMode(Motor1_dir_1, OUTPUT);
  pinMode(Motor1_dir_2, OUTPUT);
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

  Serial.begin(115200);


}



void loop() {
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  tiltX = -orientationData.orientation.z;  //theta_x  in deg
  tiltY = -orientationData.orientation.y;    //theta_y  in deg
  tiltZ = orientationData.orientation.x;
  if (tiltZ < 180) {
    tiltZ = -tiltZ;
  }
  else {
    tiltZ = 360 - tiltZ;
  }

  pidX.Compute();
  pidY.Compute();

pwm_1 = -2 / 3 * uy;
pwm_2 = 0.577 * ux + 1 / 3 * uy;
pwm_3 = -0.577 * ux + 1 / 3 * uy;

pwm_1 = constrain(pwm_1, -4095, 4095);
pwm_2 = constrain(pwm_2, -4095, 4095);
pwm_3 = constrain(pwm_3, -4095, 4095);

writeMotor(pwm_1, Motor1_dir_1, Motor1_dir_2, PWM_channel_1);
writeMotor(pwm_2, Motor2_dir_1, Motor2_dir_2, PWM_channel_2);
writeMotor(pwm_3, Motor3_dir_1, Motor3_dir_2, PWM_channel_3);




Serial.print(tiltX);
Serial.print(" ");
Serial.print(tiltY);
Serial.print(" ");
Serial.print(tiltZ);
Serial.print(" ");
Serial.print(pwm_1);
Serial.print(" ");
Serial.print(pwm_2);
Serial.print(" ");
Serial.println(pwm_3);

delay(10);

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
