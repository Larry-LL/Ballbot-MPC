#include <PID_v1.h>
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
#define PWM_resolution 8

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

volatile unsigned long lastTime_1 = 0;
volatile unsigned long currentTime_1 = 0;
volatile unsigned long lastTime_2 = 0;
volatile unsigned long currentTime_2 = 0;
volatile unsigned long lastTime_3 = 0;
volatile unsigned long currentTime_3 = 0;
const int oneSecond = 200;

double wheel_speed_1 ; 
double wheel_speed_2 ; 
double wheel_speed_3 ; 


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
unsigned long time_interval = 0.2;

//ball property
double r_ball = 10;
double r_wheel = 10;
double alpha1 = 0, alpha2 = 120, alpha3 = 240;



//PID name
double Input_1, Input_2,Input_3;
double Output_1,Output_2,Output_3,Output_x,Output_y;
double Setpoint1 = 0.9 , Setpoint2 = 0,Setpoint3 = 0,Setpoint_x = 0, setpoint_y = 0;
double v_des_y, v_des_x;
double Kp1= 700, Ki1= 70, Kd1= 20 ;
double Kp2= 0,   Ki2= 0,  Kd2= 0 ;
double Kp3= 0,   Ki3= 0,  Kd3= 0 ;
double Kp_x = 10, Ki_x = 10, Kd_x = 10,Kp_y = 10, Ki_y = 10, Kd_y = 10;



PID pid1(&wheel_speed_1, &Output_1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID pid2(&wheel_speed_2, &Output_2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);
PID pid3(&wheel_speed_3, &Output_3, &Setpoint3, Kp3, Ki3, Kd3, DIRECT);
PID tilt_x(&angleX, &Output_x, &Setpoint_x, Kp_x, Ki_x, Kd_x, DIRECT);
PID tilt_y(&angleY, &Output_y, &setpoint_y, Kp_y, Ki_y, Kd_y, DIRECT);


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
  //added pullup
  pinMode(encoder0pinA_1, INPUT_PULLUP);
  pinMode(encoder0pinB_1, INPUT_PULLUP);
  
  lastStateAB_1 = (digitalRead(encoder0pinA_1) << 1) | digitalRead(encoder0pinB_1);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA_1), encoderAB_ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0pinB_1), encoderAB_ISR1, CHANGE);



  //Motor 2 setup
  pinMode(Motor2_dir_1, OUTPUT);
  pinMode(Motor2_dir_2, OUTPUT);
  digitalWrite(Motor2_dir_1, HIGH);
  digitalWrite(Motor2_dir_2, LOW);
  ledcAttach(PWM_pin_2, PWM_freq, PWM_resolution);
  //added pullup
  pinMode(encoder0pinA_2, INPUT_PULLUP);
  pinMode(encoder0pinB_2, INPUT_PULLUP);
  lastStateAB_2 = (digitalRead(encoder0pinA_2) << 1) | digitalRead(encoder0pinB_2);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA_2), encoderAB_ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0pinB_2), encoderAB_ISR2, CHANGE);


  //Motor 3 setup 
  pinMode(Motor3_dir_1, OUTPUT);
  pinMode(Motor3_dir_2, OUTPUT);
  digitalWrite(Motor3_dir_1, HIGH);
  digitalWrite(Motor3_dir_2, LOW);
  ledcAttach(PWM_pin_3, PWM_freq, PWM_resolution);
  //added pullup
  pinMode(encoder0pinA_3, INPUT_PULLUP);
  pinMode(encoder0pinB_3, INPUT_PULLUP);
  lastStateAB_3 = (digitalRead(encoder0pinA_3) << 1) | digitalRead(encoder0pinB_3);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA_3), encoderAB_ISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0pinB_3), encoderAB_ISR3, CHANGE);


  //PID SETUP
  pid1.SetMode(AUTOMATIC);
  pid2.SetMode(AUTOMATIC);
  pid3.SetMode(AUTOMATIC);

  // pid1.SetOutputLimits(-4095, 4095);  // Example: for 12-bit PWM
  // pid2.SetOutputLimits(-4095, 4095);
  // pid3.SetOutputLimits(-4095, 4095);
  pid1.SetOutputLimits(0, 255);  // Allow full range of output


  


  Serial.begin(115200);
  //Start the encoder setup
  EncoderInit_1();
  EncoderInit_2();
  EncoderInit_3();




  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }


  sensors_event_t orientationData;
  sensors_event_t linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  rollOffset = orientationData.orientation.x;
  pitchOffset = orientationData.orientation.z;
  yawOffset = orientationData.orientation.y;
  lastTime = millis();

  delay(8000);
  Serial.print("Let's go!!!!!!!!!!!!!!!!!!!!!!!!!!");
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



  calculatRps(&pulseCount_1, &lastTime_1, &currentTime_1, &wheel_speed_1);
  calculatRps(&pulseCount_2, &lastTime_2, &currentTime_2, &wheel_speed_2);
  calculatRps(&pulseCount_3, &lastTime_3, &currentTime_3, &wheel_speed_3);


  v_des_y = r_ball * Output_y;
  v_des_x = r_ball * Output_x;


  Setpoint1 = (r_ball / r_wheel) * ( -sin(radians(alpha1)) * v_des_x + cos(radians(alpha1)) * v_des_y );
  Setpoint2 = (r_ball / r_wheel) * ( -sin(radians(alpha2)) * v_des_x + cos(radians(alpha2)) * v_des_y );
  Setpoint3 = (r_ball / r_wheel) * ( -sin(radians(alpha3)) * v_des_x + cos(radians(alpha3)) * v_des_y );

  
  
  
  
  pid1.Compute();  // returns true if it computed, false otherwise
  pid2.Compute();  // same here
  pid3.Compute();  // same here


  // pwm_1 = map(Output_1,-255,255,-4095,4095);
  // pwm_2 = map(Output_2,-255,255,-4095,4095);
  // pwm_3 = map(Output_3,-255,255,-4095,4095);

  pwm_1 = Output_1;
  pwm_2 = 0;
  pwm_3 = 0;

  // pwm_1 = constrain(pwm_1, -4095, 4095);
  // pwm_2 = constrain(pwm_2, -4095, 4095);
  // pwm_3 = constrain(pwm_3, -4095, 4095);


  // pwm_1 = 4095;
  // pwm_2 = 4095;
  // pwm_3 = 4095;

  writeMotor(pwm_1, Motor1_dir_1, Motor1_dir_2, PWM_channel_1);
  writeMotor(pwm_2, Motor2_dir_1, Motor2_dir_2, PWM_channel_2);
  writeMotor(pwm_3, Motor3_dir_1, Motor3_dir_2, PWM_channel_3);

  Serial.print(wheel_speed_1); 
  Serial.print(" ");
  Serial.println(Setpoint1);
  // Serial.print(" ");
  // Serial.println(Output_1);
  

  
  // Serial.print(ux);
  // Serial.print(" ");
  // Serial.print(uy);
  // Serial.print(" ");
  // Serial.print(pwm_1);
  // Serial.print(" ");
  // Serial.print(pwm_2);
  // Serial.print(" ");
  // Serial.println(pwm_3);

  // Serial.print(angleX);
  // Serial.print(" ");
  // Serial.print(angleY);
  // Serial.print(" ");
  // Serial.print(tiltX);
  // Serial.print(" ");
  // Serial.print(tiltY);
  // Serial.print(" ");
  // Serial.println(dt);
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

void encoderAB_ISR1(){
  currentStateAB_1 = (digitalRead(encoder0pinA_1) << 1) | digitalRead(encoder0pinB_1);
  // Update Encoder counts based on Phase switching
  if (currentStateAB_1 != lastStateAB_1) {
    if ((lastStateAB_1 == 0b00 && currentStateAB_1 == 0b10) ||
        (lastStateAB_1 == 0b10 && currentStateAB_1 == 0b11) ||
        (lastStateAB_1 == 0b11 && currentStateAB_1 == 0b01) ||
        (lastStateAB_1 == 0b01 && currentStateAB_1 == 0b00)) {
          direction1 = true; // Forward
          pulseCount_1++;
          pulseCount_distance_1++;

  } else if ((lastStateAB_1 == 0b00 && currentStateAB_1 == 0b01) ||
             (lastStateAB_1 == 0b01 && currentStateAB_1 == 0b11) ||
             (lastStateAB_1 == 0b11 && currentStateAB_1 == 0b10) ||
             (lastStateAB_1 == 0b10 && currentStateAB_1 == 0b00)) {
              direction1 = false; // Reverse
              pulseCount_1--;
              pulseCount_distance_1--;
        }
    }
    // Update lastStateAB
    lastStateAB_1 = currentStateAB_1;

}
void encoderAB_ISR2(){
  currentStateAB_2 = (digitalRead(encoder0pinA_2) << 1) | digitalRead(encoder0pinB_2);
  // Update Encoder counts based on Phase switching
  if (currentStateAB_2 != lastStateAB_2) {
    if ((lastStateAB_2 == 0b00 && currentStateAB_2 == 0b10) ||
        (lastStateAB_2 == 0b10 && currentStateAB_2 == 0b11) ||
        (lastStateAB_2 == 0b11 && currentStateAB_2 == 0b01) ||
        (lastStateAB_2 == 0b01 && currentStateAB_2 == 0b00)) {
          direction2 = true; // Forward
          pulseCount_2++;
          pulseCount_distance_2 ++;
  } else if ((lastStateAB_2 == 0b00 && currentStateAB_2 == 0b01) ||
             (lastStateAB_2 == 0b01 && currentStateAB_2 == 0b11) ||
             (lastStateAB_2 == 0b11 && currentStateAB_2 == 0b10) ||
             (lastStateAB_2 == 0b10 && currentStateAB_2 == 0b00)) {
              direction2 = false; // Reverse
              pulseCount_2--;
              pulseCount_distance_2--;
        }
    }
    // Update lastStateAB
    lastStateAB_2 = currentStateAB_2;
}

void encoderAB_ISR3(){
  currentStateAB_3 = (digitalRead(encoder0pinA_3) << 1) | digitalRead(encoder0pinB_3);
  // Update Encoder counts based on Phase switching
  if (currentStateAB_3 != lastStateAB_3) {
    if ((lastStateAB_3 == 0b00 && currentStateAB_3 == 0b10) ||
        (lastStateAB_3 == 0b10 && currentStateAB_3 == 0b11) ||
        (lastStateAB_3 == 0b11 && currentStateAB_3 == 0b01) ||
        (lastStateAB_3 == 0b01 && currentStateAB_3 == 0b00)) {
          direction3 = true; // Forward
          pulseCount_3++;
          pulseCount_distance_3++;

  } else if ((lastStateAB_3 == 0b00 && currentStateAB_3 == 0b01) ||
             (lastStateAB_3 == 0b01 && currentStateAB_3 == 0b11) ||
             (lastStateAB_3 == 0b11 && currentStateAB_3 == 0b10) ||
             (lastStateAB_3 == 0b10 && currentStateAB_3 == 0b00)) {
              direction3 = false; // Reverse
              pulseCount_3--;
              pulseCount_distance_3--;
        }
    }
    // Update lastStateAB
    lastStateAB_3 = currentStateAB_3;
}



void calculatRps(volatile int *pulseCount, volatile unsigned long *lastTime , volatile unsigned long *currentTime,double *rps){
  *currentTime = millis();
  if (*currentTime - *lastTime >= oneSecond){
    noInterrupts();
    int pulseCopy = *pulseCount;
    interrupts();
    // Serial.print("Pulse Count: ");
    // Serial.print(pulseCopy);
    // Serial.print(" | rps: ");
    *rps = (double)pulseCopy /(9600*oneSecond/1000);
    // Serial.print(rps);
    // Serial.print(" | Direction: ");
    // Serial.println(direction1 ? "Forward" : "Reverse");
    *pulseCount = 0;
    *lastTime = *currentTime;
  
  }
}
void EncoderInit_1() {
  Direction_1 = true; // Default -> Forward  
  pinMode(encoder0pinA_1, INPUT);  
  pinMode(encoder0pinB_1, INPUT);  
  // encoder0PinALast_1 = digitalRead(encoder0pinA_1); // Initialize encoder0PinALast
  // attachInterrupt(digitalPinToInterrupt(encoder0pinA_1), wheelSpeed1, CHANGE);
}

void EncoderInit_2() {
  Direction_2 = true; // Default -> Forward  
  pinMode(encoder0pinA_2, INPUT);  
  pinMode(encoder0pinB_2, INPUT);  
  // encoder0PinALast_2 = digitalRead(encoder0pinA_2); // Initialize encoder0PinALast
  // attachInterrupt(digitalPinToInterrupt(encoder0pinA_2), wheelSpeed2, CHANGE);
}

void EncoderInit_3() {
  Direction_3 = true; // Default -> Forward  
  pinMode(encoder0pinA_3, INPUT);  
  pinMode(encoder0pinB_3, INPUT);  
  // encoder0PinALast_3 = digitalRead(encoder0pinA_3); // Initialize encoder0PinALast
  // attachInterrupt(digitalPinToInterrupt(encoder0pinA_3), wheelSpeed3, CHANGE);
}
