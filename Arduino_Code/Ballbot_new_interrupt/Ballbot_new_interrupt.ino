#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <math.h>
using namespace BLA;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 2000;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


//Motor Property
#define PWM_freq 1500
#define PWM_resolution 8
int Motor_cpr = 64 ; 
int Gear_ratio = 150 ;

//Motor #1 
float pwm_1;    //motor 3 
int duration_1 = 0;
boolean Direction_1;
byte encoder0PinALast_1;
const byte encoder0pinA_1 = 15; // Pin A
const byte encoder0pinB_1 = 13; // Pin B
float omega_1 ; 
#define Motor1_dir_1 32
#define Motor1_dir_2 33
#define PWM_pin_1 10
#define PWM_channel_1 0

//Motor #2       Motor 1 
float pwm_2 ;
int duration_2 = 0;
boolean Direction_2;
byte encoder0PinALast_2;
const byte encoder0pinA_2 = 5; // Pin A
const byte encoder0pinB_2 = 18; // Pin B
float omega_2 ; 
#define Motor2_dir_1 19
#define Motor2_dir_2 23
#define PWM_pin_2 14
#define PWM_channel_2 1

//Motor #3
float pwm_3 ;
int duration_3 = 0;
boolean Direction_3;
byte encoder0PinALast_3;
const byte encoder0pinA_3 = 34; // Pin A
const byte encoder0pinB_3 = 35; // Pin B     motor 2
float omega_3 ; 
#define Motor3_dir_1 25
#define Motor3_dir_2 26 
#define PWM_pin_3 27
#define PWM_channel_3 2


//Control Variable Setup
float linear_accx_offset;
float linear_accy_offset;
float linear_accx_calib;
float linear_accy_calib;

float r_k = 0.1016 ; 
float r_w = 0.03 ;
float roll ; 
float pitch; 
float yaw; 
float gain = 1; 
float kf = 0.025; 
float ke = 1.71;
float kt = 0.873; 
float resistance_motor = 1.65; 
float velocity_x = 0 ; 
float velocity_y = 0; 
float IMU_period = 0.01; //assume 100hz
float theta_xd ; 
float theta_yd ; 
float theta_zd ; 
float acc_1; 
float acc_2; 
float acc_3; 
float k_e = 1.71; 
float ang_1 ; 
float ang_2; 
float ang_3; 
float Torque_friction = 0.3;//1.2;//1.2 ; //pwm = 0.2 //1.479;
long int wheel_distance_1 = 0  ; 
long int wheel_distance_2 = 0 ; 
long int wheel_distance_3 = 0 ; 
long int loop_time = 0; 
int loop_max = 20 ; 
float wheel_inertia = 0.0001; 
float D_pwm1  ; 
float D_pwm2  ; 
float D_pwm3  ; 
float psi_1 ; 
float psi_2 ; 
float psi_3 ; 
double wheel_speed_1 ; 
double wheel_speed_2 ; 
double wheel_speed_3 ; 
float offset_percentage = 0.1;
float new_offset_percentage = 0.18;
unsigned long previous_time = 0; 
unsigned long current_time; 
float time_interval; 
float rollOffset = 0;
float pitchOffset = 0;
float yawOffset = 0;
int k = 0 ;
float torque_1; 
float torque_2; 
float torque_3; 
int samples = 5;
float theta_dot_x; 
float theta_dot_y; 
float Phi_dot_x; 
float Phi_dot_y;
float Phi_x_tot; 
float Phi_y_tot;
float U_alpha_x; 
float U_alpha_y; 
int i = 0;



//PID Controller
double kp = 1.2, ki = 0, kd = 0; //PID Parameters 
double feedforwardPWM1; 
double feedforwardPWM2;
double feedforwardPWM3;
double pidCorrection1;
double pidCorrection2;
double pidCorrection3;
double omega_kx;
double omega_ky;
double v_x; 
double v_y ; 
double total_pwm1;
double total_pwm2;
double total_pwm3;

int count_1 = 1 ; 
int count_2 = 1 ; 
int count_3 = 1 ;  
int long duration_1_sum = 0; 
int long duration_2_sum = 0; 
int long duration_3_sum = 0; 
float duration_1_avg; 
float duration_2_avg; 
float duration_3_avg;
float Desired_Torque_1;
float Desired_Torque_2;
float Desired_Torque_3;
double Desired_current_1; 
double Desired_current_2; 
double Desired_current_3; 
double Current_current1;
double Current_current2;
double Current_current3;

float desired_alpha1;
float desired_alpha2;
float desired_alpha3;
float ratio_1; 
float ratio_2;
float ratio_3;
int j = 0;
// boolean test_direction = true;



unsigned long previousMillis = 0;   
const long interval = 5;



// PID ballbotPID1(&Current_current1, &pidCorrection1, &Desired_current_1, kp, ki, kd, DIRECT);  //input output setpoint 
// PID ballbotPID2(&Current_current2, &pidCorrection2, &Desired_current_2, kp, ki, kd, DIRECT);  //input output setpoint 
// PID ballbotPID3(&Current_current3, &pidCorrection3, &Desired_current_3, kp, ki, kd, DIRECT);  //input output setpoint 


boolean test_direction = true;

// Matrix<2, 8> K_matrix = {
//   -19.0728, -0.0141, -4.5224, -0.0607, 0, 0, 0, 0,
//   0, 0, 0, 0, -19.0728, -0.0141, -4.5224, -0.0607
// };

Matrix<2, 8> K_matrix = {
  -36.1415,   -0.0500,  -12.0185,   -0.1662, 0, 0, 0, 0,
  0, 0, 0, 0, -36.1415,   -0.0500,  -12.0185,   -0.1662
};


Matrix<8, 1> State_Matrix;
Matrix<2> Control_Input_Matrix ; 


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


  //Motor 2 setup
  pinMode(Motor2_dir_1, OUTPUT);
  pinMode(Motor2_dir_2, OUTPUT);
  digitalWrite(Motor2_dir_1, HIGH);
  digitalWrite(Motor2_dir_2, LOW);
  ledcAttach(PWM_pin_2, PWM_freq, PWM_resolution);
  //added pullup
  pinMode(encoder0pinA_2, INPUT_PULLUP);
  pinMode(encoder0pinB_2, INPUT_PULLUP);


  //Motor 3 setup
  pinMode(Motor3_dir_1, OUTPUT);
  pinMode(Motor3_dir_2, OUTPUT);
  digitalWrite(Motor3_dir_1, HIGH);
  digitalWrite(Motor3_dir_2, LOW);
  ledcAttach(PWM_pin_3, PWM_freq, PWM_resolution);
  //added pullup
  pinMode(encoder0pinA_3, INPUT_PULLUP);
  pinMode(encoder0pinB_3, INPUT_PULLUP);



  Serial.begin(115200);  // Initialize the serial port

  //start the pid setup 
  // ballbotPID1.SetMode(AUTOMATIC);
  // ballbotPID1.SetOutputLimits(-8, 8);  // Limits for PID
  // ballbotPID2.SetMode(AUTOMATIC);
  // ballbotPID2.SetOutputLimits(-8, 8);  // Limits for PID
  // ballbotPID3.SetMode(AUTOMATIC);
  // ballbotPID3.SetOutputLimits(-8, 8);  // Limits for PID
  
  //Start the encoder setup
  EncoderInit_1();
  EncoderInit_2();
  EncoderInit_3();

  //IMU SETUP
  while (!Serial) delay(10);  // wait for serial port to open!
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  delay(1000);

  sensors_event_t orientationData;
  sensors_event_t linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  //Get the first-time initial IMU DATA as offset
  rollOffset = orientationData.orientation.x;
  pitchOffset = orientationData.orientation.z;
  yawOffset = orientationData.orientation.y;
  // linear_accx_offset = linearAccelData.acceleration.z;
  // linear_accy_offset = linearAccelData.acceleration.y;


}

void loop() {
  // Wait a moment
  //IMU get data
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  // printEvent(&orientationData);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

pitch = -orientationData.orientation.z;  //theta_x     -orientationData.orientation.z
yaw = -orientationData.orientation.y;     //theta_y
roll = orientationData.orientation.x;    //theta_z

pitch = pitch + pitchOffset;
yaw = yaw + yawOffset;
roll = roll - rollOffset;

if (roll < 180) {
  roll = -roll;
} 


else{
  roll = 360-roll; 
}

//convert degrees to radians
theta_xd = pitch * PI / 180 ;
theta_yd = yaw * PI / 180;

// Serial.print(theta_xd);
// Serial.print(" ");
// Serial.println(theta_yd);


theta_zd= radians(roll);

if (theta_zd> 3.14){
  theta_zd = theta_zd - 6.28; 
}

float ang_Vx_sum = 0;  // Make sure to reset the sum each time the loop runs
float ang_Vy_sum = 0;  // Make sure to reset the sum each time the loop runs
float ang_Vz_sum = 0;



for (int i = 0; i < samples; i++) {
    ang_Vx_sum += angVelocityData.gyro.x;  // Accumulate angular velocity data
    ang_Vy_sum += angVelocityData.gyro.y;
    ang_Vz_sum += angVelocityData.gyro.z;
    if (i == samples - 1) {
        theta_dot_x = ang_Vx_sum / samples;  // Calculate and store the average
        theta_dot_y = ang_Vy_sum / samples;  // Calculate and store the average
        theta_dot_x = round(theta_dot_x*100)/100; 
        theta_dot_y = round(theta_dot_y*100)/100;
    }

}


//calculate the actual speed of wheel here 

// if (duration_1 > duration_1_sum){
//   duration_1_sum = duration_1;
// }

// if (duration_2 > duration_2_sum){
//   duration_2_sum = duration_2;
// }
// if (duration_3 > duration_3_sum){
//   duration_3_sum = duration_3;
// }


// wheel_speed_1 = duration_1 / 555 * 2* PI ; //rad/s
// wheel_speed_2 = duration_2 / 555 * 2* PI ; 
// wheel_speed_3 = duration_3 / 555 * 2* PI ; 

wheel_speed_1 = duration_1 / 83.77;
wheel_speed_2 = duration_2 / 83.77;
wheel_speed_3 = duration_3 / 83.77;

// Serial.print(duration_1);
// Serial.print(" "); 
// Serial.print(duration_2);
// Serial.print(" "); 
// Serial.print(duration_3);
// Serial.print(" "); 
// Serial.print(wheel_speed_1); 
// Serial.print(" ");
// Serial.print(wheel_speed_2); 
// Serial.print(" ");
// Serial.println(wheel_speed_3);


Phi_dot_x = 0.01528*wheel_speed_2 - 0.03055*wheel_speed_1+0.01528*wheel_speed_3; 
Phi_dot_y = 0.0529*wheel_speed_3 - 0.0529*wheel_speed_2; 

current_time = millis();
time_interval = (current_time - previous_time)/1000.00;
previous_time = current_time;

Phi_x_tot = Phi_x_tot + Phi_dot_x *time_interval; //rad
Phi_y_tot = Phi_y_tot + Phi_dot_y *time_interval; 

State_Matrix(0) = theta_xd;  
State_Matrix(1) = Phi_x_tot;  
State_Matrix(2) = theta_dot_x;  
State_Matrix(3) = Phi_dot_x ; 

State_Matrix(4) = theta_yd;  
State_Matrix(5) = Phi_y_tot ; 
State_Matrix(6) = theta_dot_y ; 
State_Matrix(7) = Phi_dot_y ; 

if (abs(theta_xd) < 0.017){
  State_Matrix(0) = 0 ;
}
if (abs(theta_yd) < 0.017){
  State_Matrix(4) = 0 ;
}
if(abs(theta_dot_x) < 0.017){

  State_Matrix(2) = 0; 
}

if(abs(theta_dot_y) < 0.017){
  State_Matrix(6) = 0;
}

Control_Input_Matrix = -K_matrix * State_Matrix ;     // return the input control Torque 

Desired_Torque_1 = -0.978 * Control_Input_Matrix(0); 
Desired_Torque_2 = 0.49 * Control_Input_Matrix(0) - 0.85 * Control_Input_Matrix(1); 
Desired_Torque_3 = 0.49 * Control_Input_Matrix(0) + 0.85 * Control_Input_Matrix(1); 

Desired_current_1 = Desired_Torque_1 / 0.87; 
Desired_current_2 = Desired_Torque_2 / 0.87; 
Desired_current_3 = Desired_Torque_3 / 0.87; 

Current_current1 = (total_pwm1/255*12 - k_e * wheel_speed_1)/1.7; 
Current_current1 = (total_pwm2/255*12 - k_e * wheel_speed_2)/1.7; 
Current_current1 = (total_pwm3/255*12 - k_e * wheel_speed_3)/1.7; 



// desired_V1 = (desired_V1 + desired_V1 + desired_alpha1*time_interval)/2; 
// desired_V2 = (desired_V2 + desired_V2 + desired_alpha2*time_interval)/2; 
// desired_V3 = (desired_V3 + desired_V3 + desired_alpha3*time_interval)/2; 


// U_alpha_x =  Control_Input_Matrix(0) / 0.0171 ;  //Ball inertia 0.0171
// U_alpha_y =  Control_Input_Matrix(1) / 0.0171 ; 
// omega_kx = (omega_kx+omega_kx + U_alpha_x *time_interval)/2; 
// omega_ky = (omega_ky+omega_ky + U_alpha_y *time_interval)/2;    //velocity of the ball 
// desired_V1 = -16.4 * omega_kx;
// desired_V2 =  16.4  * omega_kx - 9.45 * omega_ky;
// desired_V3 =  16.4  * omega_kx + 9.45 * omega_ky;


// //////////////////////////
// unsigned long currentMillis = millis();

// if (currentMillis - previousMillis >= interval) {
//   previousMillis = currentMillis;  // update the time

//   CorrectionPID1(Current_current1, Desired_current_1);
//   CorrectionPID2(Current_current2, Desired_current_2);
//   CorrectionPID3(Current_current3, Desired_current_3);
// }
// ///////////////////

// if (torque_1 > 0) {
//   total_pwm1 = (1.65 * Desired_current_1 - k_e * wheel_speed_1) / 12 * 255;
// } else {
//   total_pwm1 = (1.65 * Desired_current_1 + k_e * wheel_speed_1) / 12 * 255;
// }

// if (torque_2 > 0) {
//   total_pwm2 = (1.65 * Desired_current_2 - k_e * wheel_speed_2) / 12 * 255;}
//   else {
//     total_pwm2 = (1.65 * Desired_current_2 + k_e * wheel_speed_2) / 12 * 255;
//   }

// if (torque_3 > 0) {
//   total_pwm3 = (1.65 * Desired_current_3 - k_e * wheel_speed_3) / 12 * 255;
// } else {
//   total_pwm3 = (1.65 * Desired_current_3 + k_e * wheel_speed_3) / 12 * 255;
// }

total_pwm1 = (1.65 * Desired_current_1 + 0.5 * wheel_speed_1) / 12 * 255;

total_pwm2 = (1.65 * Desired_current_2 + 0.5 * wheel_speed_2) / 12 * 255;

total_pwm3 = (1.65 * Desired_current_3 + 0.5 * wheel_speed_3) / 12 * 255;




ratio_1 = total_pwm1/255;
ratio_2 = total_pwm2/255;
ratio_3 = total_pwm3/255;

// Serial.print(ratio_1);
// Serial.print(" ");
// Serial.print(ratio_2);
// Serial.print(" ");
// Serial.println(ratio_3);

total_pwm1 = 255; 
total_pwm2 = 255; 
total_pwm3 = 255; 

writeMotor(total_pwm1, Motor1_dir_1, Motor1_dir_2, PWM_channel_1);
writeMotor(total_pwm2, Motor2_dir_1, Motor2_dir_2, PWM_channel_2);
writeMotor(total_pwm3, Motor3_dir_1, Motor3_dir_2, PWM_channel_3);

// if (Desired_Torque_1 < 0) {
//   ratio_1 = ((Desired_Torque_1 - 0.635) * resistance_motor / kt + k_e * wheel_speed_1) / 12;

// } else {
//   ratio_1 = ((Desired_Torque_1 + 0.635) * resistance_motor / kt + k_e * wheel_speed_1) / 12;
// }

// if (Desired_Torque_2 < 0) {
//   ratio_2 = ((Desired_Torque_2 - 0.635) * resistance_motor / kt + k_e * wheel_speed_2) / 12;

// } else {
//   ratio_2 = ((Desired_Torque_2 + 0.635) * resistance_motor / kt + k_e * wheel_speed_2) / 12;
// }

// if (Desired_Torque_3 < 0) {
//   ratio_3 = ((Desired_Torque_3 - 0.635) * resistance_motor / kt + k_e * wheel_speed_3) / 12;

// } else {
//   ratio_3 = ((Desired_Torque_3 + 0.635) * resistance_motor / kt + k_e * wheel_speed_3) / 12;
// }

// pwm_1 = 255*(Desired_Torque_1*resistance_motor/kt +k_e *wheel_speed_1)/12;
// pwm_2 = 255*(Desired_Torque_2*resistance_motor/kt +k_e *wheel_speed_2)/12;
// pwm_3 = 255*(Desired_Torque_3*resistance_motor/kt +k_e *wheel_speed_3)/12;
// pwm_1 = ratio_1 * 255; 
// pwm_2 = ratio_2 *255; 
// pwm_3 = ratio_3 *255; 


// Serial.print(Desired_Torque_1);
// Serial.print(" ");
// Serial.print(Desired_Torque_2);
// Serial.print(" ");
// Serial.println(Desired_Torque_3);



// writeMotor(pwm_1, Motor1_dir_1, Motor1_dir_2, PWM_channel_1);
// writeMotor(pwm_2, Motor2_dir_1, Motor2_dir_2, PWM_channel_2);
// writeMotor(pwm_3, Motor3_dir_1, Motor3_dir_2, PWM_channel_3);

// Serial.print(pwm_1);
// Serial.print(" ");
// Serial.print(pwm_2);
// Serial.print(" ");
// Serial.println(pwm_3);

///manual testing forward and reverse
// if (test_direction) {
//   pwm_1 = 255;
//   pwm_2 = 255;
//   pwm_3 = 255;
//   writeMotor(pwm_1, Motor1_dir_1, Motor1_dir_2, PWM_channel_1);
//   writeMotor(pwm_2, Motor2_dir_1, Motor2_dir_2, PWM_channel_2);
//   writeMotor(pwm_3, Motor3_dir_1, Motor3_dir_2, PWM_channel_3);
// } else {

//   pwm_1 = -255;
//   pwm_2 = -255;
//   pwm_3 = -255;
//   writeMotor(pwm_1, Motor1_dir_1, Motor1_dir_2, PWM_channel_1);
//   writeMotor(pwm_2, Motor2_dir_1, Motor2_dir_2, PWM_channel_2);
//   writeMotor(pwm_3, Motor3_dir_1, Motor3_dir_2, PWM_channel_3);
// }

// k++; 

// if (k>=50){
//   k = 0; 
//   test_direction=!test_direction; 
// }


//PRINT COMMANDS 
// Serial.print("D_1: ");
// Serial.print(D_pwm1);
// Serial.print("D_2: ");
// Serial.print(D_pwm2);
// Serial.print("D_3: ");
// Serial.println(D_pwm3);
// Serial.print(D_pwm1);
// Serial.print(" ");
// Serial.print(D_pwm2);
// Serial.print(" ");
// Serial.print(D_pwm3);

// Serial.print(Desired_current_1);
// Serial.print(" ");
// Serial.print(Desired_current_2);
// Serial.print(" ");
// Serial.println(Desired_current_3);

// Serial.print(duration_1_sum); 
// Serial.print(" "); 
// Serial.print(duration_2_sum); 
// Serial.print(" "); 
// Serial.println(duration_3_sum); 

// Serial.print(desired_V1); 
// Serial.print(" "); 
// Serial.print(desired_V2); 
// Serial.print(" "); 
// Serial.println(desired_V3); 
// Serial.print(Control_Input_Matrix(0));
// Serial.print(" ");
// Serial.println(Control_Input_Matrix(1));
// Serial.print(" ");
// Serial.print(omega_kx);
// Serial.print(" ");
// Serial.println(omega_ky);

// Serial.print("control_input");

// Serial.print(feedforwardPWM1);
// Serial.print(" ");
// Serial.print(feedforwardPWM2);
// Serial.print(" ");
// Serial.println(feedforwardPWM2);

// Serial.print(State_Matrix(2));
// Serial.print(" ");
// Serial.println(State_Matrix(6));
// Serial.print(" ");
// Serial.println(Control_Input_Matrix(2));
// Serial.print("pwm ");
// Serial.print(pwm_1);
// Serial.print(" ");
// Serial.print(pwm_2);
// Serial.print(" ");
// Serial.println(pwm_3);
// Serial.print(psi_1);
// Serial.print(" ");
// Serial.print(psi_2);
// Serial.print(" ");
// Serial.print(psi_3);
// Serial.print(" ");

Serial.print(duration_1);
Serial.print(" ");
Serial.print(duration_2);
Serial.print(" ");
Serial.println(duration_3);


Serial.println("State Matrix:");
for (int i = 0; i < 8; i++) {
  Serial.print("State_Matrix[");
  Serial.print(i);
  Serial.print("]: ");
  Serial.println(State_Matrix(i),4);
}


// Serial.print(State_Matrix(0));
// Serial.print(" ");
// Serial.print(State_Matrix(1));
// Serial.print(" ");
// Serial.print(State_Matrix(2));
// Serial.print(" ");
// Serial.print(State_Matrix(3));
// Serial.print(" ");
// Serial.print(State_Matrix(4));
// Serial.print(" ");
// Serial.print(State_Matrix(5));
// Serial.print(" ");
// Serial.print(State_Matrix(6));
// Serial.print(" ");
// Serial.println(State_Matrix(7));

// Serial.print(total_pwm1);
// Serial.print(" ");
// Serial.print(total_pwm2);
// Serial.print(" ");
// Serial.println(total_pwm3);


// Serial.print(pidCorrection1); 
// Serial.print(" ");
// Serial.print(pidCorrection2);
// Serial.print(" ");
// Serial.println(pidCorrection3);

// Serial.print("D_1: ");
// Serial.print(D_pwm1);
// Serial.print("D_2: ");
// Serial.print(D_pwm2);
// Serial.print("D_3: ");
// Serial.println(D_pwm3);

// Reset duration
noInterrupts();  // Disable interrupts temporarily
duration_1 = 0;
duration_2 = 0;
duration_3 = 0;
interrupts();  // Re-enable interrupts

delay(100); 
}

/////////////////////////////////////////////////////////////////////////


// void CorrectionPID1(double Current_current1, double Desired_current_1) {

//   ballbotPID1.Compute();
//   // total_pwm1 = pidCorrection1*1.7/12*255;
//   total_pwm1 = (1.7*pidCorrection1+k_e*wheel_speed_1)/12*255;
//   if (total_pwm1 >0){
//     total_pwm1 += offset_percentage*255;
//   }

//   else{
//     total_pwm1 -= offset_percentage*255;
//   }
//   writeMotor(total_pwm1, Motor1_dir_1, Motor1_dir_2, PWM_channel_1);
// }

// void CorrectionPID2(double Current_current2, double Desired_current_2) {

//   ballbotPID2.Compute();
//   total_pwm2 = (1.7*pidCorrection2+k_e*wheel_speed_2)/12*255;
//   if (total_pwm2 > 0) {
//     total_pwm2 += offset_percentage * 255;
//   }

//   else {
//     total_pwm2 -= offset_percentage * 255;
//   }
//   writeMotor(total_pwm2, Motor2_dir_1, Motor2_dir_2, PWM_channel_2);
// }

// void CorrectionPID3(double Current_current3, double Desired_current3) {

//   ballbotPID3.Compute();
//   // total_pwm3 = pidCorrection3*1.7/12*255;
//   total_pwm3 = (1.7*pidCorrection3+k_e*wheel_speed_3)/12*255;
//   if (total_pwm3 > 0) {
//     total_pwm3 += offset_percentage * 255;
//   }

//   else {
//     total_pwm3 -= offset_percentage * 255;
//   }
//   writeMotor(total_pwm3, Motor3_dir_1, Motor3_dir_2, PWM_channel_3);
// }


// void CorrectionPID1(double wheel_speed, double desired_velocity) {

//   ballbotPID1.Compute();
//   total_pwm1 = k_e * pidCorrection1 / 12 * 255;
//   writeMotor(total_pwm1, Motor1_dir_1, Motor1_dir_2, PWM_channel_1);
// }

// void CorrectionPID2(double wheel_speed, double desired_velocity) {

//   ballbotPID2.Compute();
//   total_pwm2 = k_e * pidCorrection2 / 12 * 255;

//   writeMotor(total_pwm2, Motor2_dir_1, Motor2_dir_2, PWM_channel_2);
// }

// void CorrectionPID3(double wheel_speed, double desired_velocity) {

//   ballbotPID3.Compute();
//   total_pwm3 = k_e * pidCorrection3 / 12 * 255;

//   writeMotor(total_pwm3, Motor3_dir_1, Motor3_dir_2, PWM_channel_3);
// }



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

void wheelSpeed1() {
  int Lstate_1 = digitalRead(encoder0pinA_1);
  if((encoder0PinALast_1 == LOW) && Lstate_1 == HIGH) {
    int val_1 = digitalRead(encoder0pinB_1);
    if(val_1 == LOW && Direction_1) {
      Direction_1 = false; // Reverse
    } else if(val_1 == HIGH && !Direction_1) {
      Direction_1 = true; // Forward
    }
  }
  
  encoder0PinALast_1 = Lstate_1;
  
  if(!Direction_1) {
    duration_1++;
  } else {
    duration_1--;
  }
}

void wheelSpeed2() {
  int Lstate_2 = digitalRead(encoder0pinA_2);
  if((encoder0PinALast_2 == LOW) && Lstate_2 == HIGH) {
    int val_2 = digitalRead(encoder0pinB_2);
    if(val_2 == LOW && Direction_2) {
      Direction_2 = false; // Reverse
    } else if(val_2 == HIGH && !Direction_2) {
      Direction_2 = true; // Forward
    }
  }
  encoder0PinALast_2 = Lstate_2;
  
  if(!Direction_2) {
    duration_2++;
  } else {
    duration_2--;
  }
}
void wheelSpeed3() {
  int Lstate_3 = digitalRead(encoder0pinA_3);
  if((encoder0PinALast_3 == LOW) && Lstate_3 == HIGH) {
    int val_3 = digitalRead(encoder0pinB_3);
    if(val_3 == LOW && Direction_3) {
      Direction_3 = false; // Reverse
    } else if(val_3 == HIGH && !Direction_3) {
      Direction_3 = true; // Forward
    }
  }
  encoder0PinALast_3 = Lstate_3;
  
  if(!Direction_3) {
    duration_3++;
  } else {
    duration_3--;
  }
}

void EncoderInit_1() {
  Direction_1 = true; // Default -> Forward  
  pinMode(encoder0pinA_1, INPUT);  
  pinMode(encoder0pinB_1, INPUT);  
  encoder0PinALast_1 = digitalRead(encoder0pinA_1); // Initialize encoder0PinALast
  attachInterrupt(digitalPinToInterrupt(encoder0pinA_1), wheelSpeed1, CHANGE);
}

void EncoderInit_2() {
  Direction_2 = true; // Default -> Forward  
  pinMode(encoder0pinA_2, INPUT);  
  pinMode(encoder0pinB_2, INPUT);  
  encoder0PinALast_2 = digitalRead(encoder0pinA_2); // Initialize encoder0PinALast
  attachInterrupt(digitalPinToInterrupt(encoder0pinA_2), wheelSpeed2, CHANGE);
}

void EncoderInit_3() {
  Direction_3 = true; // Default -> Forward  
  pinMode(encoder0pinA_3, INPUT);  
  pinMode(encoder0pinB_3, INPUT);  
  encoder0PinALast_3 = digitalRead(encoder0pinA_3); // Initialize encoder0PinALast
  attachInterrupt(digitalPinToInterrupt(encoder0pinA_3), wheelSpeed3, CHANGE);
}

void capValue(float &omega) {
    if (omega > 7.016) {
        omega = 7.016;
    } else if (omega < -7.016) {
        omega = -7.016;
    }
}


void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

