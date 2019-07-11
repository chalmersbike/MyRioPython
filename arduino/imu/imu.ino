#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <math.h>
#define dt 0.008//0.013 //dt 0.005
#define gyroScale 131.0
#define degConv 57.29577
#define radConv 0.01745
#define a 0.997//0.995
#define b 0.8
#define h 0.43

#define accScale 16384.0


char inchar;
int pinLED = 13;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Time variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float timer, timer_reset;
float loop_time, start_time;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int gyro_address = 0x69;                   //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
float gyro_angle = 0.0;
int temperature;
long acc_x, acc_y, acc_z;
long gyro_pitch, gyro_roll, gyro_yaw;
long gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
long acc_total_vector = 0;
float angle_roll_acc, angle_roll;
float ax, ay, az, ay_roll_compensated;
float gx, gy, gz;
float angle_roll_rad, gx_rad;
float angle_roll_comp_rad, gx_filtered, gx_filtered_new, dgx, angle_roll_acc_comp, angle_roll_comp;
float gx_rad_abs, angle_roll_rad_abs,angle_roll_comp_rad_abs, ax_abs, ay_abs, az_abs, ay_roll_compensated_abs,gyro_angle_abs;
char gx_rad_sign, angle_roll_rad_sign, angle_roll_comp_rad_sign, ax_sign, ay_sign, az_sign, ay_roll_compensated_sign,gyro_angle_sign;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Calibration variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean use_manual_calibration = false;    // Set to false or true;
int calibration_number_readings = 2000;
int manual_gyro_pitch_cal_value = 0;
int manual_gyro_roll_cal_value = 0;
int manual_gyro_yaw_cal_value = 0;
int cal_int;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Error variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int imu_error_notEnoughData = 0;           //Error flag tracking if there is insufficient data to send
int imu_error_connectionLost = 1;          //Error flag tracking if Serial connection is lost ; initialize at 1 to force setup


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino setup (I/O and Serial)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(pinLED, OUTPUT); //LED
  Wire.begin();
  while(imu_error_notEnoughData || imu_error_connectionLost){
    gyro_setup();                          //Initiallize the gyro and set the correct registers.
  }

  if (!use_manual_calibration) {
    calibrate_gyro();                      //Calibrate the gyro offset.
  }
  Serial.begin(115200); //This pipes to the serial monitor
  while(!Serial);
  Serial.println("READY");
  timer_reset=micros(); //microsec
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino loop
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  // Start loop timer
  start_time = millis();


   // Read data from the IMU
  read_imu_data();
  //delayMicroseconds(1000);


  // Normalize and scale the data
  if (acc_total_vector == 0){
    acc_total_vector = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
  }
  //ax = (float)acc_x / accScale;
  //ay = (float)acc_y / accScale;
  //az = (float)acc_z / accScale;
  ax = (float)acc_x / acc_total_vector;
  ay = (float)acc_y / acc_total_vector;
  az = (float)acc_z / acc_total_vector;
  gx = (float)gyro_roll / gyroScale;
  gy = (float)gyro_pitch / gyroScale;
  gz = (float)gyro_yaw / gyroScale;


  // Compute the roll angle
  angle_roll_acc = degConv * atan2(ay, az);
  angle_roll = a * (angle_roll + gx * dt)  + (1 - a) * angle_roll_acc;  // Correct the drift of the gyro roll angle with the accelerometer roll angle.
  gyro_angle = gyro_angle + gx * dt;


  // Lateral Acceleration Compensation 
  gx_rad = gx * radConv;
  gx_filtered_new = b * gx_filtered_new + (1 - b) * gx_rad;
  dgx = (gx_filtered_new-gx_filtered) / dt;
  ay_roll_compensated = ay + (dgx * h) / 9.81;
  angle_roll_acc_comp = degConv * atan2(ay_roll_compensated, az);
  angle_roll_comp = a * (angle_roll_comp + gx * dt)  + (1 - a) * angle_roll_acc_comp;
  gx_filtered=gx_filtered_new;


  // Send data to the BeagleBone (BB)
  inchar = Serial.read(); // Check that the BB sent a message notifying that it is ready to receive data
  if(inchar=='1'){        // '1' means that the BB is ready to receive data
    // Filter out noise
    if(abs(gx_rad) < 0.0087){  //0.5 deg
      gx_rad = 0;
    }


    // Convert the angles to radians
    angle_roll_rad = angle_roll * radConv;
    angle_roll_comp_rad = angle_roll_comp * radConv;


    // Prepare the strings to send by separating absolute value and sign from the data
    gx_rad_abs = abs(gx_rad);
    angle_roll_rad_abs = abs(angle_roll_rad);
    angle_roll_comp_rad_abs = abs(angle_roll_comp_rad);
    ax_abs = abs(ax);
    ay_abs = abs(ay);
    az_abs = abs(az);
    ay_roll_compensated_abs = abs(ay_roll_compensated);
    gyro_angle_abs = abs(gyro_angle);

    gx_rad_sign = (gx > 0) ? '+' : '-';
    angle_roll_rad_sign = (angle_roll > 0) ? '+' : '-';
    angle_roll_comp_rad_sign = (angle_roll_comp > 0) ? '+' : '-';
    ax_sign = (ax > 0) ? '+' : '-';
    ay_sign  = (ay > 0) ? '+' : '-';
    az_sign  = (az > 0) ? '+' : '-';
    ay_roll_compensated_sign = (ay_roll_compensated > 0) ? '+' : '-';
    gyro_angle_sign  = (gyro_angle > 0) ? '+' : '-';

    // !!! This can be removed once the ternaries above are tested !!!
    /*
    if(gx > 0){
      gx_rad_sign = '+';
    } else {
      gx_rad_sign = '-';
    }

    if(angle_roll > 0){
      angle_roll_rad_sign = '+';
    } else {
      angle_roll_rad_sign = '-';
    }

    if(angle_roll_comp > 0){
      angle_roll_comp_rad_sign = '+';
    } else {
      angle_roll_comp_rad_sign = '-';
    }

    if(ax > 0){
      ax_sign = '+';
    } else {
      ax_sign = '-';
    }

    if(ay > 0){
      ay_sign = '+';
    } else {
      ay_sign = '-';
    }

    if(az > 0){
      az_sign = '+';
    } else {
      az_sign = '-';
    }

    if(ay_roll_compensated > 0){
      ay_roll_compensated_sign = '+';
    } else {
      ay_roll_compensated_sign = '-';
    }
    */


    // Send the data through Serial connection to the BB
    // Printed terms:
    // @ phi_roll_compensated # phi_uncompensated # phi_dot # a_x # a_y_roll_compensated # a_y # a_z $
    
    //      Serial.print(gx_filtered);
    //      Serial.print("\t");
    //      Serial.println(gx_rad);
    Serial.print("@");
    Serial.print(angle_roll_comp_rad_sign);
    Serial.print(angle_roll_comp_rad_abs,4);
    Serial.print("#");
    Serial.print(angle_roll_rad_sign);
    Serial.print(angle_roll_rad_abs,4);
    Serial.print("#");
    Serial.print(gx_rad_sign);
    Serial.print(gx_rad_abs);
    Serial.print("#");
//      Serial.print(dgx);
//      Serial.print("#");
//      Serial.print(gx_filtered);
//      Serial.print("#");
    Serial.print(ax_sign);
    Serial.print(ax_abs);
    Serial.print("#");
    Serial.print(ay_roll_compensated_sign);
    Serial.print(ay_roll_compensated_abs);
    Serial.print("#");   
    Serial.print(ay_sign);
    Serial.print(ay_abs);
    Serial.print("#");
    Serial.print(az_sign);
    Serial.print(az_abs);
//      Serial.print("#");
//      timer=millis()-timer_reset;
//      Serial.print(loop_time);
//      timer_reset=millis();
    Serial.print("#");
    Serial.print(gyro_angle_sign);
    Serial.print(gyro_angle_abs);
    Serial.println("$"); //$


    digitalWrite(13, HIGH); // !!! What is the point of this ? !!!
  }

  digitalWrite(13, LOW); // !!! What is the point of this ? !!!

  // Compute the current loop time and wait to match expected frequency
  loop_time = (millis() - start_time);
  delay(dt * 1000 - loop_time);
}
