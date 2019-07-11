#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <math.h>
#define dt 0.008//0.013 //dt 0.005
#define gyroScale 65.5
#define degConv 57.29577
#define radConv 0.01745
#define a 0.997//0.995
#define b 0.8
#define h 0.43

char inchar;
float timer, timer_reset;
boolean use_manual_calibration = false;    // Set to false or true;


int manual_gyro_pitch_cal_value = 0;
int manual_gyro_roll_cal_value = 0;
int manual_gyro_yaw_cal_value = 0;
int gyro_address = 0x69;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
long loop_t;
int cal_int;
int temperature;
long acc_x, acc_y, acc_z;
long gyro_pitch, gyro_roll, gyro_yaw;
long gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
long acc_total_vector;
float angle_roll_acc, angle_roll;
float ax, ay, az, ay_roll_compensated;
float gx, gy, gz;
float loop_time, start_time;
float gyro_angle=0.0;
float angle_roll_rad, gx_rad;
float gx_rad_abs, angle_roll_rad_abs;
char gx_rad_sign, angle_roll_rad_sign;

float angle_roll_comp_rad_abs, angle_roll_comp_rad, gx_filtered, gx_filtered_new, dgx, angle_roll_acc_comp, angle_roll_comp;
char angle_roll_comp_rad_sign;




void setup() {

  pinMode(13, OUTPUT); //LED
  Wire.begin();
//  gyro_setup();                                                 //Initiallize the gyro and set the correct registers.
  while(gyro_setup3());                                                 //Initiallize the gyro and set the correct registers.

  if (!use_manual_calibration) {
  calibrate_gyro();                                             //Calibrate the gyro offset.

  }
  Serial.begin(115200); //This pipes to the serial monitor
  while(!Serial);
  Serial.println("READY");
  timer_reset=micros(); //microsec
}


void loop(){

    start_time = millis(); //milliseconds
    gyro_signalen();
    //delayMicroseconds(1000);

    acc_total_vector = sqrt(acc_x*acc_x+acc_y*acc_y+acc_z*acc_z);
    ax = (float)acc_x/acc_total_vector;
    ay = (float)acc_y/acc_total_vector;
    az = (float)acc_z/acc_total_vector;
    gx = (float)gyro_roll / gyroScale;
    gy = (float)gyro_pitch / gyroScale;
    gz = (float)gyro_yaw / gyroScale;


    angle_roll_acc = degConv * atan2(ay, az);
    angle_roll = a * (angle_roll + gx * dt)  + (1 - a) * angle_roll_acc;      //Correct the drift of the gyro roll angle with the accelerometer roll angle.
    gyro_angle=gyro_angle+gx * dt;

////Lateral Acceleration Compensation////

    gx_rad = gx * radConv;

    gx_filtered_new=b*gx_filtered_new+(1-b)*gx_rad;

    dgx=(gx_filtered_new-gx_filtered)/dt;

    ay_roll_compensated=ay+(dgx*h)/9.81;

    angle_roll_acc_comp = degConv * atan2(ay_roll_compensated, az);
    angle_roll_comp = a * (angle_roll_comp + gx * dt)  + (1 - a) * angle_roll_acc_comp;

    gx_filtered=gx_filtered_new;


/////////////////////////////////////////



    // send data
    inchar=Serial.read();
    if(inchar=='1'){

//      // filter out noise
      if( abs(gx_rad)< 0.0087 ){    //0.5 deg
        gx_rad = 0;
      }

      angle_roll_rad = angle_roll * radConv;
      angle_roll_comp_rad = angle_roll_comp * radConv;

      gx_rad_abs=abs(gx_rad);
      angle_roll_rad_abs=abs(angle_roll_rad);
      angle_roll_comp_rad_abs=abs(angle_roll_comp_rad);


      if( gx > 0 ){
        gx_rad_sign = '+';
      } else {
        gx_rad_sign = '-';
      }


      if( angle_roll > 0 ){
        angle_roll_rad_sign = '+';
      } else {
        angle_roll_rad_sign = '-';
      }


///
      if( angle_roll_comp > 0 ){
        angle_roll_comp_rad_sign = '+';
      } else {
        angle_roll_comp_rad_sign = '-';
      }
//    Printed terms:
//    @ phi_roll_compensated # phi_uncompensated # phi_dot # a_x # a_y_roll_compensated # a_y # a_z $
//      Serial.print(gx_filtered);
//      Serial.print("\t");
//      Serial.println(gx_rad);
      Serial.print("@"); //@
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

      if( ax >= 0 ){
        Serial.print("+");
      }

      Serial.print(ax);
      Serial.print("#");

     if( ay_roll_compensated >= 0 ){
        Serial.print("+");
      }
      Serial.print( ay_roll_compensated);
      Serial.print("#");

     if( ay >= 0 ){
        Serial.print("+");
      }

      Serial.print(ay);
      Serial.print("#");
      Serial.print(az);

//      Serial.print("#");
//      timer=millis()-timer_reset;
//      Serial.print(loop_time);
//      timer_reset=millis();
      Serial.println("$"); //$


      digitalWrite(13, HIGH);
    }

    digitalWrite(13, LOW);



    loop_time=(millis()-start_time);
    delay((dt*1000-loop_time));


}