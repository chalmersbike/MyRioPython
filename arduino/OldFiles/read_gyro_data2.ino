/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////This part reads the raw gyro and accelerometer data from the MPU-6050
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void gyro_signalen2(void) {
//  Wire.beginTransmission(gyro_address2);                       //Start communication with the gyro.
//  Wire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
//  Wire.endTransmission();                                     //End the transmission.
//  Wire.requestFrom(gyro_address2, 14);                         //Request 14 bytes from the MPU 6050.
//  //while(Wire.available()<14);
//  acc_x2 = -(Wire.read() << 8 | Wire.read());                    //Add the low and high byte to the acc_x variable.
//  acc_y2 = -(Wire.read() << 8 | Wire.read());                    //Add the low and high byte to the acc_y variable.
//  acc_z2 = (Wire.read() << 8 | Wire.read());                    //Add the low and high byte to the acc_z variable.
//  temperature2 = Wire.read() << 8 | Wire.read();              //Add the low and high byte to the temperature variable.
//  gyro_roll2 = Wire.read() << 8 | Wire.read();                //Read high and low part of the angular data.
//  gyro_pitch2 = Wire.read() << 8 | Wire.read();               //Read high and low part of the angular data.
//  gyro_yaw2 = Wire.read() << 8 | Wire.read();                 //Read high and low part of the angular data.
//  gyro_roll2 *= -1;                                             //Invert the direction of the axis.
//  gyro_pitch2 *= -1;                                            //Invert the direction of the axis.
//  gyro_yaw2 *= -1;                                              //Invert the direction of the axis.
//  gyro_roll2 -= manual_gyro_roll_cal_value2;                     //Subtact the manual gyro roll calibration value.
//  gyro_pitch2 -= manual_gyro_pitch_cal_value2;                   //Subtact the manual gyro pitch calibration value.
//  gyro_yaw2 -= manual_gyro_yaw_cal_value2;                       //Subtact the manual gyro yaw calibration value.
//}