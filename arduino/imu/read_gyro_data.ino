///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_imu_data(void) {
  Wire.beginTransmission(gyro_address);                       //Start communication with the gyro.
  Wire.write(0x3B);                                           //Send the address of the register from which we will read data (3B for a MPU-6050)
  // !!! What is the meaning of this comment that was here before ? !!!      //Start reading @ register 43h and auto increment with every read.
  
  // For a MPU-6050, the data is organized as follows (register address on top, data on bottom), totalling 14 bytes to read:
  //       3B                 3C                3D                 3E                3F                 40               41               42              43                44                45                46                47                48
  // ACCEL_XOUT[15:8] | ACCEL_XOUT[7:0] | ACCEL_YOUT[15:8] | ACCEL_YOUT[7:0] | ACCEL_ZOUT[15:8] | ACCEL_ZOUT[7:0] | TEMP_OUT[15:8] | TEMP_OUT[7:0] | GYRO_XOUT[15:8] | GYRO_XOUT[7:0] | GYRO_YOUT[15:8] | GYRO_YOUT[7:0] | GYRO_ZOUT[15:8] | GYRO_ZOUT[7:0]
  
  if (Wire.endTransmission()){                                //End the transmission with the gyro.
    imu_error_connectionLost = 1;                             //Set the error flag for loss of connection to 1.
    return;                                                   //Exit function on error.
  }else{
    imu_error_connectionLost = 0;                             //Set the error flag for loss of connection to 0.
    if (Wire.requestFrom(gyro_address, 14)<14){               //Request 14 bytes from the MPU 6050.
      imu_error_notEnoughData = 1;                            //Set the error flag for insufficient data to 1.
      return;                                                 //Exit function on error.
    }else{
      imu_error_notEnoughData = 0;                            //Set the error flag for insufficient data to 0.
      acc_x = (Wire.read() << 8 | Wire.read());              //Add the low and high byte to the acc_x variable.
      acc_y = (Wire.read() << 8 | Wire.read());              //Add the low and high byte to the acc_y variable.
      acc_z = (Wire.read() << 8 | Wire.read());               //Add the low and high byte to the acc_z variable.
      temperature = Wire.read() << 8 | Wire.read();           //Add the low and high byte to the temperature variable.
      gyro_roll = Wire.read() << 8 | Wire.read();             //Read high and low part of the angular data.
      gyro_pitch = Wire.read() << 8 | Wire.read();            //Read high and low part of the angular data.
      gyro_yaw = Wire.read() << 8 | Wire.read();              //Read high and low part of the angular data.
      gyro_roll -= manual_gyro_roll_cal_value;                //Subtract the manual gyro roll calibration value.
      gyro_pitch -= manual_gyro_pitch_cal_value;              //Subtract the manual gyro pitch calibration value.
      gyro_yaw -= manual_gyro_yaw_cal_value;                  //Subtract the manual gyro yaw calibration value.
    }
  }
}
