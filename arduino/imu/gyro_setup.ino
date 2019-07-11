///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the various registers of the MPU-6050 are set.
//The function returns 0 if no error was met, otherwise it returns 1
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_setup(void){
  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  Wire.write(0x01);                                            //Set the register bits as 00000000 to activate the gyro.
  if (Wire.endTransmission()){                                 //End the transmission with the gyro.
    imu_error_connectionLost = 1;                              //Set the error flag for loss of connection to 1.
    return;                                                    //Exit function on error.
  }

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  Wire.write(0x00);                                            //Set the register bits as 00000000 (250dps full scale).
  if (Wire.endTransmission()){                                 //End the transmission with the gyro.
    imu_error_connectionLost = 1;                              //Set the error flag for loss of connection to 1.
    return;                                                    //Exit function on error.
  }

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1C hex).
  Wire.write(0x00);                                            //Set the register bits as 00000000 (+/- 2g full scale range).
  if (Wire.endTransmission()){                                 //End the transmission with the gyro.
    imu_error_connectionLost = 1;                              //Set the error flag for loss of connection to 1.
    return;                                                    //Exit function on error.
  }

  Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.

  Wire.write(0x1A);                                            //We want to write to the (gyro) CONFIG register (1A hex).

  Wire.write(0x06);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  if (Wire.endTransmission()){                                 //End the transmission with the gyro.
    imu_error_connectionLost = 1;                              //Set the error flag for loss of connection to 1.
    return;                                                    //Exit function on error.
  }

  imu_error_connectionLost = 0;                                //Set the error flag for loss of connection to 0.
}
