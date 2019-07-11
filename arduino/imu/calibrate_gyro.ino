///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the average gyro offset of calibration_number_readings readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  cal_int = 0;                                                                      //If manual calibration is not used.
  manual_gyro_pitch_cal_value = 0;                                                  //Set the manual pitch calibration variable to 0.
  manual_gyro_roll_cal_value = 0;                                                   //Set the manual roll calibration variable to 0.
  manual_gyro_yaw_cal_value = 0;                                                    //Set the manual yaw calibration variable to 0.

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < calibration_number_readings ; cal_int++) {           //Take calibration_number_readings readings for calibration.
    if (cal_int % 125 == 0) digitalWrite(pinLED, !digitalRead(pinLED));             //Change the led status every 125 readings to indicate calibration.
    read_imu_data();                                                               //Read the gyro output.
    if (!imu_error_notEnoughData && !imu_error_connectionLost){                     //Check that no error happened during the reading.
      gyro_roll_cal += gyro_roll;                                                   //Add roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                 //Add pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                     //Add yaw value to gyro_yaw_cal.
      delay(4);                                                                     //Small delay to simulate a 250Hz loop during calibration.
    }else{
      cal_int -= 1;                                                                 //Decrement the readings counter by 1 to remove this erroneous reading.
    }
  }
  gyro_roll_cal /= calibration_number_readings;                                     //Divide the roll total by the number of readings.
  gyro_pitch_cal /= calibration_number_readings;                                    //Divide the pitch total by the number of readings.
  gyro_yaw_cal /= calibration_number_readings;                                      //Divide the yaw total by the number of readings.
  manual_gyro_pitch_cal_value = gyro_pitch_cal;                                     //Set the manual pitch calibration variable to the detected value.
  manual_gyro_roll_cal_value = gyro_roll_cal;                                       //Set the manual roll calibration variable to the detected value.
  manual_gyro_yaw_cal_value = gyro_yaw_cal;                                         //Set the manual yaw calibration variable to the detected value.
}
