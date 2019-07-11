/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void calibrate_gyro2(void) {
//  if (use_manual_calibration)cal_int = 2000;                                          //If manual calibration is used set cal_int to 2000 to skip the calibration.
//  else {
//    cal_int2 = 0;                                                                      //If manual calibration is not used.
//    manual_gyro_pitch_cal_value2 = 0;                                                  //Set the manual pitch calibration variable to 0.
//    manual_gyro_roll_cal_value2 = 0;                                                   //Set the manual roll calibration variable to 0.
//    manual_gyro_yaw_cal_value2 = 0;                                                    //Set the manual yaw calibration variable to 0.
//  }
//
//  if (cal_int2 != 2000) {
//    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
//    for (cal_int2 = 0; cal_int2 < 2000 ; cal_int2 ++) {                                  //Take 2000 readings for calibration.
//      if (cal_int2 % 25 == 0) digitalWrite(13, !digitalRead(13));                    //Change the led status every 125 readings to indicate calibration.
//      gyro_signalen2();                                                                //Read the gyro output.
//      gyro_roll_cal2 += gyro_roll2;                                                     //Ad roll value to gyro_roll_cal.
//      gyro_pitch_cal2 += gyro_pitch2;                                                   //Ad pitch value to gyro_pitch_cal.
//      gyro_yaw_cal2 += gyro_yaw2;                                                       //Ad yaw value to gyro_yaw_cal.
//      delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
//    }
//     gyro_roll_cal2 /= 2000;                                                            //Divide the roll total by 2000.
//    gyro_pitch_cal2 /= 2000;                                                           //Divide the pitch total by 2000.
//    gyro_yaw_cal2 /= 2000;                                                             //Divide the yaw total by 2000.
//    manual_gyro_pitch_cal_value2 = gyro_pitch_cal2;                                     //Set the manual pitch calibration variable to the detected value.
//    manual_gyro_roll_cal_value2 = gyro_roll_cal2;                                       //Set the manual roll calibration variable to the detected value.
//    manual_gyro_yaw_cal_value2 = gyro_yaw_cal2;                                         //Set the manual yaw calibration variable to the detected value.
//  }
//}