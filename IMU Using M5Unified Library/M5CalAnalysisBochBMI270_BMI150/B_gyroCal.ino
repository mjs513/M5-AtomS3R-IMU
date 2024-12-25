////////////////////////////////////////////////////////////////////
//
// Gyro calibration (MPU9250 Library method)
//
////////////////////////////////////////////////////////////////////

void gyroCal(){
  // perform gyro calibration
  Serial.println("Gyro Calibration Underway. Do not move IMU..."); 
  calibrateGyro();
  gxb = getGyroBiasX_rads();
  gyb = getGyroBiasY_rads();
  gzb = getGyroBiasZ_rads();
  Serial.println("Gyro Calibration Complete"); 
  Serial.println(" ");
}

