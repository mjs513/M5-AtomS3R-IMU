////////////////////////////////////////////////////////////////////
//
// IMU Routines
//
////////////////////////////////////////////////////////////////////

void zeroIMUBiases(){
/*
  Imu.setAccelCalX(0.0f, 1.0f);
  Imu.setAccelCalY(0.0f, 1.0f);
  Imu.setAccelCalZ(0.0f, 1.0f);
  
  Imu.setGyroBiasX_rads(0.0f);
  Imu.setGyroBiasY_rads(0.0f);
  Imu.setGyroBiasZ_rads(0.0f);

  Imu.setMagCalX(0.0f, 1.0f);
  Imu.setMagCalY(0.0f, 1.0f);
  Imu.setMagCalZ(0.0f, 1.0f);
*/
  Serial.println("Calibration Values Reset in IMU");
  Serial.println(" ");
}

void loadBiasesIMU(){
  /*
  Imu.setAccelCalX(axb, 1.0f);
  Imu.setAccelCalY(ayb, 1.0f);
  Imu.setAccelCalZ(azb, 1.0f);
  
  Imu.setGyroBiasX_rads(gxb);
  Imu.setGyroBiasY_rads(gyb);
  Imu.setGyroBiasZ_rads(gzb);

  Imu.setMagCalX(hxb, 1.0f);
  Imu.setMagCalY(hyb, 1.0f);
  Imu.setMagCalZ(hzb, 1.0f);
*/
  Serial.println("Calibration Values Loaded in IMU");
  Serial.println(" ");
}

void printIMUBiases(){

  Serial.println("IMU Biases:");
  Serial.print("Accel X Bias:         ");Serial.println(getAccelBiasX_mss(),6);
  Serial.print("Accel X Scale Factor: ");Serial.println(getAccelScaleFactorX(),6);
  Serial.print("Accel Y Bias:         ");Serial.println(getAccelBiasY_mss(),6);
  Serial.print("Accel Y Scale Factor: ");Serial.println(getAccelScaleFactorY(),6);
  Serial.print("Accel Z Bias:         ");Serial.println(getAccelBiasZ_mss(),6);
  Serial.print("Accel Z Scale Factor: ");Serial.println(getAccelScaleFactorZ(),6);

  Serial.print("Gyro X Bias:          ");Serial.println(getGyroBiasX_rads(),6);
  Serial.print("Gyro Y Bias:          ");Serial.println(getGyroBiasY_rads(),6);
  Serial.print("Gyro Z Bias:          ");Serial.println(getGyroBiasZ_rads(),6);

  Serial.print("Mag X Bias:           ");Serial.println(getMagBiasX_uT(),6);
  Serial.print("Mag X Scale Factor:   ");Serial.println(getMagScaleFactorX(),6);
  Serial.print("Mag Y Bias:           ");Serial.println(getMagBiasY_uT(),6);
  Serial.print("Mag Y Scale Factor:   ");Serial.println(getMagScaleFactorY(),6);
  Serial.print("Mag Z Bias:           ");Serial.println(getMagBiasZ_uT(),6);
  Serial.print("Mag Z Scale Factor:   ");Serial.println(getMagScaleFactorZ(),6);
  Serial.println(" ");
}



