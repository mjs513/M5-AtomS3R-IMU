////////////////////////////////////////////////////////////////////
//
// Magnetometer calibration (MPU9250 Library method)
//
////////////////////////////////////////////////////////////////////

void magnetometerCal(){
  Serial.println("Starting Magnetometer Calibration in 2 seconds...");
  delay(2000);
  
  // calibrating magnetometer
  Serial.println("Move IMU in slow figure-8 pattern in each of the following");
  Serial.println("  orientations, about 5 sec each. Stay clear of magnetic");
  Serial.println("  fields or metal. Repeat entire process until solution");
  Serial.println("  achieved.");
  Serial.println("    +X axis up");
  Serial.println("    -X axis up");
  Serial.println("    +Y axis up");
  Serial.println("    -Y axis up");
  Serial.println("    +Z axis up");
  Serial.println("    -Z axis up");
  calibrateMag();
  Serial.println("Magnetometer calibration solution achieved.");
}

