////////////////////////////////////////////////////////////////////
//
// Accelerometer calibration (MPU9250 Library method)
//
////////////////////////////////////////////////////////////////////

void accelerometerCal(){
  // calibrating accelerometer
  Serial.println("Starting Accelerometer Calibration...");
  nextPage();
  Serial.println("Point X axis up...");
  nextPage();
  calibrateAccel();
  Serial.println("Point -X axis up...");
  nextPage();
  calibrateAccel();
  Serial.println("Point Y axis up...");
  nextPage();
  calibrateAccel();
  Serial.println("Point -Y axis up...");
  nextPage();
  calibrateAccel();
  Serial.println("Point Z axis up...");
  nextPage();
  calibrateAccel();
  Serial.println("Point -Z axis up...");
  nextPage();
  calibrateAccel();
  
  Serial.println("Accelerometer Calibration Complete");
  Serial.println(" ");
}

void nextPage()
{
  Serial.println("Press anykey to continue");
  while (Serial.read() != '\r') ;

}
