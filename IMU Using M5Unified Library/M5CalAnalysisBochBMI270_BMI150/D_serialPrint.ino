////////////////////////////////////////////////////////////////////
//
// Routine to stream IMU raw data and heading to serial
//
////////////////////////////////////////////////////////////////////

void serialPrint(){

  //Needed for Tviewer
  int textIndex;
  textIndex = 10 * 31;
  char text[textIndex];
  float values[9];
  //float headingVal, hxVal, hyVal, hzVal;
   
  readIMU(values);

  float hxVal = values[6];
  float hyVal = values[7];
  float hzVal = values[8];
  float headingVal;
  
  if (-hyVal > 0){
    headingVal = (PI/2.0f - ( atanf(hxVal/(-hyVal)) ) ) * 180.0f/PI;
  }
  else {
    headingVal = (3.0f*PI/2.0f - ( atanf(hxVal/(-hyVal)) ) ) * 180.0f/PI;
  }
   
  char ax_text[30];
  char ay_text[30];
  char az_text[30];
  char gx_text[30];
  char gy_text[30];
  char gz_text[30];
  char hx_text[30];
  char hy_text[30];
  char hz_text[30];
  char heading_text[30];

  dtostrf(values[0] * G, 10,4, ax_text);
  dtostrf(values[1] * G, 10,4, ay_text);
  dtostrf(values[2] * G, 10,4, az_text);
  dtostrf(values[3] * deg2rad, 10,4, gx_text);
  dtostrf(values[4] * deg2rad, 10,4, gy_text);
  dtostrf(values[5] * deg2rad, 10,4, gz_text);
  dtostrf(hxVal, 10,4, hx_text);
  dtostrf(hyVal, 10,4, hy_text);
  dtostrf(hzVal, 10,4, hz_text);
  dtostrf(headingVal, 10,4, heading_text);

  snprintf(text, textIndex, "%s,%s,%s,%s,%s, %s,%s,%s,%s,%s", 
        ax_text, ay_text, az_text,
        gx_text, gy_text, gz_text, 
        hx_text, hy_text, hz_text, heading_text );
  Serial.println(text);
}

