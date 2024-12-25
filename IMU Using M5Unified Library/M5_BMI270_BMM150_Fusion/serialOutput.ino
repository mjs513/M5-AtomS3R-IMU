
void telemetryPortOut(){  
  // Set textLength to the number of parameters to print * 31
  int  textLength = 22 * 31;
  char text[textLength];
  
  //Temporary text parameters
  char gyroxText[30];
  char gyroyText[30];
  char gyrozText[30];

  char accxText[30];
  char accyText[30];
  char acczText[30];

  char magxText[30];
  char magyText[30];
  char magzText[30];
  
  char pitchText[30];
  char rollText[30];
  char yawText[30];
  char fHeadingText[30];
  
  char qwText[30];
  char qxText[30];
  char qyText[30];
  char qzText[30];

  char f01[30]; char f02[30]; char f05[30];
  //char f01[30]; char f02[30]; char f03[30]; char f04[30]; char f05[30];

  dtostrf(val[3], 10, 10, gyroxText);
  dtostrf(val[4], 10, 10, gyroyText);
  dtostrf(val[5], 10, 10, gyrozText);
  dtostrf(val[0], 10, 10, accxText);
  dtostrf(val[1], 10, 10, accyText);
  dtostrf(val[2], 10, 10, acczText);
  dtostrf(val[6], 10, 10, magxText);
  dtostrf(val[7], 10, 10, magyText);
  dtostrf(val[8], 10, 10, magzText);
  dtostrf(euler.angle.pitch, 10, 10, pitchText);
  dtostrf(euler.angle.roll, 10, 10, rollText);
  dtostrf(euler.angle.yaw, 10, 10, yawText);
  dtostrf(fHeading, 10, 10, fHeadingText);
  
  dtostrf( q.element.w, 10, 6, qwText);
  dtostrf(q.element.x, 10, 6, qxText);
  dtostrf(q.element.y, 10, 6, qyText);
  dtostrf(q.element.z, 10, 6, qzText);  

  dtostrf((float)flags.initialising, 10, 6, f01);
  dtostrf((float)flags.accelerationRecovery, 10, 6, f02);
  //dtostrf((float)flags.accelerationRejectionTimeout, 10, 6, f03);
  //dtostrf((float)flags.magneticRejectionWarning, 10, 6, f04);
  dtostrf((float)flags.magneticRecovery, 10, 6, f05);

  // Create single text parameter and print it
  snprintf(text, textLength, "%s, %s, %s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
           gyroxText, gyroyText, gyrozText,
           accxText, accyText, acczText,
           magxText, magyText, magzText,
           pitchText, rollText, yawText,fHeadingText,
           qwText, qxText, qyText, qzText,
           f01, f02, f05);
/*
  dtostrf((float)flags.initialising, 10, 6, f01);
  dtostrf((float)flags.accelerationRejectionWarning, 10, 6, f02);
  dtostrf((float)flags.accelerationRejectionTimeout, 10, 6, f03);
  dtostrf((float)flags.magneticRejectionWarning, 10, 6, f04);
  dtostrf((float)flags.magneticRejectionTimeout, 10, 6, f05);

  // Create single text parameter and print it
  snprintf(text, textLength, "%s, %s, %s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
           gyroxText, gyroyText, gyrozText,
           accxText, accyText, acczText,
           magxText, magyText, magzText,
           pitchText, rollText, yawText,fHeadingText,
           qwText, qxText, qyText, qzText,
           f01, f02, f03, f04, f05);
*/
    Serial.println(text);  
  //Serial.send_now();

} 

void print_float_array(float *arr, int size) {
  if (size == 0) {
    Serial.printf("[]");
  } else {
    Serial.print('[');
    for (int i = 0; i < size - 1; i++)
      Serial.printf("%f, ", arr[i]);
    Serial.printf("%f]", arr[size - 1]);
  }
}