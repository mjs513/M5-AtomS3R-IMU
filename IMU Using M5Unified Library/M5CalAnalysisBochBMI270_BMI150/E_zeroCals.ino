////////////////////////////////////////////////////////////////////
//
// Zeros out calibrations in EEPROM and MPU9250 Library
//
////////////////////////////////////////////////////////////////////

void zeroCalValues(){
  axb = 0.0f;
  ayb = 0.0f;
  azb = 0.0f;  
  axs = 1.0f;
  ays = 1.0f;
  azs = 1.0f;
  hxb = 0.0f;
  hyb = 0.0f;
  hzb = 0.0f;  
  hxs = 1.0f;
  hys = 1.0f;
  hzs = 1.0f;
  gxb = 0.0f;
  gyb = 0.0f;
  gzb = 0.0f;


  zeroIMUBiases();
}

