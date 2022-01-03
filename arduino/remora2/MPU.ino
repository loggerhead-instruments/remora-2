#define GyroAddress 0x69
#define CompassAddress = 0x0C  //0x0C internal compass on 9150

byte mpuInit(boolean imuMode){
  byte ecode;
  if(imuMode==0)
  {
     ecode = I2Cwrite(GyroAddress, 0x06, 0x40);  //Sleep mode, internal 8 MHz oscillator  //another mode is cycle where it wakes up periodically to take a value
     return ecode;
  }
}

byte I2Cwrite(byte addr, byte reg, byte val)
{
  Wire.beginTransmission(addr);  
  Wire.write(reg);  // gyro scale, sample rate and LPF
  Wire.write(val);  
  byte ecode=Wire.endTransmission(); //end transmission
  delay(5);
  return ecode;
}
