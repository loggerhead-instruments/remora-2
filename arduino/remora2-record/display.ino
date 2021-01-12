float mAmpRec = 45;  // actual about 43 mA
float mAmpSleep = 2.8; // actual about 2.6 mA
byte nBatPacks = 1;
float mAhPerBat = 12000.0; // assume 12Ah per battery pack; good batteries should be 14000

uint32_t freeMB;
uint32_t filesPerCard;
csd_t m_csd;

void displayOn(){
  //display.ssd1306_command(SSD1306_DISPLAYON);
  display.init();
  display.setBatteryVisible(true);
} 

void displayOff(){
  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  display.print(":");
  printZero(digits);
  display.print(digits);
}

void printZero(int val){
  if(val<10) display.print('0');
}

void setTeensyTime(int hr, int mn, int sc, int dy, int mh, int yr){
  tmElements_t tm;
  tm.Year = yr - 1970;
  tm.Month = mh;
  tm.Day = dy;
  tm.Hour = hr;
  tm.Minute = mn;
  tm.Second = sc;
  time_t newtime;
  newtime = makeTime(tm); 
  Teensy3Clock.set(newtime); 
}

void cDisplay(){
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
}

void displaySettings(){
  t = getTeensy3Time();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 18);
  display.print("Rec:");
  display.print(rec_dur);
  display.println("s ");
  display.print("Sleep:");
  display.print(rec_int);
  display.println("s  ");
  display.printf("%.1f kHz",lhi_fsamps[isf]/1000.0f);
  display.print(" ");
  display.printf("%.1f",gainDb);
  display.print("dB gain");
}

void displayClock(time_t t, int loc){
  display.setTextSize(1);
  display.setCursor(0,loc);
  display.print(year(t));
  display.print('-');
  display.print(month(t));
  display.print('-');
  display.print(day(t));
  display.print("  ");
  printZero(hour(t));
  display.print(hour(t));
  printDigits(minute(t));
  printDigits(second(t));
}

void printTime(time_t t){
  Serial.print(year(t));
  Serial.print('-');
  Serial.print(month(t));
  Serial.print('-');
  Serial.print(day(t));
  Serial.print(" ");
  Serial.print(hour(t));
  Serial.print(':');
  Serial.print(minute(t));
  Serial.print(':');
  Serial.println(second(t));
}

void readEEPROM(){
  isf = EEPROM.read(0);
  gainSetting = EEPROM.read(1);
  if(isf<0 | isf>6) isf = I_SAMP;
  if(gainSetting<0 | gainSetting>13) gainSetting = 4;
}

union {
  byte b[4];
  long lval;
}u;

long readEEPROMlong(int address){
  u.b[0] = EEPROM.read(address);
  u.b[1] = EEPROM.read(address + 1);
  u.b[2] = EEPROM.read(address + 2);
  u.b[3] = EEPROM.read(address + 3);
  return u.lval;
}

void writeEEPROMlong(int address, long val){
  u.lval = val;
  EEPROM.write(address, u.b[0]);
  EEPROM.write(address + 1, u.b[1]);
  EEPROM.write(address + 2, u.b[2]);
  EEPROM.write(address + 3, u.b[3]);
}

void writeEEPROM(){
  EEPROM.write(0, isf); //byte
  EEPROM.write(1, gainSetting); //byte
}
