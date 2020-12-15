  // Copyright Loggerhead Instruments, 2020
// David Mann

// Remora2 is an underwater motion datalogger with audio recording and playback
// ATMEGA328p: low-power motion datalogging
// Dual Teensy 3.2: Audio playback and record

// To Do: 
// - Deep sensor (separate code base because running out of memory) 
// - Teensy wake and record
// Enough memory to talk to Teensy?

// Current consumption
// 20 Hz on motion sensors; clock prescaler = 2;

#include <SPI.h>
#include <SdFat.h>
#include <MsTimer2.h> 
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <prescaler.h>
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#include <Wire.h>
//#include <SoftWire.h>
#include <avr/io.h>
#include <avr/boot.h>

ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
//SoftWire Wire = SoftWire();

//
// DEV SETTINGS
//
char codeVer[12] = "2020-12-03";

int recDur = 3600; // 3600 seconds per hour
int recInt = 0;
int LED_EN = 1; //enable green LEDs flash 1x per pressure read. Can be disabled from script.

boolean HALL_EN = 1; 
boolean HALL_LED_EN = 1; //flash red LED for Hall sensor

#define pressAddress 0x76
float MS58xx_constant = 8192.0; // for 30 bar sensor
// float MS58xx_constant = 327680.0; // for 2 bar sensor; will switch to this if 30 bar fails to give good depth

// Playback
float playBackDepthThreshold = 10.0; // tag must go deeper than this depth to trigger threshold
float ascentDepthTrigger = 5.0; // after exceed playBackDepthThreshold, must ascend this amount to trigger playback
float playBackResetDepth = 2.0; // tag needs to come back above this depth before next playback can happen
int maxPlayBacks = 20; // maximum number of times to play
float maxDepth;  
byte playNow = 0;
boolean playBackDepthExceeded = 0;
int minPlayBackInterval = 120; // keep playbacks from being closer than x seconds
int longestPlayback = 30; // longest file for playback, used to power down playback board
int nPlayed = 0;

// pin assignments
#define chipSelect  10
#define LED_RED A3 
#define LED_GRN 4 
#define BURN 8     // PB0
#define TEENSY_ST 9   // PB1
#define BUTTON1 A2 // PC2
#define BAT_VOLTAGE A7// ADC7
#define HALL 3 // PD3 (INT1)
#define GPS_EN 5 //PD5
#define TEENSY_POW 6 // PD6
#define IMU_INT 7 // PD7
#define RXD2 A0 // PC0
#define TXD2 A1 // PC1


// SD file system
SdFat sd;
File dataFile;
int fileCount; 

int ssCounter; // used to get different sample rates from one timer based on imu_srate
byte clockprescaler=1;  //clock prescaler

//
// SENSORS
//
byte imuTempBuffer[20];
int imuSrate = 20; // must be integer for timer
int sensorSrate = 1; // must divide into imuSrate
int slowRateMultiple = imuSrate / sensorSrate;
int speriod = 1000 / imuSrate;

//Pressure and temp calibration coefficients
uint16_t PSENS; //pressure sensitivity
uint16_t POFF;  //Pressure offset
uint16_t TCSENS; //Temp coefficient of pressure sensitivity
uint16_t TCOFF; //Temp coefficient of pressure offset
uint16_t TREF;  //Ref temperature
uint16_t TEMPSENS; //Temperature sensitivity coefficient
byte Tbuff[3];
byte Pbuff[3];
volatile float depth, temperature, pressure_mbar;
boolean togglePress = 0; // flag to toggle conversion of temperature and pressure

//int16_t accelX, accelY, accelZ;
//int16_t magX, magY, magZ;
//int16_t gyroX, gyroY, gyroZ;

int accel_scale = 16;

// impeller spin counter
volatile int spin;

// System Modes and Status
int mode = 0; //standby = 0; running = 1
volatile float voltage;

// Time
volatile byte second = 0;
volatile byte minute = 0;
volatile byte hour = 17;
volatile byte day = 1;
volatile byte month = 1;
volatile byte year = 17;

unsigned long t, startTime, endTime, burnTime, startUnixTime, playTime;
int burnFlag = 0;
long burnSeconds;
void setup() {
  Serial.begin(115200);
  delay(5000);

  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BURN, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(HALL, INPUT);
  pinMode(BAT_VOLTAGE, INPUT);
  pinMode(GPS_EN, OUTPUT); //PD5
  pinMode(TEENSY_POW, OUTPUT); // PD6
  pinMode(IMU_INT, INPUT_PULLUP); // PD7
  pinMode(TEENSY_ST, OUTPUT); // trigger for Record Teensy
  //pinMode(RXD2, INPUT); // PC0
  //pinMode(TXD2, OUTPUT); // PC1
  pinMode(TEENSY_ST, INPUT_PULLUP);   // PB1
  
  digitalWrite(BURN,LOW);
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_GRN,HIGH);
  digitalWrite(BURN, LOW);
  digitalWrite(GPS_EN, LOW);
  digitalWrite(TEENSY_ST, LOW);
  digitalWrite(TEENSY_POW, HIGH);

  // test trigger recording
  delay(5000);
  digitalWrite(TEENSY_ST, HIGH);
  delay(100);
  digitalWrite(TEENSY_ST, LOW);
  

//  Serial.println("Remora 2");
  Wire.begin();
  Wire.setClock(400000);
  sd.begin(chipSelect, SPI_FULL_SPEED);

  loadScript(); // do this early to set time
  // recalculate sample rates in case changed from script
  slowRateMultiple = imuSrate / sensorSrate;
  speriod = 1000 / imuSrate;

  initSensors();
  
  readRTC();
  startUnixTime = t;
  logFileWrite();
  
  if(burnFlag==2){
  burnTime = t + burnSeconds;
//  Serial.print("Burn set");
//  Serial.println(burnTime);
  }

  if(startTime==0) startTime = t + 3;
//  Serial.print("Time:"); Serial.println(t);
//  Serial.print("Start Time:"); Serial.println(startTime);
  digitalWrite(LED_GRN, LOW);
  digitalWrite(LED_RED, LOW);

 // setClockPrescaler(clockprescaler); // set clockprescaler from script file
  wdtInit();  // used to wake from sleep

}

void loop() {
while(mode==0){
   // resetWdt();
    readRTC();
    checkBurn();
    Serial.print(t); Serial.print(" "); Serial.println(startTime);

    if(LED_EN){
      digitalWrite(LED_GRN, HIGH);
      digitalWrite(LED_RED, HIGH);
      delay(3);
    }
    digitalWrite(LED_GRN, LOW);
    digitalWrite(LED_RED, LOW);
    enterSleep();

    if(t >= startTime){
      endTime = startTime + recDur;
      startTime += recDur + recInt;  // this will be next start time for interval record      mpuInit(1);
      fileInit();
      updateTemp();  // get first reading ready
      mode = 1;
      startInterruptTimer(speriod, clockprescaler);
      attachInterrupt(digitalPinToInterrupt(HALL), spinCount, RISING);
    }
  } // mode = 0


  while(mode==1){
   // resetWdt();
    
    // check if time to close
    if(t>=endTime){
      stopTimer();
      dataFile.close(); // close file
      if(t - startUnixTime > 3600) LED_EN = 0; // disable green LED flashing after 3600 s
      
      if(recInt==0){  // no interval between files
        endTime += recDur;  // update end time
        fileInit();
        startInterruptTimer(speriod, clockprescaler);
      }
      else{
        mode = 0;
        wdtInit(); // start wdt
      }
    }

    // Check if stop button pressed
    if(digitalRead(BUTTON1)==0){
      delay(10); // simple deBounce
      if(digitalRead(BUTTON1)==0){
        stopTimer();
        digitalWrite(LED_RED, HIGH);
        dataFile.close();
        delay(30000);
        // wait 30 s to stop
        startInterruptTimer(speriod, clockprescaler);
        fileInit();
        digitalWrite(LED_RED, LOW);
      }
    }
  } // mode = 1
}

boolean ledState;
void spinCount(){
  ledState = !ledState;
  if(HALL_LED_EN) digitalWrite(LED_RED, ledState);
  spin++;
}

void initSensors(){
  readVoltage();
  Serial.print(voltage);
  Serial.println("V");
//  if(voltage < 3.5){
//    showFail(50); //battery voltage read fail
//  }
//  reset_alarm();

//  setTime2(12,0,0,5,12,20); 
//  readRTC();
//  int oldSecond = second;

//  digitalWrite(LED_RED, HIGH);
//  delay(1000);
//  for(int i=0; i<hour; i++){
//    delay(300);
//    digitalWrite(LED_GRN, HIGH);
//    delay(80);
//    digitalWrite(LED_GRN, LOW);
//  }
//  delay(400);
//  digitalWrite(LED_RED, LOW);
//  readRTC();
//  Serial.print(hour); Serial.print(":");
//  Serial.print(minute); Serial.print(":");
//  Serial.println(second);
//  if(second==oldSecond){
//    // showFail(100); // clock not ticking
//    Serial.println("CF");
//  }

  // Pressure/Temperature
//  if (pressInit()==0){
//    Serial.println("PF");
//   // showFail(200); // pressure sensor fail
//  }
//  Serial.println("P D T");
//  for(int x=0; x<20; x++){
//    updatePress();
//    delay(100);
//    readPress();
//    updateTemp();
//    delay(100);
//    readTemp();
//    calcPressTemp();
//    Serial.print(pressure_mbar); Serial.print(" ");
//    Serial.print(depth); Serial.print(" ");
//    Serial.println(temperature);
//  }
  Serial.print("i");
  myICM.begin( Wire, 1 );
  if( myICM.status != ICM_20948_Stat_Ok ){
      Serial.println( "ICM fail" );
      delay(500);
  }
  //icmSetup();

  for(int i=0; i<10; i++){
   if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
//    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    printImu();   // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
    }else{
    Serial.println("ICM");
    delay(500);
  }
  }
}

void showFail(int blinkInterval){
  digitalWrite(LED_GRN, LOW);
  int count = 5000 / blinkInterval;
  if(count < 100) count = 100;
  for(int n=0; n<count; n++){
    digitalWrite(LED_RED, HIGH);
    delay(blinkInterval);
    digitalWrite(LED_RED, LOW);
    delay(blinkInterval);
  }
}

//void calcImu(){
//  accelX = (int16_t) ((int16_t)imuTempBuffer[0] << 8 | imuTempBuffer[1]);    
//  accelY = (int16_t) ((int16_t)imuTempBuffer[2] << 8 | imuTempBuffer[3]);   
//  accelZ = (int16_t) ((int16_t)imuTempBuffer[4] << 8 | imuTempBuffer[5]);    
//  
// // gyroTemp = (int16_t) (((int16_t)imuTempBuffer[6]) << 8 | imuTempBuffer[7]);   
// 
//  gyroX = (int16_t)  (((int16_t)imuTempBuffer[8] << 8) | imuTempBuffer[9]);   
//  gyroY = (int16_t)  (((int16_t)imuTempBuffer[10] << 8) | imuTempBuffer[11]); 
//  gyroZ = (int16_t)  (((int16_t)imuTempBuffer[12] << 8) | imuTempBuffer[13]);   
//  
//  magX = (int16_t)  (((int16_t)imuTempBuffer[14] << 8) | imuTempBuffer[15]);   
//  magY = (int16_t)  (((int16_t)imuTempBuffer[16] << 8) | imuTempBuffer[17]);   
//  magZ = (int16_t)  (((int16_t)imuTempBuffer[18] << 8) | imuTempBuffer[19]);  
//}

void printImu(){
  Serial.print("a/m/g:\t");
  Serial.print(myICM.accX()); Serial.print("\t");
  Serial.print(myICM.accY()); Serial.print("\t");
  Serial.print(myICM.accZ()); Serial.print("\t");
  Serial.print(myICM.magX()); Serial.print("\t");
  Serial.print(myICM.magY()); Serial.print("\t");
  Serial.print(myICM.magZ()); Serial.print("\t");
  Serial.print(myICM.gyrX()); Serial.print("\t");
  Serial.print(myICM.gyrY()); Serial.print("\t");
  Serial.println(myICM.gyrZ());
}

void fileWriteImu(){
  dataFile.print(myICM.accX()); dataFile.print(",");
  dataFile.print(myICM.accY()); dataFile.print(",");
  dataFile.print(myICM.accZ()); dataFile.print(",");
  dataFile.print(myICM.magX()); dataFile.print(",");
  dataFile.print(myICM.magY()); dataFile.print(",");
  dataFile.print(myICM.magZ()); dataFile.print(",");
  dataFile.print(myICM.gyrX()); dataFile.print(",");
  dataFile.print(myICM.gyrY()); dataFile.print(",");
  dataFile.print(myICM.gyrZ());
}

void fileWriteSlowSensors(){
  dataFile.print(','); dataFile.print(year);  
  dataFile.print('-');
  if(month < 10) dataFile.print('0');
  dataFile.print(month);
  dataFile.print('-');
  if(day < 10) dataFile.print('0');
  dataFile.print(day);
  dataFile.print('T');
  if(hour) dataFile.print('0');
  dataFile.print(hour);
  dataFile.print(':');
  if(minute < 10) dataFile.print('0');
  dataFile.print(minute);
  dataFile.print(':');
  if(second < 10) dataFile.print('0');
  dataFile.print(second);
  dataFile.print("Z,");
  dataFile.print(pressure_mbar);
  dataFile.print(','); dataFile.print(depth);
  dataFile.print(','); dataFile.print(temperature);
  dataFile.print(','); dataFile.print(voltage);
  if(HALL_EN){
      dataFile.print(','); dataFile.print(spin);
  }
}

void logFileWrite()
{
   readRTC();
   
   File logFile = sd.open("log.txt", O_WRITE | O_CREAT | O_APPEND);
   logFile.print("Code version:"); logFile.println(codeVer);
   logFile.print("Serial Number: ");
   for (uint8_t i = 14; i < 24; i += 1) {
       logFile.print(boot_signature_byte_get(i), HEX);
   }
   logFile.println();
   if(MS58xx_constant == 8192.0) {
    logFile.println("30 Bar");
   }
   else{
    logFile.println("2 Bar");
   }
   
   logFile.print(year);  logFile.print("-");
   logFile.print(month); logFile.print("-");
   logFile.print(day); logFile.print("T");
   logFile.print(hour); logFile.print(":");
   logFile.print(minute); logFile.print(":");
   logFile.println(second);

   logFile.close();
}

void fileInit()
{
   char filename[60];
   sprintf(filename,"%02d%02d%02dT%02d%02d%02d.csv", year, month, day, hour, minute, second);  //filename is DDHHMM
   dataFile = sd.open(filename, O_WRITE | O_CREAT | O_APPEND);
   while (!dataFile){
    digitalWrite(LED_RED, HIGH);
    fileCount += 1;
    //sprintf(filename,"F%06d.txt",fileCount); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
   // Serial.println(filename);
    delay(100);
   }
   digitalWrite(LED_RED, LOW);
   dataFile.print("accelX,accelY,accelZ,magX,magY,magZ,gyroX,gyroY,gyroZ,date,mBar,depth,temperature,V");
   if(HALL_EN) dataFile.print(",spin");
   dataFile.println();
   SdFile::dateTimeCallback(file_date_time);
  // Serial.println(filename);
}

/********************************************************************
***        Master Interrupt Routine to Read Sensors               ***
********************************************************************/
void sampleSensors(void){  
    ssCounter++;

   // calcImu();
    myICM.getAGMT();
    fileWriteImu();

  // MS58xx start temperature conversion half-way through
  if((ssCounter>=(0.5 * slowRateMultiple))  & togglePress){ 
    readPress();   
    updateTemp();
    togglePress = 0;
  }
    
  if(ssCounter>=slowRateMultiple){
    // MS58xx pressure and temperature
    readTemp();
    updatePress();  
    togglePress = 1;
    
    if(LED_EN) digitalWrite(LED_GRN, HIGH);
    readRTC();
    checkBurn();
    calcPressTemp(); // MS58xx pressure and temperature
    readVoltage();
    fileWriteSlowSensors();
    checkPlay();
    ssCounter = 0;
    spin = 0; //reset spin counter
    digitalWrite(LED_GRN, LOW);
  }
    dataFile.println();
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  *date=FAT_DATE(year + 2000,month,day);
  *time=FAT_TIME(hour,minute,second);
}

int checkBurn(){
  if((t>=burnTime) & (burnFlag>0)){
    digitalWrite(BURN, HIGH);
  }
}

void readVoltage(){
  voltage = analogRead(BAT_VOLTAGE) * 0.0042;
}

void startInterruptTimer(int speriod, byte clockprescaler){
    MsTimer2::set(speriod>>clockprescaler, sampleSensors); // bitshift by clockprescaler...will round if not even
    MsTimer2::start();
}

void stopTimer(){
    MsTimer2::stop();
}

ISR(WDT_vect)
{
    // do nothing
}


void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}

void wdtInit(){
    /*** Setup the WDT ***/
  
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  //WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  WDTCSR = (0<<WDP3 )|(1<<WDP2 )|(1<<WDP1)|(0<<WDP0);  // 1 s
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
}
