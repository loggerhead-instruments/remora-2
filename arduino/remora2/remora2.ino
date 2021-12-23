// Copyright Loggerhead Instruments, 2021
// David Mann

// This version does not record to microSD, it will transmit data over serial
// To do:
// - read settings at start from Record Teensy
// - get time from Teensy if in settings file
// - enable clock prescaler in mode 0

// Remora2 is an underwater motion datalogger with audio recording and playback
// ATMEGA328p: low-power motion datalogging
// Dual Teensy 3.2: Audio playback and record

// Operation
// 1. Motion(ATMEGA328) controls power to playback and record Teensy booards.
// 2. Record and playback start automatically when power is turned on to Play and Rec boards from Motion Atmega.
// 3. Record stops when it receives a trigger from motion Atmega.
// 4. Play stops when file (SDTEST1.wav) finished. Play board sets PLAY_STATUS pin high when playing, low when done.
// 5. Settings file on Record card can be used to set sample rate and file length
// 6. Realtime clock is set from Atmega. The Atmega will send the time to REC Teensy over serial. Atmega has a temperature compenstated real-time clock that is more accurate than Teensy clock.

// Elephant seal 
// We plan to only have one sound per deployment.
// Plan is to have play sound on ascent - think will pick threshold of about 400m and play after ascend ~ 50 m, but still finalizing.
// Ideally would like to only expose animal once every 8 - 12 hours (still finalizing)
// Record/Playback delay: Would like an ~ 2 week delay so should be transitioning more to foraging and then record and play playback for 5 weeks. 
// We would like to set the time to be a duration that all the tags should last for so all the animals are exposed for the same amount of time (i.e. we don't want some seals exposed for 4 weeks and others 7). 
// end record x minutes after playback started): Ideally we would like to record for a few minutes before and after the playback so we know the background sound.  
// Maybe  start recording on the playback dive as soon as it depth is reached and stop recording 1-2 minutes after the playback.

// Power Consumption
// 140 mA during record and playback
// 11 mA with CP=2
// 3 mA if unplug IMU (so 8 mA is IMU)

// 0.8 mA with IMU powered down; delay start
// 2 mA recording depth sensor only with IMU board powered down

// Wiring
// - Jumper wire from R18 (ClockBat) to both Teensy Clock Bat 
// - BURN1 GND to IMU GND to power off IMU during sleep

// Current consumption
// 20 Hz on motion sensors; clock prescaler = 2;

#include <SPI.h>
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
#include <avr/io.h>
#include <avr/boot.h>

ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

//
// DEV SETTINGS
//
char codeVer[12] = "2021-12-21";

unsigned long recDur = 120; // minutes 1140 = 24 hours
int recInt = 0;
int LED_EN = 1; //enable green LEDs flash 1x per pressure read. Can be disabled from script.

boolean HALL_EN = 0; 
boolean HALL_LED_EN = 0; //flash red LED for Hall sensor

//#define pressAddress 0x76
//float MS58xx_constant = 8192.0; // for 30 bar sensor
float pressureOffset_mbar;
// float MS58xx_constant = 327680.0; // for 2 bar sensor; will switch to this if 30 bar fails to give good depth

// Playback Settings
float playBackDepthThreshold = 400.0; // tag must go deeper than this depth to trigger threshold
float ascentDepthTrigger = 100.0; // after exceed playBackDepthThreshold, must ascend this amount to trigger playback
float ascentRecordTrigger = 75.0; // after exceed playBackDepthThreshold, must ascend this amount to trigger record
float playBackResetDepth = 20.0; // tag needs to come back above this depth before next playback can happen
int maxPlayBacks = 80; // maximum number of times to play
unsigned int minPlayBackInterval = 10; // keep playbacks from being closer than x minutes default: 540
float delayRecPlayDays = 0.0; // delay record/playback for x days. Default 14
float maxPlayDays = 42.0; // maximum time window for playbacks from tag on; e.g. 42 days
byte recMinutesAfterPlay = 2;

// Playback status
float maxDepth;  
byte playNow = 0;
byte playBackDepthExceeded = 0;
volatile unsigned int nPlayed = 0;
volatile boolean REC_STATE, PLAY_STATE;
float daysFromStart;

boolean simulateDepth = 1;
#define nDepths 10
float depthProfile[] = {0.1, 500.0, 450.0, 420.0, 300.0, 10.0, 5.0, 50.0, 100.0, 200.0
                      }; //simulated depth profile; one value per minute; max of 10 values because running out of memory
byte depthIndex = 0;
byte oldMinute;

// pin assignments
#define chipSelect  10
#define LED_RED A3 
#define LED_GRN 4 
#define BURN 8     // PB0
#define REC_ST 9   // PB1 REC Start/Stop
#define BUTTON1 A2 // PC2
#define BAT_VOLTAGE A7// ADC7
#define HALL 3 // PD3 (INT1)
#define REC_POW 6 // PD6
#define IMU_INT 7 // PD7
#define PLAY_POW 5 // PD5 Rev 2 of board
#define PLAY_ST A1 // PC1 Rev 2 of board Play Start/Stop
#define REC_STATUS A0 // Rev 2 of board
#define PLAY_STATUS A6 // Rev 2 of board


int ssCounter; // used to get different sample rates from one timer based on imu_srate
byte clockprescaler=2;  //clock prescaler

//
// SENSORS
//
//byte imuTempBuffer[20];
int imuSrate = 20; // must be integer for timer
int sensorSrate = 1; // must divide into imuSrate
int slowRateMultiple = imuSrate / sensorSrate;
int speriod = 1000 / imuSrate;

////Pressure and temp calibration coefficients
//uint16_t PSENS; //pressure sensitivity
//uint16_t POFF;  //Pressure offset
//uint16_t TCSENS; //Temp coefficient of pressure sensitivity
//uint16_t TCOFF; //Temp coefficient of pressure offset
//uint16_t TREF;  //Ref temperature
//uint16_t TEMPSENS; //Temperature sensitivity coefficient
// byte Tbuff[3];
// byte Pbuff[3];
volatile float depth = 0.0;
volatile float temperature, pressure_mbar;
boolean togglePress = 0; // flag to toggle conversion of temperature and pressure

//int16_t accelX, accelY, accelZ;
//int16_t magX, magY, magZ;
//int16_t gyroX, gyroY, gyroZ;

//int accel_scale = 16;

// impeller spin counter
volatile int spin;

// System Modes and Status
int mode = 0; //standby = 0; running = 1
volatile float voltage;
volatile boolean writeSlowSensorsFlag = 0;
volatile boolean writeMotionSensorsFlag = 0;
volatile boolean changeToModeZero = 0;

// Time
volatile byte second = 0;
volatile byte minute = 0;
volatile byte hour = 17;
volatile byte day = 1;
volatile byte month = 1;
volatile byte year = 17;

volatile unsigned long t, endTime, startUnixTime, playTime;
volatile unsigned long startTime = 0;
int burnFlag = 0;
long burnSeconds;
void setup() {
  Serial.begin(115200);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BURN, OUTPUT);  // used to control power to IMU board
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(HALL, INPUT);
  pinMode(BAT_VOLTAGE, INPUT); 
  pinMode(IMU_INT, INPUT_PULLUP); // PD7
  pinMode(PLAY_ST, OUTPUT); // trigger for Play Teensy
  pinMode(REC_ST, OUTPUT); // trigger for Record Teensy
  pinMode(REC_STATUS, INPUT);
  pinMode(PLAY_STATUS, INPUT);
  pinMode(REC_POW, OUTPUT);
  pinMode(PLAY_POW, OUTPUT); // PD6
  digitalWrite(PLAY_ST, LOW);
  digitalWrite(REC_ST, LOW);
  digitalWrite(REC_POW, HIGH);  // turn on Teensy so can reprogram
  digitalWrite(PLAY_POW, HIGH);
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_GRN,HIGH);
  digitalWrite(BURN, HIGH);

  Wire.begin();
  Wire.setClock(400000);
  
  // recalculate sample rates in case changed from script
  slowRateMultiple = imuSrate / sensorSrate;
  speriod = 1000 / imuSrate;

  initSensors();
  // this sometimes fails
  // critical to update time here
  while(readRTC()==0){ 
    delay(100);
  }
  startUnixTime = t; // time tag turned on
  playTime = t;
  digitalWrite(BURN, LOW); // power down IMU

  if(startTime==0) startTime = t + 5;
//  Serial.print("Time:"); Serial.println(t);
//  Serial.print("Start Time:"); Serial.println(startTime);
//  Serial.print("UT:"); Serial.println( );
//  Serial.println(simulateDepth);
  delay(10);

  wdtInit();  // used to wake from sleep
  //setClockPrescaler(clockprescaler); // set clockprescaler from script file; this affects the baud rate
  oldMinute = minute;
}

void loop() {

  // wait in mode 0 while delay to play is in effect and while checkPlay is waiting to start record
  while(mode==0){
    // resetWdt();
    digitalWrite(BURN, HIGH); // power on IMU because may be messing with I2C
    readRTC();
    digitalWrite(BURN, LOW); // power on IMU
    if((t - startUnixTime) > 3600){
      LED_EN = 0; // disable green LED flashing after 3600 s
    }
   
    if(LED_EN){
      digitalWrite(LED_GRN, HIGH);
      digitalWrite(LED_RED, HIGH);
      delay(3);
    }
    digitalWrite(LED_GRN, LOW);
    digitalWrite(LED_RED, LOW);
    
    enterSleep();

    daysFromStart = (float) (t - startUnixTime) / 86400.0;
    if((daysFromStart >= delayRecPlayDays) & (daysFromStart < maxPlayDays)){
      if(simulateDepth){
        if (minute != oldMinute){
          oldMinute = minute;
          depthIndex++;
          if(depthIndex>=nDepths) depthIndex = 0;
          depth = depthProfile[depthIndex];
          delay(100);
          Serial.print("Min since last play:");
          Serial.println((t - playTime)/60);
          serialWriteSlowSensors();
          delay(100);
        }
      }
      else{
        kellerConvert(); // start new depth reading
        delay(100);
        kellerRead(); // read new depth value
      }

      // start tracking depths once minPlayBackInterval is exceeded
      if (((t - playTime)/60 > minPlayBackInterval)){
        // check if time to start recording; recording will start when checkPlay returns 1
        if(checkPlay()==1){
          digitalWrite(BURN, HIGH); // power on IMU
          delay(5);
          myICM.begin( Wire, 1 );
         // icmSetup();
         // updateTemp();  // get first reading ready
          mode = 1;
          setClockPrescaler(0); // run full speed during data acquisition so have full bandwidth serial
          startInterruptTimer(speriod, 0);
        }
      }
    }
  } // mode = 0

  //
  // Send Motion Data Out Serial Port and Wait for Record to End
  //
  while(mode==1){
    if(simulateDepth){
        if (minute != oldMinute){
          oldMinute = minute;
          depthIndex++;
          if(depthIndex>=nDepths) depthIndex = 0;
        }
        depth = depthProfile[depthIndex];
      }
   // resetWdt();
   if((t - startUnixTime) > 3600) LED_EN = 0; // disable green LED flashing after 3600 s

   // check if data to write over serial
   if(writeMotionSensorsFlag){
      serialWriteImu(); // write IMU to Serial
      writeMotionSensorsFlag = 0;
   }
   if(writeSlowSensorsFlag){
    serialWriteSlowSensors();
    writeSlowSensorsFlag = 0;
   }

   if (changeToModeZero == 1){
    stopTimer();
   // setClockPrescaler(clockprescaler);  // slow down clock to save power
    mode = 0;        
    changeToModeZero = 0;  
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
  delay(2000);
  readVoltage();
//  Serial.print(voltage);
//  Serial.println("V");

  // Sends DT to Record Teensy
  readRTC();
  Serial.print("DT "); Serial.println(t);
  Serial.print("T "); Serial.print(hour); Serial.print(":");
  Serial.print(minute); Serial.print(":");
  Serial.println(second);
  Serial.flush();
  delay(5000);

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

  float pressureSum = 0;

  if(kellerInit()){
    kellerConvert();
    delay(20);
    kellerRead();
    for(int n=1; n<10; n++){
        kellerConvert();
        delay(20);
        kellerRead();
        delay(100);
        
        pressureSum+= pressure_mbar;
        pressureOffset_mbar = pressureSum / n;
    }
  }
//  Serial.print("mBar "); Serial.println(pressure_mbar);
//  Serial.print("Off "); Serial.println(pressureOffset_mbar);
//  Serial.print("Depth "); Serial.println(depth);
//  Serial.print("Temp "); Serial.println(temperature);
//  Serial.flush();
//  delay(5000);

//  myICM.begin( Wire, 1 );
//  if( myICM.status != ICM_20948_Stat_Ok ){
//      Serial.println( "ICM fail" );
//      delay(500);
//  }


  digitalWrite(REC_ST, HIGH);  // start recording
  delay(3000);
//  for(int i = 0; i<5; i++){
//    Serial.print(digitalRead(REC_STATUS));
//    delay(500);
//  }
  digitalWrite(REC_ST, LOW); // stop recording
  for(int i = 0; i<10; i++){
   // Serial.print(digitalRead(REC_STATUS));
    delay(500);
  }
  digitalWrite(REC_POW, LOW);
  digitalWrite(PLAY_POW, LOW);
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

//void printImu(){
//  Serial.print("a/m/g:\t");
//  Serial.print(myICM.accX()); Serial.print("\t");
//  Serial.print(myICM.accY()); Serial.print("\t");
//  Serial.print(myICM.accZ()); Serial.print("\t");
//  Serial.print(myICM.magX()); Serial.print("\t");
//  Serial.print(myICM.magY()); Serial.print("\t");
//  Serial.print(myICM.magZ()); Serial.print("\t");
//  Serial.print(myICM.gyrX()); Serial.print("\t");
//  Serial.print(myICM.gyrY()); Serial.print("\t");
//  Serial.println(myICM.gyrZ());
//}

void serialWriteImu(){
  Serial.println(); // force start on new line
  Serial.print(myICM.accX()); Serial.print(",");
  Serial.print(myICM.accY()); Serial.print(",");
  Serial.print(myICM.accZ()); Serial.print(",");
  Serial.print(myICM.magX()); Serial.print(",");
  Serial.print(myICM.magY()); Serial.print(",");
  Serial.print(myICM.magZ()); Serial.print(",");
  Serial.print(myICM.gyrX()); Serial.print(",");
  Serial.print(myICM.gyrY()); Serial.print(",");
  Serial.print(myICM.gyrZ());
}

void serialWriteSlowSensors(){
  //Serial.println(depth);
  Serial.print(','); Serial.print(year);  
  Serial.print('-');
  if(month < 10) Serial.print('0');
  Serial.print(month);
  Serial.print('-');
  if(day < 10) Serial.print('0');
  Serial.print(day);
  Serial.print('T');
  if(hour < 10) Serial.print('0');
  Serial.print(hour);
  Serial.print(':');
  if(minute < 10) Serial.print('0');
  Serial.print(minute);
  Serial.print(':');
  if(second < 10) Serial.print('0');
  Serial.print(second);
  Serial.print(",");
  Serial.print(pressure_mbar);
  Serial.print(','); Serial.print(depth);
  Serial.print(','); Serial.print(temperature);
  Serial.print(','); Serial.print(voltage);
  Serial.print(','); Serial.print(REC_STATE);
  Serial.print(','); Serial.print(PLAY_STATE);
  if(HALL_EN){
      Serial.print(','); Serial.print(spin);
  }
}

/********************************************************************
***        Master Interrupt Routine to Read Sensors               ***
********************************************************************/
void sampleSensors(void){  
    ssCounter++;

   // calcImu();
    myICM.getAGMT();

//  // MS58xx start temperature conversion half-way through
  if((ssCounter>=(0.5 * slowRateMultiple))  & togglePress){ 
//    readPress();   
//    updateTemp();
    togglePress = 0;
    if(simulateDepth==0) kellerConvert();
  }
    
  if(ssCounter>=slowRateMultiple){
//    // MS58xx pressure and temperature
//    readTemp();
//    updatePress();  
    if(simulateDepth==0) kellerRead();
    togglePress = 1;

    if(LED_EN) digitalWrite(LED_GRN, HIGH);
    readRTC();
    //calcPressTemp(); // MS58xx pressure and temperature
    readVoltage();
    writeSlowSensorsFlag = 1;
    
    int retVal = checkPlay();
    if(retVal==0) changeToModeZero = 1; // when done recording change to mode 0
    ssCounter = 0;
    spin = 0; //reset spin counter
    digitalWrite(LED_GRN, LOW);
  }
  writeMotionSensorsFlag = 1; // always write motion sensors, set flag after slow sensors flag
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
