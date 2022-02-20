// Copyright Loggerhead Instruments, 2021
// David Mann

// This version does not record to microSD, it will transmit data over serial

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
// 72 mA during record (96 MHz clock rate) LED ON
// 60 mA during record (96 MHz clock rate) LED OFF
// 70 mA during record (72 MHz clock rate) LED ON
// 0.4 mA reading depth sensor and IMU powered down

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
char codeVer[12] = "2022-02-20";

unsigned long recDur = 120; // minutes 1140 = 24 hours
int recInt = 0;
int LED_EN = 1; //enable green LEDs flash 1x per pressure read. Can be disabled from script.

float pressureOffset_mbar;

// Playback Settings
int16_t playBackDepthThreshold = 275; // tag must be deeper than this depth to start playback. Default 275
int16_t ascentRateTrigger = 100; // tag must ascend this amount in 3 minutes to trigger playback. Default 100
int16_t maxPlayBacks = 80; // maximum number of times to play. Default 80
uint16_t minPlayBackInterval = 5; // minutes from end of one rec/playback session to start of next. Default: 540
float delayRecPlayDays = 0.0; // delay record/playback for x days. Default 20
byte recMinutes = 2; // record this many minutes Default 20
byte playDelaySeconds = 30;  // seconds to start playback after start recording

// Playback status
unsigned int nPlayed = 0;
byte PLAY_STATE, REC_STATE = 0;
float daysFromStart;

boolean simulateDepth = 1;
#define nDepths 10
int16_t depthProfile[] = {2, 2, 4, 0, 0, -2, -2, -4, 0, 0
                      }; //delta depth per second, value changes once per minutebyte depthIndex = 0;
byte simulateIndex = 0;
byte simulateIndexCounter = 0;
byte checkSimulateCounter = 0;
#define checkSimulatePeriod 60
uint16_t oldDepth;
byte oldMinute;

// track depth for N_HISTORY seconds
// this is used to calculate an ascent rate over an N_HISTORY period
// e.g. N_HISTORY 180 will store 3 minutes of depth data in depthHistory[]
#define N_HISTORY 18
volatile byte depthHistoryIndex = 0;
int16_t depthHistory[N_HISTORY];
int16_t deltaDepth = 0;
byte checkDepthCounter = 0;
#define checkDepthPeriod 10

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

//Pressure and temperature
volatile float depth = 0.0;
volatile float temperature, pressure_mbar;
boolean togglePress = 0; // flag to toggle conversion of temperature and pressure

// System Modes and Status
byte loopMode = 0; //standby = 0; running = 1
float voltage;
volatile boolean writeSlowSensorsFlag = 0;
volatile boolean writeMotionSensorsFlag = 0;

// Time
volatile byte second = 0;
volatile byte minute = 0;
volatile byte hour = 17;
volatile byte day = 1;
volatile byte month = 1;
volatile byte year = 17;

volatile unsigned long t, endTime, startUnixTime, playTime, recTime, oldDepthCheckTime;
volatile unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BURN, OUTPUT);  // used to control power to IMU board
  pinMode(BUTTON1, INPUT_PULLUP);
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

  delay(100);

  Wire.begin();
  Wire.setClock(100000); // running at 100 kHz to make reading clock more reliable
  
  // recalculate sample rates in case changed from script
  slowRateMultiple = imuSrate / sensorSrate;
  speriod = 1000 / imuSrate;

  initSensors();
  setTime2(0, 0, 0, 0, 1, 1);

  if(simulateDepth) digitalWrite(REC_POW, HIGH);  // if simulate depth leave record Teensy on to monitor serial
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
//  Serial.print("Start Unix Time:"); Serial.println(startUnixTime);
//  Serial.print("Start Time:"); Serial.println(startTime);
//  Serial.print("UT:"); Serial.println( );
//  Serial.println(simulateDepth);
  delay(10);

  wdtInit();  // used to wake from sleep
  setClockPrescaler(clockprescaler); // set clockprescaler from script file; this affects the baud rate
  oldMinute = minute;
  depth = 0;
  oldDepth = depth;
  oldDepthCheckTime = t;
}

void loop() {
  // wait in loopMode 0 while waiting to start playback
  while(loopMode==0){
    // resetWdt();
    digitalWrite(BURN, HIGH); // power on IMU because may be messing with I2C
    delay(10);
    if(readRTC()){ 
      delay(10);
      unsigned long temp_t = t;
      if(readRTC()) if(t - temp_t < 2){ // make sure two subsequent readings of RTC are within 1 second of each other
        kellerConvert(); // start new depth reading
        digitalWrite(BURN, LOW); // power off IMU
        
  //      Serial.print("t2:");
  //      Serial.println(t - startUnixTime);
  //      delay(100);
  //      Serial.println(LED_EN);
        
        if((t - startUnixTime) > 3600) LED_EN = 0; // disable green LED flashing after 3600 s
        digitalWrite(LED_GRN, LOW);
        digitalWrite(LED_RED, LOW);
    
        setClockPrescaler(clockprescaler);
        
        enterSleep();  // sleep 0.25 s (because need this to loop faster than 1s)
        //delay(150);
    
        setClockPrescaler(0);
    
        if(LED_EN){
          digitalWrite(LED_GRN, HIGH);
          digitalWrite(LED_RED, HIGH);
        }
    
        // EVERY SECOND
        // - calculate or read new depth
        // - store depth history
        // - calculate ascent rate over 3 minutes
        if(t - oldDepthCheckTime >= 1){
          oldDepthCheckTime = t;
          if(simulateDepth) depth = depth + (float) depthProfile[simulateIndex];  // calculate new depth
          else{
            digitalWrite(BURN, HIGH); // power up IMU so can use I2C
            kellerRead(); // read new depth value
            digitalWrite(BURN, LOW); // power down IMU
          }
          
          // EVERY 10 SECONDS timed with checkDepthCounter
          // - update depth history
          checkDepthCounter++;
          if(checkDepthCounter>=checkDepthPeriod){
            checkDepthCounter = 0;
            // store depth history
            depthHistory[depthHistoryIndex] = (uint16_t) depth;
            depthHistoryIndex++;
            if(depthHistoryIndex>=N_HISTORY) depthHistoryIndex = 0;
            checkDepthCounter = 0;
      
            // calculate ascent rate over 3 minutes
            oldDepth = depthHistory[depthHistoryIndex];
            deltaDepth = oldDepth - (uint16_t) depth;
          }
      
          // EVERY MINUTE
          // - if simulate depth, update ascent rate
          checkSimulateCounter++;
          if(checkSimulateCounter>=checkSimulatePeriod){
            checkSimulateCounter = 0;
             if(simulateDepth){
                simulateIndex++;
                if(simulateIndex>=nDepths) simulateIndex = 0;
             }
          }
    
          //      Serial.print("Min since last play:");
          //      Serial.println((t - playTime)/60);
    //      Serial.print(" oD:");Serial.print(oldDepth);
          Serial.print(" D:"); Serial.print(depth);
          Serial.print(" dd:"); Serial.print(deltaDepth);
          Serial.print(" "); Serial.println(second);
          //      Serial.print(" DepthT:");Serial.print(playBackDepthThreshold);
          //      Serial.print(" ascentT:"); Serial.println(ascentRateTrigger);
          Serial.flush();
        }
    
        daysFromStart = (float) (t - startUnixTime) / 86400.0;
        // check if time to start record/playback sequence
        if((daysFromStart >= delayRecPlayDays) & (nPlayed < maxPlayBacks) & (((t - playTime)/60) >= minPlayBackInterval)){          
          // check if depths satisfy playback sequence
          if((depth > playBackDepthThreshold) & (deltaDepth > ascentRateTrigger)){
            digitalWrite(BURN, HIGH); // power on IMU
            delay(100);
            myICM.begin( Wire, 1 );
            myICM.getAGMT();  // for some reason need this so when read from interrupt get good readings
            loopMode = 1;
            digitalWrite(REC_POW, HIGH); // turn on recorder
            digitalWrite(REC_ST, HIGH);  // start recording
            REC_STATE = 1;
            PLAY_STATE = 0;
            startInterruptTimer(speriod, 0);
            recTime = t;
            oldMinute = minute; // used to update simulate depth during playback
            // Serial.print("loopMode:"); Serial.println(loopMode);
            digitalWrite(LED_GRN, LOW);
            digitalWrite(LED_RED, LOW);
          }
        }
      } // second read of RTC
      else{
        enterSleep();  // second read failed
      }
    } // first readRTC
    else{
      enterSleep();  // sleep 0.25 s (because need this to loop faster than 1s)
    }
  } // loopMode = 0

  //
  // loopMode 1: Send Motion Data Out Serial Port and Wait for Record to End
  //
  while(loopMode==1){
    if(simulateDepth){
      if (minute != oldMinute){
        oldMinute = minute;
        simulateIndex++;
        if(simulateIndex>=nDepths) simulateIndex = 0;
      }
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
      if(simulateDepth) depth = depth + (float) depthProfile[simulateIndex];  // calculate new depth once per second
   }

   // start playback
   if(REC_STATE==1 & PLAY_STATE==0){
        if(t-recTime > playDelaySeconds){
          digitalWrite(PLAY_POW, HIGH);  // play will start automatically
          PLAY_STATE = 1;
          nPlayed++;
        }
    }

    if(PLAY_STATE==1){
       // check whether to stop playback 10s after starts
       if (t-recTime > (playDelaySeconds + 10)){
          if(analogRead(PLAY_STATUS) < 200){
            digitalWrite(PLAY_POW, LOW);  // stop playback
            PLAY_STATE = 2;
          }
       }
    }

   // check if time to stop recording and reset flags and counters
   if ((t-recTime)/60 >= recMinutes){
    stopTimer();
    digitalWrite(PLAY_POW, LOW); // power down playback
    digitalWrite(REC_ST, LOW);  // stop recording
    delay(2000);
    if(simulateDepth==0) digitalWrite(REC_POW, LOW); // turn off recorder
    loopMode = 0;    
    playTime = t; // reset playTime to when recording ended    
    PLAY_STATE = 0;
    REC_STATE = 0;
    checkDepthCounter = 0;
    Serial.println("End Rec");
    Serial.print("loopMode:"); Serial.println(loopMode);
   }
  } // loopMode = 1
}


void initSensors(){
  delay(2000);
  readVoltage();

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

  myICM.begin( Wire, 1 );
  if( myICM.status != ICM_20948_Stat_Ok ){
      Serial.println( "ICM fail" );
      delay(500);
  }

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
  Serial.print(','); Serial.print(PLAY_STATE);
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
    togglePress = 0;
    if(simulateDepth==0) kellerConvert();
  }
    
  if(ssCounter>=slowRateMultiple){  
    if(simulateDepth==0) kellerRead();
    togglePress = 1;

    if(LED_EN) digitalWrite(LED_GRN, HIGH);
    readRTC();
    //calcPressTemp(); // MS58xx pressure and temperature
    readVoltage();
    writeSlowSensorsFlag = 1;
    ssCounter = 0;
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
  // WDTCSR = (0<<WDP3 )|(1<<WDP2 )|(1<<WDP1)|(0<<WDP0);  // 1 s
  WDTCSR = (0<<WDP3 )|(1<<WDP2 )|(0<<WDP1)|(0<<WDP0);  // .25 s
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
}
