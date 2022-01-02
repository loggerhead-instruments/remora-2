//
// Remora2 Record Teensy
//
// Loggerhead Instruments
// 2016-2021
// David Mann
// 
// Modified from PJRC audio code
// http://www.pjrc.com/store/teensy3_audio.html
//
// Compile with 96 MHz Fastest

// Major Changes:
// dumps Serial input to a separate file from audio file
// WFI removed to avoid dropping motion samples
// Modified serial1.c line 43 to increase serial receive buffer size to the following:
// #define SERIAL1_RX_BUFFER_SIZE     256 // number of incoming bytes to buffer; default=64
// C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3\serial1.c

// Modified by WMXZ 15-05-2018 for SdFS anf multiple sampling frequencies
// Optionally uses SdFS from Bill Greiman https://github.com/greiman/SdFs; but has higher current draw in sleep

char codeVersion[12] = "2022-01-02";
static boolean printDiags = 1;  // 1: serial print diagnostics; 0: no diagnostics

#define USE_SDFS 0  // to be used for exFAT but works also for FAT16/32
#define MQ 100 // to be used with LHI record queue (modified local version)
//#define USE_LONG_FILE_NAMES

#include "LHI_record_queue.h"
#include "control_sgtl5000.h"

#if USE_SDFS==0
  #include "input_i2s.h"
//  #include "LHI_record_queue.h"
//  #include "control_sgtl5000.h"
#else
  #include <Audio.h>  //this also includes SD.h from lines 89 & 90
#endif
#include <Wire.h>
#include <SPI.h>
#if USE_SDFS==1
  #include "SdFs.h"
#else
  #include "SdFat.h"
#endif
#include "amx32.h"

#include <Snooze.h>  //using https://github.com/duff2013/Snooze; uncomment line 62 #define USE_HIBERNATE

#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>
#include <EEPROM.h>

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // modify so calls i2c_t3 not Wire.h
#include <Adafruit_FeatherOLED.h>

#define OLED_RESET -1
#define displayLine1 0
#define displayLine2 8
#define displayLine3 16
#define displayLine4 25
Adafruit_FeatherOLED display = Adafruit_FeatherOLED();
#define BOTTOM 25

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1

#define NREC 32 // increase disk buffer to speed up disk access

static uint32_t myID[2];

unsigned long baud = 115200;

#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400

#define MODE_NORMAL 0
#define MODE_DIEL 1

// GUItool: begin automatically generated code
AudioInputI2S            i2s2;           //xy=105,63
LHIRecordQueue           queue1;         //xy=281,63
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// GUItool: end automatically generated code

const int myInput = AUDIO_INPUT_LINEIN;
unsigned int gainSetting = 4; //default gain setting; can be overridden in setup file
int noDC = 0; // 0 = freezeDC offset; 1 = remove DC offset

// Pin Assignments
#define BURN_REC 5
#define REC_ST 8
#define ledGreen 17
#define SGTL_EN_R 6
#define REC_STATUS 2
const int SDSW = 0;

// Pins used by audio shield
// https://www.pjrc.com/store/teensy3_audio.html
// MEMCS 6
// MOSI 7
// BCLK 9
// SDCS 10
// MCLK 11
// MISO 12
// RX 13
// SCLK 14
// VOL 15
// SDA 18
// SCL 19
// TX 22
// LRCLK 23

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording, 2=playing
time_t startTime;
time_t stopTime;
time_t t;
time_t burnTime;
byte startHour, startMinute, endHour, endMinute; //used in Diel mode

boolean audioFlag = 1;

volatile boolean LEDSON=1;
boolean introperiod=1;  //flag for introductory period; used for keeping LED on for a little while

int32_t lhi_fsamps[7] = {8000, 16000, 32000, 44100, 48000, 96000, 192000};
#define I_SAMP 3   // 0 is 8 kHz; 1 is 16 kHz; 2 is 32 kHz; 3 is 44.1 kHz; 4 is 48 kHz; 5 is 96 kHz; 6 is 192 kHz

float audio_srate = lhi_fsamps[I_SAMP];
int isf = I_SAMP;

//WMXZ float audioIntervalSec = 256.0 / audio_srate; //buffer interval in seconds
//WMXZ unsigned int audioIntervalCount = 0;
float gainDb;

int recMode = MODE_NORMAL;
long rec_dur = 3600;
long rec_int = 0;
int wakeahead = 5;  //wake from snooze to give hydrophone and camera time to power up
int snooze_hour;
int snooze_minute;
int snooze_second;
int buf_count;
long nbufs_per_file;
boolean settingsChanged = 0;

long file_count;
char filename[60];
char dirname[20];
int folderMonth;

SnoozeAlarm alarm;
SnoozeAudio snooze_audio;
SnoozeBlock config_teensy32(snooze_audio, alarm);


// The file where data is recorded
#if USE_SDFS==1
  FsFile frec;
  FsFile frecMotion;
  SdFs sd;
#else
  File frec;
  File frecMotion;
  SdFat sd;
  //SdFile file; // not used(WMXZ)
#endif

typedef struct {
    char    rId[4];
    unsigned int rLen;
    char    wId[4];
    char    fId[4];
    unsigned int    fLen;
    unsigned short nFormatTag;
    unsigned short nChannels;
    unsigned int nSamplesPerSec;
    unsigned int nAvgBytesPerSec;
    unsigned short nBlockAlign;
    unsigned short  nBitsPerSamples;
    char    dId[4];
    unsigned int    dLen;
} HdrStruct;

HdrStruct wav_hdr;

unsigned char prev_dtr = 0;


void setup() {
  Serial.begin(baud);
  pinMode(ledGreen, OUTPUT);
  digitalWrite(ledGreen, LOW);
  pinMode(REC_ST, INPUT);
  pinMode(SGTL_EN_R, OUTPUT);
  pinMode(REC_STATUS, OUTPUT);
  digitalWrite(REC_STATUS, LOW);
  Serial1.begin(baud); // talk to Atmega

  Serial.println(RTC_TSR);
  Serial.println(RTC_TSR);
  Serial.println(RTC_TSR);

  readEEPROM();

  RTC_CR = 0; // disable RTC
  delay(100);
  // Serial.println(RTC_CR,HEX);
  // change capacitance to 26 pF (12.5 pF load capacitance)
  RTC_CR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P; 
  delay(100);
  RTC_CR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P | RTC_CR_OSCE;
  // setSyncProvider(getTeensy3Time);
  
  Wire.begin();  

    // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  if (!(sd.begin(10))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    
    while (1) {
      displayOn();
      cDisplay(); 
      display.println("SD error. Restart.");
      display.display();
      delay(1000);  
      //resetFunc();
    }
  }
  //SdFile::dateTimeCallback(file_date_time);
  LoadScript(); // settings accessible from the card
  writeEEPROM();
  if(LEDSON) digitalWrite(ledGreen, HIGH);

  if((digitalRead(REC_ST)==LOW)){
    displayOn();
    cDisplay();
    display.print("Remora ");
    display.print(lhi_fsamps[isf]); 
    display.println("Hz");
    display.display();
 }
  int curLine = 0;  
  char incomingText[100];
  int i = 0;
  while(digitalRead(REC_ST)==LOW){
    // display text from Atmega
    while(Serial1.available()>0){
      incomingText[i] = Serial1.read();
      Serial.print(i); Serial.print(':'); Serial.println(incomingText[i]);
//      Serial.write(incomingText);
//      display.write(incomingText);
      if(incomingText[i] == '\r') {
        Serial.println(incomingText);
        display.print(incomingText);
        display.display();
        i = 0;
        curLine += 1;
        if(incomingText[1] == 'D' and incomingText[2] == 'T'){   
          time_t newTime ;
          //incomingText[14] = '\0'; // null character
          char str[10];
          sscanf(incomingText, "%s %u", str, &newTime);
          Serial.print("New Time: ");
          Serial.print(str); 
          Serial.println(newTime);
          Teensy3Clock.set(newTime); 
          displayClock(newTime, BOTTOM);
        }
      }
      else{
        i++;
        if(i == 100) i = 0;
      }
      
    }
    display.display();
//    if(curLine > 4){
//        delay(500);
//        cDisplay();
//        curLine = 0;
//    }
  }

  read_myID();
  
  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  // initialize now to estimate DC offset during setup
  AudioMemory(MQ+10);
  
  audio_srate = lhi_fsamps[isf];
//WMXZ  audioIntervalSec = 256.0 / audio_srate; //buffer interval in seconds

  AudioInit(isf); // this calls Wire.begin() in control_sgtl5000.cpp

  logFileHeader();
  displayOff();

  t = getTeensy3Time();

  nbufs_per_file = (long) (ceil(((rec_dur * audio_srate / 256.0) / (float) NREC)) * (float) NREC);                 
  mode = 0;
  Serial.print("Bufs:"); Serial.println(nbufs_per_file);
  // create first folder to hold data
  folderMonth = -1;  //set to -1 so when first file made will create directory
}

//
// MAIN LOOP
//

int recLoopCount;  //for debugging when does not start record
  
void loop() {
  // Standby
  if(mode == 0)
  {
      t = getTeensy3Time();
      digitalWrite(ledGreen, LOW);
      if(noDC==0) {
        audio_freeze_adc_hp(); // this will lower the DC offset voltage, and reduce noise
        noDC = -1;
      }
      Serial.println("Record Start.");
      mode = 1;
      motionFileInit();
      startRecording(); // this will init audio file
  }

  // Record mode
  if (mode == 1) {
    continueRecording();  // download data  

  byte incomingByte;
  while (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    frecMotion.write(incomingByte);
    // Serial.write(incomingByte);
  }
    
    if(LEDSON) digitalWrite(ledGreen, HIGH);
    
    if(buf_count >= nbufs_per_file){       // time for new file?
        frec.close();
        FileInit();
        buf_count = 0;
    }

    // stop when low detected
    if(digitalRead(REC_ST)==0){
      delay(1);
      if(digitalRead(REC_ST)==0){
        stopRecording();
        mode = 2;
      }
    }
  }
  
  // asm("wfi"); // reduce power between interrupts
}

void startRecording() {
  if (printDiags)  Serial.println("startRecording");
  FileInit();
  buf_count = 0;
  queue1.begin();
  digitalWrite(REC_STATUS, HIGH);
  if (printDiags)  Serial.println("Queue Begin");
}


byte buffer[NREC*512];
void continueRecording() {
  if (queue1.available() >= NREC * 2) {
    // Fetch 2 blocks (or multiples) from the audio library and copy
    // into a 512 byte buffer.  micro SD disk access
    // is most efficient when full (or multiple of) 512 byte sector size
    // writes are used.
    if(LEDSON) digitalWrite(ledGreen, HIGH);
    for(int ii=0;ii<NREC;ii++)
    { byte *ptr = buffer+ii*512;
      memcpy(ptr, queue1.readBuffer(), 256);
      queue1.freeBuffer();
      memcpy(ptr+256, queue1.readBuffer(), 256);
      queue1.freeBuffer();
    }
    digitalWrite(ledGreen, LOW);  
    if(frec.write(buffer, NREC*512)==-1) resetFunc(); //audio to .wav file
    buf_count += NREC;
  }
}

void stopRecording() {
  if (printDiags) Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  if (printDiags) Serial.print("Audio Memory Max");
  if (printDiags) Serial.println(maxblocks);
  queue1.end();
  // update wav file header
  wav_hdr.rLen = 36 + buf_count * 256 * 2;
  wav_hdr.dLen = buf_count * 256 * 2;
  frec.seek(0);
  frec.write((uint8_t *)&wav_hdr, 44);
  AudioMemoryUsageMaxReset();
  //frec.timestamp(T_WRITE,(uint16_t) year(t),month(t),day(t),hour(t),minute(t),second);
  frec.close();
  frecMotion.close();
  delay(100);
  digitalWrite(REC_STATUS, LOW);
}


void sdInit(){
     if (!(sd.begin(10))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    
    cDisplay();
    display.println("SD error. Restart.");
    displayClock(getTeensy3Time(), BOTTOM);
    display.display();
    delay(1000);
  }
}

void motionFileInit(){
  sd.chdir(); // only to be sure to start from root
  char motionFileName[100];
  sprintf(motionFileName,"%04d%02d%02dT%02d%02d%02d_%lu%lu.csv", year(t), month(t), day(t), hour(t), minute(t), second(t), myID[0], myID[1]);  //filename is DDHHMMSS

  #if USE_SDFS==1
    if(frecMotion = sd.open(motionFileName,  O_CREAT | O_APPEND | O_WRITE)){
  #else
    if(frecMotion = sd.open(motionFileName,  O_CREAT | O_APPEND | O_WRITE)){
  #endif
    frecMotion.println("accelX,accelY,accelZ,magX,magY,magZ,gyroX,gyroY,gyroZ,date,mBar,depth,temp,V,rec,play");
   }
}

void FileInit()
{
   t = getTeensy3Time();
   
   if (folderMonth != month(t)){
    if(printDiags) Serial.println("New Folder");
    folderMonth = month(t);
    sprintf(dirname, "/%04d-%02d", year(t), folderMonth);
    #if USE_SDFS==1
      FsDateTime::callback = file_date_time;
    #else
      SdFile::dateTimeCallback(file_date_time);
    #endif
    sd.mkdir(dirname);
   }

   // open file 
   sd.chdir(dirname);
   sprintf(filename,"%04d%02d%02dT%02d%02d%02d_%lu%lu_%2.1f.wav", year(t), month(t), day(t), hour(t), minute(t), second(t), myID[0], myID[1], gainDb);  //filename is DDHHMMSS


   // log file
  #if USE_SDFS==1
    FsDateTime::callback = file_date_time;
  #else
    SdFile::dateTimeCallback(file_date_time);
  #endif

  sd.chdir(); // only to be sure to start from root
  #if USE_SDFS==1
    if(FsFile logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
  #else
    if(File logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
  #endif
      logFile.print(filename);
      logFile.print(',');
      logFile.println(codeVersion);
      logFile.close();
   }
   else{
    if(printDiags) Serial.print("Log open fail.");
    resetFunc();
   }

    
   sd.chdir(dirname);
   frec = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
   Serial.println(filename);
   
   while (!frec){
    file_count += 1;
    sprintf(filename,"F%06d.wav",file_count); //if can't open just use count
    sd.chdir(dirname);
    frec = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
    delay(10);
    if(file_count>1000) resetFunc(); // give up after many tries
   }


    //intialize .wav file header
    sprintf(wav_hdr.rId,"RIFF");
    wav_hdr.rLen=36;
    sprintf(wav_hdr.wId,"WAVE");
    sprintf(wav_hdr.fId,"fmt ");
    wav_hdr.fLen=0x10;
    wav_hdr.nFormatTag=1;
    wav_hdr.nChannels=1;
    wav_hdr.nSamplesPerSec=audio_srate;
    wav_hdr.nAvgBytesPerSec=audio_srate*2;
    wav_hdr.nBlockAlign=2;
    wav_hdr.nBitsPerSamples=16;
    sprintf(wav_hdr.dId,"data");
    wav_hdr.rLen = 36 + nbufs_per_file * 256 * 2;
    wav_hdr.dLen = nbufs_per_file * 256 * 2;
  
    frec.write((uint8_t *)&wav_hdr, 44);

  Serial.print("Buffers: ");
  Serial.println(nbufs_per_file);
}

void logFileHeader(){

   sd.chdir(); // only to be sure to star from root
#if USE_SDFS==1
  if(FsFile logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
#else
  if(File logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
#endif
      logFile.println("filename, Version");
      logFile.close();
  }
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  t = getTeensy3Time();
  #if USE_SDFS==1
    *date=FS_DATE(year(t),month(t),day(t));
    *time=FS_TIME(hour(t),minute(t),second(t));
  #else
    *date=FAT_DATE(year(t),month(t),day(t));
    *time=FAT_TIME(hour(t),minute(t),second(t));
  #endif
}

void AudioInit(int ifs){
 // Instead of using audio library enable; do custom so only power up what is needed in sgtl5000_LHI
  I2S_modification(lhi_fsamps[ifs], 16);
  Wire.begin();
  audio_enable(ifs);
}

void calcGain(){
    switch(gainSetting){
    case 0: gainDb = -20 * log10(3.12 / 2.0); break;
    case 1: gainDb = -20 * log10(2.63 / 2.0); break;
    case 2: gainDb = -20 * log10(2.22 / 2.0); break;
    case 3: gainDb = -20 * log10(1.87 / 2.0); break;
    case 4: gainDb = -20 * log10(1.58 / 2.0); break;
    case 5: gainDb = -20 * log10(1.33 / 2.0); break;
    case 6: gainDb = -20 * log10(1.11 / 2.0); break;
    case 7: gainDb = -20 * log10(0.94 / 2.0); break;
    case 8: gainDb = -20 * log10(0.79 / 2.0); break;
    case 9: gainDb = -20 * log10(0.67 / 2.0); break;
    case 10: gainDb = -20 * log10(0.56 / 2.0); break;
    case 11: gainDb = -20 * log10(0.48 / 2.0); break;
    case 12: gainDb = -20 * log10(0.40 / 2.0); break;
    case 13: gainDb = -20 * log10(0.34 / 2.0); break;
    case 14: gainDb = -20 * log10(0.29 / 2.0); break;
    case 15: gainDb = -20 * log10(0.24 / 2.0); break;
  }
}


time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1451606400; // Jan 1 2016
} 
  
// Calculates Accurate UNIX Time Based on RTC Timestamp
unsigned long RTCToUNIXTime(TIME_HEAD *tm){
    int i;
    unsigned const char DaysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    unsigned long Ticks = 0;

    long yearsSince = tm->year + 30; // Years since 1970
    long numLeaps = yearsSince >> 2; // yearsSince / 4 truncated

    if((!(tm->year%4)) && (tm->month>2))
            Ticks+=SECONDS_IN_DAY;  //dm 8/9/2012  If current year is leap, add one day

    // Calculate Year Ticks
    Ticks += (yearsSince-numLeaps)*SECONDS_IN_YEAR;
    Ticks += numLeaps * SECONDS_IN_LEAP;

    // Calculate Month Ticks
    for(i=0; i < tm->month-1; i++){
         Ticks += DaysInMonth[i] * SECONDS_IN_DAY;
    }

    // Calculate Day Ticks
    Ticks += (tm->day - 1) * SECONDS_IN_DAY;

    // Calculate Time Ticks CHANGES ARE HERE
    Ticks += (ULONG)tm->hour * SECONDS_IN_HOUR;
    Ticks += (ULONG)tm->minute * SECONDS_IN_MINUTE;
    Ticks += tm->sec;

    return Ticks;
}

void resetFunc(void){
  EEPROM.write(20, 1); // reset indicator register
  CPU_RESTART
}
    
void read_myID() {
//  myID[0] = SIM_UIDH;
  myID[0] = SIM_UIDMH;
  myID[1] = SIM_UIDML;
//  myID[3] = SIM_UIDL;
}
