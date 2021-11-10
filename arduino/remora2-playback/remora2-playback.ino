// Simple WAV file player example
//
// Three types of output may be used, by configuring the code below.
//
//   1: Digital I2S - Normally used with the audio shield:
//         http://www.pjrc.com/store/teensy3_audio.html
//
//   2: Digital S/PDIF - Connect pin 22 to a S/PDIF transmitter
//         https://www.oshpark.com/shared_projects/KcDBKHta
//
//   3: Analog DAC - Connect the DAC pin to an amplified speaker
//         http://www.pjrc.com/teensy/gui/?info=AudioOutputAnalog
//
// To configure the output type, first uncomment one of the three
// output objects.  If not using the audio shield, comment out
// the sgtl5000_1 lines in setup(), so it does not wait forever
// trying to configure the SGTL5000 codec chip.
//
// The SD card may connect to different pins, depending on the
// hardware you are using.  Uncomment or configure the SD card
// pins to match your hardware.
//
// Data files to put on your SD card can be downloaded here:
//   http://www.pjrc.com/teensy/td_libs_AudioDataFiles.html
//
// This example code is in the public domain.

#include <Audio.h> // Do not use Loggerhead Audio; use original audio
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

AudioPlaySdWav           playWav1;
AudioOutputI2S           audioOutput;
AudioInputI2S            i2s2;           //xy=105,63
AudioRecordQueue         queue1;         //xy=281,63
AudioConnection          patchCord1(playWav1, 0, audioOutput, 0);
AudioConnection          patchCord2(playWav1, 1, audioOutput, 1);
AudioControlSGTL5000     sgtl5000_1;

const int myInput = AUDIO_INPUT_LINEIN;

// Use these with the audio adaptor board
#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14

#define LED_GREEN 17
#define PLAY_STATUS 2

// Use these for the SD+Wiz820 or other adaptors
//#define SDCARD_CS_PIN    4
//#define SDCARD_MOSI_PIN  11
//#define SDCARD_SCK_PIN   13
// The file where data is recorded
File frec;

void setup() {
  Serial.begin(9600);

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(110);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(PLAY_STATUS, OUTPUT);
  digitalWrite(PLAY_STATUS, HIGH);  // just set high at start

  // Comment these out if not using the audio adaptor board.
  // This may wait forever if the SDA & SCL pins lack
  // pullup resistors
  sgtl5000_1.enable();
  //sgtl5000_1.lineOutLevel(31);

//  lineOutLevel(both);
//
//Adjust the line level output voltage range. The following settings are possible:
//
//13: 3.16 Volts p-p
//14: 2.98 Volts p-p
//15: 2.83 Volts p-p
//16: 2.67 Volts p-p
//17: 2.53 Volts p-p
//18: 2.39 Volts p-p
//19: 2.26 Volts p-p
//20: 2.14 Volts p-p
//21: 2.02 Volts p-p
//22: 1.91 Volts p-p
//23: 1.80 Volts p-p
//24: 1.71 Volts p-p
//25: 1.62 Volts p-p
//26: 1.53 Volts p-p
//27: 1.44 Volts p-p
//28: 1.37 Volts p-p
//29: 1.29 Volts p-p  (default)
//30: 1.22 Volts p-p
//31: 1.16 Volts p-p
//lineOutLevel(left, right);
//
//Adjust the line level outout voltage range, with separate settings for left and right. The same settings (13 to 31) are available.

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
  playFile("SDTEST1.wav");  // filenames are always uppercase 8.3 format
  delay(5000);
}

void playFile(const char *filename)
{
  // digitalWrite(LED_GREEN, HIGH);
  Serial.print("Playing file: ");
  Serial.println(filename);
  // Start playing the file.  This sketch continues to
  // run while the file plays.
  //startRecording();
  playWav1.play(filename);

  // A brief delay for the library read WAV info
  delay(5);
  
  // Simply wait for the file to finish playing.
  while (playWav1.isPlaying()) {
    // record wav
   // continueRecording();
  }
 // stopRecording();
 digitalWrite(LED_GREEN, LOW);
 digitalWrite(PLAY_STATUS, LOW);
}


void loop() {
  Serial.println("Done");
  while(1);
}

void startRecording() {
  Serial.println("startRecording");
  if (SD.exists("RECORD.RAW")) {
    // The SD library writes new data to the end of the
    // file, so to start a new recording, the old file
    // must be deleted before new data is written.
    SD.remove("RECORD.RAW");
  }
  frec = SD.open("RECORD.RAW", FILE_WRITE);
  if (frec) {
    queue1.begin();
  }
}

#define NREC 20
byte buffer[NREC*512];
void continueRecording() {
  if (queue1.available() >= NREC * 2) {
    // Fetch 2 blocks (or multiples) from the audio library and copy
    // into a 512 byte buffer.  micro SD disk access
    // is most efficient when full (or multiple of) 512 byte sector size
    // writes are used.
    for(int ii=0;ii<NREC;ii++)
    { byte *ptr = buffer+ii*512;
      memcpy(ptr, queue1.readBuffer(), 256);
      queue1.freeBuffer();
      memcpy(ptr+256, queue1.readBuffer(), 256);
      queue1.freeBuffer();
    }

    frec.write(buffer, NREC*512);
  }
}

void stopRecording() {
  Serial.println("stopRecording");
  queue1.end();
  while (queue1.available() > 0) {
    frec.write((byte*)queue1.readBuffer(), 256);
    queue1.freeBuffer();
  }
  frec.close();

}
