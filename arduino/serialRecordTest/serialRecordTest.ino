#define HWSERIAL Serial1
#define REC_ST 8
#define LED_GRN 17

void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(115200);
  pinMode(REC_ST, INPUT);
  pinMode(LED_GRN, OUTPUT);
}

void loop() {
  byte incomingByte;

  if (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    if (incomingByte==20) Serial.println();
    else Serial.write(incomingByte);
  }

  digitalWrite(LED_GRN, digitalRead(REC_ST));
  
}
