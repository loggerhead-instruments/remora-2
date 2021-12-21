#define HWSERIAL Serial1

void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(115200);

}

void loop() {
  byte incomingByte;

  if (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    if (incomingByte==20) Serial.println();
    else Serial.write(incomingByte);
    
  }

}
