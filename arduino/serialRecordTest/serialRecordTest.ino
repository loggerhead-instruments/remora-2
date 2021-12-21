#define HWSERIAL Serial1

void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(115200);

}

void loop() {
  int incomingByte;

  if (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    if (incomingByte==13) Serial.println();
    else Serial.write(incomingByte);
    
  }

}
