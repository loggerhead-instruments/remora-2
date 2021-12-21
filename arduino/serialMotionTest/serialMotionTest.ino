// test send buffer of values
// max write buffer is 64 bytes


void setup() {
  Serial.begin(115200);
}

float depth = 0.0;
void loop() {
  Serial.println(depth);
  depth=depth + 12.4;
  delay(1000);
}
