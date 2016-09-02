byte buf[1000];
int bIndex = 0;
byte a = 0;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
}

void loop() {
  while(Serial.available()) {
    a = Serial.read();
    if (a < 100) {
      Serial.write(a);
    }
    if (a >= 100 && a != 255)
      buf[bIndex++] = a;
    if (a == 255) {
      Serial.write(buf, 1000);
      bIndex = 0;
    }
  }
}
