#include <EEPROM.h>

#define SERIAL_MAX_LENGTH 20

char serialArray[SERIAL_MAX_LENGTH]; // = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
String serialString = "";

void setup() {
  Serial.begin(115200);  
}

void loop() {
  Serial.println("");
  Serial.println("TYPE IN SERIAL NUMBER AND PRESS ENTER (END WITH NEWLINE CHAR)...");
  Serial.println("");
  while(!Serial.available()) {
   // wait
  }
  serialString = Serial.readStringUntil('\n');
  // WRITE SERIAL
  if (serialString.substring(0,4) == "ROB-" && serialString.length() <= SERIAL_MAX_LENGTH) {
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
    }
    for (int i = 0 ; i < serialString.length() ; i++) {
      EEPROM.write(i, serialString[i]);
    }
  }
  // READ SERIAL
  EEPROM.get(0, serialArray);
  Serial.println("CURRENT SERIAL NUMBER IS: ");
  Serial.println(serialArray);
  for(int i = 0; i < SERIAL_MAX_LENGTH; i++) {
    Serial.print(serialArray[i], DEC);
    Serial.print(" ");
  }
  Serial.println();
}
