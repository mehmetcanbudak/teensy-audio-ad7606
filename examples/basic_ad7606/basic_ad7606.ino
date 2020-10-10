#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <input_ad7606.h>

// GUItool: begin automatically generated code
AudioOutputTDM           tdm1;           //xy=514,291
AudioInputAD7606         ad7606;           //xy=514,291
/*
AudioConnection          patchCord1(ad7606, 0, tdm1, 0);
AudioConnection          patchCord2(ad7606, 1, tdm1, 2);
AudioConnection          patchCord3(ad7606, 2, tdm1, 4);
AudioConnection          patchCord4(ad7606, 3, tdm1, 6);
AudioConnection          patchCord5(ad7606, 4, tdm1, 8);
AudioConnection          patchCord6(ad7606, 5, tdm1, 10);
AudioConnection          patchCord7(ad7606, 6, tdm1, 12);
AudioConnection          patchCord8(ad7606, 7, tdm1, 14);
*/
// GUItool: end automatically generated code

void setup() {
  Serial.begin(9600);
  AudioMemory(60);
  while (!Serial) {
    delay(1);
  }
  Serial.println("Started...");
}

void loop() {
  delay(1000);
  //Serial.print(".");
}
