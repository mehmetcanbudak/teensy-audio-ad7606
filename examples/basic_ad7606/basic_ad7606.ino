#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <input_ad7606.h>

// GUItool: begin automatically generated code
AudioOutputTDM           tdm1;           //xy=514,291
AudioRecordQueue         audioRecordQueue;
AudioInputAD7606         ad7606;           //xy=514,291

AudioConnection          patchCord1(ad7606, 0, audioRecordQueue, 0);

// GUItool: end automatically generated code

void setup() {
  Serial.begin(9600);
  AudioMemory(60);
  while (!Serial) {
    delay(1);
  }
  Serial.println("Started...");
  audioRecordQueue.begin();
}

void loop() {
  if (audioRecordQueue.available() >= 1) {
      int16_t buffer[128]; //256 bytes
      // Fetch 2 blocks from the audio library and copy
      // into a 512 byte buffer.  The Arduino SD library
      // is most efficient when full 512 byte sector size
      // writes are used.
      memcpy(buffer, audioRecordQueue.readBuffer(), 256);
      audioRecordQueue.freeBuffer();

      for (int i=0; i<128; i++) {
          Serial.println(buffer[i]);
      }
//      Serial.println();
  }

}
