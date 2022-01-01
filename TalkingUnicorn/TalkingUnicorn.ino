/*******************************************************************
    Talking Unicorn
    ESP32 as a bluetooth speaker that tries to move the mouth
    in sync with the vocal audio

    Heavily Based on https://github.com/thingpulse/esp32-icon64-a2dp

    Modifications from the original sketch:
    - Updated to the most recent version of the bluetooth audio lib
    - Removed graphics
    - Added the threshold for moving the mouth

    Parts Used:
    ESP32
    Adafruit Max 98357A I2S Amp

    If you find what I do useful and would like to support me,
    please consider becoming a sponsor on Github
    https://github.com/sponsors/witnessmenow/

    Adapted by Brian Lough
    YouTube: https://www.youtube.com/brianlough
    Tindie: https://www.tindie.com/stores/brianlough/
    Twitter: https://twitter.com/witnessmenow
 *******************************************************************/

 
// Original Codes header:

/*
  MIT License

  Copyright (c) 2020 ThingPulse GmbH (B.L.: Do I leave this here?)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

// ----------------------------
// Additional Libraries - each one of these will need to be installed.
// ----------------------------


#include "BluetoothA2DPSink.h"
// Library to make the ESP32 act as a Bluetooth speaker

// Think it needs to be installed through Github
// https://github.com/pschatzmann/ESP32-A2DP

#include <arduinoFFT.h>
// Library to do the FFT (seperating the audio into different freuencies) 

// Can be installed through the library manager
// https://github.com/kosme/arduinoFFT

BluetoothA2DPSink a2dp_sink;


// FFT Settings
#define NUM_BANDS  8
#define SAMPLES 512
#define SAMPLING_FREQUENCY 44100

// Threshold for moving the mouth
#define VOCAL_CHANNEL 3 // Which band to base the vocal audio on
#define VOCAL_THRESHOLD 7 // what threshold it needs to meet to be considered high. 8 is max

arduinoFFT FFT = arduinoFFT();

int32_t peak[] = {0, 0, 0, 0, 0, 0, 0, 0};
double vReal[SAMPLES];
double vImag[SAMPLES];

QueueHandle_t queue;

int16_t sample_l_int;
int16_t sample_r_int;

float amplitude = 200.0;

uint32_t animationCounter = 0;

int visualizationCounter = 0;
int32_t lastVisualizationUpdate = 0;

void createBands(int i, int dsize) {
  uint8_t band = 0;
  if (i <= 2) {
    band =  0; // 125Hz
  } else if (i <= 5) {
    band =   1; // 250Hz
  } else if (i <= 7)  {
    band =  2; // 500Hz
  } else if (i <= 15) {
    band =  3; // 1000Hz
  } else if (i <= 30) {
    band =  4; // 2000Hz
  } else if (i <= 53) {
    band =  5; // 4000Hz
  } else if (i <= 106) {
    band =  6;// 8000Hz
  } else {
    band = 7;
  }
  int dmax = amplitude;
  if (dsize > dmax)
    dsize = dmax;
  if (dsize > peak[band])
  {
    peak[band] = dsize;
  }
}

void renderFFT(void * parameter) {
  int item = 0;
  for (;;) {
    if (uxQueueMessagesWaiting(queue) > 0) {

      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

      for (uint8_t band = 0; band < NUM_BANDS; band++) {
        peak[band] = 0;
      }

      for (int i = 2; i < (SAMPLES / 2); i++) { // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
        if (vReal[i] > 2000) { // Add a crude noise filter, 10 x amplitude or more
          createBands(i, (int)vReal[i] / amplitude);
        }
      }

      // Release handle
      xQueueReceive(queue, &item, 0);

      uint8_t intensity;

      //      FastLED.clear();
      //      FastLED.setBrightness(BRIGHTNESS);

      for (byte band = 0; band < NUM_BANDS; band++) {
        intensity = map(peak[band], 1, amplitude, 0, 8);
//        Serial.print(intensity);
//        Serial.print(" | ");
        if(band == VOCAL_CHANNEL){
          digitalWrite(5, (intensity >= VOCAL_THRESHOLD));
        }

      }

//      Serial.println("");


      if ((millis() - lastVisualizationUpdate) > 1000) {
        log_e("Fps: %f", visualizationCounter / ((millis() - lastVisualizationUpdate) / 1000.0));
        visualizationCounter = 0;
        lastVisualizationUpdate = millis();
        //hueOffset += 5;
      }
      visualizationCounter++;
    }
  }
}

void avrc_metadata_callback(uint8_t data1, const uint8_t *data2) {
  Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
}

// Then somewhere in your sketch:
void read_data_stream(const uint8_t *data, uint32_t length)
{
  int item = 0;
  // Only prepare new samples if the queue is empty
  if (uxQueueMessagesWaiting(queue) == 0) {
    //log_e("Queue is empty, adding new item");
    int byteOffset = 0;
    for (int i = 0; i < SAMPLES; i++) {
      sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset + 2)));
      vReal[i] = (sample_l_int + sample_r_int) / 2.0f;;
      vImag[i] = 0;
      byteOffset = byteOffset + 4;
    }

    // Tell the task in core 1 that the processing can start
    xQueueSend(queue, &item, portMAX_DELAY);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(5, OUTPUT);

  // The queue is used for communication between A2DP callback and the FFT processor
  queue = xQueueCreate( 1, sizeof( int ) );
  if (queue == NULL) {
    Serial.println("Error creating the queue");
  }

  xTaskCreatePinnedToCore(
    renderFFT,          // Function that should be called
    "FFT Renderer",     // Name of the task (for debugging)
    10000,              // Stack size (bytes)
    NULL,               // Parameter to pass
    1,                  // Task priority
    NULL,               // Task handle
    1                   // Core you want to run the task on (0 or 1)
  );

  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_stream_reader(read_data_stream);
  a2dp_sink.start("MyMusic");
}

void loop() {
}
