// EMG Envelope + Motion Detection via Std Deviation
// Based on Muscle bioAmp patchy sensor by Upside Down Labs
#include<Servo.h>

#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN A0

// Envelope buffer for smoothing
#define ENVELOPE_BUFFER_SIZE 64
int envelope_buffer[ENVELOPE_BUFFER_SIZE];
int envelope_index = 0;
int envelope_sum = 0;

// Buffer for standard deviation check
#define STD_BUFFER_SIZE 30
float std_buffer[STD_BUFFER_SIZE];
int std_index = 0;
bool std_buffer_filled = false;

// Threshold to detect motion (tune this value)
#define STD_THRESHOLD 5.0

#define thumb 9
#define index 10
#define middleRing 11
#define pinki 12

Servo Sthumb;
Servo Sindex;
Servo SmiddleRing;
Servo Spinki;

void setup() {
  Serial.begin(BAUD_RATE);

  Sthumb.attach(thumb);
  Sindex.attach(index);
  SmiddleRing.attach(middleRing);
  Spinki.attach(pinki);

}

void loop() {
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  static long timer = 0;
  timer -= interval;

  if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;

    // Read and filter EMG signal
    int raw_signal = analogRead(INPUT_PIN);
    int filtered_signal = EMGFilter(raw_signal);
    int abs_filtered = abs(filtered_signal);
    int envelope = getEnvelope(abs_filtered);

    // Print for Serial Plotter
    Serial.print(filtered_signal);
    Serial.print(",");
    Serial.println(envelope);

    // Update std deviation buffer
    std_buffer[std_index] = envelope;
    std_index = (std_index + 1) % STD_BUFFER_SIZE;
    if (std_index == 0) std_buffer_filled = true;

    // Calculate std deviation if buffer filled
    if (std_buffer_filled) {
      float mean = 0;
      for (int i = 0; i < STD_BUFFER_SIZE; i++) {
        mean += std_buffer[i];
      }
      mean /= STD_BUFFER_SIZE;

      float variance = 0;
      for (int i = 0; i < STD_BUFFER_SIZE; i++) {
        variance += pow(std_buffer[i] - mean, 2);
      }
      variance /= STD_BUFFER_SIZE;

      float stdDev = sqrt(variance);
      Serial.print("StdDev:");
      Serial.println(stdDev);

      if(envelope > 10){
        
        Sthumb.write(60);
        Sindex.write(140);
        SmiddleRing.write(170);
        Spinki.write(0);

      }else{

        Sthumb.write(0);
        Sindex.write(0);
        SmiddleRing.write(0);
        Spinki.write(80);
      }
    }
  }
}

// Envelope detection using moving average
int getEnvelope(int abs_emg) {
  envelope_sum -= envelope_buffer[envelope_index];
  envelope_sum += abs_emg;
  envelope_buffer[envelope_index] = abs_emg;
  envelope_index = (envelope_index + 1) % ENVELOPE_BUFFER_SIZE;
  return (envelope_sum / ENVELOPE_BUFFER_SIZE) * 2;
}

// BioAmp EXG Band-Pass Filter (Butterworth IIR)
float EMGFilter(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - 0.05159732 * z1 - 0.36347401 * z2;
    output = 0.01856301 * x + 0.03712602 * z1 + 0.01856301 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -0.53945795 * z1 - 0.39764934 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - 0.47319594 * z1 - 0.70744137 * z2;
    output = 1.00000000 * x + 2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -1.00211112 * z1 - 0.74520226 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}