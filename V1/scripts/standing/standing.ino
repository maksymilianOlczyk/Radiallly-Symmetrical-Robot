#include <Servo.h>

unsigned long time = millis();

// Global Definitions
const int BUFFER = 32;
const int NUM_SERVOS = 12;
const int ST = time;
const int seqlen = 4;
const int sequence[seqlen + 1][NUM_SERVOS + 1] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 60, 40, 0, 65, 50, 0, 60, 40, 0, 60, 40, 2000},
  {0, 60, 40, 0, 65, 50, 0, 60, 40, 0, 60, 40, 2000},
  {0, 60, 40, 0, 65, 50, 0, 60, 40, 0, 60, 40, 2000},
  {0, 60, 40, 0, 65, 50, 0, 60, 40, 0, 60, 40, 2000},
};
// angles followed by cumulative time of ending

int target_angles[NUM_SERVOS] = {0};
int current_angles[NUM_SERVOS] = {0};
int step_angles[NUM_SERVOS] = {0};
unsigned long last_time = 0;
const float SMOOTHING_FACTOR = 0.1; // Adjust this value to change the amount of smoothing

Servo ml[NUM_SERVOS];     // motor list

// Function Definitions
void servosetup() {
  for (int i = 2; i < NUM_SERVOS + 2; i++) {
    ml[i - 2].attach(i);
    ml[i - 2].write(0);
  }
}

void setup() {
  servosetup();
}

void executeSequence() {
  for (int i = 1; i < seqlen + 1; i++) {
    last_time = millis();
    while ((millis() - last_time) < sequence[i][NUM_SERVOS]) {
      float t = (millis() - last_time) / (float)sequence[i][NUM_SERVOS];
      for (int j = 0; j < NUM_SERVOS; j++) {
        int target = sequence[i][j] * t + current_angles[j] * (1 - t);
        target_angles[j] = target_angles[j] * (1 - SMOOTHING_FACTOR) + target * SMOOTHING_FACTOR; // Apply low-pass filter
      }
      for (int j = 0; j < NUM_SERVOS; j++) {
        if (target_angles[j] != current_angles[j]) {
          if (target_angles[j] > current_angles[j]) {
            step_angles[j] = 1;
          } else {
            step_angles[j] = -1;
          }
          current_angles[j] += step_angles[j];
          ml[j].write(current_angles[j]);
        }
      }
    }
    for (int j = 0; j < NUM_SERVOS; j++) {
      ml[j].write(sequence[i][j]);
    }
    memcpy(current_angles, sequence[i], NUM_SERVOS * sizeof(int));
  }
}

// Main
void loop() {
  executeSequence();
}
