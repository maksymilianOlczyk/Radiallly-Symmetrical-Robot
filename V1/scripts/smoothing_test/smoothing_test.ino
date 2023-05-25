#include <Servo.h>

unsigned long startTime = 0;

// Global Definitions
const int BUFFER = 32;
const int NUM_SERVOS = 12;
const int seqlen = 3;
int prevDis = 0;
const int sequence[seqlen + 1][NUM_SERVOS + 1] = {
  {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 2000},
  {40, 90, 0, 40, 90, 0, 40, 90, 0, 40, 90, 0, 2000},
  {140, 0, 0, 140, 0, 0, 140, 0, 0, 140, 0, 0, 2000},
};
// angles followed by cumulative time of ending

int target_angles[NUM_SERVOS] = {sequence[0]};
unsigned long last_time = 0;
const double SMOOTHING_FACTOR = -4;

Servo ml[NUM_SERVOS]; // motor list

// Function Definitions
void servosetup() {
  for (int i = 2; i < NUM_SERVOS + 2; i++) {
    ml[i - 2].attach(i);
    ml[i - 2].write(0);
  }
}

double pow(double base, double exponent) {
  double result = 1.0;
  if (exponent > 0) {
    for (int i = 0; i < exponent; ++i) {
      result *= base;
    }
  }
  else {
    for (int i = 0; i > exponent; --i) {
      result /= base;
    }
  }
  return result;
}

int calculateNextAngle(int current_angle, int target_angle, unsigned long t, float SMOOTHING_FACTOR, int seqTim) {
  unsigned long time_diff = t;
  float anDiff = abs(current_angle - target_angle);
  float time_to_reach = -log(0.5 / anDiff) * pow(10, -SMOOTHING_FACTOR);
  if (anDiff == 0) {
    time_to_reach = 0;
  }
  double SFtemp = SMOOTHING_FACTOR;
  if (time_to_reach > seqTim) {
    SFtemp = -log(seqTim/-log(0.5/anDiff));
  }
  float smoothing = exp(-pow(10, SMOOTHING_FACTOR) * time_diff);

  // Calculate the next angle using exponential decrease
  int next_angle = target_angle + (current_angle - target_angle) * smoothing;

  return next_angle;
}

void setup() {
  servosetup();
  Serial.begin(9600);
}

void executeSequence() {
  for (int i = 0; i < seqlen + 1; i++) {
    last_time = millis();
    while ((millis() - last_time) < sequence[i][NUM_SERVOS]) {
      float t = (millis() - last_time);
      for (int j = 0; j < NUM_SERVOS; j++) {
        target_angles[j] = calculateNextAngle(target_angles[j], sequence[i][j], t, SMOOTHING_FACTOR, sequence[i][NUM_SERVOS]);
        ml[j].write(target_angles[j]);
        if (!(target_angles[0] == prevDis)) {
          prevDis = target_angles[0];
           Serial.println(target_angles[0]);
        }
      }
    }
    for (int j = 0; j < NUM_SERVOS; j++) {
      target_angles[j] = sequence[i][j];
      ml[j].write(sequence[i][j]);
    }
  }
}

// Main
void loop() {
  executeSequence();
}
