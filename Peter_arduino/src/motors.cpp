#include "motors.h"
#include "config.h"
#include <Arduino.h>

// Pines confirmados por test físico ('1'-'4'):
#define M1_A 3   // físico: FL
#define M1_B 4
#define M2_A 6   // físico: RL
#define M2_B 5
#define M3_A 10  // físico: FR
#define M3_B 9
#define M4_A 11  // físico: RR
#define M4_B 12

static void _drive(uint8_t pinA, uint8_t pinB, int speed) {
  if (speed > 0) {
    analogWrite(pinB, 0);
    analogWrite(pinA, speed);
  } else if (speed < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -speed);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

void motors_init() {
  int pins[] = {M1_A, M1_B, M2_A, M2_B, M3_A, M3_B, M4_A, M4_B};
  for (int p : pins) {
    pinMode(p, OUTPUT);
  }
  motors_stop();
}

// Parámetros en orden lógico: FR, FL, RR, RL
// Enrutados a los pines físicos confirmados por test.
void motor_set(int fr, int fl, int rr, int rl) {
  _drive(M3_A, M3_B, M3_DIR * fr); // FR – pines 11,12
  _drive(M1_A, M1_B, M1_DIR * fl); // FL – pines 3,4
  _drive(M4_A, M4_B, M4_DIR * rr); // RR – pines 10,9
  _drive(M2_A, M2_B, M2_DIR * rl); // RL – pines 6,5
}

void motors_stop() { motor_set(0, 0, 0, 0); }
