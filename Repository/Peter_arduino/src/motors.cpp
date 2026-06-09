#include "motors.h"
#include <Arduino.h>

#define M1_A 3
#define M1_B 4
#define M2_A 6
#define M2_B 5
#define M3_A 11
#define M3_B 12
#define M4_A 10
#define M4_B 9

// Función interna para controlar un solo motor
static void _drive(uint8_t pinA, uint8_t pinB, int speed) {
  if (speed > 0) {
    analogWrite(pinA, speed);
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    digitalWrite(pinA, LOW);
    analogWrite(pinB, -speed);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void motors_init() {
  int pins[] = {M1_A, M1_B, M2_A, M2_B, M3_A, M3_B, M4_A, M4_B};
  for (int p : pins) {
    pinMode(p, OUTPUT);
  }
  motors_stop();
}

void motor_set(int m1, int m2, int m3, int m4) {
  _drive(M1_A, M1_B, m1); // FR
  _drive(M2_A, M2_B, m2); // FL
  _drive(M3_A, M3_B, m3); // RR
  _drive(M4_A, M4_B, m4); // RL
}

void motors_stop() { motor_set(0, 0, 0, 0); }
