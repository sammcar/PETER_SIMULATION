#pragma once
#include <stdint.h>

void motors_init();

// speed: -255 a +255 por motor
// Orden: M1(FR), M2(FL), M3(RR), M4(RL)
void motor_set(int m1, int m2, int m3, int m4);

void motors_stop();
