#pragma once
#include <stdint.h>

void  imu_init();                  // detecta y configura el MPU-6050
void  imu_calibrate();             // captura la posición actual como referencia cero
void  imu_update(uint32_t now_ms); // debe llamarse cada iteración del loop
float imu_get_pitch();             // grados respecto al cero calibrado (+ = nariz arriba)
float imu_get_roll();              // grados respecto al cero calibrado (+ = lado izq arriba)
bool  imu_ok();                    // false si el sensor no respondió al inicio
