#pragma once
#include <stdint.h>

void imu_init();                   // detecta y configura el MPU-6050
void imu_update(uint32_t now_ms);  // debe llamarse cada iteración del loop
bool imu_ok();                     // false si el sensor no respondió al inicio
