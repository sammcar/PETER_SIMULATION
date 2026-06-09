#pragma once
#include <stdint.h>

#include "config.h" // Necesario para exponer RobotMode

void gait_set_mode(RobotMode mode);
RobotMode gait_get_mode();

enum GaitDir {
  GAIT_STOP = 0,
  GAIT_FORWARD,
  GAIT_BACKWARD,
  GAIT_TURN_LEFT,
  GAIT_TURN_RIGHT
};

// Inicializa la marcha y lleva el robot a posición de reposo.
void gait_init();

// Cambia la dirección de marcha.
// GAIT_STOP inicia un retorno suave a la posición de reposo.
// Cualquier otra dirección cancela el retorno si estaba en curso.
void gait_set_dir(GaitDir dir);

// Debe llamarse desde loop() con millis().
// Actualiza la máquina de estados de la marcha y mueve los servos.
void gait_update(uint32_t now_ms);
