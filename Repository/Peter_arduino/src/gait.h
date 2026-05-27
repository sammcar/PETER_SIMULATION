#pragma once
#include <stdint.h>

// Direcciones de movimiento (igual que teleop_twist_keyboard de ROS 2)
enum GaitDir {
    GAIT_STOP = 0,
    GAIT_FORWARD,
    GAIT_BACKWARD,
    GAIT_LEFT,
    GAIT_RIGHT,
    GAIT_FWD_LEFT,
    GAIT_FWD_RIGHT,
    GAIT_BCK_LEFT,
    GAIT_BCK_RIGHT,
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

// Lee la posición actual del pie de una pata (frame cuerpo, metros).
void gait_get_foot_pos(int leg, float out[3]);
