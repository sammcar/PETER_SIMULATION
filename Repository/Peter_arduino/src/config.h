#pragma once
#include <stdint.h>

// ── I2C / PCA9685 ──────────────────────────────────────────────────────
#define I2C_SDA  8
#define I2C_SCL  7
#define PCA_ADDR 0x40
#define PWM_FREQ 50

// ── Índices de patas y articulaciones ─────────────────────────────────
#define LEG_FL 0   // Frontal  Izquierda
#define LEG_FR 1   // Frontal  Derecha
#define LEG_RL 2   // Trasera  Izquierda
#define LEG_RR 3   // Trasera  Derecha
#define COXA   0
#define FEMUR  1
#define TIBIA  2

// ── Parámetros físicos (metros) ────────────────────────────────────────
#define BODY_LEN  0.145f
#define BODY_WID  0.105f
#define L_COXA    0.042f
#define L_FEMUR   0.068f
#define L_TIBIA   0.123f
#define REST_X    0.11f   // mínimo seguro: 0.10f (θ_tibia=-90°); con 0.00f el servo clipa
#define REST_Z   -0.100f

// ── Parámetros de marcha crawl ─────────────────────────────────────────
#define STEP_LEN      0.040f  // metros — longitud de paso adelante/atrás
#define STEP_LEN_LAT  0.020f  // metros — paso lateral
#define STEP_H        0.035f  // metros — altura de elevación del pie

#define LEG_SPEED     0.006f  // metros por iteración del loop — velocidad de la pata en vuelo
#define BODY_SPEED    0.003f  // metros por iteración del loop — velocidad durante fase de cuerpo
#define REACH_TOL     0.002f  // metros — tolerancia para considerar una posición alcanzada

#define TIBIA_BIAS_DEG  22.0f  // grados extra de doblado de rodilla para compensar cedencia del servo

// ── IMU MPU-6050 ───────────────────────────────────────────────────────
#define USE_IMU        0       // 1 = habilita el IMU | 0 = deshabilita completamente (sin corrección ni parada de seguridad)
#define IMU_ALPHA      0.98f   // filtro complementario (0.95-0.99); más alto = más giróscopo, menos deriva
#define IMU_GAIN_X     0.003f  // m de corrección en X por grado de pitch (cambiar signo si está invertido)
#define IMU_GAIN_Y    -0.003f  // m de corrección en Y por grado de roll  (cambiar signo si está invertido)
#define IMU_MAX_TILT   25.0f   // grados máximos de inclinación antes de detener la marcha por seguridad

// ── Calibración de servos ──────────────────────────────────────────────
// Fórmula:  ang_efectivo = sign * angulo_deg - offset
//           Luego mapea [0,180] → [min_us, max_us]
struct ServoCalib {
    uint8_t  channel;
    uint16_t min_us;
    uint16_t max_us;
    int8_t   sign;
    float    offset;
};

extern const ServoCalib CALIB[4][3];
