#pragma once
#include <stdint.h>

// ── I2C / PCA9685 ──────────────────────────────────────────────────────
#define I2C_SDA 8
#define I2C_SCL 7
#define PCA_ADDR 0x40
#define PWM_FREQ 50

// ── Índices de patas y articulaciones ─────────────────────────────────
#define LEG_FL 0 // Frontal  Izquierda
#define LEG_FR 1 // Frontal  Derecha
#define LEG_RL 2 // Trasera  Izquierda
#define LEG_RR 3 // Trasera  Derecha
#define COXA 0
#define FEMUR 1
#define TIBIA 2

// ── Parámetros físicos (metros) ────────────────────────────────────────
#define BODY_LEN 0.145f
#define BODY_WID 0.105f
#define L_COXA 0.042f
#define L_FEMUR 0.068f
#define L_TIBIA 0.123f
#define REST_X                                                                 \
  0.00f // mínimo seguro: 0.10f (θ_tibia=-90°); con 0.00f el servo clipa
#define REST_Z -0.120f

// ── Parámetros de marcha crawl ─────────────────────────────────────────
#define STEP_LEN 0.040f     // metros — longitud de paso adelante/atrás
#define STEP_LEN_LAT 0.020f // metros — paso lateral
#define STEP_H 0.060f       // metros — altura de elevación del pie

#define LEG_SPEED                                                              \
  0.006f // metros por iteración del loop — velocidad de la pata en vuelo
#define BODY_SPEED                                                             \
  0.003f // metros por iteración del loop — velocidad durante fase de cuerpo
#define REACH_TOL                                                              \
  0.002f // metros — tolerancia para considerar una posición alcanzada

#define TIBIA_BIAS_DEG                                                         \
  22.0f // grados extra de doblado de rodilla para compensar cedencia del servo

// ── IMU MPU-6050 ───────────────────────────────────────────────────────
#define USE_IMU                                                                \
  0 // 1 = habilita el IMU | 0 = deshabilita completamente (sin corrección ni
    // parada de seguridad)
#define IMU_ALPHA                                                              \
  0.98f // filtro complementario (0.95-0.99); más alto = más giróscopo, menos
        // deriva
#define IMU_GAIN_X                                                             \
  0.003f // m de corrección en X por grado de pitch (cambiar signo si está
         // invertido)
#define IMU_GAIN_Y                                                             \
  -0.003f // m de corrección en Y por grado de roll  (cambiar signo si está
          // invertido)
#define IMU_MAX_TILT                                                           \
  25.0f // grados máximos de inclinación antes de detener la marcha por
        // seguridad

// ── Geometría Absoluta (Peter Controller) ──────────────────────────────
#define SIDE_m 0.1105f   // Alcance lateral (r_total validado en Python)
#define DIAG_m 0.078135f // Proyección a 45° (SIDE_m * 0.707106)

// Constantes precalculadas para los giros in-place (Turn)
// Derivadas de las ecuaciones de _calc_peter_turn_constants()
#define TURN_X1 0.08588f
#define TURN_Y1 0.04013f
#define TURN_X0 0.00872f
#define TURN_Y0 0.10956f

// ── Calibración de servos ──────────────────────────────────────────────
// Fórmula:  ang_efectivo = sign * angulo_deg - offset
//           Luego mapea [0,180] → [min_us, max_us]
struct ServoCalib {
  uint8_t channel;
  uint16_t min_us;
  uint16_t max_us;
  int8_t sign;
  float offset;
};

extern const ServoCalib CALIB[4][3];

// ── Modo de robot ──────────────────────────────────────────────────────
enum RobotMode { MODE_SPIDER = 0, MODE_VEHICLE, MODE_OMNI };

// ── Profundidad del pie en modos de conducción ─────────────────────────
#define FOLD_H_Z                                                               \
  -0.005f // Altura para Modo H: profundidad tibia bajo cuerpo (m)
#define FOLD_X_BODY_H                                                          \
  0.025f // Altura cuerpo sobre ruedas en Modo X (m) — máx: L_FEMUR = 0.068

// ── Multiplicador de velocidad para transiciones de regreso a Spider ──
#define TRANS_TO_SPIDER_MULT 1.5f

// ── Velocidad base de motores ─────────────────────────────────────────
#define MOTOR_BASE_SPEED 130 // 0-255

// ── Dirección física por motor (+1 = normal, -1 = invertido) ─────────
// Cambiar a -1 para invertir un motor sin tocar la lógica de control.
#define M1_DIR -1 // pines 3,4  – físicamente FL (Frontal Izquierda)
#define M2_DIR 1  // pines 6,5  – físicamente RL (Trasera Izquierda)
#define M3_DIR 1  // pines 10,9  – físicamente FR (Frontal Derecha)
#define M4_DIR -1 // pines 11,12 – físicamente RR (Trasera Derecha)
