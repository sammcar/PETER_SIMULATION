#include "gait.h"
#include "config.h"
#include "imu.h"
#include "kinematics.h"
#include "motors.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

#define TICK_INTERVAL_MS 33

// ── Estado interno ─────────────────────────────────────────────────────
static float site_now[4][3];
static float site_expect[4][3];
static float site_rest[4][3];

static GaitDir current_dir = GAIT_STOP;
static uint32_t _last_tick_ms = 0;

static int _state_idx = 0;
static const char *_stable_pose = "LEFT_Y";
static GaitDir _next_dir = GAIT_STOP;
static bool _is_moving = false;

static RobotMode _robot_mode = MODE_SPIDER;

RobotMode gait_get_mode() { return _robot_mode; }
// ── Forward declarations ───────────────────────────────────────────────
static void get_peter_target(uint8_t leg, float local_x, float local_y,
                             float local_z, float out[3]);

extern void mover(uint8_t leg, uint8_t joint, float angulo_deg);

// ── Helpers ────────────────────────────────────────────────────────────

// Mueve site_now hacia site_expect a la velocidad dada.
// Retorna true cuando TODAS las patas alcanzaron su objetivo.
static bool tick_toward_expect(float speed) {
  bool all_done = true;
  for (uint8_t l = 0; l < 4; l++) {
    for (uint8_t j = 0; j < 3; j++) {
      float diff = site_expect[l][j] - site_now[l][j];
      if (fabsf(diff) > REACH_TOL) {
        all_done = false;
        float step = fminf(fabsf(diff), speed);
        site_now[l][j] += (diff > 0.0f ? step : -step);
      } else {
        site_now[l][j] = site_expect[l][j];
      }
    }
  }
  return all_done;
}

static void apply_ik_all() {
  for (uint8_t l = 0; l < 4; l++) {
    float q[3];
    if (leg_ik(l, site_now[l][0], site_now[l][1], site_now[l][2], q)) {
      mover(l, COXA, q[0]);
      mover(l, FEMUR, q[1]);
      mover(l, TIBIA, q[2] - TIBIA_BIAS_DEG);
    }
  }
}

// Función auxiliar bloqueante para animar las transiciones
static void _animate_to_expect(float speed) {
  while (!tick_toward_expect(speed)) {
    apply_ik_all();
    delay(TICK_INTERVAL_MS);
  }
  apply_ik_all(); // Un último pulso para asegurar precisión
}

// ── Helpers de posición para transiciones ──────────────────────────────

// Calcula posición X-mode para un leg y la pone en site_expect
static void _expect_x_pos(uint8_t leg) {
  float rhoriz =
      L_COXA + L_FEMUR + sqrtf(L_TIBIA * L_TIBIA - FOLD_X_Z * FOLD_X_Z);
  float rdiag = rhoriz * 0.70711f;
  float sx = BODY_LEN / 2.0f + rdiag;
  float sy = BODY_WID / 2.0f + rdiag;
  float hy = (leg == LEG_FL || leg == LEG_RL) ? +sy : -sy;
  float hx = (leg == LEG_FL || leg == LEG_FR) ? +sx : -sx;
  site_expect[leg][0] = hx;
  site_expect[leg][1] = hy;
  site_expect[leg][2] = FOLD_X_Z;
}

// Pone las 4 patas en H-mode
static void _expect_all_h() {
  float rhoriz =
      L_COXA + L_FEMUR + sqrtf(L_TIBIA * L_TIBIA - FOLD_H_Z * FOLD_H_Z);
  float sx = BODY_LEN / 2.0f;
  float sy = BODY_WID / 2.0f + rhoriz;
  site_expect[LEG_FL][0] = +sx;
  site_expect[LEG_FL][1] = +sy;
  site_expect[LEG_FL][2] = FOLD_H_Z;
  site_expect[LEG_FR][0] = +sx;
  site_expect[LEG_FR][1] = -sy;
  site_expect[LEG_FR][2] = FOLD_H_Z;
  site_expect[LEG_RL][0] = -sx;
  site_expect[LEG_RL][1] = +sy;
  site_expect[LEG_RL][2] = FOLD_H_Z;
  site_expect[LEG_RR][0] = -sx;
  site_expect[LEG_RR][1] = -sy;
  site_expect[LEG_RR][2] = FOLD_H_Z;
}

// Pone las 4 patas en X-mode
static void _expect_all_x() {
  _expect_x_pos(LEG_FL);
  _expect_x_pos(LEG_FR);
  _expect_x_pos(LEG_RL);
  _expect_x_pos(LEG_RR);
}

// Pone las 4 patas en LEFT_Y x,y pero con z personalizado
static void _expect_left_y_at_z(float z) {
  get_peter_target(LEG_FL, SIDE_m, 0.0f, z, site_expect[LEG_FL]);
  get_peter_target(LEG_FR, DIAG_m, DIAG_m, z, site_expect[LEG_FR]);
  get_peter_target(LEG_RL, SIDE_m, 0.0f, z, site_expect[LEG_RL]);
  get_peter_target(LEG_RR, DIAG_m, DIAG_m, z, site_expect[LEG_RR]);
}

// ── Animaciones de transición ──────────────────────────────────────────

// ── Animaciones de transición ──────────────────────────────────────────

static void _trans_spider_to_H() {
  bool left_y = (strcmp(_stable_pose, "LEFT_Y") == 0);

  // Paso 1: patas diagonales → X-mode (camino directo, dentro del workspace)
  if (left_y) { _expect_x_pos(LEG_FR); _expect_x_pos(LEG_RR); }
  else         { _expect_x_pos(LEG_FL); _expect_x_pos(LEG_RL); }
  _animate_to_expect(BODY_SPEED);

  // Paso 2: todas → H-mode (FL/RL desde REST_Z, FR/RR rotan coxa — simultáneo)
  _expect_all_h();
  _animate_to_expect(BODY_SPEED);
}

static void _trans_spider_to_X() {
  bool left_y = (strcmp(_stable_pose, "LEFT_Y") == 0);

  // Paso 1: patas diagonales → X-mode (camino directo, dentro del workspace)
  if (left_y) { _expect_x_pos(LEG_FR); _expect_x_pos(LEG_RR); }
  else         { _expect_x_pos(LEG_FL); _expect_x_pos(LEG_RL); }
  _animate_to_expect(BODY_SPEED);

  // Paso 2: patas laterales → X-mode (camino directo)
  if (left_y) { _expect_x_pos(LEG_FL); _expect_x_pos(LEG_RL); }
  else         { _expect_x_pos(LEG_FR); _expect_x_pos(LEG_RR); }
  _animate_to_expect(BODY_SPEED);
}

static void _trans_H_to_spider() {
  // Paso 1: FR+RR ajustan coxa a diagonal (Z no cambia, están en el suelo)
  float rhoriz =
      L_COXA + L_FEMUR + sqrtf(L_TIBIA * L_TIBIA - FOLD_X_Z * FOLD_X_Z);
  float rdiag = rhoriz * 0.70711f;
  float sx = BODY_LEN / 2.0f + rdiag;
  float sy = BODY_WID / 2.0f + rdiag;
  site_expect[LEG_FR][0] = +sx;
  site_expect[LEG_FR][1] = -sy;
  site_expect[LEG_FR][2] = FOLD_H_Z;
  site_expect[LEG_RR][0] = -sx;
  site_expect[LEG_RR][1] = -sy;
  site_expect[LEG_RR][2] = FOLD_H_Z;
  _animate_to_expect(BODY_SPEED);

  // Paso 2: Todas a LEFT_Y xy, Z sin cambio (arrastre sobre ruedas)
  _expect_left_y_at_z(FOLD_H_Z);
  _animate_to_expect(BODY_SPEED);

  // Paso 3: Subir Z al reposo (levanta el chasis)
  for (uint8_t l = 0; l < 4; l++)
    site_expect[l][2] = REST_Z;
  _animate_to_expect(BODY_SPEED);
}

static void _trans_X_to_spider() {
  // Paso 1: FL+RL cierran a LEFT_Y lateral (Z no cambia)
  get_peter_target(LEG_FL, SIDE_m, 0.0f, FOLD_X_Z, site_expect[LEG_FL]);
  get_peter_target(LEG_RL, SIDE_m, 0.0f, FOLD_X_Z, site_expect[LEG_RL]);
  _animate_to_expect(BODY_SPEED);

  // Paso 2: FR+RR arrastran a LEFT_Y diagonal (Z no cambia)
  get_peter_target(LEG_FR, DIAG_m, DIAG_m, FOLD_X_Z, site_expect[LEG_FR]);
  get_peter_target(LEG_RR, DIAG_m, DIAG_m, FOLD_X_Z, site_expect[LEG_RR]);
  _animate_to_expect(BODY_SPEED);

  // Paso 3: Subir Z al reposo (levanta el chasis)
  for (uint8_t l = 0; l < 4; l++)
    site_expect[l][2] = REST_Z;
  _animate_to_expect(BODY_SPEED);
}

static void _trans_H_to_X() {
  _expect_all_x();
  // Están a la misma altura o casi (ambas FOLD_*_Z), basta un paso
  _animate_to_expect(BODY_SPEED);
}

static void _trans_X_to_H() {
  _expect_all_h();
  _animate_to_expect(BODY_SPEED);
}

// ── Control Principal de Modos ─────────────────────────────────────────

// ── Control Principal de Modos ─────────────────────────────────────────

void gait_set_mode(RobotMode target_mode) {
  if (_robot_mode == target_mode)
    return;

  // Esperar checkpoint si estamos en marcha cuadrúpedo
  if (_robot_mode == MODE_SPIDER && _is_moving) {
    _next_dir = GAIT_STOP;
    while (_is_moving) {
      gait_update(millis());
      delay(TICK_INTERVAL_MS);
    }
  }

  current_dir = GAIT_STOP;
  _is_moving = false;
  motors_stop();

  RobotMode from = _robot_mode;

  // Delegar a las rutinas seguras de transición divididas
  if (from == MODE_SPIDER && target_mode == MODE_VEHICLE)
    _trans_spider_to_H();
  else if (from == MODE_SPIDER && target_mode == MODE_OMNI)
    _trans_spider_to_X();
  else if (from == MODE_VEHICLE && target_mode == MODE_SPIDER)
    _trans_H_to_spider();
  else if (from == MODE_OMNI && target_mode == MODE_SPIDER)
    _trans_X_to_spider();
  else if (from == MODE_VEHICLE && target_mode == MODE_OMNI)
    _trans_H_to_X();
  else if (from == MODE_OMNI && target_mode == MODE_VEHICLE)
    _trans_X_to_H();

  _robot_mode = target_mode;

  if (target_mode == MODE_SPIDER) {
    _stable_pose = "LEFT_Y";
    for (uint8_t l = 0; l < 4; l++) {
      memcpy(site_rest[l], site_expect[l], sizeof(float[3]));
    }
  }
}

// Mapea coordenadas locales de Peter a coordenadas de marco del cuerpo (Body
// Frame)
static void get_peter_target(uint8_t leg, float local_x, float local_y,
                             float local_z, float out[3]) {
  float hx =
      (leg == LEG_FL || leg == LEG_FR) ? (BODY_LEN / 2.0f) : (-BODY_LEN / 2.0f);
  float hy =
      (leg == LEG_FL || leg == LEG_RL) ? (BODY_WID / 2.0f) : (-BODY_WID / 2.0f);

  switch (leg) {
  case LEG_FL: // LU (Left Up) en Peter
    out[0] = hx + local_y;
    out[1] = hy + local_x;
    break;
  case LEG_FR: // RU (Right Up) en Peter
    out[0] = hx + local_y;
    out[1] = hy - local_x;
    break;
  case LEG_RL: // LD (Left Down) en Peter
    out[0] = hx - local_y;
    out[1] = hy + local_x;
    break;
  case LEG_RR: // RD (Right Down) en Peter
    out[0] = hx - local_y;
    out[1] = hy - local_x;
    break;
  }
  out[2] = local_z;
}

static void snap_to_left_y() {
  get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z, site_rest[LEG_FL]);
  get_peter_target(LEG_FR, DIAG_m, DIAG_m, REST_Z, site_rest[LEG_FR]);
  get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z, site_rest[LEG_RL]);
  get_peter_target(LEG_RR, DIAG_m, DIAG_m, REST_Z, site_rest[LEG_RR]);

  for (uint8_t l = 0; l < 4; l++) {
    memcpy(site_now[l], site_rest[l], sizeof(float[3]));
    memcpy(site_expect[l], site_rest[l], sizeof(float[3]));
  }

  apply_ik_all();

  Serial.println("\n--- SNAP A LEFT-Y ---");
  Serial.printf(" FL = [%.4f, %.4f]\n", site_now[LEG_FL][0],
                site_now[LEG_FL][1]);
  Serial.printf(" FR = [%.4f, %.4f]\n", site_now[LEG_FR][0],
                site_now[LEG_FR][1]);
  Serial.printf(" RL = [%.4f, %.4f]\n", site_now[LEG_RL][0],
                site_now[LEG_RL][1]);
  Serial.printf(" RR = [%.4f, %.4f]\n", site_now[LEG_RR][0],
                site_now[LEG_RR][1]);
}

static void snap_to_right_y() {
  get_peter_target(LEG_FL, DIAG_m, DIAG_m, REST_Z, site_rest[LEG_FL]);
  get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z, site_rest[LEG_FR]);
  get_peter_target(LEG_RL, DIAG_m, DIAG_m, REST_Z, site_rest[LEG_RL]);
  get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z, site_rest[LEG_RR]);

  for (uint8_t l = 0; l < 4; l++) {
    memcpy(site_now[l], site_rest[l], sizeof(float[3]));
    memcpy(site_expect[l], site_rest[l], sizeof(float[3]));
  }

  apply_ik_all();

  Serial.println("\n--- SNAP A RIGHT-Y ---");
  Serial.printf(" FL = [%.4f, %.4f]\n", site_now[LEG_FL][0],
                site_now[LEG_FL][1]);
  Serial.printf(" FR = [%.4f, %.4f]\n", site_now[LEG_FR][0],
                site_now[LEG_FR][1]);
  Serial.printf(" RL = [%.4f, %.4f]\n", site_now[LEG_RL][0],
                site_now[LEG_RL][1]);
  Serial.printf(" RR = [%.4f, %.4f]\n", site_now[LEG_RR][0],
                site_now[LEG_RR][1]);
}

void _update_state_machine() {
  if (current_dir == GAIT_FORWARD) {
    switch (_state_idx) {
    case 0:
      get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_FL]);
      break;
    case 1:
      get_peter_target(LEG_FL, DIAG_m, 2.0f * DIAG_m, REST_Z + STEP_H,
                       site_expect[LEG_FL]);
      break;
    case 2:
      get_peter_target(LEG_FL, DIAG_m, 2.0f * DIAG_m, REST_Z,
                       site_expect[LEG_FL]);
      break;
    case 3:
      get_peter_target(LEG_FL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FR]);
      get_peter_target(LEG_RL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, DIAG_m, 2.0f * DIAG_m, REST_Z,
                       site_expect[LEG_RR]);
      break;
    case 4:
      get_peter_target(LEG_RR, DIAG_m, 2.0f * DIAG_m, REST_Z + STEP_H,
                       site_expect[LEG_RR]);
      break;
    case 5:
      get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_RR]);
      break;
    case 6:
      get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RR]);
      break;
    case 7:
      get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_FR]);
      break;
    case 8:
      get_peter_target(LEG_FR, DIAG_m, 2.0f * DIAG_m, REST_Z + STEP_H,
                       site_expect[LEG_FR]);
      break;
    case 9:
      get_peter_target(LEG_FR, DIAG_m, 2.0f * DIAG_m, REST_Z,
                       site_expect[LEG_FR]);
      break;
    case 10:
      get_peter_target(LEG_FR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FR]);
      get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_RR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RR]);
      get_peter_target(LEG_RL, DIAG_m, 2.0f * DIAG_m, REST_Z,
                       site_expect[LEG_RL]);
      break;
    case 11:
      get_peter_target(LEG_RL, DIAG_m, 2.0f * DIAG_m, REST_Z + STEP_H,
                       site_expect[LEG_RL]);
      break;
    case 12:
      get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_RL]);
      break;
    case 13:
      get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RL]);
      break;
    }
  } else if (current_dir == GAIT_BACKWARD) {
    switch (_state_idx) {
    // --- PRIMERA MITAD (Left-Y -> Right-Y) ---
    case 0:
      get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_RL]);
      break;
    case 1:
      get_peter_target(LEG_RL, DIAG_m, 2.0f * DIAG_m, REST_Z + STEP_H,
                       site_expect[LEG_RL]);
      break;
    case 2:
      get_peter_target(LEG_RL, DIAG_m, 2.0f * DIAG_m, REST_Z,
                       site_expect[LEG_RL]);
      break;
    case 3:
      get_peter_target(LEG_FL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, DIAG_m, 2.0f * DIAG_m, REST_Z,
                       site_expect[LEG_FR]); // Rezagada
      get_peter_target(LEG_RL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RR]);
      break;
    case 4:
      get_peter_target(LEG_FR, DIAG_m, 2.0f * DIAG_m, REST_Z + STEP_H,
                       site_expect[LEG_FR]);
      break;
    case 5:
      get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_FR]);
      break;
    case 6:
      get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FR]);
      break;

    // --- SEGUNDA MITAD (Right-Y -> Left-Y) ---
    case 7:
      get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_RR]);
      break;
    case 8:
      get_peter_target(LEG_RR, DIAG_m, 2.0f * DIAG_m, REST_Z + STEP_H,
                       site_expect[LEG_RR]);
      break;
    case 9:
      get_peter_target(LEG_RR, DIAG_m, 2.0f * DIAG_m, REST_Z,
                       site_expect[LEG_RR]);
      break;
    case 10:
      get_peter_target(LEG_FL, DIAG_m, 2.0f * DIAG_m, REST_Z,
                       site_expect[LEG_FL]); // Rezagada
      get_peter_target(LEG_FR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FR]);
      get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RR]);
      break;
    case 11:
      get_peter_target(LEG_FL, DIAG_m, 2.0f * DIAG_m, REST_Z + STEP_H,
                       site_expect[LEG_FL]);
      break;
    case 12:
      get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_FL]);
      break;
    case 13:
      get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FL]);
      break;
    }
  } else if (current_dir == GAIT_TURN_RIGHT) {
    switch (_state_idx) {
    // --- PRIMERA MITAD (Left-Y -> Right-Y) --- FL lidera
    case 0:
      get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_FL]);
      break;
    case 1:
      get_peter_target(LEG_FL, TURN_X0, TURN_Y0, REST_Z + STEP_H,
                       site_expect[LEG_FL]);
      get_peter_target(LEG_FR, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_FR]);
      get_peter_target(LEG_RL, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_RR]);
      break;
    case 2:
      get_peter_target(LEG_FL, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_FR]);
      get_peter_target(LEG_RL, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_RR]);
      break;
    case 3:
      get_peter_target(LEG_FR, TURN_X0, TURN_Y0, REST_Z + STEP_H,
                       site_expect[LEG_FR]);
      break;
    case 4:
      get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_FR]);
      get_peter_target(LEG_FL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_RL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RR]);
      break;
    case 5: // LOWER FR -> Right-Y estable
      get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FR]);
      get_peter_target(LEG_FL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_RL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RR]);
      break;

    // --- SEGUNDA MITAD (Right-Y -> Left-Y) --- RR lidera
    case 6:
      get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_RR]);
      break;
    case 7:
      get_peter_target(LEG_RR, TURN_X0, TURN_Y0, REST_Z + STEP_H,
                       site_expect[LEG_RR]);
      get_peter_target(LEG_RL, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_FL, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_FR]);
      break;
    case 8:
      get_peter_target(LEG_RR, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_RR]);
      get_peter_target(LEG_RL, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_FL, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_FR]);
      break;
    case 9:
      get_peter_target(LEG_RL, TURN_X0, TURN_Y0, REST_Z + STEP_H,
                       site_expect[LEG_RL]);
      break;
    case 10:
      get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_RL]);
      get_peter_target(LEG_RR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RR]);
      get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FR]);
      break;
    case 11: // LOWER RL -> Left-Y estable
      get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RR]);
      get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FR]);
      break;
    }

  } else if (current_dir == GAIT_TURN_LEFT) {
    switch (_state_idx) {
    // --- PRIMERA MITAD (Right-Y -> Left-Y) --- FR lidera
    case 0:
      get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_FR]);
      break;
    case 1:
      get_peter_target(LEG_FR, TURN_X0, TURN_Y0, REST_Z + STEP_H,
                       site_expect[LEG_FR]);
      get_peter_target(LEG_FL, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_RL, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_RR]);
      break;
    case 2:
      get_peter_target(LEG_FR, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_FR]);
      get_peter_target(LEG_FL, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_RL, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_RR]);
      break;
    case 3:
      get_peter_target(LEG_FL, TURN_X0, TURN_Y0, REST_Z + STEP_H,
                       site_expect[LEG_FL]);
      break;
    case 4:
      get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_FL]);
      get_peter_target(LEG_FR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FR]);
      get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RR]);
      break;
    case 5: // LOWER FL -> Left-Y estable
      get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FR]);
      get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RR]);
      break;

    // --- SEGUNDA MITAD (Left-Y -> Right-Y) --- RL lidera
    case 6:
      get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_RL]);
      break;
    case 7:
      get_peter_target(LEG_RL, TURN_X0, TURN_Y0, REST_Z + STEP_H,
                       site_expect[LEG_RL]);
      get_peter_target(LEG_RR, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_RR]);
      get_peter_target(LEG_FL, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_FR]);
      break;
    case 8:
      get_peter_target(LEG_RL, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_RR, TURN_X0, TURN_Y0, REST_Z, site_expect[LEG_RR]);
      get_peter_target(LEG_FL, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, TURN_X1, TURN_Y1, REST_Z, site_expect[LEG_FR]);
      break;
    case 9:
      get_peter_target(LEG_RR, TURN_X0, TURN_Y0, REST_Z + STEP_H,
                       site_expect[LEG_RR]);
      break;
    case 10:
      get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z + STEP_H,
                       site_expect[LEG_RR]);
      get_peter_target(LEG_RL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_FL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FR]);
      break;
    case 11: // LOWER RR -> Right-Y estable
      get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_RR]);
      get_peter_target(LEG_RL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_RL]);
      get_peter_target(LEG_FL, DIAG_m, DIAG_m, REST_Z, site_expect[LEG_FL]);
      get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z, site_expect[LEG_FR]);
      break;
    }
  }
}

// ── API pública ────────────────────────────────────────────────────────

void gait_init() { snap_to_left_y(); }

void gait_set_dir(GaitDir dir) {
  if (dir == current_dir)
    return;

  _next_dir = dir;
  if (dir == GAIT_STOP)
    return;

  if (!_is_moving && (dir == GAIT_FORWARD || dir == GAIT_BACKWARD ||
                      dir == GAIT_TURN_LEFT || dir == GAIT_TURN_RIGHT)) {
    current_dir = dir;
    _is_moving = true;
    bool is_turn = (dir == GAIT_TURN_LEFT || dir == GAIT_TURN_RIGHT);

    if (strcmp(_stable_pose, "RIGHT_Y") == 0) {
      _state_idx = is_turn ? (dir == GAIT_TURN_RIGHT ? 6 : 0) : 7;
      snap_to_right_y();
    } else {
      _state_idx = is_turn ? (dir == GAIT_TURN_RIGHT ? 0 : 6) : 0;
      _stable_pose = "LEFT_Y";
      snap_to_left_y();
    }

    _update_state_machine();
    Serial.println("\n>>> INICIANDO MARCHA / GIRO <<<");
  }
}

void gait_update(uint32_t now_ms) {
  if (now_ms - _last_tick_ms < TICK_INTERVAL_MS)
    return;
  _last_tick_ms = now_ms;

  if (!_is_moving)
    return;

  bool is_turn =
      (current_dir == GAIT_TURN_LEFT || current_dir == GAIT_TURN_RIGHT);

  // Ajuste de velocidad: Los giros usan BODY_SPEED en las fases 1, 4, 7, 10
  float speed = LEG_SPEED;
  if (is_turn) {
    if (_state_idx == 1 || _state_idx == 4 || _state_idx == 7 ||
        _state_idx == 10)
      speed = BODY_SPEED;
  } else {
    if (_state_idx == 3 || _state_idx == 10)
      speed = BODY_SPEED;
  }

  bool done = tick_toward_expect(speed);
  apply_ik_all();

  if (done) {
    bool checkpoint_reached = false;

    // Evaluar Checkpoints
    if (is_turn) {
      if (_state_idx == 5) {
        _stable_pose = (current_dir == GAIT_TURN_RIGHT) ? "RIGHT_Y" : "LEFT_Y";
        checkpoint_reached = true;
      } else if (_state_idx == 11) {
        _stable_pose = (current_dir == GAIT_TURN_RIGHT) ? "LEFT_Y" : "RIGHT_Y";
        checkpoint_reached = true;
      }
    } else {
      if (_state_idx == 6) {
        _stable_pose = "RIGHT_Y";
        checkpoint_reached = true;
      } else if (_state_idx == 13) {
        _stable_pose = "LEFT_Y";
        checkpoint_reached = true;
      }
    }

    // Procesar transición de checkpoint
    if (checkpoint_reached) {
      if (_next_dir == GAIT_STOP) {
        current_dir = GAIT_STOP;
        _is_moving = false;
        Serial.printf(">>> STOP SUAVE (Asentado en %s) <<<\n", _stable_pose);
        return;
      } else if (_next_dir != current_dir) {
        // PUENTE DINÁMICO: Cambiar modo de marcha al vuelo adaptando el índice
        // a la pose física actual
        current_dir = _next_dir;
        bool next_is_turn =
            (current_dir == GAIT_TURN_LEFT || current_dir == GAIT_TURN_RIGHT);

        if (strcmp(_stable_pose, "RIGHT_Y") == 0) {
          _state_idx =
              next_is_turn ? (current_dir == GAIT_TURN_RIGHT ? 6 : 0) : 7;
        } else {
          _state_idx =
              next_is_turn ? (current_dir == GAIT_TURN_RIGHT ? 0 : 6) : 0;
        }
      } else {
        // Mismo comando continuo, incrementar con wrap respectivo
        _state_idx++;
        if (is_turn && _state_idx > 11)
          _state_idx = 0;
        else if (!is_turn && _state_idx > 13)
          _state_idx = 0;
      }
    } else {
      // Fases intermedias puras
      _state_idx++;
    }

    _update_state_machine();
  }
}
