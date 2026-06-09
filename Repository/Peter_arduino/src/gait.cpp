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

// Mueve las patas a la posición lateral (Vehículo H)
static void _set_fold_h_expect() {
  // Con fémur horizontal, la extensión se deriva geométricamente de FOLD_H_Z
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

// Mueve las patas a la posición diagonal (Omni X)
static void _set_fold_x_expect() {
  // Mismo r_horiz que H, pero distribuido en diagonal → multiplicar por
  // cos(45°)
  float rhoriz =
      L_COXA + L_FEMUR + sqrtf(L_TIBIA * L_TIBIA - FOLD_X_Z * FOLD_X_Z);
  float rdiag = rhoriz * 0.70711f;
  float sx = BODY_LEN / 2.0f + rdiag;
  float sy = BODY_WID / 2.0f + rdiag;

  site_expect[LEG_FL][0] = +sx;
  site_expect[LEG_FL][1] = +sy;
  site_expect[LEG_FL][2] = FOLD_X_Z;
  site_expect[LEG_FR][0] = +sx;
  site_expect[LEG_FR][1] = -sy;
  site_expect[LEG_FR][2] = FOLD_X_Z;
  site_expect[LEG_RL][0] = -sx;
  site_expect[LEG_RL][1] = +sy;
  site_expect[LEG_RL][2] = FOLD_X_Z;
  site_expect[LEG_RR][0] = -sx;
  site_expect[LEG_RR][1] = -sy;
  site_expect[LEG_RR][2] = FOLD_X_Z;
}

// (Temporal para la prueba de la Etapa 2. Luego lo reemplazaremos con la
// máquina de transiciones de la Etapa 3)

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

void gait_set_mode(RobotMode mode) {
  _robot_mode = mode;
  if (mode == MODE_VEHICLE) {
    _set_fold_h_expect();
  } else if (mode == MODE_OMNI) {
    _set_fold_x_expect();
  } else {
    return;
  }
  // Snap inmediato: copiar expect a now y aplicar IK
  for (uint8_t l = 0; l < 4; l++)
    memcpy(site_now[l], site_expect[l], sizeof(float[3]));
  apply_ik_all();
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
