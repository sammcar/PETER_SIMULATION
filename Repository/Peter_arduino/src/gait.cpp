#include "gait.h"
#include "config.h"
#include "imu.h"
#include "kinematics.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

#define TICK_INTERVAL_MS 33

// Orden diagonal-wave: FL → RR → FR → RL
static const uint8_t SWING_ORDER[4] = {LEG_FL, LEG_RR, LEG_FR, LEG_RL};

// ── Fases de la máquina de estados ────────────────────────────────────
typedef enum {
  PHASE_IDLE = 0,
  PHASE_LIFT,   // sube la pata de balanceo (solo Z)
  PHASE_MOVE,   // avanza la pata de balanceo (X/Y a altura levantada)
  PHASE_LOWER,  // baja la pata de balanceo al suelo
  PHASE_BODY,   // mueve todas las patas a la posición de apoyo posterior
  PHASE_RETURN, // retorno suave a reposo
} Phase;

// ── Estado interno ─────────────────────────────────────────────────────
static float site_now[4][3]; // posición actual del pie (frame cuerpo, metros)
static float site_expect[4][3]; // posición objetivo del pie
static float site_rest[4][3];   // posición de reposo de cada pata

static Phase phase = PHASE_IDLE;
static uint8_t swing_idx = 0; // índice 0-3 en SWING_ORDER
static GaitDir current_dir = GAIT_STOP;
static float step_dx = 0.0f;
static float step_dy = 0.0f;
static uint32_t _last_tick_ms = 0;

extern void mover(uint8_t leg, uint8_t joint, float angulo_deg);

// ── Helpers ────────────────────────────────────────────────────────────

static void dir_to_step(GaitDir dir, float &dx, float &dy) {
  float sl = STEP_LEN, sll = STEP_LEN_LAT;
  float d = sl * 0.707f;
  switch (dir) {
  case GAIT_FORWARD:
    dx = sl;
    dy = 0;
    return;
  case GAIT_BACKWARD:
    dx = -sl;
    dy = 0;
    return;
  case GAIT_LEFT:
    dx = 0;
    dy = sll;
    return;
  case GAIT_RIGHT:
    dx = 0;
    dy = -sll;
    return;
  case GAIT_FWD_LEFT:
    dx = d;
    dy = d;
    return;
  case GAIT_FWD_RIGHT:
    dx = d;
    dy = -d;
    return;
  case GAIT_BCK_LEFT:
    dx = -d;
    dy = d;
    return;
  case GAIT_BCK_RIGHT:
    dx = -d;
    dy = -d;
    return;
  default:
    dx = 0;
    dy = 0;
    return;
  }
}

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

// Fija el objetivo de UNA pata; las demás no cambian.
static void set_site_one(uint8_t leg, float x, float y, float z) {
  site_expect[leg][0] = x;
  site_expect[leg][1] = y;
  site_expect[leg][2] = z;
}

// Todas las patas a su posición de reposo.
static void expect_all_rest() {
  for (uint8_t l = 0; l < 4; l++)
    memcpy(site_expect[l], site_rest[l], sizeof(float[3]));
}

// Todas las patas a la posición de apoyo posterior: rest - step/2.
// Esto hace avanzar el cuerpo medio paso al llegar ahí.
static void expect_all_stride_back(float dx, float dy) {
  for (uint8_t l = 0; l < 4; l++) {
    site_expect[l][0] = site_rest[l][0] - dx * 0.5f;
    site_expect[l][1] = site_rest[l][1] - dy * 0.5f;
    site_expect[l][2] = site_rest[l][2];
  }
}

// Sincroniza los expects de las patas en apoyo a su posición actual
// (evita movimiento no deseado al comenzar una nueva marcha desde IDLE).
static void freeze_stance_legs() {
  for (uint8_t l = 0; l < 4; l++) {
    if (l != SWING_ORDER[swing_idx])
      memcpy(site_expect[l], site_now[l], sizeof(float[3]));
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

void snap_to_left_y() {
  // 1. Asignar las coordenadas absolutas basadas en la geometría local
  get_peter_target(LEG_FL, SIDE_m, 0.0f, REST_Z, site_rest[LEG_FL]);
  get_peter_target(LEG_FR, DIAG_m, DIAG_m, REST_Z, site_rest[LEG_FR]);
  get_peter_target(LEG_RL, SIDE_m, 0.0f, REST_Z, site_rest[LEG_RL]);
  get_peter_target(LEG_RR, DIAG_m, DIAG_m, REST_Z, site_rest[LEG_RR]);

  // 2. Forzar el estado actual y esperado
  for (uint8_t l = 0; l < 4; l++) {
    memcpy(site_now[l], site_rest[l], sizeof(float[3]));
    memcpy(site_expect[l], site_rest[l], sizeof(float[3]));
  }

  // 3. Resolver cinemática y actualizar motores
  apply_ik_all();
  phase = PHASE_IDLE;

  // 4. Imprimir la validación cartesiana
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

void snap_to_right_y() {
  // 1. Asignar las coordenadas absolutas invertidas
  get_peter_target(LEG_FL, DIAG_m, DIAG_m, REST_Z, site_rest[LEG_FL]);
  get_peter_target(LEG_FR, SIDE_m, 0.0f, REST_Z, site_rest[LEG_FR]);
  get_peter_target(LEG_RL, DIAG_m, DIAG_m, REST_Z, site_rest[LEG_RL]);
  get_peter_target(LEG_RR, SIDE_m, 0.0f, REST_Z, site_rest[LEG_RR]);

  // 2. Forzar el estado actual y esperado
  for (uint8_t l = 0; l < 4; l++) {
    memcpy(site_now[l], site_rest[l], sizeof(float[3]));
    memcpy(site_expect[l], site_rest[l], sizeof(float[3]));
  }

  // 3. Resolver cinemática y actualizar motores
  apply_ik_all();
  phase = PHASE_IDLE;

  // 4. Imprimir la validación cartesiana
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

// Variables internas para el modo manual
static int _state_idx = 0;
static const char *_stable_pose = "LEFT_Y";
static GaitDir _next_dir = GAIT_STOP; // Buffer para no frenar en seco
static bool _is_moving = false;       // Bandera de ejecución continua

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
  }
}

void gait_step_once() {
  if (_state_idx > 13) {
    _state_idx = 0; // Permite loopear la marcha manual cerrando el ciclo
  }

  Serial.printf("\n>>> Calculando EXPECT para Estado %d...\n", _state_idx);
  _update_state_machine();

  bool done = false;
  while (!done) {
    // Los estados 3 y 10 son fases de CUERPO, usan BODY_SPEED
    float speed =
        (_state_idx == 3 || _state_idx == 10) ? BODY_SPEED : LEG_SPEED;
    done = tick_toward_expect(speed);
    apply_ik_all();
    delay(20);
  }

  Serial.printf("<<< Estado %d COMPLETADO.\n", _state_idx);

  // Actualizar marcadores de estabilidad
  if (_state_idx == 6) {
    _stable_pose = "RIGHT_Y";
  } else if (_state_idx == 13) {
    _stable_pose = "LEFT_Y";
  }

  print_gait_state();

  _state_idx++;
  if (_state_idx > 13) {
    Serial.println(
        "\n*** CICLO COMPLETO CERRADO. Robot de vuelta en Left-Y. ***\n");
  }
}

void print_gait_state() {
  Serial.printf("\n--- ESTADO ACTUAL ---\n");
  Serial.printf(" _state_idx   : %d\n", _state_idx);
  Serial.printf(" _stable_pose : %s\n", _stable_pose);

  const char *names[] = {"FL", "FR", "RL", "RR"};
  for (int i = 0; i < 4; i++) {
    Serial.printf(" %s = [%.4f, %.4f, %.4f]\n", names[i], site_now[i][0],
                  site_now[i][1], site_now[i][2]);
  }

  float t[3];
  if (_state_idx <= 6) {
    get_peter_target(LEG_FL, DIAG_m, 2.0f * DIAG_m, REST_Z, t);
    Serial.printf("\n (INFO) EXPECTED REACH FL -> [%.4f, %.4f]\n", t[0], t[1]);
  } else {
    get_peter_target(LEG_FR, DIAG_m, 2.0f * DIAG_m, REST_Z, t);
    Serial.printf("\n (INFO) EXPECTED REACH FR -> [%.4f, %.4f]\n", t[0], t[1]);
  }
  Serial.println("---------------------\n");
}

// ── Inicio de cada sub-fase ────────────────────────────────────────────

static void start_lift() {
  uint8_t sl = SWING_ORDER[swing_idx];
  dir_to_step(current_dir, step_dx, step_dy);
  // Levanta solo la coordenada Z; X/Y quedan donde está.
  set_site_one(sl, site_now[sl][0], site_now[sl][1], site_rest[sl][2] + STEP_H);
}

static void start_move() {
  uint8_t sl = SWING_ORDER[swing_idx];
  // Avanza a la posición delantera (rest + medio paso) a altura levantada.
  set_site_one(sl, site_rest[sl][0] + step_dx * 0.5f,
               site_rest[sl][1] + step_dy * 0.5f, site_rest[sl][2] + STEP_H);
}

static void start_lower() {
  uint8_t sl = SWING_ORDER[swing_idx];
  // Baja al suelo en la posición delantera.
  set_site_one(sl, site_rest[sl][0] + step_dx * 0.5f,
               site_rest[sl][1] + step_dy * 0.5f, site_rest[sl][2]);
}

// ── API pública ────────────────────────────────────────────────────────

void gait_init() {
  // Reutilizamos el snap para la pose de arranque
  snap_to_left_y();
}

void gait_set_dir(GaitDir dir) {
  if (dir == current_dir)
    return;

  _next_dir = dir;

  if (dir == GAIT_STOP) {
    return;
  }

  // Si estamos en reposo y piden arrancar (adelante o atrás)
  if (!_is_moving && (dir == GAIT_FORWARD || dir == GAIT_BACKWARD)) {
    current_dir = dir;
    _is_moving = true;

    if (strcmp(_stable_pose, "RIGHT_Y") == 0) {
      _state_idx = 7;
      snap_to_right_y();
    } else {
      _state_idx = 0;
      _stable_pose = "LEFT_Y";
      snap_to_left_y();
    }

    _update_state_machine();
    if (dir == GAIT_FORWARD)
      Serial.println("\n>>> INICIANDO MARCHA CONTINUA ADELANTE <<<");
    else
      Serial.println("\n>>> INICIANDO MARCHA CONTINUA ATRAS <<<");
  }
}

void gait_update(uint32_t now_ms) {
  if (now_ms - _last_tick_ms < TICK_INTERVAL_MS)
    return;
  _last_tick_ms = now_ms;

  if (!_is_moving)
    return;

  float speed = (_state_idx == 3 || _state_idx == 10) ? BODY_SPEED : LEG_SPEED;
  bool done = tick_toward_expect(speed);
  apply_ik_all();

  if (done) {
    // --- CHECKPOINTS DE ESTABILIDAD ---
    if (_state_idx == 6) {
      _stable_pose = "RIGHT_Y";
      if (_next_dir == GAIT_STOP) {
        current_dir = GAIT_STOP;
        _is_moving = false;
        Serial.println(">>> STOP SUAVE (Robot asentado en Right-Y) <<<");
        return;
      } else {
        current_dir =
            _next_dir; // <-- Permite cambio dinámico de dirección sin frenar
      }
    } else if (_state_idx == 13) {
      _stable_pose = "LEFT_Y";
      if (_next_dir == GAIT_STOP) {
        current_dir = GAIT_STOP;
        _is_moving = false;
        Serial.println(">>> STOP SUAVE (Robot asentado en Left-Y) <<<");
        return;
      } else {
        current_dir =
            _next_dir; // <-- Permite cambio dinámico de dirección sin frenar
      }
    }

    // --- TRANSICIÓN AL SIGUIENTE ESTADO ---
    _state_idx++;
    if (_state_idx > 13) {
      _state_idx = 0;
    }

    _update_state_machine();
  }
}

void gait_get_foot_pos(int leg, float out[3]) {
  memcpy(out, site_now[leg], sizeof(float[3]));
}
