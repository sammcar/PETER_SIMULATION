#include "gait.h"
#include "kinematics.h"
#include "config.h"
#include "imu.h"
#include <string.h>
#include <math.h>

// Orden diagonal-wave: FL → RR → FR → RL
static const uint8_t SWING_ORDER[4] = { LEG_FL, LEG_RR, LEG_FR, LEG_RL };

// ── Fases de la máquina de estados ────────────────────────────────────
typedef enum {
    PHASE_IDLE = 0,
    PHASE_LIFT,    // sube la pata de balanceo (solo Z)
    PHASE_MOVE,    // avanza la pata de balanceo (X/Y a altura levantada)
    PHASE_LOWER,   // baja la pata de balanceo al suelo
    PHASE_BODY,    // mueve todas las patas a la posición de apoyo posterior
    PHASE_RETURN,  // retorno suave a reposo
} Phase;

// ── Estado interno ─────────────────────────────────────────────────────
static float site_now[4][3];    // posición actual del pie (frame cuerpo, metros)
static float site_expect[4][3]; // posición objetivo del pie
static float site_rest[4][3];   // posición de reposo de cada pata

static Phase   phase       = PHASE_IDLE;
static uint8_t swing_idx   = 0;           // índice 0-3 en SWING_ORDER
static GaitDir current_dir = GAIT_STOP;
static float   step_dx     = 0.0f;
static float   step_dy     = 0.0f;

extern void mover(uint8_t leg, uint8_t joint, float angulo_deg);

// ── Helpers ────────────────────────────────────────────────────────────

static void dir_to_step(GaitDir dir, float &dx, float &dy) {
    float sl = STEP_LEN, sll = STEP_LEN_LAT;
    float d  = sl * 0.707f;
    switch (dir) {
        case GAIT_FORWARD:   dx =  sl;  dy =  0;    return;
        case GAIT_BACKWARD:  dx = -sl;  dy =  0;    return;
        case GAIT_LEFT:      dx =  0;   dy =  sll;  return;
        case GAIT_RIGHT:     dx =  0;   dy = -sll;  return;
        case GAIT_FWD_LEFT:  dx =  d;   dy =  d;    return;
        case GAIT_FWD_RIGHT: dx =  d;   dy = -d;    return;
        case GAIT_BCK_LEFT:  dx = -d;   dy =  d;    return;
        case GAIT_BCK_RIGHT: dx = -d;   dy = -d;    return;
        default:             dx =  0;   dy =  0;    return;
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
            mover(l, COXA,  q[0]);
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

// ── Inicio de cada sub-fase ────────────────────────────────────────────

static void start_lift() {
    uint8_t sl = SWING_ORDER[swing_idx];
    dir_to_step(current_dir, step_dx, step_dy);
    // Levanta solo la coordenada Z; X/Y quedan donde está.
    set_site_one(sl,
        site_now[sl][0],
        site_now[sl][1],
        site_rest[sl][2] + STEP_H);
}

static void start_move() {
    uint8_t sl = SWING_ORDER[swing_idx];
    // Avanza a la posición delantera (rest + medio paso) a altura levantada.
    set_site_one(sl,
        site_rest[sl][0] + step_dx * 0.5f,
        site_rest[sl][1] + step_dy * 0.5f,
        site_rest[sl][2] + STEP_H);
}

static void start_lower() {
    uint8_t sl = SWING_ORDER[swing_idx];
    // Baja al suelo en la posición delantera.
    set_site_one(sl,
        site_rest[sl][0] + step_dx * 0.5f,
        site_rest[sl][1] + step_dy * 0.5f,
        site_rest[sl][2]);
}

// ── API pública ────────────────────────────────────────────────────────

void gait_init() {
    for (uint8_t l = 0; l < 4; l++) {
        leg_rest_pos(l, site_rest[l]);
        memcpy(site_now[l],    site_rest[l], sizeof(float[3]));
        memcpy(site_expect[l], site_rest[l], sizeof(float[3]));
    }
    apply_ik_all();
    phase = PHASE_IDLE;
}

void gait_set_dir(GaitDir dir) {
    if (dir == current_dir) return;
    current_dir = dir;

    if (dir == GAIT_STOP) {
        if (phase != PHASE_IDLE) {
            phase = PHASE_RETURN;
            expect_all_rest();
        }
        return;
    }

    // Arrancar desde reposo o cancelar un retorno en curso.
    if (phase == PHASE_IDLE || phase == PHASE_RETURN) {
        swing_idx = 0;
        freeze_stance_legs();
        phase = PHASE_LIFT;
        start_lift();
    }
    // Si ya está en marcha, la nueva dirección toma efecto en el próximo start_lift().
}

void gait_update(uint32_t now_ms) {
    (void)now_ms;

    // Parada de seguridad por IMU.
    if (USE_IMU && imu_ok() && phase != PHASE_IDLE && phase != PHASE_RETURN) {
        if (fabsf(imu_get_pitch()) > IMU_MAX_TILT || fabsf(imu_get_roll()) > IMU_MAX_TILT) {
            current_dir = GAIT_STOP;
            phase       = PHASE_RETURN;
            expect_all_rest();
        }
    }

    switch (phase) {

        case PHASE_IDLE:
            return;

        case PHASE_RETURN:
            if (tick_toward_expect(BODY_SPEED)) phase = PHASE_IDLE;
            apply_ik_all();
            return;

        case PHASE_LIFT:
            if (tick_toward_expect(LEG_SPEED)) {
                phase = PHASE_MOVE;
                start_move();
            }
            break;

        case PHASE_MOVE:
            if (tick_toward_expect(LEG_SPEED)) {
                phase = PHASE_LOWER;
                start_lower();
            }
            break;

        case PHASE_LOWER:
            if (tick_toward_expect(LEG_SPEED)) {
                phase = PHASE_BODY;
                expect_all_stride_back(step_dx, step_dy);
            }
            break;

        case PHASE_BODY:
            if (tick_toward_expect(BODY_SPEED)) {
                if (current_dir == GAIT_STOP) {
                    phase = PHASE_RETURN;
                    expect_all_rest();
                } else {
                    swing_idx = (swing_idx + 1) % 4;
                    phase     = PHASE_LIFT;
                    start_lift();
                }
            }
            break;
    }

    apply_ik_all();
}

void gait_get_foot_pos(int leg, float out[3]) {
    memcpy(out, site_now[leg], sizeof(float[3]));
}
