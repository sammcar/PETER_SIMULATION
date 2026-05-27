#include "gait.h"
#include "kinematics.h"
#include "config.h"
#include "imu.h"
#include <string.h>
#include <math.h>

// Orden de balanceo diagonal-wave crawl: FL → RR → FR → RL
// (mientras una pata balancea, las tres contrarias forman un triángulo estable)
static const uint8_t SWING_ORDER[4] = { LEG_FL, LEG_RR, LEG_FR, LEG_RL };

// ── Estado interno ─────────────────────────────────────────────────────

static float foot_pos[4][3];        // objetivo actual del pie en frame cuerpo
static float foot_rest[4][3];       // posición de reposo en frame cuerpo

// Datos fijados al inicio de cada fase de balanceo
static float swing_start[3];        // posición del pie de balanceo al inicio
static float swing_end[3];          // posición destino del pie de balanceo
static float stance_start[4][3];    // posición de las patas en apoyo al inicio de fase
static float phase_dx, phase_dy;    // vector de paso capturado al inicio de la fase

static GaitDir  current_dir   = GAIT_STOP;
static uint8_t  current_phase = 0;       // índice 0-3 en SWING_ORDER
static uint32_t phase_start_ms = 0;
static bool     active    = false;
static bool     returning = false;
static float    return_start[4][3];      // foot_pos al inicio del retorno a reposo

static float body_shift[2]        = {0.0f, 0.0f};  // desplazamiento actual del cuerpo (x,y)
static float body_shift_start[2]  = {0.0f, 0.0f};  // desplazamiento al inicio de la fase
static float body_shift_tgt[2]    = {0.0f, 0.0f};  // desplazamiento objetivo de la fase
static float return_body_shift[2] = {0.0f, 0.0f};  // desplazamiento al inicio del retorno a reposo

// Implementada en peter_arduino.ino
extern void mover(uint8_t leg, uint8_t joint, float angulo_deg);

static inline float smoothstep(float t) { return t * t * (3.0f - 2.0f * t); }

// ── Helpers ────────────────────────────────────────────────────────────

// Vector de paso (dx, dy) en metros según la dirección pedida.
// Para giros: patas izquierdas van hacia atrás, derechas hacia adelante (skid-steer).
static void dir_to_step(GaitDir dir, uint8_t leg, float& dx, float& dy) {
    float sl  = STEP_LEN;
    float sll = STEP_LEN_LAT;
    float d   = sl * 0.707f;   // diagonal normalizada

    switch (dir) {
        case GAIT_FORWARD:   dx =  sl;  dy =  0;   return;
        case GAIT_BACKWARD:  dx = -sl;  dy =  0;   return;
        case GAIT_LEFT:      dx =  0;   dy =  sll; return;
        case GAIT_RIGHT:     dx =  0;   dy = -sll; return;
        case GAIT_FWD_LEFT:  dx =  d;   dy =  d;   return;
        case GAIT_FWD_RIGHT: dx =  d;   dy = -d;   return;
        case GAIT_BCK_LEFT:  dx = -d;   dy =  d;   return;
        case GAIT_BCK_RIGHT: dx = -d;   dy = -d;   return;
        default:             dx =  0;   dy =  0;   return;
    }
}

// Envía los ángulos IK actuales de todas las patas a los servos.
// Se resta body_shift para que el cuerpo se desplace hacia el polígono de soporte.
static void apply_ik_all() {
    for (uint8_t l = 0; l < 4; l++) {
        float q[3];
        float fx = foot_pos[l][0] - body_shift[0];
        float fy = foot_pos[l][1] - body_shift[1];
        if (leg_ik(l, fx, fy, foot_pos[l][2], q)) {
            mover(l, COXA,  q[0]);
            mover(l, FEMUR, q[1]);
            mover(l, TIBIA, q[2] - TIBIA_BIAS_DEG);
        }
    }
}

// Prepara los datos de la fase de balanceo del ciclo actual.
static void start_phase() {
    uint8_t sl = SWING_ORDER[current_phase];

    // Vector de paso para esta fase (congelado hasta la siguiente fase)
    dir_to_step(current_dir, sl, phase_dx, phase_dy);

    // Guardar posiciones de inicio
    for (uint8_t l = 0; l < 4; l++)
        memcpy(stance_start[l], foot_pos[l], sizeof(float[3]));

    // La pata de balanceo parte de su posición actual y aterriza medio paso
    // por delante de su posición de reposo
    memcpy(swing_start, foot_pos[sl], sizeof(float[3]));
    swing_end[0] = foot_rest[sl][0] + phase_dx * 0.5f;
    swing_end[1] = foot_rest[sl][1] + phase_dy * 0.5f;
    swing_end[2] = foot_rest[sl][2];   // sin elevación al aterrizar

    // Centroide del triángulo de soporte → objetivo del desplazamiento del cuerpo
    float cx = 0.0f, cy = 0.0f;
    for (uint8_t l = 0; l < 4; l++) {
        if (l != sl) { cx += foot_pos[l][0]; cy += foot_pos[l][1]; }
    }
    cx /= 3.0f;  cy /= 3.0f;
    body_shift_start[0] = body_shift[0];
    body_shift_start[1] = body_shift[1];
    bool lat = fabsf(phase_dy) > fabsf(phase_dx);
    body_shift_tgt[0]   = cx * (lat ? BODY_SHIFT_LAT_X : BODY_SHIFT_FWD_X);
    body_shift_tgt[1]   = cy * (lat ? BODY_SHIFT_LAT_Y : BODY_SHIFT_FWD_Y);

    // Corrección adicional por inclinación del IMU.
    // Se suma al desplazamiento del centroide para compensar superficies inclinadas.
    // Si la corrección está invertida, cambiar el signo de IMU_GAIN_X / IMU_GAIN_Y en config.h.
    if (USE_IMU && imu_ok()) {
        body_shift_tgt[0] += imu_get_pitch() * IMU_GAIN_X;
        body_shift_tgt[1] += imu_get_roll()  * IMU_GAIN_Y;
    }
}

// ── API pública ────────────────────────────────────────────────────────

void gait_init() {
    for (uint8_t l = 0; l < 4; l++) {
        leg_rest_pos(l, foot_rest[l]);
        memcpy(foot_pos[l], foot_rest[l], sizeof(float[3]));
    }
    apply_ik_all();
}

void gait_set_dir(GaitDir dir) {
    if (dir == GAIT_STOP) {
        current_dir = GAIT_STOP;
        if (active || returning) {
            active    = false;
            returning = true;
            phase_start_ms = 0;
            for (uint8_t l = 0; l < 4; l++)
                memcpy(return_start[l], foot_pos[l], sizeof(float[3]));
            return_body_shift[0] = body_shift[0];
            return_body_shift[1] = body_shift[1];
        }
        return;
    }

    current_dir = dir;
    if (!active) {
        returning      = false;   // cancela retorno si estaba en curso
        active         = true;
        current_phase  = 0;
        phase_start_ms = 0;       // se fijará en el primer gait_update()
        start_phase();
    }
    // Si ya está activo: la nueva dirección toma efecto en la próxima fase
}

void gait_update(uint32_t now_ms) {

    // ── Retorno suave a reposo ─────────────────────────────────────────
    if (returning) {
        if (phase_start_ms == 0) phase_start_ms = now_ms;
        float t = (float)(now_ms - phase_start_ms) / RETURN_MS;
        if (t >= 1.0f) {
            returning      = false;
            phase_start_ms = 0;
            for (uint8_t l = 0; l < 4; l++)
                memcpy(foot_pos[l], foot_rest[l], sizeof(float[3]));
            body_shift[0] = 0.0f;
            body_shift[1] = 0.0f;
        } else if (t <= RETURN_LAND_RATIO) {
            // Fase 1: solo bajar las patas al suelo (solo Z).
            // Garantiza que ninguna pata quede en el aire antes de reposicionar.
            float tl = smoothstep(t / RETURN_LAND_RATIO);
            for (uint8_t l = 0; l < 4; l++) {
                foot_pos[l][0] = return_start[l][0];
                foot_pos[l][1] = return_start[l][1];
                foot_pos[l][2] = return_start[l][2] + (foot_rest[l][2] - return_start[l][2]) * tl;
            }
            body_shift[0] = return_body_shift[0];
            body_shift[1] = return_body_shift[1];
        } else {
            // Fase 2: con patas en tierra, reposicionar X/Y y centrar el cuerpo.
            float tr = smoothstep((t - RETURN_LAND_RATIO) / (1.0f - RETURN_LAND_RATIO));
            for (uint8_t l = 0; l < 4; l++) {
                foot_pos[l][0] = return_start[l][0] + (foot_rest[l][0] - return_start[l][0]) * tr;
                foot_pos[l][1] = return_start[l][1] + (foot_rest[l][1] - return_start[l][1]) * tr;
                foot_pos[l][2] = foot_rest[l][2];
            }
            body_shift[0] = return_body_shift[0] * (1.0f - tr);
            body_shift[1] = return_body_shift[1] * (1.0f - tr);
        }
        apply_ik_all();
        return;
    }

    // Detención de seguridad: si la inclinación supera el límite, parar la marcha.
    if (USE_IMU && active && imu_ok()) {
        if (fabsf(imu_get_pitch()) > IMU_MAX_TILT || fabsf(imu_get_roll()) > IMU_MAX_TILT) {
            gait_set_dir(GAIT_STOP);
        }
    }

    if (!active) return;

    // ── Marcha activa ──────────────────────────────────────────────────
    if (phase_start_ms == 0) phase_start_ms = now_ms;

    uint32_t elapsed = now_ms - phase_start_ms;
    float t = (elapsed < (uint32_t)STEP_MS)
              ? (float)elapsed / STEP_MS
              : 1.0f;

    uint8_t swing_leg = SWING_ORDER[current_phase];

    if (t <= BODY_PREP_RATIO) {
        // ── Sub-fase 1: preparación ────────────────────────────────────
        // El cuerpo se desplaza hacia el centroide del triángulo de soporte
        // con las 4 patas todavía en el suelo.
        float tp = smoothstep(t / BODY_PREP_RATIO);   // 0→1, suavizado para cero velocidad al inicio y al fin
        body_shift[0] = body_shift_start[0] + (body_shift_tgt[0] - body_shift_start[0]) * tp;
        body_shift[1] = body_shift_start[1] + (body_shift_tgt[1] - body_shift_start[1]) * tp;
        for (uint8_t l = 0; l < 4; l++) {
            foot_pos[l][0] = stance_start[l][0];
            foot_pos[l][1] = stance_start[l][1];
        }
    } else {
        // ── Sub-fase 2: balanceo ───────────────────────────────────────
        // Cuerpo fijo en el centroide; la pata sube y avanza.
        body_shift[0] = body_shift_tgt[0];
        body_shift[1] = body_shift_tgt[1];
        float ts = (t - BODY_PREP_RATIO) / (1.0f - BODY_PREP_RATIO);   // 0→1 dentro del balanceo

        // 16·t²·(1-t)²: derivada nula en t=0 y t=1, despegue y aterrizaje sin golpe
        float lift  = STEP_H * 16.0f * ts * ts * (1.0f - ts) * (1.0f - ts);
        float ts_sm = smoothstep(ts);   // posicionamiento horizontal con arranque/parada suave
        foot_pos[swing_leg][0] = swing_start[0] + (swing_end[0] - swing_start[0]) * ts_sm;
        foot_pos[swing_leg][1] = swing_start[1] + (swing_end[1] - swing_start[1]) * ts_sm;
        foot_pos[swing_leg][2] = swing_end[2] + lift;

        for (uint8_t l = 0; l < 4; l++) {
            if (l == swing_leg) continue;
            foot_pos[l][0] = stance_start[l][0] - phase_dx * 0.25f * ts_sm;
            foot_pos[l][1] = stance_start[l][1] - phase_dy * 0.25f * ts_sm;
        }
    }

    apply_ik_all();

    // ── Fin de fase ────────────────────────────────────────────────────
    if (elapsed >= (uint32_t)STEP_MS) {
        // Fijar la pata de balanceo en su posición final
        memcpy(foot_pos[swing_leg], swing_end, sizeof(float[3]));

        current_phase  = (current_phase + 1) % 4;
        phase_start_ms = now_ms;

        if (current_dir == GAIT_STOP) {
            active    = false;
            returning = true;
            for (uint8_t l = 0; l < 4; l++)
                memcpy(return_start[l], foot_pos[l], sizeof(float[3]));
            return_body_shift[0] = body_shift[0];
            return_body_shift[1] = body_shift[1];
            phase_start_ms = 0;
        } else {
            start_phase();
        }
    }
}

void gait_get_foot_pos(int leg, float out[3]) {
    memcpy(out, foot_pos[leg], sizeof(float[3]));
}
