#include "kinematics.h"
#include "config.h"
#include <math.h>

// Offsets del centro del cuerpo al pivote de la coxa (mismo que Python _build_legs)
static const float LEG_OFFSET[4][3] = {
    { BODY_LEN/2,  BODY_WID/2, 0.0f },   // FL
    { BODY_LEN/2, -BODY_WID/2, 0.0f },   // FR
    {-BODY_LEN/2,  BODY_WID/2, 0.0f },   // RL
    {-BODY_LEN/2, -BODY_WID/2, 0.0f },   // RR
};

// Ángulo de montaje de la coxa respecto al eje X del cuerpo (radianes)
// Mismo que coxa_angle_body en Python SpiderQuadruped._build_legs
static const float COXA_ANGLE[4] = {
    M_PI * 0.25f,    // FL  45°
    M_PI *-0.25f,    // FR -45°
    M_PI * 0.75f,    // RL 135°
    M_PI * 1.25f,    // RR 225°
};

// ── Cinemática inversa ─────────────────────────────────────────────────
// Puerto directo del método SpiderLeg::ik() en robot_kinematics.py
bool leg_ik(int leg, float fx, float fy, float fz, float q_out[3]) {
    float a = COXA_ANGLE[leg];
    float c = cosf(a), s = sinf(a);

    // Transformar al frame de la pata: R_body2leg * (foot_body - body_offset)
    float dx = fx - LEG_OFFSET[leg][0];
    float dy = fy - LEG_OFFSET[leg][1];
    float dz = fz - LEG_OFFSET[leg][2];
    float px =  c * dx + s * dy;
    float py = -s * dx + c * dy;
    float pz = dz;

    // Coxa (yaw en el plano XY del frame de la pata)
    float theta_coxa  = atan2f(py, px);
    float r_horiz     = sqrtf(px*px + py*py);
    float reach       = r_horiz - L_COXA;
    float D           = sqrtf(reach*reach + pz*pz);

    float L1 = L_FEMUR, L2 = L_TIBIA;
    if (D > (L1 + L2) || D < fabsf(L1 - L2) || D < 1e-6f)
        return false;

    // Tibia (ley del coseno, rodilla hacia abajo)
    float cos_t      = fmaxf(-1.0f, fminf(1.0f,
                            (D*D - L1*L1 - L2*L2) / (2.0f * L1 * L2)));
    float theta_tib  = -acosf(cos_t);

    // Fémur
    float alpha      = atan2f(pz, reach);
    float beta       = atan2f(L2 * sinf(theta_tib),
                               L1 + L2 * cosf(theta_tib));
    float theta_fem  = alpha - beta;

    q_out[0] = theta_coxa * (180.0f / M_PI);
    q_out[1] = theta_fem  * (180.0f / M_PI);
    q_out[2] = theta_tib  * (180.0f / M_PI);
    return true;
}

