#pragma once
#include <stdint.h>

// Calcula la cinemática inversa para una pata.
// leg  : 0=FL  1=FR  2=RL  3=RR
// fx,fy,fz : posición objetivo del pie en frame del cuerpo (metros)
// q_out[3] : ángulos resultantes [coxa, fémur, tibia] en GRADOS
// Retorna true si el punto es alcanzable; false si está fuera de rango.
// elbow_up=false → rodilla hacia abajo (por defecto, todos los modos salvo OMNI)
// elbow_up=true  → rodilla hacia arriba (modo OMNI: fémur abajo, tibia horizontal)
bool leg_ik(int leg, float fx, float fy, float fz, float q_out[3], bool elbow_up = false);
