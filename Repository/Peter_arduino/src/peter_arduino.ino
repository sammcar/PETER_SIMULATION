/*
 * peter_arduino.ino — Controlador principal del robot cuadrúpedo Peter
 * ════════════════════════════════════════════════════════════════════════
 * Marcha diagonal-wave con giros in-place.
 *
 * Teclas de movimiento (un solo carácter por serial):
 *
 *   i = Adelante        , = Atrás
 *   u = Giro izquierda  o = Giro derecha
 *   k = Stop (retorno suave a reposo)
 *
 * Comando de articulación directa (detiene la marcha):
 *   leg,joint,angulo    ej: 0,1,30  → pata FL, fémur, 30°
 */

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "gait.h"
#include "imu.h"
#include "kinematics.h"
#include "motors.h"

// ── Calibración de servos ──────────────────────────────────────────────
// Fórmula:  ang_ef = sign * ang - offset  → mapea a [min_us, max_us]
// [pata][articulacion]   canal  min_us  max_us  sign  offset
const ServoCalib CALIB[4][3] = {
    {
        // LEG_FL – Frontal Izquierda
        {12, 580, 2340, -1, -90}, // Coxa
        {13, 593, 2323, 1, -90},  // Fémur
        {14, 675, 2335, -1, -30}, // Tibia
    },
    {
        // LEG_FR – Frontal Derecha
        {0, 663, 2413, -1, -90}, // Coxa
        {1, 595, 2295, -1, -90}, // Fémur
        {2, 665, 2285, 1, -150}, // Tibia
    },
    {
        // LEG_RL – Trasera Izquierda
        {8, 593, 2263, -1, -90},  // Coxa
        {9, 580, 2340, -1, -90},  // Fémur
        {10, 538, 2308, 1, -150}, // Tibia
    },
    {
        // LEG_RR – Trasera Derecha
        {4, 720, 2360, -1, -90}, // Coxa
        {5, 655, 2355, 1, -90},  // Fémur
        {6, 645, 2265, -1, -30}, // Tibia
    },
};

Adafruit_PWMServoDriver pca(PCA_ADDR, Wire);

// ── Control de servo individual ────────────────────────────────────────

static int us_a_ticks(int us) { return (int)(us * 4096.0f / 20000.0f); }

// Mueve una articulación al ángulo indicado en grados [-90, +90].
// Llamada desde gait.cpp vía extern.
void mover(uint8_t leg, uint8_t joint, float angulo) {
  const ServoCalib &c = CALIB[leg][joint];
  angulo = constrain(angulo, -90.0f, 90.0f);
  float ang_ef = c.sign * angulo - c.offset;
  ang_ef = constrain(ang_ef, 0.0f, 180.0f);
  int us = (int)(c.min_us + (ang_ef / 180.0f) * (c.max_us - c.min_us));
  pca.setPWM(c.channel, 0, us_a_ticks(us));
}

// ── Parser serial ──────────────────────────────────────────────────────

static const char *leg_name(int l) {
  switch (l) {
  case LEG_FL:
    return "FL";
  case LEG_FR:
    return "FR";
  case LEG_RL:
    return "RL";
  case LEG_RR:
    return "RR";
  default:
    return "??";
  }
}

static const char *dir_name(GaitDir d) {
  switch (d) {
  case GAIT_FORWARD:
    return "ADELANTE";
  case GAIT_BACKWARD:
    return "ATRAS";
  case GAIT_TURN_LEFT:
    return "GIRO IZQUIERDA";
  case GAIT_TURN_RIGHT:
    return "GIRO DERECHA";
  default:
    return "STOP";
  }
}

// Interpreta un carácter de teleop y ajusta la dirección de marcha.

static float _speed = 1.0f; // Multiplicador linear
static float _turn = 1.0f;  // Multiplicador angular

void omni_drive(float vx, float vy, float wz) {
  int m1 = (int)((1.15f * vx + wz + vy) * MOTOR_BASE_SPEED * _speed);
  int m2 = (int)((-1.15f * vx + wz + vy) * MOTOR_BASE_SPEED * _speed);
  int m3 = (int)((1.15f * vx + wz - vy) * MOTOR_BASE_SPEED * _speed);
  int m4 = (int)((-1.15f * vx + wz - vy) * MOTOR_BASE_SPEED * _speed);
  motor_set(constrain(m1, -255, 255), constrain(m2, -255, 255),
            constrain(m3, -255, 255), constrain(m4, -255, 255));
}

static void handle_teleop(char key) {
  // 1. Teclas globales: Control de velocidad y Cambio de modos
  switch (key) {
  case 'z':
    gait_set_mode(MODE_VEHICLE);
    Serial.println("MODO: VEHICLE");
    return;
  case 'x':
    gait_set_mode(MODE_OMNI);
    Serial.println("MODO: OMNI");
    return;
  case 'c':
    gait_set_mode(MODE_SPIDER);
    Serial.println("MODO: SPIDER");
    return;

  case 'q':
    _speed *= 1.1f;
    _turn *= 1.1f;
    Serial.println("+Speed Global");
    return;
  case 'a':
    _speed *= 0.9f;
    _turn *= 0.9f;
    Serial.println("-Speed Global");
    return;
  case 'w':
    _speed *= 1.1f;
    Serial.println("+Linear Speed");
    return;
  case 's':
    _speed *= 0.9f;
    Serial.println("-Linear Speed");
    return;
  case 'e':
    _turn *= 1.1f;
    Serial.println("+Turn Speed");
    return;
  case 'd':
    _turn *= 0.9f;
    Serial.println("-Turn Speed");
    return;
  }

  // 2. Control de movimiento dependiendo del modo actual
  RobotMode mode = gait_get_mode();

  if (mode == MODE_SPIDER) {
    switch (key) {
    case 'i':
      gait_set_dir(GAIT_FORWARD);
      break;
    case ',':
      gait_set_dir(GAIT_BACKWARD);
      break;
    case 'u':
    case 'm':
      gait_set_dir(GAIT_TURN_LEFT);
      break;
    case 'o':
    case '.':
      gait_set_dir(GAIT_TURN_RIGHT);
      break;
    case 'k':
      gait_set_dir(GAIT_STOP);
      break;
    }
  } else if (mode == MODE_VEHICLE) {
    int V = (int)(MOTOR_BASE_SPEED * _speed);
    int Vt = (int)(MOTOR_BASE_SPEED * _turn);
    switch (key) {
    case 'i':
      motor_set(+V, -V, +V, -V);
      break;
    case ',':
      motor_set(-V, +V, -V, +V);
      break;
    case 'u':
    case 'm':
      motor_set(-Vt, -Vt, +Vt, +Vt);
      break;
    case 'o':
    case '.':
      motor_set(+Vt, +Vt, -Vt, -Vt);
      break;
    case 'k':
      motors_stop();
      break;
    }
  } else if (mode == MODE_OMNI) {
    switch (key) {
    case 'i':
      omni_drive(+1.0f, 0.0f, 0.0f);
      break;
    case ',':
      omni_drive(-1.0f, 0.0f, 0.0f);
      break;
    case 'j':
      omni_drive(0.0f, +1.0f, 0.0f);
      break;
    case 'l':
      omni_drive(0.0f, -1.0f, 0.0f);
      break;
    case 'u':
    case 'm':
      omni_drive(0.0f, 0.0f, +1.0f);
      break;
    case 'o':
    case '.':
      omni_drive(0.0f, 0.0f, -1.0f);
      break;
    case 'k':
      motors_stop();
      break;
    }
  }
}

// Interpreta "leg,joint,angulo" — mismo formato que joint_mapper.py
static void handle_joint_cmd(const String &line) {
  int sep1 = line.indexOf(',');
  int sep2 = (sep1 >= 0) ? line.indexOf(',', sep1 + 1) : -1;
  if (sep1 < 0 || sep2 < 0) {
    Serial.println("ERROR formato: usa leg,joint,angulo");
    return;
  }

  int leg = line.substring(0, sep1).toInt();
  int joint = line.substring(sep1 + 1, sep2).toInt();
  float ang = line.substring(sep2 + 1).toFloat();

  if (leg < 0 || leg > 3) {
    Serial.println("ERROR pata fuera de rango (0-3)");
    return;
  }
  if (joint < 0 || joint > 2) {
    Serial.println("ERROR articulacion fuera de rango (0-2)");
    return;
  }

  // Detiene la marcha antes del comando directo para evitar conflicto
  gait_set_dir(GAIT_STOP);
  mover((uint8_t)leg, (uint8_t)joint, ang);

  Serial.print("CMD=");
  Serial.print(line);
  Serial.print(" pata=");
  Serial.print(leg_name(leg));
  Serial.print(" art=");
  Serial.print(joint);
  Serial.print(" canal=");
  Serial.print(CALIB[leg][joint].channel);
  Serial.print(" ang=");
  Serial.println(ang, 1);
}

void procesar_serial() {
  static String buffer_serial = "";

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      buffer_serial.trim();

      if (buffer_serial.length() > 0) {
        if (buffer_serial.length() == 1) {
          handle_teleop(buffer_serial[0]);
        } else if (buffer_serial.indexOf(',') >= 0) {
          handle_joint_cmd(buffer_serial);
        } else {
          Serial.print("ERROR desconocido: ");
          Serial.println(buffer_serial);
        }
        buffer_serial = "";
      }
    } else {
      buffer_serial += c;
    }
  }
}

// ── Setup y Loop ───────────────────────────────────────────────────────

void setup() {
  delay(5000);
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(PWM_FREQ);
  delay(100);

  if (USE_IMU) {
    imu_init();
    if (imu_ok()) {
      Serial.println("IMU MPU-6050 detectado OK");
    } else {
      Serial.println(
          "AVISO: MPU-6050 no detectado (verifica conexion I2C en 0x68)");
    }
  } else {
    Serial.println("IMU deshabilitado (USE_IMU=0 en config.h)");
  }

  gait_init(); // IK hacia posición de reposo + envío inicial a servos

  // --- PRUEBA ETAPA 0 ---
  motors_init();
  // motor_set(150, -150, 150, -150);
  //  ----------------------

  Serial.println("Running Peter Controller by Sam :D");
  Serial.println("LISTO");
  Serial.println("  i=Adelante  ,=Atras  u=Giro-Izq  o=Giro-Der  k=Stop");
  Serial.println("  leg,joint,ang  -> comando directo de servo");
}

void loop() {
  uint32_t now = millis();
  if (USE_IMU)
    imu_update(now);
  gait_update(now);

  if (now < 500) {
    while (Serial.available())
      Serial.read();
    return;
  }

  procesar_serial();
}
