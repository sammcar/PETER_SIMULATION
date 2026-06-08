/*
 * peter_arduino.ino — Controlador principal del robot cuadrúpedo Peter
 * ════════════════════════════════════════════════════════════════════════
 * Marcha crawl diagonal-wave con compensación de centro de masa e IMU.
 *
 * Teclas de movimiento (un solo carácter por serial):
 *
 *      u    i    o
 *      j    k    l
 *      m    ,    .
 *
 *   i = Adelante        , = Atrás          k = Stop (retorno a reposo)
 *   j = Strafe izq      l = Strafe der
 *   u = Adelante+izq    o = Adelante+der
 *   m = Atrás+izq       . = Atrás+der
 *   c = Calibrar IMU
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
  case GAIT_LEFT:
    return "IZQUIERDA";
  case GAIT_RIGHT:
    return "DERECHA";
  case GAIT_FWD_LEFT:
    return "ADELANTE+IZQ";
  case GAIT_FWD_RIGHT:
    return "ADELANTE+DER";
  case GAIT_BCK_LEFT:
    return "ATRAS+IZQ";
  case GAIT_BCK_RIGHT:
    return "ATRAS+DER";
  default:
    return "STOP";
  }
}

// Interpreta un carácter de teleop y ajusta la dirección de marcha.
static void handle_teleop(char key) {
  GaitDir dir;
  switch (key) {
  case 'n':
    gait_step_once();
    return;
  case 'p':
    print_gait_state();
    return;
  case 'y':
    snap_to_left_y();
    return;
  case 'r':
    snap_to_right_y();
    return;
  case 'i':
    dir = GAIT_FORWARD;
    break;
  case ',':
    dir = GAIT_BACKWARD;
    break;
  case 'j':
    dir = GAIT_LEFT;
    break;
  case 'l':
    dir = GAIT_RIGHT;
    break;
  case 'k':
    dir = GAIT_STOP;
    break;
  case 'c':
    Serial.print("IMU calibrando (antes: pitch=");
    Serial.print(imu_get_pitch(), 1);
    Serial.print("  roll=");
    Serial.print(imu_get_roll(), 1);
    Serial.println(")");
    imu_calibrate();
    Serial.println("IMU calibrado — nueva referencia = posicion actual");
    return;
  case 'u':
    dir = GAIT_FWD_LEFT;
    break;
  case 'o':
    dir = GAIT_FWD_RIGHT;
    break;
  case 'm':
    dir = GAIT_BCK_LEFT;
    break;
  case '.':
    dir = GAIT_BCK_RIGHT;
    break;
  default:
    return;
  }
  gait_set_dir(dir);
  Serial.print("CMD=");
  Serial.print(key);
  Serial.print(" DIR=");
  Serial.println(dir_name(dir));
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

  Serial.println("LISTO");
  Serial.println("  i=Adelante  ,=Atras   j=Izq  l=Der  k=Stop");
  Serial.println("  u=Del+Izq   o=Del+Der  m=At+Izq  .=At+Der");
  Serial.println("  c=Calibrar IMU (poner robot nivelado y presionar c)");
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
