#include "imu.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

// ── Registros MPU-6050 ─────────────────────────────────────────────────
#define MPU_ADDR   0x68
#define REG_CONFIG 0x1A   // DLPF (filtro pasa-bajos digital)
#define REG_GYRO   0x1B   // configuración del giróscopo
#define REG_ACCEL  0x1C   // configuración del acelerómetro
#define REG_DATA   0x3B   // inicio del bloque de datos (accel + temp + gyro)
#define REG_PWR    0x6B   // gestión de energía
#define REG_WHOAMI 0x75   // identificador del chip (debe devolver 0x68)

// Factores de escala según configuración
#define GYRO_LSB   131.0f     // LSB por °/s  a ±250°/s
#define ACCEL_LSB  16384.0f   // LSB por g    a ±2g
#define RAD2DEG    (180.0f / (float)M_PI)

// ── Estado interno ─────────────────────────────────────────────────────
static bool     _ok         = false;
static float    _pitch      = 0.0f;
static float    _roll       = 0.0f;
static float    _pitch_bias = 0.0f;
static float    _roll_bias  = 0.0f;
static uint32_t _last_ms    = 0;

// ── I2C helpers ────────────────────────────────────────────────────────

static void write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static bool read_sensors(int16_t& ax, int16_t& ay, int16_t& az,
                         int16_t& gx, int16_t& gy, int16_t& gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) < 14) return false;

    ax = (int16_t)(Wire.read() << 8 | Wire.read());
    ay = (int16_t)(Wire.read() << 8 | Wire.read());
    az = (int16_t)(Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read();   // temperatura — descartar
    gx = (int16_t)(Wire.read() << 8 | Wire.read());
    gy = (int16_t)(Wire.read() << 8 | Wire.read());
    gz = (int16_t)(Wire.read() << 8 | Wire.read());
    return true;
}

// ── API pública ────────────────────────────────────────────────────────

void imu_init() {
    write_reg(REG_PWR,   0x00);   // salir del modo sleep
    write_reg(REG_CONFIG, 0x03);  // DLPF 44 Hz — atenúa vibraciones mecánicas
    write_reg(REG_GYRO,  0x00);   // giróscopo ±250°/s
    write_reg(REG_ACCEL, 0x00);   // acelerómetro ±2g
    delay(150);

    // Verificar que el chip responde correctamente
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_WHOAMI);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    uint8_t who = Wire.available() ? Wire.read() : 0x00;
    _ok = (who == 0x68);
    if (!_ok) return;

    // Inicializar el filtro complementario con los ángulos del acelerómetro.
    // 20 lecturas × 10 ms = 200 ms de calentamiento para estabilizar el filtro
    // antes de que empiece la marcha.
    for (int i = 0; i < 20; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (read_sensors(ax, ay, az, gx, gy, gz)) {
            float fax = ax / ACCEL_LSB;
            float fay = ay / ACCEL_LSB;
            float faz = az / ACCEL_LSB;
            _pitch = atan2f(fax, sqrtf(fay*fay + faz*faz)) * RAD2DEG;
            _roll  = atan2f(-fay, faz) * RAD2DEG;
        }
        delay(10);
    }
    _last_ms = millis();
}

void imu_update(uint32_t now_ms) {
    if (!_ok) return;

    int16_t ax, ay, az, gx, gy, gz;
    if (!read_sensors(ax, ay, az, gx, gy, gz)) return;

    float dt = (now_ms - _last_ms) * 0.001f;
    _last_ms = now_ms;
    if (dt <= 0.0f || dt > 0.2f) return;   // descarta saltos de tiempo anómalos

    // Ángulos desde el acelerómetro (lentos pero sin deriva)
    float fax = ax / ACCEL_LSB;
    float fay = ay / ACCEL_LSB;
    float faz = az / ACCEL_LSB;
    float accel_pitch = atan2f(fax, sqrtf(fay*fay + faz*faz)) * RAD2DEG;
    float accel_roll  = atan2f(-fay, faz) * RAD2DEG;

    // Tasas angulares del giróscopo (rápidas pero con deriva a largo plazo)
    float gx_dps = gx / GYRO_LSB;
    float gy_dps = gy / GYRO_LSB;

    // Filtro complementario: combina giróscopo (alta frecuencia) y acelerómetro (baja frecuencia)
    _pitch = IMU_ALPHA * (_pitch + gx_dps * dt) + (1.0f - IMU_ALPHA) * accel_pitch;
    _roll  = IMU_ALPHA * (_roll  + gy_dps * dt) + (1.0f - IMU_ALPHA) * accel_roll;
}

void imu_calibrate() {
    _pitch_bias = _pitch;
    _roll_bias  = _roll;
}

float imu_get_pitch() { return _pitch - _pitch_bias; }
float imu_get_roll()  { return _roll  - _roll_bias;  }
bool  imu_ok()        { return _ok; }
