#pragma once

#include <cmath>
#include <vector>
#include <chrono>
#include <optional>
#include <stdexcept>

/**
 * @file robot_metrics.hpp
 * @brief Métricas de rendimiento para navegación de robot móvil.
 *
 * Cuatro métricas independientes extraídas del nodo ROS network_publisher.py:
 *
 *  1. ResponseTimeMetric   — Tiempo desde detección de terreno rugoso hasta estabilización.
 *  2. ModeSwitchMetric     — Delay entre comando de cambio de modo y estabilización.
 *  3. RollRmsMetric        — RMS acumulado del ángulo de roll (oscilación lateral).
 *  4. PitchRmsMetric       — RMS acumulado del ángulo de pitch (oscilación frontal).
 *
 * Ninguna clase depende de ROS; usan <chrono> para tiempo de alta resolución.
 */

// ─────────────────────────────────────────────────────────────────────────────
// Estructuras de entrada
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Señales de estabilidad del robot necesarias para determinar si el sistema
 * se ha asentado después de una perturbación.
 *
 * @param accel_std   Desviación estándar de la magnitud de aceleración (m/s²).
 * @param pitch_deg   Ángulo de pitch en grados (valor absoluto).
 * @param roll_deg    Ángulo de roll en grados (0–360, referenciado a 180°).
 *
 * Condición de estabilidad (equivalente al Python):
 *   accel_std < 2.0  &&  pitch_deg < 1.5  &&  roll_deg > 178.0
 */
struct StabilitySignals {
    double accel_std = 0.0;  ///< std dev de aceleración total [m/s²]
    double pitch_deg = 0.0;  ///< pitch absoluto [°]
    double roll_deg  = 0.0;  ///< roll 0-360 referenciado a 180° [°]
};

// ─────────────────────────────────────────────────────────────────────────────
// 1. TIEMPO DE RESPUESTA
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class ResponseTimeMetric
 * @brief Mide cuánto tarda el robot en estabilizarse tras detectar terreno rugoso.
 *
 * Uso típico:
 * @code
 *   ResponseTimeMetric metric;
 *
 *   // Cuando el detector de terreno dispara:
 *   metric.on_rough_terrain_detected();
 *
 *   // En cada ciclo de control, pasar las señales de estabilidad:
 *   metric.update(signals);
 *
 *   // Consultar resultado:
 *   if (metric.is_measured()) {
 *       double t = metric.get_response_time_s();
 *   }
 * @endcode
 */
class ResponseTimeMetric {
public:
    /**
     * @brief Señala que el terreno rugoso acaba de ser detectado.
     *        Inicia el cronómetro y habilita la medición.
     *        Llamar exactamente una vez por evento de terreno.
     */
    void on_rough_terrain_detected();

    /**
     * @brief Actualiza la métrica con las señales actuales del robot.
     *        Debe llamarse en cada ciclo de control mientras !is_measured().
     *
     * @param signals  Señales de estabilidad del ciclo actual.
     */
    void update(const StabilitySignals& signals);

    /**
     * @brief Reinicia la métrica para un nuevo experimento.
     */
    void reset();

    /**
     * @return true si la medición ya fue completada.
     */
    bool is_measured() const { return measured_; }

    /**
     * @return Tiempo de respuesta en segundos.
     * @throws std::logic_error si la medición aún no está lista.
     */
    double get_response_time_s() const;

private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    bool       active_   = false;   ///< true después de on_rough_terrain_detected()
    bool       measured_ = false;
    TimePoint  t_start_;
    double     result_s_ = 0.0;

    static bool is_stable(const StabilitySignals& s);
};

// ─────────────────────────────────────────────────────────────────────────────
// 2. DELAY DE CAMBIO DE MODO
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class ModeSwitchMetric
 * @brief Mide el tiempo entre la emisión de un comando de cambio de modo
 *        y la estabilización efectiva del robot.
 *
 * Uso típico:
 * @code
 *   ModeSwitchMetric metric;
 *
 *   // Al emitir el comando de cambio de modo:
 *   metric.on_mode_command_issued();
 *
 *   // En cada ciclo:
 *   metric.update(signals);
 *
 *   if (metric.is_measured()) {
 *       double t = metric.get_switch_time_s();
 *   }
 * @endcode
 */
class ModeSwitchMetric {
public:
    /**
     * @brief Señala que se acaba de emitir un comando de cambio de modo.
     *        Reinicia el cronómetro interno.
     */
    void on_mode_command_issued();

    /**
     * @brief Actualiza la métrica con las señales actuales del robot.
     *
     * @param signals  Señales de estabilidad del ciclo actual.
     */
    void update(const StabilitySignals& signals);

    /**
     * @brief Reinicia la métrica para un nuevo experimento.
     */
    void reset();

    /** @return true si la medición fue completada. */
    bool is_measured() const { return measured_; }

    /**
     * @return Delay de cambio de modo en segundos.
     * @throws std::logic_error si la medición aún no está lista.
     */
    double get_switch_time_s() const;

private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    bool       active_   = false;
    bool       measured_ = false;
    TimePoint  t_cmd_;
    double     result_s_ = 0.0;

    static bool is_stable(const StabilitySignals& s);
};

// ─────────────────────────────────────────────────────────────────────────────
// 3. RMS DE OSCILACIÓN — ROLL
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class RollRmsMetric
 * @brief Calcula el RMS acumulado del ángulo de roll centrado en 0°.
 *
 * La señal de roll viene en el rango 0–360° referenciada a 180°
 * (igual que en el código Python: roll_centered = roll - 180.0).
 * El RMS se calcula sobre todas las muestras recibidas.
 *
 * Uso típico:
 * @code
 *   RollRmsMetric metric;
 *   // En cada ciclo IMU:
 *   metric.update(roll_deg);   // roll_deg en 0-360 (referenciado a 180)
 *   double rms = metric.get_rms_deg();
 * @endcode
 */
class RollRmsMetric {
public:
    /**
     * @brief Ingresa una nueva muestra de roll.
     *
     * @param roll_deg  Ángulo de roll en grados (rango 0–360, ref 180°).
     *                  La función lo centra internamente: centered = roll_deg - 180.
     */
    void update(double roll_deg);

    /**
     * @brief Reinicia el histórico acumulado.
     */
    void reset();

    /**
     * @return RMS del roll centrado en grados. Devuelve 0.0 si no hay muestras.
     */
    double get_rms_deg() const;

    /** @return Número de muestras acumuladas. */
    std::size_t sample_count() const { return n_; }

private:
    double      sum_sq_ = 0.0;  ///< Suma de (roll_centered)²
    std::size_t n_      = 0;    ///< Número de muestras
};

// ─────────────────────────────────────────────────────────────────────────────
// 4. RMS DE OSCILACIÓN — PITCH
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class PitchRmsMetric
 * @brief Calcula el RMS acumulado del ángulo de pitch.
 *
 * El pitch ya viene como valor absoluto en grados (igual que en Python).
 * El RMS se calcula sobre todas las muestras recibidas.
 *
 * Uso típico:
 * @code
 *   PitchRmsMetric metric;
 *   // En cada ciclo IMU:
 *   metric.update(pitch_deg);  // pitch_deg >= 0
 *   double rms = metric.get_rms_deg();
 * @endcode
 */
class PitchRmsMetric {
public:
    /**
     * @brief Ingresa una nueva muestra de pitch.
     *
     * @param pitch_deg  Ángulo de pitch en grados (valor absoluto, >= 0).
     */
    void update(double pitch_deg);

    /**
     * @brief Reinicia el histórico acumulado.
     */
    void reset();

    /**
     * @return RMS del pitch en grados. Devuelve 0.0 si no hay muestras.
     */
    double get_rms_deg() const;

    /** @return Número de muestras acumuladas. */
    std::size_t sample_count() const { return n_; }

private:
    double      sum_sq_ = 0.0;
    std::size_t n_      = 0;
};