#pragma once

#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <random>

// ============================================================================
// RUIDO
// ============================================================================

// Añadir ruido gaussiano a un escalar
inline double addGaussianNoise(
    double value,
    double sigma_fraction = 0.05,
    unsigned int seed = std::random_device{}())
{
    std::mt19937 gen(seed);

    double sigma_abs =
        std::max(std::abs(value) * sigma_fraction, 1e-4);

    std::normal_distribution<double> dist(0.0, sigma_abs);

    return value + dist(gen);
}

// Añadir ruido gaussiano a un vector
inline std::vector<double> addGaussianNoiseArray(
    const std::vector<double>& signal,
    double sigma_fraction = 0.05,
    unsigned int seed = std::random_device{}())
{
    std::mt19937 gen(seed);

    std::vector<double> noisy(signal.size());

    for(size_t i = 0; i < signal.size(); i++)
    {
        double sigma_abs =
            std::max(std::abs(signal[i]) * sigma_fraction, 1e-4);

        std::normal_distribution<double> dist(0.0, sigma_abs);

        noisy[i] = signal[i] + dist(gen);
    }

    return noisy;
}

// ============================================================================
// MÉTRICAS DEL ROBOT
// ============================================================================

// Tiempo de respuesta
inline double responseTime(
    double t_stimulus,
    double t_stable)
{
    return t_stable - t_stimulus;
}

// Delay de cambio de modo
inline double modeSwitchDelay(
    double t_command,
    double t_stable)
{
    return t_stable - t_command;
}

// RMS auxiliar
inline double rms(const std::vector<double>& signal)
{
    if(signal.empty())
        return 0.0;

    double sum_sq = 0.0;

    for(double x : signal)
        sum_sq += x * x;

    return std::sqrt(sum_sq / signal.size());
}

// Roll RMS y Pitch RMS
inline std::pair<double,double> rollPitchRMS(
    const std::vector<double>& roll_signal,
    const std::vector<double>& pitch_signal)
{
    return {
        rms(roll_signal),
        rms(pitch_signal)
    };
}

// RMSE de trayectoria
inline double trajectoryRMSE(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<double>& x_ref,
    const std::vector<double>& y_ref)
{
    size_t N = std::min({
        x.size(),
        y.size(),
        x_ref.size(),
        y_ref.size()
    });

    if(N == 0)
        return 0.0;

    double sum_error_sq = 0.0;

    for(size_t i = 0; i < N; i++)
    {
        double dx = x[i] - x_ref[i];
        double dy = y[i] - y_ref[i];

        double error = std::sqrt(dx*dx + dy*dy);

        sum_error_sq += error * error;
    }

    return std::sqrt(sum_error_sq / N);
}

// Energía consumida
inline double consumedEnergy(
    const std::vector<double>& current_signal,
    double voltage,
    double dt)
{
    double energy = 0.0;

    for(double current : current_signal)
        energy += voltage * current * dt;

    return energy;
}

// ============================================================================
// MÉTRICAS NEURONALES
// ============================================================================

// Latencia de decisión
inline double decisionLatency(
    double t_stimulus,
    double t_trigger)
{
    return t_trigger - t_stimulus;
}

// Latencia media
inline double meanDecisionLatency(
    const std::vector<double>& latencies)
{
    if(latencies.empty())
        return 0.0;

    double sum =
        std::accumulate(
            latencies.begin(),
            latencies.end(),
            0.0);

    return sum / latencies.size();
}

// Varianza de una neurona
inline double neuronVariance(
    const std::vector<double>& signal)
{
    if(signal.size() < 2)
        return 0.0;

    double mean =
        std::accumulate(
            signal.begin(),
            signal.end(),
            0.0) / signal.size();

    double var = 0.0;

    for(double x : signal)
        var += (x - mean) * (x - mean);

    return var / signal.size();
}

// Varianza de disparo neuronal
inline double firingVariance(
    const std::vector<std::vector<double>>& activity_matrix)
{
    if(activity_matrix.empty())
        return 0.0;

    size_t neurons = activity_matrix[0].size();

    double total_var = 0.0;

    for(size_t n = 0; n < neurons; n++)
    {
        std::vector<double> neuron_signal;

        for(const auto& row : activity_matrix)
            neuron_signal.push_back(row[n]);

        total_var += neuronVariance(neuron_signal);
    }

    return total_var / neurons;
}

// ============================================================================
// CONSISTENCIA TEMPORAL
// ============================================================================

inline double temporalConsistency(
    const std::vector<std::vector<double>>& activity_matrix)
{
    if(activity_matrix.size() < 2)
        return 0.0;

    std::vector<double> correlations;

    for(size_t i = 1; i < activity_matrix.size(); i++)
    {
        const auto& a = activity_matrix[i - 1];
        const auto& b = activity_matrix[i];

        double dot = 0.0;
        double na = 0.0;
        double nb = 0.0;

        for(size_t j = 0; j < a.size(); j++)
        {
            dot += a[j] * b[j];
            na += a[j] * a[j];
            nb += b[j] * b[j];
        }

        na = std::sqrt(na);
        nb = std::sqrt(nb);

        if(na < 1e-9 || nb < 1e-9)
            continue;

        correlations.push_back(dot / (na * nb));
    }

    if(correlations.empty())
        return 0.0;

    double sum =
        std::accumulate(
            correlations.begin(),
            correlations.end(),
            0.0);

    return sum / correlations.size();
}

// ============================================================================
// EFICIENCIA λ
// ============================================================================

inline double lambdaEfficiency(
    double command_magnitude,
    double blue_intensity,
    double red_intensity)
{
    // Sin conflicto
    if(red_intensity <= 0.0 ||
       blue_intensity <= 0.0)
    {
        return 1.0;
    }

    double conflict_intensity =
        red_intensity + blue_intensity;

    if(conflict_intensity <= 1e-9)
        return 1.0;

    return std::min(
        1.0,
        command_magnitude / conflict_intensity);
}