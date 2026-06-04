#Lista de métricas para el robot real

import numpy as np


# Añadir ruido gaussiano a un escalar
def add_gaussian_noise(value, sigma_fraction=0.05, seed=None):
    """
    value: valor original
    sigma_fraction: porcentaje de ruido (0.05 = 5%)
    """

    rng = np.random.default_rng(seed)

    sigma_abs = max(abs(value) * sigma_fraction, 1e-4)

    noise = rng.normal(0.0, sigma_abs)

    return value + noise


# Añadir ruido gaussiano a un escalar

def add_gaussian_noise_array(signal, sigma_fraction=0.05, seed=None):
    """
    signal: array numpy
    sigma_fraction: porcentaje de ruido
    """

    rng = np.random.default_rng(seed)

    signal = np.asarray(signal)

    sigma_abs = np.maximum(
        np.abs(signal) * sigma_fraction,
        1e-4
    )

    noise = rng.normal(0.0, sigma_abs)

    return signal + noise

#tiempo de respuesta
def response_time(t_stimulus, t_stable):
    """
    t_stimulus : instante de aparición de la perturbación (medición manual) tiempo que le toma al robot en llegar al terreno rocoso sin detectarlo
    t_stable   : instante en que vuelve a estabilizarse
    """

    return t_stable - t_stimulus

#delay de cambio de modo
def mode_switch_delay(t_command, t_stable):
    """
    t_command : instante en que se ordena el cambio
    t_stable  : instante en que el robot se estabiliza
    """

    return t_stable - t_command



#Roll y pitch RMS

def rollpitch_rms(roll_signal, pitch_signal):

    roll_signal = np.asarray(roll_signal)
    pitch_signal = np.asarray(pitch_signal)

    return np.sqrt(np.mean(np.square(roll_signal))), np.sqrt(np.mean(np.square(pitch_signal)))


#error RMSE (este requiere posición del robot)

def trajectory_rmse(x, y, x_ref, y_ref):

    error = np.sqrt((np.asarray(x) - np.asarray(x_ref))**2 +(np.asarray(y) - np.asarray(y_ref))**2)

    return np.sqrt(np.mean(error**2))


def consumed_energy(current_signal, voltage, dt):
    """
    Energía consumida [J]

    current_signal : array de corrientes [A]
    voltage : voltaje [V]
    dt : tiempo entre muestras [s]
    """

    power_signal = voltage * np.asarray(current_signal)

    return np.sum(power_signal) * dt


# METRICAS NEURONALES ------------------------------------------------------------------------------------------------------------------------

#latencia de decisión
def decision_latency(t_stimulus, t_trigger):
    """
    t_stimulus : instante de aparición del estímulo
    t_trigger  : instante del primer comando motor
    """

    return t_trigger - t_stimulus


def mean_decision_latency(latencies):
    """
    latencies: lista de latencias individuales
    """

    return np.mean(latencies)


#varianza del disparo neuronal

def firing_variance(activity_matrix):
    """
    shape = (tiempo, neuronas)
    """
    activity_matrix = np.asarray(activity_matrix)

    per_neuron_var = np.var(activity_matrix, axis=0)

    return np.mean(per_neuron_var)


#varianza de una neurona específica
def neuron_variance(neuron_signal):

    neuron_signal = np.asarray(neuron_signal)

    return np.var(neuron_signal)



#consistencia temporal


def temporal_consistency(activity_matrix):

    activity_matrix = np.asarray(activity_matrix)

    if len(activity_matrix) < 2:
        return 0.0

    corr = []

    for i in range(1, len(activity_matrix)):

        a = activity_matrix[i-1]
        b = activity_matrix[i]

        na = np.linalg.norm(a)
        nb = np.linalg.norm(b)

        if na < 1e-9 or nb < 1e-9:
            continue

        corr.append(np.dot(a, b)/(na*nb))

    return np.mean(corr) if corr else 0.0


#eficiencia 
def lambda_efficiency(command_magnitude,blue_intensity,red_intensity):

    # Sin conflicto
    if red_intensity <= 0.0 or blue_intensity <= 0.0:
        return 1.0

    conflict_intensity = red_intensity + blue_intensity

    if conflict_intensity <= 1e-9:
        return 1.0

    return min(1.0, command_magnitude / conflict_intensity)




"""
Aquí está la explicación de la implementación de cada una de las funciones:


LOCOMOTORAS Y CONTROL

1. gaussian_noise debe usarse solo en la prueba de robustez, estas funciones se usan para añadir ruido a 
los sensores, cuando se haga la prueba de robustez deben medir el RMSE de posición con ruido = 0%, 5%, 10%, 15%
con esto, pueden calcular el índice de robustez (manualmente)


2. response_time: hay que medir el tiempo que le toma al robot en la prueba en llegar al terreno rocoso (esto sin detectarlo así que deben
medirlo por su cuenta), luego se hace la diferencia de este tiempo y el que le toma a estabilizarse en el nuevo terreno, esto usando un criterio de estabilidad,
mi criterio se basó en pitch roll y desviación estándar de la vibración 

stable_condition = (self.accel_std < 2 and self.pitch < 1.5 and self.roll > 178) #Estabilización cond

una vez sea true, se calcula el tiempo de respuesta

3. switch_delay: similar a la anterior pero se hace la diferencia entre el tiempo que el robot ordena un cambio de modo de locomoción y el tiempo que se estabiliza en el
nuevo terreno, se usa el mismo criterio de estabilidad que el antes mostrado, pueden variar los valores a conveniencia

4. rollypitch RMS, medición adicional para el pitch y roll, se puede usar para gráficas

5. RMSE de trayectoria: este valor es usado junto a las funciones gaussianas para calcular el RMSE, al finalizar la prueba deben hacer una media de las mediciones
para calcular el índice de robustez

RI = 1.0 - (((rmse_5 - rmse_0) + (rmse_10 - rmse_0)) / (2.0 * rmse_0) 

este es un ejemplo de cómo calcularlo con el RMSE usando pruebas con ruido de 5% y 10%
RI ≈ 1  -> muy robusto
RI ≈ 0  -> degradación severa
RI < 0  -> colapso ante ruido

6. Energía: esta medición usaría corriente y un voltaje para calcular la energia que usó el robot en la prueba
puede ser interesante comparar esta medición con un caso donde el robot no tenga cambios de locomoción como prueba que nuestro método es 
más eficiente energéticamente hablando



NEURONALES


1. decision_latency: tiempo que le toma a una o varias neuronas decidir, en este caso se usaría en cuanto a un estímulo y la toma de decisión

2. firing_variance (Varianza de disparo): promedio de la varianza por neurona en la ventana temporal.
Con N muestras de actividad (cada una es un vector de neuronas),
calculamos la varianza temporal de cada neurona y promediamos.

activity_matrix tiene shape Tiempo, neuronas

3. temporal_consistency:
Consistencia temporal: correlación media entre vectores de actividad
en ciclos consecutivos dentro de la ventana.

Valor ∈ [-1, 1]:  1.0 = perfectamente consistente.




4. efficiency lambda:
Eficiencia de resolución de conflictos λ.

Definición operativa:
    λ = (decisión tomada) / (intensidad total de estímulos en conflicto)

- Si solo hay un estímulo activo: λ = 1.0 (no hay conflicto).
- Si hay dos estímulos simultáneos: λ = cmd_mag / (R_intensity + B_intensity).
    Valores altos (→1) indican que el sistema tomó una decisión clara y eficiente
    a pesar del conflicto; valores bajos indican parálisis o indecisión.
- Normalizado a [0, 1].


"""