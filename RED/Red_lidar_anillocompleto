import numpy as np
import matplotlib.pyplot as plt

# Parámetros Globales
n_neuronas = 60  # Número de neuronas (direcciones preferidas) en un anillo completo
sigma = 0.06     # Parámetro de la gausiana
umbral = 0.95    # Umbral para discriminar la activación

dt = 0.1         # Paso temporal
t_total = 500    # Duración total de la simulación (ms)
n_steps = int(t_total/dt)
tau = 5          # Constante de tiempo para la dinámica neuronal

# Función gaussiana para la respuesta direccional
def gausiana(theta, omega):
    return np.exp((np.vdot(theta, omega) - 1) / (2 * (sigma**2)))

# Definir direcciones preferidas en todo el círculo (0° a 360°)
Directions = np.linspace(0, 360, n_neuronas, endpoint=False) * np.pi/180  # en radianes
Om = np.vstack((np.cos(Directions), np.sin(Directions)))    # Vectores unitarios

# Solicitar al usuario la cantidad de pares a generar
num_pares = int(input("Ingrese la cantidad de pares de datos a generar: "))

# Generar num_pares de datos aleatorios:
# Ángulo en el rango de [0,360]° y distancia en [0.05,0.35]
datos = []
for _ in range(num_pares):
    ang = np.random.uniform(0, 360)       
    dist = np.random.uniform(0.05, 0.35)
    datos.append((ang, dist))

print("Datos generados (ángulo, distancia):")
for d in datos:
    print(f"Ángulo = {d[0]:.2f}°, Distancia = {d[1]:.2f}")

# Preparar la entrada neuronal a lo largo del tiempo
entrada_neuronal = np.zeros((n_neuronas, n_steps))
duracion_pulso = int(30/dt)          # Duración de cada pulso (30 ms)
espaciado = n_steps // num_pares     # Espaciado entre inyecciones

# Lista para almacenar las activaciones (gaussianas) de cada par, para graficarlas luego.
activaciones_par = []

# Para cada par de datos, calcular activaciones y definir el aporte a inyectar
for i, (ang_input, distancia) in enumerate(datos):
    # Convertir el ángulo a vector unitario
    ang_rad = np.deg2rad(ang_input)
    vector_input = np.array([np.cos(ang_rad), np.sin(ang_rad)])
    
    # Calcular la activación para cada neurona mediante la función gausiana
    activaciones = np.array([gausiana(vector_input.reshape(2,1), Om[:, j]) 
                              for j in range(n_neuronas)])
    # Guardar las activaciones para graficar posteriormente
    activaciones_par.append((ang_input, distancia, activaciones))
    
    # Definir el aporte: si la activación > umbral, aporte = 1 * distancia, sino 0.
    aporte_neuronal = np.where(activaciones > umbral, 1 * distancia, 0)
    
    # Determinar el intervalo temporal en el que se inyecta esta entrada:
    t_inicio = i * espaciado
    t_fin = t_inicio + duracion_pulso
    
    # Inyectar el aporte en cada neurona durante ese intervalo
    for j in range(n_neuronas):
        entrada_neuronal[j, t_inicio:t_fin] += aporte_neuronal[j]

# =============================================================================
# 1. Graficar las activaciones de la función gaussiana para cada par de datos
# =============================================================================
plt.figure(figsize=(12, 8))
pref_directions_deg = np.degrees(Directions)
for (ang_input, distancia, activaciones) in activaciones_par:
    plt.stem(pref_directions_deg, activaciones, linefmt='grey', markerfmt='o', basefmt=" ", 
             label=f"Ángulo {ang_input:.1f}° / Dist {distancia:.2f}")
    # Marcar en rojo las neuronas cuya activación supera el umbral
    indices = np.where(activaciones > umbral)[0]
    plt.plot(pref_directions_deg[indices], activaciones[indices], 'ro')
plt.xlabel('Dirección preferida (°)')
plt.ylabel('Activación gaussiana')
plt.title('Activaciones de la función gaussiana para cada par (rojo si > 0.95)')
plt.legend(loc='upper right', fontsize='small')
plt.grid(True)
plt.show()

# =============================================================================
# 2. Simulación de la dinámica temporal neuronal
# =============================================================================
z = np.zeros((n_neuronas, n_steps))
for i in range(n_steps - 1):
    z[:, i+1] = z[:, i] + (dt/tau) * (-z[:, i] + entrada_neuronal[:, i])

plt.figure(figsize=(12, 8))
tiempo = np.arange(0, t_total, dt)
for j in range(n_neuronas):
    plt.plot(tiempo, z[j, :], label=f"Neurona {j} ({np.degrees(Directions[j]):.1f}°)")
plt.xlabel("Tiempo (ms)")
plt.ylabel("Actividad z(t)")
plt.title("Dinámica temporal neuronal (aporte condicionado a activación > umbral)")
plt.legend(loc="upper right", fontsize="small", ncol=2)
plt.grid(True)
plt.show()

# =============================================================================
# 3. Graficar las entradas inyectadas a cada neurona
# =============================================================================
plt.figure(figsize=(12, 4))
for j in range(n_neuronas):
    plt.plot(tiempo, entrada_neuronal[j, :], label=f"Neurona {j}")
plt.xlabel("Tiempo (ms)")
plt.ylabel("Entrada inyectada")
plt.title("Entradas inyectadas a cada neurona")
plt.legend(loc="upper right", fontsize="small", ncol=2)
plt.grid(True)
plt.show()
