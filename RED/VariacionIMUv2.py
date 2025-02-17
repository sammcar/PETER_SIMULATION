import numpy as np
import matplotlib.pyplot as plt


def naka(u,sigma):
    return((max(u,0)**2)/(sigma**2 + max(u,0)**2))

# --------------- V A R I A B L E S ---------------------- #

Time = 500

sigma_az = 0.14 #Variación estandar de la aceleración en z recibida por el IMU
Usigma_az = 0.1790 #Ejemplo de variación estándar de un IMU

# DATOS DEL IMU
pitch = 18
Upitch = 20 #Umbral
roll = 18
Uroll = 20 #Umbral

# Sacado del módulo de arbitraje
BlueStim = 0.3
GreenStim = 0.4

# ---------------------------------- RED ------------------------ #

Neu = np.zeros((6, Time+1))
Tao1 = 10
Tao2 = 20

for i in range(Time):
    Neu[0, i+1] = np.clip((Neu[0,i] + (1/Tao1) * (-Neu[0,i] + sigma_az - Usigma_az)), 0, None)
    Neu[1, i+1] = np.clip((Neu[1,i] + (1/Tao1) * (-Neu[1,i] + pitch - Upitch)), 0, None)
    Neu[2, i+1] = np.clip((Neu[2,i] + (1/Tao1) * (-Neu[2,i] + roll -   Uroll)), 0, None)

    #Por si acaso implementación con naka
    #Neu[3, i+1] = np.clip((Neu[3,i] + (1/Tao2) * (-Neu[3,i] + 10*Neu[0,i] + Neu[1,i] + Neu[2,i] + naka(-Neu[4,i]-Neu[5,i] ,2))), 0, None) 
    #Neu[4, i+1] = np.clip((Neu[4,i] + (1/Tao2) * (-Neu[4,i] + BlueStim  + naka(-Neu[3,i]-Neu[5,i] ,2))), 0, None)
    #Neu[5, i+1] = np.clip((Neu[5,i] + (1/Tao2) * (-Neu[5,i] + GreenStim + naka(-Neu[3,i]-Neu[4,i] ,2))), 0, None)
    
    Neu[3, i+1] = np.clip((Neu[3,i] + (1/Tao2) * (-Neu[3,i] + 20*Neu[0,i] + 0.7*Neu[1,i] + 0.7*Neu[2,i] -Neu[4,i]-Neu[5,i])), 0, None)
    Neu[4, i+1] = np.clip((Neu[4,i] + (1/Tao2) * (-Neu[4,i] + BlueStim  - Neu[3,i]-Neu[5,i])), 0, None)
    Neu[5, i+1] = np.clip((Neu[5,i] + (1/Tao2) * (-Neu[5,i] + GreenStim - Neu[3,i]-Neu[4,i])), 0, None)


# -------------------------------- G R Á F I C A S ----------------------------#

fig, ax = plt.subplots(2, 1, figsize=(10, 10))  # 2 filas, 1 columna

# 🔹 Primer subplot: Red de cambio de Modo
ax[0].plot(Neu[3], label="Modo Cuad", color='r')
ax[0].plot(Neu[4], label="Modo H", color='b')
ax[0].plot(Neu[5], label="Modo X", color='g')
ax[0].set_xlabel("Tiempo")
ax[0].set_ylabel("Actividad neuronal")
ax[0].set_title("Red de cambio de Modo")
ax[0].legend()
ax[0].grid()

# 🔹 Segundo subplot: Entradas del IMU
ax[1].plot(Neu[0], label="Sigma az", color='r')
ax[1].plot(Neu[1], label="Pitch", color='b')
ax[1].plot(Neu[2], label="Roll", color='g')
ax[1].set_xlabel("Tiempo")
ax[1].set_ylabel("Actividad neuronal")
ax[1].set_title("Entradas del IMU")
ax[1].legend()
ax[1].grid()


plt.tight_layout()  # Ajusta los espacios entre subplots

plt.show()

