# Métricas y prueba de robustez

## Inyección de ruido artificial

La red neuronal ahora tiene la facultad de añadir ruido artificial a los sensores para hacer la prueba de robustez (esta prueba deberá hacerse independiente a las otras). Con el siguiente comando puedes poner la cantidad de ruido que se añadirá a la prueba:

```bash
ros2 run peter_robot red_neuronal --ros-args -p nl:=0
```

Donde `nl` corresponde a **Noise Level** con las siguientes opciones:

| nl | Ruido |
|----|--------|
| 0 | 0% de ruido (este es el `nl` por defecto si no se anexa como argumento del run) |
| 1 | 5% de ruido |
| 2 | 10% de ruido |

---

## Índice de Robustez (RI)

Hay que tener en cuenta que durante todas las pruebas hay que tener el nodo `RobustnessMetric` activo. Una vez reciba el RMSE de todas ellas (mediante el nodo `RMSENode`) soltará el valor de RI (**Índice de Robustez**).

Esta es la forma de interpretar el resultado:

| Valor de RI | Interpretación |
|------------|---------------|
| RI ≈ 1 | Muy robusto |
| RI ≈ 0 | Degradación severa |
| RI < 0 | El sistema colapsa con ruido |

---

## Configuración del RMSENode

Hay que tener algo en cuenta y es que el `RMSENode` (en su función `__init__`) tiene 3 variables llamadas:

```python
self.start
self.goal
self.duration
```

Estas indican:

- **self.start**: posición inicial del robot.
- **self.goal**: posición que tiene que alcanzar (podría ser el estímulo apetente en este caso).
- **self.duration**: duración en segundos que le toma al robot llegar ahí cuando `nl = 0`.

La duración se deberá definir de forma manual y la posición final puede ser el estímulo apetente o el punto donde el robot llegue después de un debido tiempo.

Por ejemplo, se podrían registrar las coordenadas del robot y la duración en segundos cuando el robot cruce el terreno rocoso y definir ese punto como `self.goal`.

> **Importante:** La prueba de robustez es la única métrica que se debe medir de forma independiente y con pruebas que no tengan obstáculos o trayectorias muy variantes, ya que el RMSE traza una línea recta entre la posición inicial y final y la define como trayectoria óptima. Esto se hace así por efectos de simplicidad y tiempo.

---

## Tópico `/Metrics`

Las demás métricas se pueden medir en cualquier prueba para medir el desempeño del robot durante la misma o simplemente para tener una idea de los cambios de posición, rotación u otras cosas que pasaron durante la prueba.

El siguiente array corresponde al tópico `/Metrics`:

```python
self.metricsArr = [
    self.Tresponse,
    self.Tswitch,
    self.roll_rms,
    self.pitch_rms,
    self.ACTIVE_NOISE_LEVEL_IDX
]
```

---

# Explicación de las métricas

## Tresponse (Tiempo de respuesta)
Corresponde al tiempo que tarda la red neuronal en generar una respuesta ante la aparición de un estímulo relevante. Esta se mide la diferencia entre el momento en el que el robot llega al tereeno rocoso (SIN HABERLO DETECTADO) hasta que stable_condition es True, esta condición se define en la linea 554 de la red neuronal, si se ve que esta métrica no converge, puedes mover los valores donde se interpreta convergencia, los cuales dependen del roll pitch y vibración

## Tswitch (Tiempo de conmutación)
Corresponde al tiempo que tarda el robot en cambiar de modo desde que se envía la instrucción de cambio de modo, empieza desde que se envía la instrucción de cambio de modo hasta cuando es estable en teoría es el tiempo que se tarda el robot en cambiar de un modo a otro

## roll_rms
Corresponde al valor RMS del ángulo de roll del robot durante toda la prueba.

## pitch_rms
Corresponde al valor RMS del ángulo de pitch del robot durante toda la prueba.

## RMSE de posición
Este corresponde al tópico /rmse_ct e indica qué tanto el robot se desvía de la ruta óptima hacia un punto fijo

