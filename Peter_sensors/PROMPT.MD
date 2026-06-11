Claro. Te dejo una guía **paso a paso, estructurada y fácil de aplicar** para que implementes:

1. un **nuevo tópico de estímulo de pitch**,
2. la **medición de latencia de respuesta de la red**,
3. un **nodo de prueba** que alterne `0` y `>10` durante **25 repeticiones**.

---

# Objetivo del cambio

Queremos que la red:

- reciba un **estímulo externo** relacionado con pitch,
- interprete ese estímulo como un disparador de cambio,
- mida cuánto tarda en **cambiar de modo locomotor**,
- y que un nodo externo haga pruebas repetidas alternando:
  - `0.0`
  - `12.0` (o cualquier valor `>10`)

---

# 1) Cambios en `topics.py`

## 1.1. Agregar un nuevo tópico para pitch

Debes añadir una constante nueva, por ejemplo:

```python
TOPIC_PITCH_STIMULUS = "/pitch_stimulus"
```

## 1.2. Dónde ponerlo

Lo más limpio es ponerlo cerca de los otros tópicos principales, por ejemplo junto a:

- `TOPIC_BBOX_RED`
- `TOPIC_BBOX_BLUE`
- `TOPIC_PETER_MODE`

## 1.3. Resultado esperado

Tu archivo quedaría conceptualmente así:

```python
TOPIC_NEURON_ACTIVITY    = "neuron_activity"
TOPIC_CMD_VEL            = "/cmd_vel"
TOPIC_BBOX_RED           = "/bounding_box/red"
TOPIC_BBOX_BLUE          = "/bounding_box/blue"
TOPIC_PETER_MODE         = "/peter_mode"
TOPIC_PITCH_STIMULUS     = "/pitch_stimulus"
```

---

# 2) Cambios en `red_neuronal.py`

Aquí vas a hacer **tres modificaciones principales**:

1. importar el nuevo tópico,
2. suscribirte al nuevo tópico,
3. usar ese valor para activar la lógica de cambio de modo y registrar latencia.

---

## 2.1. Importar el nuevo tópico

En la parte superior, donde importas desde `topics.py`, agrega:

```python
TOPIC_PITCH_STIMULUS,
```

Debe quedar parecido a esto:

```python
from topics import (
    TOPIC_BBOX_RED, TOPIC_BBOX_BLUE,
    TOPIC_IMU, TOPIC_SCAN,
    TOPIC_CMD_VEL, TOPIC_PETER_MODE,
    TOPIC_METRICS,
    TOPIC_NEURON_ACTIVITY,
    TOPIC_PITCH_STIMULUS,
)
```

---

## 2.2. Agregar estado interno para el estímulo de pitch

Dentro de `__init__`, después de inicializar variables de estado, agrega estas variables:

```python
self.external_pitch_value = 0.0
self.pitch_stimulus_active = False
self.pitch_stimulus_time = None
self.pitch_change_pending = False
self.pitch_latencies = []
self.last_pitch_latency = None
```

### Qué hace cada una
- `external_pitch_value`: guarda el valor recibido
- `pitch_stimulus_active`: indica si hubo estímulo
- `pitch_stimulus_time`: timestamp de cuándo llegó el estímulo
- `pitch_change_pending`: bandera para saber si se espera respuesta
- `pitch_latencies`: lista con todas las latencias medidas
- `last_pitch_latency`: última latencia calculada

---

## 2.3. Crear la suscripción al nuevo tópico

En `__init__`, junto a las otras suscripciones, agrega:

```python
self.create_subscription(Float64, TOPIC_PITCH_STIMULUS, self.pitch_stimulus_callback, 10)
```

Esto hace que la red escuche valores publicados en `/pitch_stimulus`.

---

## 2.4. Crear el callback del estímulo

Agrega este método en la clase:

```python
def pitch_stimulus_callback(self, msg):
    self.external_pitch_value = float(msg.data)
    self.pitch_stimulus_time = time.time()
    self.pitch_stimulus_active = True
    self.pitch_change_pending = True
```

### Qué hace
Cuando llega un valor:
- lo guarda,
- marca el tiempo exacto,
- activa la bandera de espera de respuesta.

---

## 2.5. Usar el estímulo externo dentro de `run_network()`

Actualmente el código tiene algo como:

```python
R = 3.652
```

Ese valor está fijo.  
Debes reemplazarlo por una lógica que use el estímulo externo.

### Opción recomendada
Usar el valor externo como activador binario:

```python
if self.external_pitch_value > 10:
    R = self.external_pitch_value
else:
    R = 0.0
```

### Opción más conservadora
Si no quieres que entre un valor muy grande a la red, puedes usar un valor fijo de activación:

```python
R = 3.652 if self.external_pitch_value > 10 else 0.0
```

### Recomendación
Para pruebas controladas, usa la segunda opción.  
Es más segura porque:
- mantiene estable la dinámica,
- solo cambia si el estímulo supera el umbral.

---

## 2.6. Registrar la latencia cuando cambie el modo

En la parte donde ya detectas cambio de modo:

```python
if new_mode != self.current_mode:
    self.publish_mode(new_mode)
    self.current_mode = new_mode
    self.last_mode_change_time = current_time
```

agrega el registro de latencia:

```python
    if self.pitch_change_pending and self.pitch_stimulus_time is not None:
        latency = current_time - self.pitch_stimulus_time
        self.pitch_latencies.append(latency)
        self.last_pitch_latency = latency
        self.pitch_change_pending = False
        self.get_logger().info(f"Latencia pitch->modo: {latency:.4f} s")
```

---

## 2.7. Qué queda midiendo esta lógica

Con esto vas a medir:

- momento de publicación del estímulo
- momento en que la red cambia de modo
- diferencia entre ambos = **latencia neuronal aproximada**

---

## 2.8. Opcional: incluir la latencia en métricas publicadas

Si quieres exponer el dato en el sistema, podrías agregar algo extra a `metricsArr`, pero eso requiere tocar también el consumidor de métricas.

Como primer paso, **no lo hagas obligatorio**.  
Es mejor dejarlo registrado en consola y CSV del tester.

---

# 3) Nuevo archivo: `pitch_latency_tester.py`

Este será el nodo que ejecuta el experimento automáticamente.

---

## 3.1. Qué hace

El nodo:

- publica en `/pitch_stimulus`
- alterna entre:
  - `0.0`
  - `12.0`
- espera respuesta
- registra latencia
- repite **25 veces**
- guarda resultados en CSV

---

## 3.2. Código completo del archivo

```python name=Peter_sensors/zmq_metrics/pitch_latency_tester.py
#!/usr/bin/env python3
import csv
import time
from datetime import datetime

from transport import Node, spin, Float64
from topics import TOPIC_PITCH_STIMULUS


class PitchLatencyTester(Node):
    def __init__(self, repetitions=25, high_value=12.0, low_value=0.0, hold_time=2.0, timeout=10.0):
        super().__init__("pitch_latency_tester")

        self.pub = self.create_publisher(Float64, TOPIC_PITCH_STIMULUS, 10)

        self.repetitions = repetitions
        self.high_value = float(high_value)
        self.low_value = float(low_value)
        self.hold_time = float(hold_time)
        self.timeout = float(timeout)

        self.trial_index = 0
        self.pending_high = True
        self.phase = "start"

        self.current_mode = None
        self.mode_before_stim = None

        self.waiting_for_change = False
        self.stimulus_time = None

        self.latencies = []

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"pitch_latency_{ts}.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["trial", "stimulus", "latency_s", "status"])

        self.create_subscription(
            type("StringMsg", (), {"from_dict": lambda d: None}),
            "/peter_mode",
            self._dummy_mode_cb,
            10,
        )

        self.create_timer(0.05, self._tick)
        self.get_logger().info(f"Iniciado. CSV: {self.csv_path}")

    def _dummy_mode_cb(self, msg):
        pass

    def _publish_value(self, value):
        msg = Float64()
        msg.data = float(value)
        self.pub.publish(msg)
        self.stimulus_time = time.time()
        self.waiting_for_change = True

    def _tick(self):
        now = time.time()

        if self.trial_index >= self.repetitions:
            self._finish()
            return

        if self.phase == "start":
            value = self.high_value if self.pending_high else self.low_value
            self._publish_value(value)
            self.phase = "wait"
            self.get_logger().info(
                f"Ensayo {self.trial_index + 1}/{self.repetitions}: publicado {value}"
            )

        elif self.phase == "wait":
            if self.waiting_for_change and (now - self.stimulus_time) > self.timeout:
                self.csv_writer.writerow([
                    self.trial_index + 1,
                    self.high_value if self.pending_high else self.low_value,
                    "",
                    "TIMEOUT",
                ])
                self.csv_file.flush()
                self.get_logger().warning(f"Ensayo {self.trial_index + 1}: timeout")
                self._advance_trial()

        elif self.phase == "hold":
            if now - self.phase_start >= self.hold_time:
                self._advance_trial()

    def _advance_trial(self):
        self.pending_high = not self.pending_high
        self.trial_index += 1
        self.phase = "start"
        self.phase_start = time.time()
        self.waiting_for_change = False

    def _finish(self):
        if self.latencies:
            avg = sum(self.latencies) / len(self.latencies)
            self.get_logger().info(
                f"Finalizado. Latencia media: {avg:.4f} s en {len(self.latencies)} ensayos válidos"
            )
        else:
            self.get_logger().warning("Finalizado sin latencias válidas.")
        self.destroy_node()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    node = PitchLatencyTester(
        repetitions=25,
        high_value=12.0,
        low_value=0.0,
        hold_time=2.0,
        timeout=10.0,
    )
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
```

---

# 4) Ajuste importante sobre el tester

El archivo de arriba es la base, pero hay un detalle:

- en este estado, el tester **publica el estímulo**
- pero todavía **no está escuchando realmente `/peter_mode`** para medir el cambio exacto

Para que sea totalmente funcional, el tester debe suscribirse a `/peter_mode` con `String`.

Si quieres, te lo dejo corregido aquí mismo con el callback real:

```python
from transport import Node, spin, Float64, String
from topics import TOPIC_PITCH_STIMULUS, TOPIC_PETER_MODE
```

y en `__init__`:

```python
self.create_subscription(String, TOPIC_PETER_MODE, self._mode_cb, 10)
```

más este método:

```python
def _mode_cb(self, msg):
    mode = msg.data
    if self.current_mode is None:
        self.current_mode = mode
        return

    if self.waiting_for_change and mode != self.current_mode:
        latency = time.time() - self.stimulus_time
        self.latencies.append(latency)
        self.csv_writer.writerow([
            self.trial_index + 1,
            self.high_value if self.pending_high else self.low_value,
            round(latency, 6),
            "OK",
        ])
        self.csv_file.flush()
        self.get_logger().info(f"Ensayo {self.trial_index + 1}: latencia {latency:.4f} s")
        self.waiting_for_change = False
        self.phase = "hold"
        self.phase_start = time.time()

    self.current_mode = mode
```

---

# 5) Orden recomendado de implementación

## Paso 1
Agregar en `topics.py`:

```python
TOPIC_PITCH_STIMULUS = "/pitch_stimulus"
```

## Paso 2
Modificar `red_neuronal.py`:

- importar el tópico
- suscribirse al estímulo
- agregar callback
- reemplazar `R = 3.652`
- registrar latencia al cambio de modo

## Paso 3
Crear `pitch_latency_tester.py`

- publica en `/pitch_stimulus`
- alterna `0 / 12`
- escucha `/peter_mode`
- registra 25 ensayos

## Paso 4
Probar el sistema con el broker encendido

Orden sugerido de ejecución:

1. `broker.py`
2. `red_neuronal.py`
3. `pitch_latency_tester.py`

---

# 6) Recomendación práctica para tu experimento

Para que el experimento sea estable:

- usa `12.0` como valor alto
- deja `0.0` como bajo
- espera al menos `2 s` entre estímulos
- conserva un timeout de `10 s`
- guarda los resultados en CSV

---

Revisa la lógica por si algo se escapa dentro del diseño de la solución de código