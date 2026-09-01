# Análisis Comparativo: Arbitraje BG/WTA vs. Baseline FSM — Familia C (Terreno)

**Objetivo:** responder al comentario de Reviewer #2 ("add finite state machine (FSM) gait arbitration baseline comparisons under identical experimental conditions") comparando el controlador propuesto (BG/WTA) contra el baseline FSM, restringido al alcance asignado: `familia_c1_terreno_rugoso` y `familia_c2_pendiente`.

**Fuentes de datos:**
- BG/WTA: `familia_.zip` ya documentado (nodo `NetworkPublisher`, código "ROBUSTEZ REVISION 2"), 27 trials en c1 / 15 en c2.
- FSM: `familia_c.zip` (nodo `FSMGaitArbitration`, código `fsm_gait_arbitration.py`), 25 trials en c1 / 21 en c2, carpetas `familia_c1_terreno_rugoso_fsm` / `familia_c2_pendiente_fsm`.

---

## 1. Advertencia metodológica — leer antes de usar estos resultados en el paper

Antes de cualquier comparación de métricas, encontré **tres diferencias de configuración entre ambos códigos** que afectan la validez de una comparación "bajo condiciones idénticas" tal como la pide el reviewer. Las señalo con evidencia de código exacta para que el equipo decida si re-ejecutar con los valores igualados o documentar la discrepancia explícitamente en el paper.

### 1.1 El umbral de pitch (`Upitch`) no es el mismo en ambos códigos

- BG/WTA: `else: self.Upitch = 0.9 #uMbral pitch INCLINADO`
- FSM: `else: self.Upitch = 1.30 #uMbral pitch INCLINADO`

Este umbral gobierna cuándo el sistema considera que el terreno está "inclinado" (dispara el modo H tanto en el circuito neuronal `z[1]` como en la regla FSM `pitch_rms > self.Upitch → H`). Con un umbral más alto, la FSM necesita una inclinación mayor para reaccionar — esto puede hacer que la FSM parezca más lenta o más tardía en activar H de lo que sería con el umbral original del BG, o a la inversa, más estable ante pequeñas oscilaciones. **No es la misma condición experimental.**

### 1.2 El tiempo de referencia para "tiempo de respuesta" (`tchange`) difiere

- BG/WTA: `else: self.tchange = self.starttime + 98.0`
- FSM: `else: self.tchange = self.starttime + 80.0`

`Tresponse` se calcula como `time.time() - self.tchange`, así que un `tchange` distinto desplaza el origen de la medición en 18 segundos entre ambos sistemas. **Los valores de `Tresponse` de BG y FSM no son directamente comparables sin corregir este desfase.**

### 1.3 La fórmula de `Tswitch` es distinta entre ambos códigos

- BG/WTA: `self.Tswitch = (time.time() - self.tcmd) + 1` (con un offset fijo de **+1 segundo**, según el propio comentario del código: *"se añade un seg por si la condición de estabilidad se cumple mientras cambia"*)
- FSM: `self.Tswitch = time.time() - self.tcmd` (sin offset)

Además, en el código BG/WTA, `self.tcmd` **solo se actualiza explícitamente cuando el sistema transiciona al modo C** (`if self.z[15,1] > 0.5: ...; self.tcmd = time.time()`), mientras que en `publish_mode()` la condición que originalmente fijaba `tcmd` en cualquier cambio de modo está comentada (`if self.current_mode != mode:# and (...)`) — es decir, la actualización de `tcmd` es asimétrica entre modos en el código BG. La FSM, en cambio, fija `tcmd` de forma consistente en `publish_mode()` para cualquier cambio de modo real. **Esta es probablemente la causa principal de la brecha de casi 1 segundo que se observa entre los `Tswitch` de ambos sistemas (Sección 4)** — no se puede asumir que sea una diferencia real de latencia de arbitraje sin antes corregir esta asimetría de medición.

### 1.4 Lo que SÍ es comparable

El circuito de ganglios basales (`StN`/`Gpi`/`Gpe`/`StR`, arbitraje R/G/B) está **presente e idéntico en ambos códigos** — la FSM no lo modifica ni lo remueve. Lo único que cambia es el módulo de arbitraje de **modo de locomoción** (`z[14]`/`z[15]`/`z[16]`, que en la FSM se fuerzan a `0.0` y se reemplazan por lógica explícita de prioridad + persistencia). Esto es exactamente el diseño experimental correcto que buscaba el Reviewer #2: **aislar el mecanismo de arbitraje de marcha como la única variable independiente**, sin tocar el resto del pipeline sensorial-perceptual. Este punto es una fortaleza real del diseño ejecutado y debe destacarse en el paper.

**Recomendación:** antes de publicar estos números, o bien (a) volver a correr ambos códigos con `Upitch`, `tchange` y la fórmula/condición de `Tswitch` igualadas, o (b) si no hay tiempo para re-ejecutar, documentar explícitamente en el paper estas tres diferencias como limitación metodológica de la comparación, en vez de presentar la brecha de `Tswitch`/`Tresponse` como una ventaja de diseño de la FSM.

---

## 2. Confirmación: ninguno de los dos conjuntos tuvo ruido real activo

Confirmaste que las pruebas FSM no tuvieron ruido — esto es verificable directamente en el código:

```python
self.ACTIVE_NOISE_LEVEL_IDX = 0   # fijo, no se lee de un parámetro ROS
...
self._sigma_noise = self.NoiseLevel[self.ACTIVE_NOISE_LEVEL_IDX]   # siempre 0.0
```

El manifiesto de las suites FSM sí registra `noise_level_idx` de 0 a 4 (heredado del mismo arnés de pruebas usado para el lote BG), pero **ese índice no tiene ningún efecto en el código FSM** — es un campo vestigial del harness, no una condición experimental real. Verifiqué esto empíricamente: no hay tendencia de degradación de métricas a medida que sube el `noise_level_idx` etiquetado en los datos FSM (ver tablas de la Sección 4), consistente con que no se aplicó ninguna perturbación real.

Esto significa que la comparación válida contra el lote BG/WTA es específicamente contra su condición **`noise_level_idx = 0`** (la única genuinamente sin ruido en ese conjunto), no contra los niveles 1–4 del lote BG, que sí tuvieron deriva de IMU y ruido de LiDAR reales.

---

## 3. Verdicts — conteo de trials

| Familia | Sistema | SUCCESS (retenidos) | FAILURE_TIMEOUT | FAILURE_TIPOVER | FAILURE_CRASH | N comparable |
|---|---|---|---|---|---|---|
| c1 (terreno rugoso) | BG/WTA — solo `noise_level_idx=0` | 3 | 0 | 2 | 1 | 6 intentos |
| c1 (terreno rugoso) | FSM — todos los "niveles" (sin efecto real) | 15 | 0 | 8 | 2 | 25 intentos |
| c2 (pendiente) | BG/WTA — solo `noise_level_idx=0` | 3 | 0 | 0 | 0 | 3 intentos |
| c2 (pendiente) | FSM — todos los "niveles" (sin efecto real) | 15 | 0 | 6 | 0 | 21 intentos |

**Observación directa (conteo, no tasa):** en `familia_c2_pendiente`, el lote BG/WTA con `noise_level_idx=0` no registró ningún `FAILURE_TIPOVER` en los 3 intentos disponibles, mientras que el lote FSM (con 21 intentos, muestra más grande) sí registró 6. Dado que el BG solo tiene 3 intentos en esta condición frente a los 21 de la FSM, **no es posible afirmar con esta evidencia que el BG sea más estable en pendiente** — la diferencia de tamaño muestral (3 vs. 21) es demasiado grande para sacar esa conclusión; lo único que puede afirmarse es que, en la muestra disponible, el BG no mostró vuelcos y la FSM sí. Si el equipo quiere una comparación defendible en el paper, se necesitan más réplicas de BG en `noise_level_idx=0` (o de la FSM restringidas a las primeras 3 réplicas, para igualar N).

En `familia_c1_terreno_rugoso`, ambos sistemas muestran fallos (BG: 2 TIPOVER + 1 CRASH en 6 intentos; FSM: 8 TIPOVER + 2 CRASH en 25 intentos) — el terreno rugoso es exigente para ambos mecanismos de arbitraje.

---

## 4. Resultados crudos — comparación lado a lado (trials `SUCCESS`)

### 4.1 familia_c1_terreno_rugoso

**BG/WTA — `noise_level_idx = 0` (n=3):**

| trial | mode | sim_time_s | tresponse | tswitch | roll_rms | pitch_rms | tr |
|---|---|---|---|---|---|---|---|
| 1 | C | 51.11 | 4.5695 | 0.0004 | 1.7889 | 1.3899 | 0.6049 |
| 11 | C | 56.5 | 3.3306 | 1.0003 | 1.8684 | 1.358 | 0.6049 |
| 6 | C | 57.21 | 4.242 | 1.0003 | 1.8237 | 1.2494 | 0.6049 |

**FSM — todos los "niveles" (sin efecto real, n=15):**

| trial | noise_level_idx (etiqueta, sin efecto) | mode | sim_time_s | tresponse | tswitch | roll_rms | pitch_rms | tr |
|---|---|---|---|---|---|---|---|---|
| 1 | 0 | C | 54.2 | 6.045 | 0.0013 | 1.6177 | 1.3463 | 0.0 |
| 11 | 0 | C | 55.71 | 5.7713 | 0.0017 | 1.9808 | 1.2775 | 0.0 |
| 6 | 0 | C | 52.28 | 6.6653 | 0.0028 | 1.6655 | 1.3205 | 0.5797 |
| 12 | 1 | C | 52.05 | 2.5914 | 0.0016 | 1.6342 | 1.2759 | 0.0 |
| 2 | 1 | H | 72.34 | 5.2815 | 0.0008 | 1.8241 | 1.4161 | 0.0 |
| 7 | 1 | C | 57.13 | 4.0613 | 0.0017 | 1.8415 | 1.3703 | 0.6049 |
| 13 | 2 | C | 42.09 | 18.7173 | 0.0008 | 1.5666 | 1.2922 | 0.0 |
| 3 | 2 | C | 53.43 | 5.3023 | 0.0013 | 1.6542 | 1.2305 | 0.0 |
| 8 | 2 | C | 53.76 | 5.5144 | 0.0011 | 1.6908 | 1.614 | 0.0 |
| 14 | 3 | C | 49.64 | 12.3902 | 0.0017 | 1.5446 | 1.4252 | 0.0 |
| 4 | 3 | C | 50.62 | 8.2563 | 0.0017 | 1.9116 | 1.301 | 0.5797 |
| 9 | 3 | C | 61.98 | 5.0017 | 0.0014 | 1.6931 | 1.6169 | 0.0 |
| 10 | 4 | C | 50.06 | 6.5987 | 0.0012 | 1.6521 | 1.2987 | 0.0 |
| 15 | 4 | C | 49.64 | 17.492 | 0.0036 | 1.9952 | 1.4007 | 0.6049 |
| 5 | 4 | C | 76.4 | 2.865 | 0.0014 | 1.8154 | 1.5352 | 0.0 |

**Observaciones directas (c1):**
- `mode` final es `C` en prácticamente todos los trials de ambos sistemas (14 de 15 en FSM, 3 de 3 en BG) — consistente comportamiento de marcha cuadrúpeda en terreno rugoso en ambos mecanismos.
- `roll_rms`/`pitch_rms` están en rangos similares entre ambos sistemas (roll_rms ~1.5–2.0°, pitch_rms ~1.2–1.6° en ambos) — no se observa una diferencia grande de estabilidad postural entre BG y FSM en esta familia, con la evidencia disponible.
- `tswitch` es sistemáticamente ~1.0 s en BG (2 de 3 trials) vs. ~0.001–0.004 s en FSM (los 15 trials) — ver advertencia de la Sección 1.3 antes de interpretar esto como ventaja de la FSM.
- `tresponse` (BG: 3.3–4.6 s) vs. FSM (2.6–18.7 s, rango mucho más amplio) — recordar el desfase de `tchange` (Sección 1.2) antes de comparar directamente.

### 4.2 familia_c2_pendiente

**BG/WTA — `noise_level_idx = 0` (n=3):**

| trial | mode | sim_time_s | tresponse | tswitch | roll_rms | pitch_rms | tr |
|---|---|---|---|---|---|---|---|
| 1 | H | 75.67 | 63.384 | 1.0002 | 1.613 | 3.6763 | 0.6049 |
| 11 | H | 76.68 | 11.8386 | 1.0002 | 1.7643 | 2.3605 | 0.0 |
| 6 | C | 78.7 | 14.8398 | 1.0005 | 16.8761 | 5.7351 | 0.0 |

**FSM — todos los "niveles" (sin efecto real, n=15):**

| trial | noise_level_idx (etiqueta, sin efecto) | mode | sim_time_s | tresponse | tswitch | roll_rms | pitch_rms | tr |
|---|---|---|---|---|---|---|---|---|
| 1 | 0 | H | 60.39 | 15.5874 | 3.1482 | 1.4591 | 1.6534 | 0.6049 |
| 11 | 0 | H | 65.07 | 10.4835 | 0.0024 | 1.4333 | 2.7232 | 0.6049 |
| 6 | 0 | H | 66.06 | 18.2913 | 0.0017 | 1.3574 | 3.1105 | 0.0 |
| 12 | 1 | H | 65.05 | 15.2942 | 0.0013 | 1.3584 | 2.4951 | 0.6049 |
| 2 | 1 | H | 64.26 | 1.3314 | 0.0013 | 1.4252 | 1.6316 | 0.0 |
| 7 | 1 | H | 65.08 | 18.2895 | 0.0013 | 1.9878 | 2.7943 | 0.0 |
| 13 | 2 | H | 65.05 | 15.7515 | 0.0015 | 17.3789 | 8.1197 | 0.0 |
| 3 | 2 | H | 63.11 | 12.4379 | 0.0012 | 1.4107 | 1.5464 | 0.6049 |
| 8 | 2 | H | 65.04 | 17.6913 | 0.0012 | 1.3692 | 2.162 | 0.6049 |
| 14 | 3 | H | 65.05 | 11.5331 | 0.002 | 1.4106 | 1.6679 | 0.6049 |
| 4 | 3 | H | 58.8 | 13.7933 | 0.0012 | 18.8597 | 10.735 | 0.5797 |
| 9 | 3 | H | 65.04 | 10.1821 | 0.0069 | 1.4447 | 1.6028 | 0.0 |
| 10 | 4 | H | 72.1 | 23.9851 | 0.0011 | 27.7427 | 4.7364 | 0.0 |
| 15 | 4 | H | 65.07 | 13.489 | 0.0016 | 1.4468 | 1.7232 | 0.0 |
| 5 | 4 | H | 62.84 | 13.1906 | 0.0011 | 1.3585 | 1.5781 | 0.5797 |

**Observaciones directas (c2):**
- `mode` final es `H` en 14 de 15 trials FSM y en 2 de 3 trials BG — el trial 6 del BG terminó en modo `C`, coincidiendo con su valor atípico de `roll_rms = 16.88` (ya señalado en el informe anterior como dato real, no filtrado).
- **Ambos sistemas presentan valores atípicos de `roll_rms` muy altos en pendiente**: BG trial 6 (`roll_rms=16.88`) y FSM trials 13, 4 y 10 (`roll_rms` = 17.38, 18.86, 27.74 respectivamente). Esto sugiere que la pendiente es un escenario límite para ambos mecanismos de arbitraje, no solo para uno — vale la pena que el equipo revise si estos trials corresponden a eventos de inestabilidad genuina (posible precursor de vuelco no capturado por el `verdict` binario) antes de decidir si se excluyen o se discuten explícitamente.
- El primer trial FSM (`trial=1`, `noise_level_idx` etiqueta 0) es el único con `tswitch=3.1482` — un valor mucho más alto que el resto de trials FSM (~0.001–0.007 s); esto es consistente con ser el primer cambio de modo del trial, posiblemente medido bajo una condición transitoria distinta al resto — dato real, señalado para que el equipo lo revise antes de tratarlo como típico.

---

## 5. Qué se puede afirmar y qué no, con esta evidencia

**Se puede afirmar (con la muestra disponible, respetando las advertencias de la Sección 1):**
- La FSM implementada aísla correctamente el módulo de arbitraje de modo de locomoción (`z[14]`–`z[16]`) sin alterar el circuito de ganglios basales que arbitra R/G/B — el diseño experimental logra el aislamiento de variable independiente que pide el Reviewer #2.
- En terreno rugoso (c1), ambos sistemas mantienen mayoritariamente el modo `C` y muestran rangos de `roll_rms`/`pitch_rms` similares.
- En pendiente (c2), ambos sistemas muestran episodios de inestabilidad postural severa (`roll_rms` > 15°) en al menos un trial cada uno.
- El sistema BG/WTA no registró ningún `FAILURE_TIPOVER` en sus 3 intentos disponibles en `noise_level_idx=0` para c2, mientras que la FSM sí registró 6 sobre 21 intentos — diferencia observada, no concluyente por el desbalance de N.

**No se puede afirmar todavía, con esta evidencia:**
- Que la FSM tenga menor latencia de conmutación de marcha (`Tswitch`) que el BG/WTA — la brecha observada (~1 s) coincide casi exactamente con el offset de +1 s hardcodeado en la fórmula de `Tswitch` del código BG, así que es probable (aunque no 100% seguro sin re-ejecutar) que sea mayormente un artefacto de medición, no una diferencia real de mecanismo.
- Que un sistema sea más rápido en "tiempo de respuesta" (`Tresponse`) que el otro — el origen temporal (`tchange`) difiere en 18 s entre ambos códigos.
- Que el BG/WTA sea más robusto ante vuelcos en pendiente que la FSM — el tamaño de muestra de BG en la condición comparable (`noise_level_idx=0`) es de solo 3 intentos, insuficiente para esa conclusión frente a los 21 de la FSM.

---

## 6. Recomendación concreta para el equipo de escritura

Antes de incluir esta comparación en el paper como respuesta al Reviewer #2:

1. **Prioridad alta:** decidir entre (a) re-ejecutar ambos sistemas con `Upitch`, `tchange` y la fórmula de `Tswitch` igualadas, para tener una comparación limpia; o (b) mantener los datos actuales pero describir explícitamente en el paper las tres diferencias de la Sección 1 como limitación metodológica de este baseline, para no sobre-representar la comparación como "condiciones idénticas" cuando no lo fueron del todo.
2. **Prioridad media:** aumentar las réplicas de BG/WTA en `noise_level_idx=0` para `familia_c2_pendiente` (actualmente solo 3) si se quiere una comparación de tasa de vuelco defendible frente a los 21 intentos disponibles de la FSM.
3. **Prioridad media:** revisar los trials con `roll_rms` atípicamente alto (BG c2 trial 6; FSM c2 trials 13/4/10) para confirmar si son eventos de inestabilidad genuina relevantes para la discusión, o casos a excluir con justificación explícita.
4. Si se opta por (b) en el punto 1, la figura/tabla comparativa del paper debe presentar `Tswitch` y `Tresponse` con una nota al pie que aclare la diferencia de fórmula/offset entre ambos sistemas, en vez de presentarlos como directamente comparables.

**Reviewer comment addressed:** #2.2 (FSM baseline). Cobertura parcial: la comparación arquitectónica y los datos de terreno están completos, pero la comparación cuantitativa fina (`Tswitch`, `Tresponse`) requiere la corrección de la Sección 1 antes de poder presentarse como evidencia sólida.