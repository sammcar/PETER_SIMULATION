# Plan de Revisión — "Multimodal robot with bio-inspired gait arbitration across simulated and real environments"

**Basado en:** auditoría completa del PDF (88 páginas, V2, 05/06/2026) + comentarios de Reviewer #1, #2 y #3 (segunda ronda).

**Nota de contexto importante:** el manuscrito ya incluye una Sección 3.1.7 ("Quantitative Stability Analysis of Locomotion Mode Transitions") cuyo primer párrafo dice textualmente *"To address the reviewer's request for systematic quantitative metrics on mode-switching behaviour..."* Esto confirma que ya hubo una ronda previa de revisión cuantitativa (probablemente en respuesta a Reviewer #2/#3 de la ronda 1). El paper actual, por tanto, ya es bastante denso en cifras — lo cual es precisamente la raíz de la queja de verbosidad del Reviewer #2.

---

## STEP 1 — AUDITORÍA DEL PAPER (mapa de elementos existentes)

| Elemento | Ubicación exacta |
|---|---|
| Arquitectura general del controlador (4 módulos) | Sec. 2.3, párrafo introductorio |
| Módulo sensorial de obstáculos (anillo LiDAR de 16 unidades) | Sec. 2.3.1, Fig. 5–6, Ec. (1)–(9), Tabla 1 |
| Módulo de ganglios basales (STN/GPe/GPi/STR, WTA por disinhibición) | Sec. 2.3.2, Fig. 7, Ec. (10)–(13), Tabla 2 |
| Mecanismo WTA | Implementado dentro de 2.3.2; descrito conceptualmente en Introducción (disinhibición) |
| Conexiones inhibitorias | Ec. (10)–(13) (Gpi inhibe STN, Gpe inhibe Gpi, etc.) |
| Módulo de locomoción (dirección de movimiento) | Sec. 2.3.3, Fig. 8–9, Ec. (14)–(25), Tabla 3 |
| Módulo de decisión de marcha (gait-decision) + MLP | Sec. 2.3.4 (implícita, dentro de 2.3.3 extendida), Fig. 10–12, Ec. (26)–(37), Tabla 4–5 |
| Modalidades sensoriales | Cámara (RGB, umbral de color), LiDAR (anillo de 16 neuronas), IMU (roll, pitch, σ_az) — Sec. 2.1, 2.3.1, 2.3.4 |
| Clasificador MLP de terreno | Sec. 2.3.4, Fig. 10–12; evaluación cuantitativa en Sec. 3.2.3, Fig. 41–46 |
| Modos de locomoción | Cuadrupedal (C), Rodante diferencial (H), Omnidireccional (X) — Sec. 2.1 |
| Experimentos de simulación | Sec. 3.1 completa (single-stimulus 3.1.1–3.1.2; multi-stimulus 3.1.3–3.1.4; topografía variable 3.1.5–3.1.6; transiciones de modo 3.1.7) |
| Experimentos físicos | Sec. 3.2 completa + Sec. 3.3 (perfilado energético y de latencia) |
| Escenarios de inspección | Solo mencionados como motivación (Introducción, Conclusiones); **no hay un experimento dedicado con métricas de inspección** |
| Experimentos de ruido/robustez | Ruido gaussiano sintético σ∈[0,4] inyectado en el seguimiento visual, Tabla 6 (Sec. 3.1.2/3.1.4) — **limitado a un tipo de perturbación, no cubre IMU/LiDAR/cámara por separado ni la cadena completa** |
| Experimentos de energía/cómputo | Fig. 46 (latencia de inferencia del MLP por muestra); Fig. 58 (energía de las **transiciones** C↔H, no de los tres modos en estado estacionario) |
| Limitaciones actuales | Sec. 3.4 ("Limitations of the Present Work") **y** repetidas parcialmente en Sec. 4 (Conclusiones, párrafo "Several limitations must be acknowledged...") — **duplicación real** |
| Revisión de literatura | Integrada en la Introducción (prosa, sin tabla comparativa ni clasificación horizontal) |
| Figuras totales | 59 figuras (Fig. 1–59) |
| Tablas totales | 7 tablas (todas de hiperparámetros excepto Tabla 6 y 7, que son de resultados) |
| Métricas de evaluación actuales | Tiempo de misión, latencia de decisión (~47 ms), Roll/Pitch RMS, Tipover Risk (TR), Normalised Stability Margin (SM), tasa de éxito (100%), latencia de conmutación (Tswitch), varianza de disparo neuronal (Var(z)) |

**Conclusión de la auditoría:** el paper tiene una base experimental sólida en estabilidad, latencia y repetibilidad, pero carece completamente de (a) baselines comparativos (FSM, threshold), (b) ablación del propio módulo de ganglios basales/WTA, (c) robustez multisensor sistemática, y (d) métricas orientadas a tarea de inspección. Estos son exactamente los puntos que pide Reviewer #3.

---

## STEP 2 — TABLA DE GAP ANALYSIS (Reviewer por Reviewer)

| Reviewer | Comentario | Cobertura actual | Estado | Evidencia faltante | Acción requerida | Prioridad |
|---|---|---|---|---|---|---|
| #1 | Sin comentarios adicionales | — | **FULLY ADDRESSED** | Ninguna | Ninguna | — |
| #2.1 | Reducir verbosidad, raster plots y curvas de estabilidad duplicadas, unificar definiciones de métricas, acortar sección del almacén | El paper repite definiciones de SM/TR en 3.1.6 y 3.1.7; repite estructura de figura ECDF+neural+RMS 4 veces (Fig. 54–57); repite limitaciones en 3.4 y en Conclusiones | **PARTIALLY ADDRESSED / EDITORIAL CHANGE ONLY** | No falta evidencia; es un problema de presentación | Consolidar (ver Step 6) | **CRITICAL** |
| #2.2 | Baseline FSM bajo mismas condiciones | No existe ningún baseline comparativo en todo el manuscrito | **NOT ADDRESSED** | Falta 100%: no hay FSM implementada ni comparada | Implementar y ejecutar Experimento A | **CRITICAL** |
| #2.3 | Consolidar limitaciones en una subsección, eliminar discusiones dispersas | Existe Sec. 3.4 dedicada, pero Sec. 4 repite limitaciones parcialmente | **PARTIALLY ADDRESSED** | No falta evidencia nueva; falta depuración editorial | Fusionar y eliminar duplicado en Conclusiones | **HIGH** |
| #2.4a | Energía a largo plazo de los 3 modos de locomoción | Solo existe energía de **transiciones** (Fig. 58: C→H y H→C); no hay medición en estado estacionario de C, H y X por separado | **PARTIALLY ADDRESSED** | Falta consumo estacionario (W, mAh, J/m) de cada uno de los 3 modos | Ejecutar Experimento F | **HIGH** |
| #2.4b | Overhead computacional embebido del MLP | Fig. 46 da latencia de inferencia por muestra (1.64 µs); no hay CPU%, RAM, flash, ni comparación MLP-on vs MLP-off en el ESP32-S3 real | **PARTIALLY ADDRESSED** | Falta footprint de memoria/CPU en hardware objetivo | Ejecutar Experimento G | **HIGH** |
| #3.1 | Falta análisis de ablación de la red de arbitración de ganglios basales | No existe ninguna ablación del módulo BG/WTA. (Existe una ablación de **arquitectura del MLP** — Fig. 44 — que es un módulo distinto y no debe confundirse con esto) | **NOT ADDRESSED** | Falta 100% | Ejecutar Experimento B | **CRITICAL** |
| #3.2 | Faltan pruebas comparativas que remuevan capas/inhibición para probar la necesidad del WTA frente a lógica de umbral simple | No existe comparación WTA vs. umbral simple en ningún punto del texto | **NOT ADDRESSED** | Falta 100% | Ejecutar Experimentos B y C | **CRITICAL** |
| #3.3 | Restricciones de servos de bajo torque y defectos de marcha en lazo abierto solo mencionados, sin estrategia de compensación | Reconocido explícitamente en Sec. 3.4, sin propuesta de compensación ni experimento correctivo | **PARTIALLY ADDRESSED** (como reconocimiento) / **NOT ADDRESSED** (como solución) | Falta decidir si se implementa compensación real o se refuerza como limitación justificada | Ver Step 4 (probablemente opción C: discusión, no rediseño) | **MEDIUM** |
| #3.4 | Sin experimentos de robustez a ruido (deriva IMU, ruido de distancia LiDAR, distorsión de iluminación de cámara) en el pipeline completo | Existe ruido gaussiano σ∈[0,4] pero aplicado únicamente al seguimiento visual/estímulo, no a IMU ni LiDAR, y no se mide su propagación hasta la decisión de marcha | **PARTIALLY ADDRESSED** | Falta perturbar IMU y LiDAR específicamente y medir efecto en selección de marcha, no solo en cinemática | Ejecutar Experimento D | **CRITICAL** |
| #3.5 | Faltan indicadores orientados a inspección (tracking accuracy, coverage rate, re-planificación tras bloqueo) | Solo existe comportamiento de "approach" con señal de parada (X17); no hay tracking cuantitativo, cobertura, ni replanificación tras bloqueo | **NOT ADDRESSED** | Falta 100% | Ejecutar Experimento E | **HIGH** |
| #3.6 | Falta revisión sistemática de literatura bio-inspirada (BG, CPG, RL) con tablas de clasificación comparativa | Existe revisión narrativa extensa en la Introducción, pero sin tabla horizontal ni tabla comparativa de innovación | **PARTIALLY ADDRESSED** | Falta forma tabular; el contenido narrativo de base ya existe y es reutilizable | Ejecutar Step 5 | **HIGH** |

---

## STEP 3 — NEW EXPERIMENTS REQUIRED (resumen ejecutivo)

Se requieren **cinco nuevos experimentos** (A, B, D, E, F/G combinados) para cerrar los comentarios NOT ADDRESSED / PARTIALLY ADDRESSED con déficit real de evidencia. No se inventará ningún resultado; cada experimento se describe con su diseño completo para que el equipo lo ejecute.

---

## EXPERIMENTO A — BASELINE FSM (Reviewer #2.2)

**Objetivo:** demostrar que la arbitración BG/WTA aporta ventajas medibles frente a un selector de marcha determinista clásico, bajo condiciones idénticas.

**Pregunta científica:** ¿la arquitectura BG/WTA reduce latencia de conmutación, oscilaciones de marcha (dithering) y riesgo de vuelco frente a una máquina de estados finita con los mismos umbrales sensoriales?

**Diseño de la FSM (propuesta, para que sea un baseline justo y no débil artificialmente):**
- Estados: `Quadrupedal (C)`, `Rolling (H)`, `Omnidirectional (X)`.
- Transiciones gobernadas por **los mismos umbrales ya calibrados en el paper**: `U_σaz`, `U_Roll`, `U_Pitch`, `U_N0`, `U_N1` (Tabla 3 y 5), y la misma salida del MLP de terreno (N0, N1, N2).
- Reglas de prioridad explícitas (para evitar conflictos, ej.: obstáculo > aversivo > apetitivo > terreno), documentadas en una tabla de transición de estados.
- Debounce/histéresis configurable para que la FSM no quede en desventaja injusta frente al WTA (que ya tiene supresión inhibitoria implícita); de lo contrario la comparación estaría sesgada.
- Debe usar exactamente los mismos sensores (cámara, LiDAR, IMU, MLP), el mismo hardware/simulación, y los mismos escenarios (single-stimulus, multi-stimulus, topografía variable) ya presentes en el paper.
- La única variable independiente debe ser el **mecanismo de arbitración** (BG/WTA vs. FSM).

**Variables dependientes / métricas (ordenadas por relevancia para demostrar la ventaja del BG):**
1. **Tasa de selecciones de marcha incorrectas** — la más importante: mide directamente calidad de arbitración.
2. **Frecuencia de conmutación / dithering (oscilación entre modos)** — clave porque la literatura BG (Girard et al., ya citado en el paper) predice justamente que BG evita esto y un WTA/FSM simple no.
3. **Latencia de conmutación (Tswitch)** — comparable directamente con los datos ya existentes (Tabla 6, Fig. 33b, 54–57a).
4. Tipover Risk (TR) y Roll/Pitch RMS bajo conflicto multiestímulo.
5. Tiempo de misión y tasa de éxito.
6. Energía consumida (si Experimento F ya está implementado, reutilizar la instrumentación).

**Condiciones idénticas obligatorias:** mismo terreno (los tres del paper: plano/grava/pendiente + Perlin), mismos estímulos apetitivo/aversivo, mismo número de ensayos que las réplicas ya reportadas (≥15 en simulación, ≥3 en físico, para mantener comparabilidad estadística).

**Reviewer atendido:** #2.2 (crítico), refuerza #3.2.

---

## EXPERIMENTO B — ABLACIÓN DE GANGLIOS BASALES (Reviewer #3.1, #3.2)

**Objetivo:** aislar la contribución causal de cada componente del módulo BG (STN, GPe, GPi, inhibición cruzada) a la calidad de arbitración.

**Selección mínima de ablaciones (no todas las combinaciones posibles, solo las científicamente necesarias):**

| Modelo | BG completo | WTA (competencia) | Inhibición (Gpe→Gpi, Gpi→STN) | Propósito |
|---|---|---|---|---|
| M0 — Completo (propuesto) | ✔ | ✔ | ✔ | Referencia |
| M1 — Sin inhibición cruzada (Ec. 10–13 con términos de inhibición anulados) | ✔ | parcial | ✘ | Prueba si la inhibición es necesaria para evitar selección múltiple/conflicto |
| M2 — Sin STN (entrada directa Stimulus→Gpi) | ✘ (parcial) | ✔ | ✔ | Prueba el rol del STN como nodo de relevancia |
| M3 — WTA reemplazado por lógica de umbral simple (`argmax(Stimulus_i) si Stimulus_i > U`) | ✘ | ✘ | ✘ | Es el "simple threshold decision logic" que Reviewer #3 pide explícitamente comparar (ver Experimento C) |
| M4 — FSM (referida de Experimento A) | ✘ | ✘ | ✘ | Baseline externo de referencia |

No se requiere una ablación exhaustiva de GPe/GPi por separado si M1 y M2 ya aíslan el efecto de inhibición vs. relevancia — agregar más variantes sin una hipótesis específica dispersaría el experimento sin aportar evidencia adicional.

**Métricas:** las mismas que en Experimento A (selección incorrecta, dithering, Tswitch, TR, Roll/Pitch RMS), evaluadas bajo escenarios de **conflicto multiestímulo** (el escenario de Sec. 3.1.3/3.2.2, que ya existe y puede reutilizarse como protocolo, no como resultado).

**Estadística:** comparación entre M0–M4 con ANOVA o Kruskal-Wallis (según distribución) sobre cada métrica, con al menos 15 ensayos por condición en simulación (mismo N que 3.1.6–3.1.7) y 3 réplicas físicas si el tiempo lo permite (opcional, ver Step 10).

**Figura/tabla esperada:** una tabla tipo "Modelo × Métrica" + una figura de cajas (boxplot) comparando M0–M4.

**Reviewer atendido:** #3.1, #3.2 (ambos CRITICAL).

---

## EXPERIMENTO C — WTA vs. LÓGICA DE UMBRAL (Reviewer #3.2)

Este experimento es en la práctica la comparación M0 vs. M3 vs. M4 del Experimento B, presentada como un análisis independiente porque responde a una pregunta distinta y más específica que pide el Reviewer #3: *¿aporta la competencia WTA algo que un umbral simple no aporte?*

**Foco de análisis (specífico, no repetir todo Experimento B):**
- Comandos conflictivos emitidos simultáneamente (contar instantes en que dos o más canales superan el umbral de activación motora al mismo tiempo).
- Transiciones de marcha inestables (definidas como transición seguida de TR > 0.7 en menos de 1 s).
- Robustez ante empates de estímulo (dos estímulos de intensidad casi idéntica) — este es el caso de uso clásico donde WTA supera a un umbral fijo, y es fácilmente reproducible controlando la distancia relativa de los estímulos apetitivo/aversivo.

**Reviewer atendido:** #3.2 (CRITICAL). Comparte datos con Experimento B — no dupliques la ejecución, solo el análisis.

---

## EXPERIMENTO D — ROBUSTEZ SENSORIAL DE EXTREMO A EXTREMO (Reviewer #3.4)

**Objetivo:** demostrar que el pipeline completo (sensor → percepción → arbitración → marcha → tarea) sigue siendo funcional bajo perturbaciones realistas, no solo evaluar precisión de percepción aislada.

**Perturbaciones a introducir (niveles sugeridos, ajustables según el hardware real):**

| Sensor | Perturbación | Niveles sugeridos |
|---|---|---|
| IMU | Deriva de bias en roll/pitch | 0°, ±1°, ±3°, ±5° de offset acumulado |
| IMU | Ruido gaussiano adicional en σ_az | σ = 0, 0.5, 1.0, 2.0 (m/s²) |
| LiDAR | Ruido de distancia | 0%, 5%, 10%, 20% del rango medido |
| Cámara | Distorsión de iluminación | Iluminación nominal, −30% brillo, +30% brillo, sombra parcial |

**Cadena de propagación a medir (no limitarse a precisión de clasificación):**
`Sensor → percepción (ring/MLP) → representación neuronal → arbitración BG → decisión de marcha → locomoción → resultado de tarea`

**Métricas por nivel de perturbación:**
- Precisión de percepción (ya existe el marco de la Tabla 6 para reutilizar como plantilla de reporte).
- Precisión de selección de marcha (comparar contra el "ground truth" de terreno/estímulo).
- Tasa de transiciones falsas de marcha.
- Latencia de conmutación (Tswitch).
- Tipover Risk / Roll-Pitch RMS.
- Tasa de colisión.
- Éxito de misión y tiempo de finalización.

**Diseño experimental:** factorial reducido — perturbar **un sensor a la vez** (para atribución causal) más **una condición combinada worst-case** (los tres sensores degradados simultáneamente), evitando una matriz factorial completa que sería innecesariamente costosa.

**Presentación sugerida:** una figura compuesta de 4 subpaneles (uno por tipo de perturbación) con el eje X = nivel de ruido y el eje Y = precisión de selección de marcha / tasa de éxito, más una tabla resumen tipo Tabla 6 extendida.

**Reutilización:** el protocolo de inyección de ruido gaussiano ya usado en 3.1.2/3.1.4 (σ∈[0,4]) puede adaptarse directamente para la perturbación de cámara; no es necesario reconstruirlo desde cero.

**Reviewer atendido:** #3.4 (CRITICAL).

---

## EXPERIMENTO E — TAREA DE INSPECCIÓN (Reviewer #3.5)

**Objetivo:** validar el sistema en una tarea con métricas de ingeniería de inspección, no solo comportamiento apetitivo genérico.

**Elementos mínimos factibles con el hardware/setup actual:**
- Detección de objetivo (ya existe vía estímulo apetitivo/color).
- Aproximación al objetivo con parada a distancia de inspección (ya existe: neurona X17).
- **Nuevo:** bloqueo por obstáculo entre el robot y el objetivo, forzando re-planificación de trayectoria (no existe actualmente — el escenario multi-estímulo actual coloca obstáculo, aversivo y apetitivo en posiciones fijas no bloqueantes).
- **Nuevo:** reencuentro del objetivo tras el rodeo del obstáculo.

**Indicadores cuantitativos — esenciales vs. opcionales:**

*Esenciales (mínimo defendible):*
- Precisión de aproximación (distancia final al objetivo, error de posición).
- Tasa de éxito de evasión de obstáculo.
- Tasa de reencuentro del objetivo tras bloqueo.
- Tiempo total de la tarea.

*Opcionales (mejoran el paper pero no son indispensables para responder al Reviewer):*
- Cobertura (coverage rate) — solo tiene sentido si se define un área de inspección explícita, lo cual no existe actualmente en el diseño experimental; requeriría rediseñar el escenario.
- Error de orientación respecto al objetivo.
- Tiempo/tasa de éxito de re-planificación como métrica separada del tiempo total.

**Recomendación:** ejecutar la versión "esencial" primero (mínimo defendible para responder a Reviewer #3.5); la cobertura y el error de orientación se dejan como extensión opcional si el tiempo de revisión lo permite.

**Reviewer atendido:** #3.5 (HIGH).

---

## EXPERIMENTO F — CONSUMO ENERGÉTICO A LARGO PLAZO (Reviewer #2.4a)

**Objetivo:** reportar el consumo energético **en estado estacionario** de cada uno de los tres modos de locomoción (no solo el de las transiciones, que ya está en Fig. 58).

**Diseño:**
- Duración: recorrido de distancia fija (ej. 2 m) o tiempo fijo (ej. 60 s) por modo, en terreno plano controlado — para que los tres modos sean comparables.
- Repeticiones: mínimo 5 réplicas por modo (consistente con el número de réplicas ya usado en otras secciones del paper, aunque puede alinearse a 3 si se homogeniza con Sec. 3.3).
- Instrumentación: reutilizar el sensor INA ya usado para Fig. 58 (mismo hardware, sin necesidad de nueva instrumentación).
- Medir: voltaje, corriente, potencia promedio (W), energía total (J o Wh), y **energía normalizada por distancia recorrida (J/m)** — esto es crítico porque los tres modos tienen velocidades distintas, y comparar solo potencia instantánea sería engañoso.

**Cómo hacer la comparación significativa pese a velocidades distintas:** reportar tanto potencia media (W) como energía por metro (J/m); el segundo es el indicador relevante para "eficiencia energética" y es el que debe destacarse en la discusión, ya que es agnóstico a la velocidad de cada modo.

**Figura/tabla esperada:** tabla con columnas [Modo, Potencia media (W), Energía total (J), Distancia (m), Energía/m (J/m)] + una figura de barras con barras de error.

**Reviewer atendido:** #2.4a (HIGH).

---

## EXPERIMENTO G — OVERHEAD COMPUTACIONAL EMBEBIDO DEL MLP (Reviewer #2.4b)

**Auditoría de lo ya existente:** Fig. 46 reporta la latencia de inferencia por muestra del MLP (1.64 µs) comparada con otros clasificadores. **Esto NO es suficiente** para responder al Reviewer, porque no dice nada sobre uso de CPU, RAM o tamaño de modelo en el ESP32-S3 real, que es el punto central de la pregunta ("embedded computing overhead... for onboard deployment analysis").

**Mediciones adicionales necesarias:**
- Latencia de inferencia **medida en el ESP32-S3 real** (si Fig. 46 se generó off-device, debe repetirse on-device; si ya es on-device, aclararlo explícitamente en el texto, porque actualmente no lo especifica).
- Uso de CPU (%) durante inferencia, relativo al lazo de control completo.
- RAM ocupada por el modelo y sus buffers de entrada (recordar: entrada de 135 valores por muestra).
- Tamaño en flash del modelo serializado (KB).
- Frecuencia de inferencia sostenida (Hz) compatible con el lazo de control a 3.3 kHz ya mencionado en el paper (Sec. 3.1.6).
- Comparación **MLP habilitado vs. deshabilitado**: overhead relativo de CPU/RAM cuando el MLP está activo frente a cuando el sistema opera solo con lógica de umbral IMU directa (esto también puede reutilizar datos del Experimento C si se define M3 como "MLP deshabilitado").

**Qué constituye evidencia suficiente:** una tabla simple [Métrica, Valor, % del presupuesto disponible del ESP32-S3] es suficiente — no se requiere un profiling exhaustivo, solo demostrar que el modelo cabe cómodamente en el presupuesto de memoria/cómputo del microcontrolador.

**Reviewer atendido:** #2.4b (HIGH).

---

## STEP 4 — LIMITACIONES DE CONTROL (servos de bajo torque, marcha en lazo abierto)

**Ya reconocido en el paper:** Sec. 3.4 ya declara explícitamente que el uso de servos de bajo torque y marcha en lazo abierto es una limitación de diseño de bajo costo, no del controlador neuronal.

**Recomendación (mínimo científicamente creíble):**

- **A. Experimentos realmente necesarios:** **ninguno adicional es estrictamente obligatorio.** El reviewer pide "supplementary improved control strategies or compensating algorithms", pero no exige que se implementen — exige que la limitación no quede "solo brevemente mencionada". Esto es, en el fondo, una petición de profundidad de discusión, no necesariamente de nuevo hardware/control.
- **B. Compensaciones de control realmente implementables en el plazo de revisión (si el equipo decide ir más allá del mínimo):** un lazo de retroalimentación simple de postura (ej., ajuste de ganancia de paso según error de pitch medido) sobre el gait ya existente, sin rediseñar toda la arquitectura. Esto es opcional y de alto costo relativo — recomendable solo si hay margen de tiempo (ver Step 10).
- **C. Declaración de discusión/trabajo futuro (suficiente si no se implementa control nuevo):** expandir el párrafo actual de Sec. 3.4 con: (i) una cuantificación explícita del déficit de torque de los MG996R frente al par requerido estimado para el peso del robot en pendiente máxima probada (25°/45° ya mencionados en Sec. 2.2), y (ii) una descripción concreta —aunque no implementada— de qué estrategia de control cerrado se propondría (ej. control de impedancia en la fase de apoyo), dejándolo como línea de trabajo futuro con mayor especificidad que la actual.

**No se recomienda rediseñar el robot.** El Reviewer no lo exige literalmente; pide que la limitación deje de estar "solo mencionada", lo cual se resuelve con B (opcional) o C (mínimo, suficiente).

**Reviewer atendido:** #3.3 (MEDIUM).

---

## STEP 5 — REVISIÓN DE LITERATURA (Reviewer #3.6)

**Contenido reutilizable:** la Introducción ya contiene material narrativo sustancial sobre ANYmal, ANYmal-on-wheels, CENTAURO, Ascento, el modelo GPR de Gurney et al., Prescott et al., Girard et al., Kamali Sarvestani et al., y los modelos de tres vías de Baston & Ursino. Esto es la base de contenido; falta **tabularlo**.

**TABLA 1 — Clasificación horizontal de métodos de selección de acción/arbitración de marcha bio-inspirados:**

Columnas sugeridas: Método/Referencia | Inspiración biológica | Mecanismo de selección de acción | Mecanismo de generación de marcha | ¿Requiere aprendizaje? | Modalidades sensoriales | Validación en robot real | Validación en simulación | Mecanismo de conmutación | Interpretabilidad | Requisitos computacionales | Evaluación de robustez | Capacidad de inspección | Limitaciones

Filas mínimas a incluir (todas ya citadas en el paper, solo requiere extraer y tabular, marcado [búsqueda de literatura] donde falte un dato puntual no mencionado en el texto actual):
- Bjelonic et al. (ANYmal-on-wheels, WBC) [8]
- Bjelonic et al. (MPC extendido) [9,10]
- Lee et al. (RL end-to-end) [11]
- CENTAURO [12,13]
- Ascento [14]
- Prescott et al. (GPR embebido en robot) [23]
- Girard et al. (BG vs. WTA clásico) [24]
- Kamali Sarvestani et al. (base del presente trabajo) [25]
- Baston & Ursino (3-vías Go/NoGo/hiperdirecta) [26]
- **Presente trabajo**

**TABLA 2 — Comparación directa con métodos representativos:** subconjunto de la Tabla 1 (los 3–4 más cercanos: ANYmal-on-wheels, Lee et al. RL, Kamali Sarvestani et al., presente trabajo), con foco en remarcar la novedad ya reclamada en la Introducción: arbitración neurobiológicamente interpretable aplicada específicamente a switching de locomoción híbrida (a diferencia de RL end-to-end o lógica de umbral).

**Nota:** cualquier dato de la tabla que no esté ya verificado en las referencias citadas debe marcarse como tarea de búsqueda bibliográfica adicional antes de publicar — no se debe completar con valores inventados.

**Ubicación en el manuscrito:** se recomienda crear una subsección "2.0 Related Work" (ver Step 8) separada de la Introducción, para alojar las tablas sin sobrecargar la Sección 1.

**Reviewer atendido:** #3.6 (HIGH).

---

## STEP 6 — DEPURACIÓN DEL MANUSCRITO (Reviewer #2.1)

| Elemento | Acción | Justificación |
|---|---|---|
| Fig. 19, 20, 24, 38, 39 (raster plots de activaciones neuronales, simulación vs. físico, single vs. multi-stimulus) | **MERGE** | Repiten el mismo tipo de gráfico (raster de activación) 5 veces con estructura idéntica; consolidar en una figura multipanel por escenario (sim vs. físico) en lugar de figuras separadas |
| Fig. 54(b), 55(b), 56(b), 57(b) (dinámica neuronal Var(z) en 4 escenarios físicos) | **MERGE** | Mismo tipo de panel (b) repetido 4 veces con estructura idéntica; consolidar en una figura de 4 subpaneles (a,b,c,d) en vez de 4 figuras completas |
| Fig. 54(c), 55(c), 56(c), 57(c) (Roll/Pitch RMS en 4 escenarios físicos) | **MERGE** | Misma razón; consolidar como panel adicional en la figura fusionada anterior |
| Fig. 54(a), 55(a), 56(a), 57(a) (ECDF de latencia en 4 escenarios) | **MERGE** | Misma razón; unificar en una sola figura con 4 curvas ECDF superpuestas (más informativa y más corta que 4 figuras separadas) |
| Definiciones de SM y TR (repetidas en Sec. 3.1.6 y 3.1.7) | **MERGE** | Definir una sola vez (recomendado: al inicio de 3.1.6) y referenciar en 3.1.7 sin repetir la fórmula |
| Descripción del "symbolic warehouse simulation" (Fig. 17, con descargo de responsabilidad repetido en el propio texto de la figura y en Sec. 3.4) | **MERGE / SHORTEN** | El descargo de que el almacén no es una réplica certificada aparece tanto en el pie de figura como en el cuerpo del texto (líneas ~1007–1016) y de nuevo en Sec. 3.4 (líneas ~1860–1872); mantener solo una mención completa, la otra puede reducirse a una remisión cruzada |
| Limitaciones en Sec. 3.4 vs. párrafo de limitaciones en Sec. 4 (Conclusiones) | **MOVE TO SUPPLEMENTARY (dentro del propio manuscrito) → consolidar todo en la nueva subsección "Limitations and Deployment Considerations"** | Ver Step 7 |
| Fig. 27–32 (topografía variable, simulación) vs. Fig. 47–53 (topografía variable, físico) | **KEEP** | Aunque son estructuralmente paralelas, contienen evidencia única (simulación vs. hardware real) que no debe eliminarse — es la validación cruzada central del paper |
| Fig. 33 vs. Fig. 59 (perfiles cuantitativos de estabilidad/latencia, simulación vs. físico) | **KEEP** | Igual razón — evidencia única por dominio (sim/físico), central para el argumento del paper |
| Tabla 6 (métricas consolidadas de simulación) | **KEEP, pero unificar formato con la nueva Tabla de robustez (Experimento D)** | Evita crear un formato de tabla nuevo cuando ya existe uno reutilizable |

**Plan de consolidación de figuras propuesto (ejemplo concreto):**

```
Figuras actuales 54(a) + 55(a) + 56(a) + 57(a)  →  Nueva Figura "ECDF consolidado" con panel único, 4 curvas
Figuras actuales 54(b) + 55(b) + 56(b) + 57(b)  →  Nueva Figura "Dinámica neuronal consolidada", panel (a)-(d)
Figuras actuales 54(c) + 55(c) + 56(c) + 57(c)  →  Nueva Figura "Roll/Pitch RMS consolidado", panel (a)-(d)
```

Esto reduce 4 figuras completas (54–57) a 3 figuras compactas sin perder ningún dato, cumpliendo directamente la petición de Reviewer #2.1.

**No se debe eliminar** ninguna figura que contenga la única evidencia física de un escenario (p. ej. Fig. 47–53, Fig. 34–36), ya que hacerlo debilitaría la validación físico-simulación que es el eje central del paper.

---

## STEP 7 — SUBSECCIÓN DE LIMITACIONES CONSOLIDADA

**Nueva subsección propuesta: "Limitations and Deployment Considerations"** (reemplaza y fusiona Sec. 3.4 + el párrafo de limitaciones de Sec. 4).

Categorías a incluir (todas ya existen disperso en el texto actual, solo requieren reubicación):

| Categoría | Contenido ya existente a mover | Ubicación actual |
|---|---|---|
| Limitaciones de actuación | Servos de bajo torque, marcha en lazo abierto | Sec. 3.4 |
| Limitaciones de percepción | Segmentación por color bajo iluminación controlada, sin evaluación bajo iluminación variable | Sec. 4 (párrafo de limitaciones) |
| Limitaciones de fusión sensorial | Ausencia de un framework formal de fusión (Kalman, Bayesiano); la fusión emerge implícitamente vía WTA | Sec. 4 |
| Limitaciones ambientales | Validación restringida a laboratorio estructurado, sin exteriores ni obstáculos dinámicos | Sec. 3.4 |
| Limitaciones de inspección | Sin indicadores de inspección formales (referenciar Experimento E como respuesta parcial, si se implementa) | Nueva, resultado del Step 3 |
| Limitaciones computacionales | Referenciar resultados del Experimento G una vez ejecutado | Nueva |
| Parámetros ajustados manualmente | Sin optimización adaptativa de umbrales | Sec. 4 |
| Falta de validación exterior a largo plazo | Mencionado en Conclusiones como trabajo futuro | Sec. 4 |

**Qué eliminar de Resultados tras mover:** ninguna limitación aparece actualmente incrustada dentro de la Sección 3 de Resultados (los autores ya fueron disciplinados en eso); el problema real de dispersión está entre Sec. 3.4 y Sec. 4, no dentro de Resultados. Por tanto, la tarea es más simple de lo que el comentario del Reviewer podría sugerir: **fusionar dos bloques, no rastrear decenas de menciones dispersas.**

---

## STEP 8 — ESTRUCTURA PROPUESTA DEL NUEVO MANUSCRITO

```
1. Introduction (recortada — mover clasificación de literatura a Sec. 2)
2. Related Work                                    [NUEVO — Tablas 1 y 2 del Step 5]
3. System Architecture / Methodology (= actual Sec. 2, sin cambios mayores)
   3.1 Platform Design
   3.2 Gazebo Simulation
   3.3 Neuronal Control System
   3.4 Physical Platform Construction and Experimental Setup
4. Experimental Setup                              [nuevo, breve, resume terrenos/hardware/réplicas comunes a todos los experimentos, evita repetirlo en cada subsección de Resultados]
5. Results
   5.1 Baseline and Ablation Studies                [NUEVO — Experimentos A, B, C]
   5.2 Robustness Evaluation                        [NUEVO — Experimento D]
   5.3 Locomotion Performance (simulation + physical, consolidado — Steps 6)
   5.4 Inspection-Oriented Evaluation                [NUEVO — Experimento E]
   5.5 Energy and Computational Analysis             [Experimentos F, G + Fig. 46/58 ya existentes]
6. Discussion
7. Limitations and Deployment Considerations        [Step 7]
8. Conclusions
```

**Nota:** esta estructura no es obligatoria tal cual — es una propuesta; el equipo puede optar por mantener "Simulation Results" / "Physical Implementation Results" como sub-bloques dentro de 5.3 en lugar de fusionarlos, si prefieren preservar la separación sim/físico que ya funciona bien en el resto del paper.

---

## STEP 9 — LISTA DE FIGURAS Y TABLAS

**Mantener sin cambios:** Fig. 1–17, 27–36, 41–53 (contienen evidencia única de diseño, simulación de topografía y validación física).

**Fusionar:** Fig. 19+20+24+38+39 → 1–2 figuras multipanel; Fig. 54–57 → 3 figuras consolidadas (ver Step 6).

**Nuevas figuras/tablas requeridas:**

| # | Título propuesto | Propósito | Datos requeridos | Reviewer |
|---|---|---|---|---|
| Nueva Fig. | Comparación BG/WTA vs. FSM vs. umbral simple (boxplots multi-métrica) | Resultado de Experimentos A+B+C | Ejecución de Exp. A–C | #2.2, #3.1, #3.2 |
| Nueva Tabla | Modelo × Métrica de ablación BG (M0–M4) | Resultado tabular de Experimento B | Ejecución de Exp. B | #3.1, #3.2 |
| Nueva Fig. | Robustez multisensor (4 subpaneles: IMU drift, IMU noise, LiDAR noise, cámara) | Resultado de Experimento D | Ejecución de Exp. D | #3.4 |
| Nueva Tabla | Métricas de tarea de inspección | Resultado de Experimento E | Ejecución de Exp. E | #3.5 |
| Nueva Tabla/Fig. | Energía en estado estacionario por modo (C, H, X) | Resultado de Experimento F | Ejecución de Exp. F | #2.4a |
| Nueva Tabla | Overhead computacional embebido del MLP (CPU/RAM/flash) | Resultado de Experimento G | Ejecución de Exp. G | #2.4b |
| Tabla 1 (nueva) | Clasificación horizontal de arbitración bio-inspirada | Literatura ya citada, tabulada | Extracción de Introducción actual + búsqueda puntual | #3.6 |
| Tabla 2 (nueva) | Comparación directa con métodos representativos | Subconjunto de Tabla 1 | Igual que arriba | #3.6 |

---

## STEP 10 — PRIORIDAD Y CARGA DE TRABAJO

| Tarea | Prioridad | Trabajo experimental | Trabajo de código | Análisis de datos | Edición de manuscrito |
|---|---|---|---|---|---|
| Depuración/fusión de figuras y texto redundante (Step 6) | CRITICAL | — | LOW | LOW | HIGH |
| Consolidar limitaciones (Step 7) | HIGH | — | — | LOW | MEDIUM |
| Experimento A (FSM baseline) | CRITICAL | HIGH | HIGH | MEDIUM | MEDIUM |
| Experimento B (ablación BG) | CRITICAL | HIGH | MEDIUM | MEDIUM | MEDIUM |
| Experimento C (WTA vs. umbral) | CRITICAL | (reutiliza datos de B) | LOW | LOW | LOW |
| Experimento D (robustez sensorial) | CRITICAL | HIGH | MEDIUM | MEDIUM | MEDIUM |
| Experimento E (tarea de inspección) | HIGH | MEDIUM–HIGH | MEDIUM | MEDIUM | MEDIUM |
| Experimento F (energía por modo) | HIGH | MEDIUM | LOW | LOW | LOW |
| Experimento G (overhead MLP embebido) | HIGH | MEDIUM | MEDIUM | LOW | LOW |
| Step 4 (opción C, discusión de limitación de torque) | MEDIUM | — | — | LOW | MEDIUM |
| Step 5 (tablas de literatura) | HIGH | — | — | LOW | HIGH |
| Step 4 (opción B, compensación de control real) | LOW (opcional) | VERY HIGH | HIGH | MEDIUM | LOW |

---

## STEP 11 — CHECKLIST FINAL

```
[ ] FSM baseline implementada (Experimento A)
[ ] Experimento FSM completado bajo condiciones idénticas
[ ] Ablación de ganglios basales completada (M0–M4)
[ ] Comparación WTA vs. umbral simple completada (Experimento C)
[ ] Robustez a deriva de IMU completada
[ ] Robustez a ruido de distancia LiDAR completada
[ ] Robustez a distorsión de iluminación de cámara completada
[ ] Condición combinada worst-case (3 sensores) completada
[ ] Escenario de inspección con bloqueo y re-planificación implementado
[ ] Métricas de inspección (aproximación, evasión, reencuentro, tiempo) medidas
[ ] Energía en estado estacionario de los 3 modos medida (Experimento F)
[ ] Overhead computacional embebido del MLP medido en hardware real (Experimento G)
[ ] Tabla 1 de clasificación de literatura añadida
[ ] Tabla 2 de comparación directa añadida
[ ] Subsección "Related Work" creada
[ ] Subsección "Limitations and Deployment Considerations" creada y limitaciones duplicadas eliminadas de Conclusiones
[ ] Figuras 19/20/24/38/39 fusionadas
[ ] Figuras 54–57 consolidadas en 3 figuras compactas
[ ] Definiciones de SM/TR unificadas (una sola aparición completa)
[ ] Descargo del "symbolic warehouse" reducido a una sola mención completa
[ ] Sección de Discusión actualizada con resultados de ablación y robustez
[ ] Carta de respuesta a revisores (point-by-point) preparada
```

---

## MINIMUM SET OF NEW WORK NEEDED FOR ACCEPTANCE

Dado que Reviewer #1 ya aprobó el paper y Reviewer #2 solo pide ajustes menores + dos experimentos acotados, **el cuello de botella real para la aceptación es Reviewer #3**, que exige tres piezas de evidencia experimental genuinamente ausentes. El conjunto mínimo defendible es:

1. **Experimento B + C combinados** (ablación BG + comparación WTA/umbral), ejecutados como una sola campaña experimental sobre los escenarios de conflicto multiestímulo ya existentes en el paper (Sec. 3.1.3/3.2.2) — responde simultáneamente a #3.1 y #3.2, los dos comentarios más graves de Reviewer #3.
2. **Experimento A** (FSM baseline), que puede compartir infraestructura y escenarios con (1) — responde a #2.2, el único punto verdaderamente experimental de Reviewer #2.
3. **Experimento D** (robustez sensorial), en su versión reducida (perturbar un sensor a la vez + un caso combinado, sin matriz factorial completa) — responde a #3.4.
4. **Experimento E**, en su versión "esencial" (sin cobertura ni error de orientación) — responde a #3.5.
5. **Experimentos F y G**, ambos de bajo costo relativo porque reutilizan instrumentación ya existente (sensor INA de Fig. 58; banco de pruebas del MLP de Fig. 41–46) — responden a #2.4a/b.
6. **Tabla 1 y Tabla 2 de literatura** (Step 5) — trabajo de escritura, no experimental; responde a #3.6.
7. **Fusión de figuras 54–57 y 19/20/24/38/39 + consolidación de limitaciones** (Steps 6–7) — trabajo puramente editorial; responde a #2.1 y #2.3.

**Lo que se puede omitir sin poner en riesgo la aceptación** (a menos que el tiempo lo permita):
- Cobertura de inspección y error de orientación en Experimento E (opcionales, Step 3-E).
- Implementación real de una estrategia de control compensatorio para los servos de bajo torque (Step 4, opción B) — basta con la opción C (discusión ampliada y cuantificada).
- Ablaciones adicionales de GPe/GPi por separado más allá de M1/M2 en Experimento B — el conjunto M0–M4 ya es suficiente para sostener la hipótesis central.

Este conjunto cubre el 100% de los comentarios NOT ADDRESSED y PARTIALLY ADDRESSED de los tres revisores sin ampliar el alcance del paper más allá de lo que los propios comentarios exigen.
