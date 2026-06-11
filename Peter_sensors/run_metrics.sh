#!/bin/bash
# run_metrics.sh — Lanza los nodos de métricas y guarda CSVs numerados.
#
# Uso:
#   ./run_metrics.sh [--experiment NAME] [--test N]
#
# Opciones:
#   --experiment NAME   nombre del experimento (default: exp)
#   --test N            número de prueba; si se omite, busca el siguiente libre
#
# Crea:
#   ~/peter_experiments/<experiment>_t<NN>/
#     neural_metrics.csv   — actividad neuronal + latencia
#     combined_metrics.csv — todas las métricas agregadas
#
# Requiere que el broker ZMQ ya esté corriendo (run_pc.sh lo inicia).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$HOME/peter_experiments"

# ── Argumentos ────────────────────────────────────────────────────────────────
EXPERIMENT="exp"
TEST_NUM=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --experiment) EXPERIMENT="$2"; shift 2 ;;
        --experiment=*) EXPERIMENT="${1#*=}"; shift ;;
        --test)       TEST_NUM="$2"; shift 2 ;;
        --test=*)     TEST_NUM="${1#*=}"; shift ;;
        *) echo "Argumento desconocido: $1"; exit 1 ;;
    esac
done

# Buscar siguiente número de prueba libre si no se especificó
if [[ -z "$TEST_NUM" ]]; then
    TEST_NUM=1
    while [[ -d "$BASE_DIR/${EXPERIMENT}_t$(printf '%02d' $TEST_NUM)" ]]; do
        TEST_NUM=$((TEST_NUM + 1))
    done
fi

OUT_DIR="$BASE_DIR/${EXPERIMENT}_t$(printf '%02d' $TEST_NUM)"
mkdir -p "$OUT_DIR"

echo "Experimento : $EXPERIMENT"
echo "Prueba      : $TEST_NUM"
echo "Directorio  : $OUT_DIR"
echo ""

# ── Limpieza al salir ─────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Deteniendo nodos de métricas..."
    kill $PID_NEURAL $PID_COMBINED $PID_INA 2>/dev/null
    wait $PID_NEURAL $PID_COMBINED $PID_INA 2>/dev/null
    echo "CSVs guardados en: $OUT_DIR"
    exit 0
}

trap cleanup SIGINT SIGTERM

# ── Nodos ─────────────────────────────────────────────────────────────────────
echo "[1/2] Neural recorder..."
uv run "$SCRIPT_DIR/zmq_metrics/neural_recorder.py" \
    --experiment-type "$EXPERIMENT" \
    --output-dir "$OUT_DIR" &
PID_NEURAL=$!

echo "[2/3] Metrics recorder..."
uv run "$SCRIPT_DIR/zmq_metrics/metrics_recorder.py" \
    --experiment "$EXPERIMENT" \
    --test "$TEST_NUM" \
    --output-dir "$OUT_DIR" &
PID_COMBINED=$!

echo "[3/3] IMU + INA recorder..."
uv run "$SCRIPT_DIR/zmq_metrics/imu_ina_recorder.py" \
    --output-dir "$OUT_DIR" &
PID_INA=$!

echo ""
echo "Nodos corriendo:"
echo "  Neural recorder   PID $PID_NEURAL   → $OUT_DIR/neural_metrics.csv"
echo "  Metrics recorder  PID $PID_COMBINED → $OUT_DIR/combined_metrics.csv"
echo "  IMU+INA recorder  PID $PID_INA      → $OUT_DIR/imu_ina_<timestamp>.csv"
echo ""
echo "Presiona Ctrl+C para detener y guardar."

wait $PID_NEURAL $PID_COMBINED $PID_INA
