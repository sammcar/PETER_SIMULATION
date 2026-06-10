#!/bin/bash
# run_pc.sh — Lanza todos los nodos receptores del PC (sin la red neuronal).
#
# Uso:
#   ./run_pc.sh [opciones]
#
# Opciones generales:
#   --show-lidar          mostrar plot polar del lidar en tiempo real
#   --show-camera         mostrar ventana de debug de la cámara
#   --show-imu            imprimir lecturas IMU en consola
#   --csv                 guardar CSV con lecturas IMU/INA
#   --threshold T         umbral de velocidad para cmd_translator (default 0.1)
#
# Parámetros del lidar (ya tienen los valores del robot físico como default):
#   --rmax R              distancia máxima en metros        (default 0.45)
#   --rmin R              distancia mínima en metros        (default 0.2)
#   --expiry S            segundos para expirar celda       (default 0.3)
#   --keepalive S         reenviar último cmd al ESP cada S segundos (default 0.4)
#
# La red neuronal (red_neuronal.py) se inicia por separado.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Argumentos opcionales ──────────────────────────────────────────────────────
SHOW_LIDAR=""
SHOW_CAM=""
SHOW_IMU=""
CSV_FLAG=""
THRESHOLD="0.1"
KEEPALIVE="0.4"
LIDAR_RMAX="0.45"
LIDAR_RMIN="0.2"
LIDAR_EXPIRY="0.3"

for arg in "$@"; do
    case $arg in
        --show-lidar)    SHOW_LIDAR="--show" ;;
        --show-camera)   SHOW_CAM="--show" ;;
        --show-imu)      SHOW_IMU="--show" ;;
        --csv)           CSV_FLAG="--csv" ;;
        --threshold=*)   THRESHOLD="${arg#*=}" ;;
        --keepalive=*)   KEEPALIVE="${arg#*=}" ;;
        --rmax=*)        LIDAR_RMAX="${arg#*=}" ;;
        --rmin=*)        LIDAR_RMIN="${arg#*=}" ;;
        --expiry=*)      LIDAR_EXPIRY="${arg#*=}" ;;
        --threshold)     shift; THRESHOLD="$1" ;;
        --keepalive)     shift; KEEPALIVE="$1" ;;
        --rmax)          shift; LIDAR_RMAX="$1" ;;
        --rmin)          shift; LIDAR_RMIN="$1" ;;
        --expiry)        shift; LIDAR_EXPIRY="$1" ;;
    esac
done

# ── Limpieza al salir ──────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Deteniendo nodos del PC..."
    kill $PID_BROKER $PID_LIDAR $PID_IMU $PID_CAM $PID_CMD_SRV $PID_TRANSLATOR 2>/dev/null
    wait $PID_BROKER $PID_LIDAR $PID_IMU $PID_CAM $PID_CMD_SRV $PID_TRANSLATOR 2>/dev/null
    echo "Listo."
    exit 0
}

trap cleanup SIGINT SIGTERM

# ── Broker ZMQ (debe arrancar primero) ────────────────────────────────────────
echo "[1/6] Broker ZMQ..."
uv run "$SCRIPT_DIR/zmq_metrics/broker.py" &
PID_BROKER=$!
sleep 0.5   # dar tiempo a que haga bind en los puertos

# ── Nodos receptores ──────────────────────────────────────────────────────────
echo "[2/6] Lidar node  (UDP:8888, rmax=$LIDAR_RMAX rmin=$LIDAR_RMIN expiry=$LIDAR_EXPIRY)..."
uv run "$SCRIPT_DIR/Receptor_datos/lidar_node.py" \
    --rmax "$LIDAR_RMAX" --rmin "$LIDAR_RMIN" --expiry "$LIDAR_EXPIRY" \
    $SHOW_LIDAR &
PID_LIDAR=$!

echo "[3/6] IMU/INA node (UDP:9999)..."
uv run "$SCRIPT_DIR/Receptor_datos/imu_ina_node.py" $SHOW_IMU $CSV_FLAG &
PID_IMU=$!

echo "[4/6] Camera node  (TCP:8080)..."
uv run "$SCRIPT_DIR/Receptor_datos/camera_node.py" $SHOW_CAM &
PID_CAM=$!

echo "[5/6] Cmd vel server (TCP:8003)..."
uv run "$SCRIPT_DIR/Receptor_datos/cmd_vel_server.py" &
PID_CMD_SRV=$!

echo "[6/6] Cmd translator..."
uv run "$SCRIPT_DIR/Receptor_datos/cmd_translator.py" --threshold "$THRESHOLD" --keepalive "$KEEPALIVE" &
PID_TRANSLATOR=$!

# ── Info ───────────────────────────────────────────────────────────────────────
echo ""
echo "Nodos corriendo:"
echo "  Broker ZMQ      PID $PID_BROKER"
echo "  Lidar node      PID $PID_LIDAR"
echo "  IMU/INA node    PID $PID_IMU"
echo "  Camera node     PID $PID_CAM"
echo "  Cmd vel server  PID $PID_CMD_SRV   ← espera conexión de la Pi en :8003"
echo "  Cmd translator  PID $PID_TRANSLATOR"
echo ""
echo "Cuando todo esté listo, corre en otra terminal:"
echo "  uv run zmq_metrics/red_neuronal.py"
echo ""
echo "Presiona Ctrl+C para detener."

wait $PID_BROKER $PID_LIDAR $PID_IMU $PID_CAM $PID_CMD_SRV $PID_TRANSLATOR
