#!/usr/bin/env bash
# export_results.sh — Exporta resultados de experimentos del contenedor al host
#
# Uso:
#   ./export_results.sh [--last] [--list] [--clean]
#
#   --last   Exporta automáticamente el resultado más reciente
#   --list   Solo lista los resultados disponibles (sin exportar)
#   --clean  Elimina todos los resultados del contenedor (confirma antes)
#
# Sin argumentos: menú interactivo para seleccionar qué exportar.
#
# Variables configurables (se pueden sobreescribir desde el entorno):
#   CONTAINER           Nombre del contenedor  (por defecto: peter_sim_dev)
#   CONTAINER_RESULTS   Ruta de resultados dentro del contenedor
#   HOST_RESULTS        Ruta de destino en el host

set -euo pipefail

# ── Configuración ─────────────────────────────────────────────────────────────
CONTAINER="${CONTAINER:-peter_sim_dev}"
CONTAINER_RESULTS="${CONTAINER_RESULTS:-/root/peter_experiments}"
# Destino en el host: ruta absoluta relativa al script → src/peter_robot/docs/resultados
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts/ está dentro de src/peter_robot/; subir dos niveles llega a src/peter_robot/
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
HOST_RESULTS="${HOST_RESULTS:-${PACKAGE_DIR}/docs/resultados}"

# ── Colores ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${CYAN}[INFO]${RESET} $*"; }
ok()      { echo -e "${GREEN}[OK]${RESET}   $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET} $*"; }
err()     { echo -e "${RED}[ERROR]${RESET} $*" >&2; }

# ── Comprobaciones previas ────────────────────────────────────────────────────
check_container() {
    if ! docker inspect --format '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
        err "El contenedor '${CONTAINER}' no está corriendo."
        err "Arránalo con: docker start ${CONTAINER}"
        exit 1
    fi
}

check_results_dir() {
    if ! docker exec "$CONTAINER" test -d "$CONTAINER_RESULTS" 2>/dev/null; then
        warn "La carpeta de resultados '${CONTAINER_RESULTS}' no existe dentro del contenedor."
        warn "Aún no se han generado experimentos. Lanza una simulación con 'make sim-single' o 'make sim-multi'."
        exit 0
    fi
}

# ── Obtener lista de sesiones ─────────────────────────────────────────────────
get_sessions() {
    # Lista subdirectorios de resultados, ordenados del más reciente al más antiguo
    docker exec "$CONTAINER" bash -c \
        "ls -1dt ${CONTAINER_RESULTS}/*/ 2>/dev/null | xargs -I{} basename {}" \
        2>/dev/null || true
}

# ── Menú interactivo (fzf si está disponible, else select bash) ───────────────
interactive_select() {
    local sessions_array=("$@")

    if command -v fzf &>/dev/null; then
        # fzf: selección múltiple con Tab, confirmación con Enter
        printf '%s\n' "${sessions_array[@]}" | \
            fzf --multi \
                --prompt="Selecciona resultado(s) [Tab=marcar, Enter=confirmar]: " \
                --header="Resultados en contenedor '${CONTAINER}:${CONTAINER_RESULTS}'"
    else
        # Fallback: menú numerado en bash puro
        echo ""
        echo -e "${BOLD}Resultados disponibles en el contenedor:${RESET}"
        echo "  [0] Exportar TODOS"
        local i=1
        for s in "${sessions_array[@]}"; do
            printf "  [%d] %s\n" "$i" "$s"
            ((i++))
        done
        echo ""
        read -rp "Selecciona número(s) separados por espacio (ej: 1 3): " raw_input

        if [[ "$raw_input" == "0" ]]; then
            printf '%s\n' "${sessions_array[@]}"
            return
        fi

        local selected=()
        for num in $raw_input; do
            if [[ "$num" =~ ^[0-9]+$ ]] && (( num >= 1 && num <= ${#sessions_array[@]} )); then
                selected+=("${sessions_array[$((num - 1))]}")
            else
                warn "Número inválido ignorado: $num"
            fi
        done

        if [[ ${#selected[@]} -eq 0 ]]; then
            err "No se seleccionó ningún resultado válido."
            exit 1
        fi

        printf '%s\n' "${selected[@]}"
    fi
}

# ── Resolver colisiones de nombre en el destino ───────────────────────────────
resolve_dest_name() {
    local name="$1"
    local dest="${HOST_RESULTS}/${name}"
    if [[ -e "$dest" ]]; then
        local i=1
        while [[ -e "${HOST_RESULTS}/${name}_${i}" ]]; do
            ((i++))
        done
        echo "${name}_${i}"
    else
        echo "$name"
    fi
}

# ── Exportar una sola sesión ──────────────────────────────────────────────────
export_session() {
    local session="$1"
    local final_name="$2"   # puede ser igual a session o un nombre nuevo

    local src_path="${CONTAINER_RESULTS}/${session}"
    local tmp_tar="/tmp/peter_export_$$.tar"
    local dest_path="${HOST_RESULTS}/${final_name}"

    info "Exportando: ${CONTAINER}:${src_path}"
    info "      → ${dest_path}"

    # Crear directorio destino en host si no existe
    mkdir -p "$HOST_RESULTS"

    # Copiar via docker cp (soporta carpetas, nombres con espacios, etc.)
    if docker cp "${CONTAINER}:${src_path}" "$dest_path"; then
        ok "Exportado exitosamente: ${dest_path}"
        return 0
    else
        err "Falló la copia de '${session}'. No se eliminará del contenedor."
        return 1
    fi
}

# ── Eliminar sesión del contenedor ────────────────────────────────────────────
delete_from_container() {
    local session="$1"
    local src_path="${CONTAINER_RESULTS}/${session}"
    info "Eliminando del contenedor: ${src_path}"
    if docker exec "$CONTAINER" rm -rf "$src_path"; then
        ok "Eliminado del contenedor: ${session}"
    else
        warn "No se pudo eliminar '${session}' del contenedor."
    fi
}

# ── Modo: listar ──────────────────────────────────────────────────────────────
cmd_list() {
    check_container
    check_results_dir
    local sessions
    mapfile -t sessions < <(get_sessions)
    if [[ ${#sessions[@]} -eq 0 ]]; then
        warn "No hay resultados en '${CONTAINER_RESULTS}'."
        exit 0
    fi
    echo ""
    echo -e "${BOLD}Resultados en ${CONTAINER}:${CONTAINER_RESULTS}${RESET}"
    local i=1
    for s in "${sessions[@]}"; do
        # Mostrar tamaño del directorio si es posible
        local size
        size=$(docker exec "$CONTAINER" bash -c "du -sh '${CONTAINER_RESULTS}/${s}' 2>/dev/null | cut -f1" || echo "?")
        printf "  %2d.  %-40s  %s\n" "$i" "$s" "$size"
        ((i++))
    done
    echo ""
}

# ── Modo: exportar el más reciente ────────────────────────────────────────────
cmd_export_last() {
    check_container
    check_results_dir
    local sessions
    mapfile -t sessions < <(get_sessions)
    if [[ ${#sessions[@]} -eq 0 ]]; then
        warn "No hay resultados para exportar."
        exit 0
    fi

    local last="${sessions[0]}"
    info "Resultado más reciente: ${last}"

    # Preguntar rename
    local final_name
    final_name=$(resolve_dest_name "$last")
    read -rp "Nombre destino [${final_name}]: " user_name
    user_name="${user_name:-$final_name}"
    # Sanear el nombre introducido
    user_name=$(echo "$user_name" | tr ' ' '_' | tr -cd '[:alnum:]_.-')
    if [[ -z "$user_name" ]]; then
        user_name="$final_name"
    fi
    final_name=$(resolve_dest_name "$user_name")

    if export_session "$last" "$final_name"; then
        echo ""
        read -rp "¿Eliminar '${last}' del contenedor? [s/N]: " confirm
        if [[ "${confirm,,}" == "s" ]]; then
            delete_from_container "$last"
        fi
    fi
}

# ── Modo: exportar interactivo ────────────────────────────────────────────────
cmd_export_interactive() {
    check_container
    check_results_dir
    local sessions
    mapfile -t sessions < <(get_sessions)
    if [[ ${#sessions[@]} -eq 0 ]]; then
        warn "No hay resultados para exportar."
        exit 0
    fi

    local selected_sessions
    mapfile -t selected_sessions < <(interactive_select "${sessions[@]}")

    if [[ ${#selected_sessions[@]} -eq 0 ]]; then
        warn "No se seleccionó ningún resultado."
        exit 0
    fi

    echo ""
    info "Destino base: ${HOST_RESULTS}"
    echo ""

    local exported=()
    local failed=()

    for session in "${selected_sessions[@]}"; do
        local final_name
        final_name=$(resolve_dest_name "$session")

        if [[ ${#selected_sessions[@]} -eq 1 ]]; then
            # Solo uno: ofrecer rename
            read -rp "Nombre destino para '${session}' [${final_name}]: " user_name
            user_name="${user_name:-$final_name}"
            user_name=$(echo "$user_name" | tr ' ' '_' | tr -cd '[:alnum:]_.-')
            if [[ -n "$user_name" ]]; then
                final_name=$(resolve_dest_name "$user_name")
            fi
        fi

        if export_session "$session" "$final_name"; then
            exported+=("$session")
        else
            failed+=("$session")
        fi
    done

    echo ""
    if [[ ${#exported[@]} -gt 0 ]]; then
        ok "${#exported[@]} resultado(s) exportado(s) a: ${HOST_RESULTS}"

        read -rp "¿Eliminar los resultados exportados del contenedor? [s/N]: " confirm
        if [[ "${confirm,,}" == "s" ]]; then
            for session in "${exported[@]}"; do
                delete_from_container "$session"
            done
        fi
    fi

    if [[ ${#failed[@]} -gt 0 ]]; then
        err "${#failed[@]} resultado(s) fallaron al exportar:"
        for f in "${failed[@]}"; do
            err "  - $f"
        done
        exit 1
    fi
}

# ── Modo: limpiar todos los resultados del contenedor ─────────────────────────
cmd_clean() {
    check_container
    check_results_dir
    local sessions
    mapfile -t sessions < <(get_sessions)
    if [[ ${#sessions[@]} -eq 0 ]]; then
        warn "No hay resultados que limpiar."
        exit 0
    fi

    echo ""
    echo -e "${RED}${BOLD}¡ATENCIÓN! Se eliminarán TODOS los resultados del contenedor:${RESET}"
    for s in "${sessions[@]}"; do
        echo "  - $s"
    done
    echo ""
    read -rp "¿Confirmar eliminación de ${#sessions[@]} resultado(s)? [s/N]: " confirm
    if [[ "${confirm,,}" != "s" ]]; then
        info "Operación cancelada."
        exit 0
    fi

    docker exec "$CONTAINER" rm -rf "${CONTAINER_RESULTS:?}/"*
    ok "Todos los resultados eliminados de '${CONTAINER_RESULTS}'."
}

# ── Punto de entrada ──────────────────────────────────────────────────────────
MODE="interactive"

for arg in "$@"; do
    case "$arg" in
        --last)  MODE="last"  ;;
        --list)  MODE="list"  ;;
        --clean) MODE="clean" ;;
        --help|-h)
            echo "Uso: $(basename "$0") [--last|--list|--clean|--help]"
            echo ""
            echo "  (sin args)  Menú interactivo para seleccionar resultados a exportar"
            echo "  --last      Exporta automáticamente el resultado más reciente"
            echo "  --list      Lista los resultados disponibles en el contenedor"
            echo "  --clean     Elimina todos los resultados del contenedor"
            echo ""
            echo "Variables de entorno:"
            echo "  CONTAINER          Nombre del contenedor   (por defecto: peter_sim_dev)"
            echo "  CONTAINER_RESULTS  Ruta de resultados      (por defecto: /root/peter_experiments)"
            echo "  HOST_RESULTS       Ruta destino en host    (por defecto: <package>/docs/resultados)"
            exit 0
            ;;
        *) warn "Argumento desconocido ignorado: $arg" ;;
    esac
done

echo -e "${BOLD}═══════════════════════════════════════════════════════${RESET}"
echo -e "${BOLD}   PETER — Exportación de resultados experimentales${RESET}"
echo -e "${BOLD}═══════════════════════════════════════════════════════${RESET}"
echo -e "  Contenedor : ${CYAN}${CONTAINER}${RESET}"
echo -e "  Origen     : ${CYAN}${CONTAINER_RESULTS}${RESET}"
echo -e "  Destino    : ${CYAN}${HOST_RESULTS}${RESET}"
echo ""

case "$MODE" in
    interactive) cmd_export_interactive ;;
    last)        cmd_export_last        ;;
    list)        cmd_list               ;;
    clean)       cmd_clean              ;;
esac
