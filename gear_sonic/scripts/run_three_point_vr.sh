#!/usr/bin/env bash
# One-terminal, SIM-only SONIC TELEOP + locomotion planner.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"
PY="${SONIC_PYTHON:-$REPO/.venv/bin/python}"
SOURCE="${INPUT_SOURCE:-pico}"
MODELS="${THREE_POINT_MODELS:-${SONIC_HOME:-$HOME/.cache/sonic}/x2/three_point_vr}"
DRY=0; PREPARE=0; CHECKPOINT=""; PASS=()
while (( $# )); do
    case "$1" in
        --dry-run) DRY=1; shift ;;
        --prepare-models) PREPARE=1; shift ;;
        --checkpoint) CHECKPOINT="${2:?missing checkpoint}"; shift 2 ;;
        --input-source) SOURCE="${2:?missing input source}"; shift 2 ;;
        --no-viewer) export SIMSTACK_NO_VIEWER=1; shift ;;
        --seconds) PASS+=(--seconds "${2:?missing seconds}"); shift 2 ;;
        -h|--help)
            echo "Usage: sim_onnx_planner.sh --three-point-vr [--input-source pico|quest3]"
            echo "       [--prepare-models] [--checkpoint native.pt] [--dry-run] [--no-viewer] [--seconds N]"
            echo "SIM only. Models default to a matched public X2 14000 TELEOP set."
            exit 0 ;;
        *) echo "Unsupported three-point SIM option: $1" >&2; exit 2 ;;
    esac
done
[[ "$SOURCE" == "pico" || "$SOURCE" == "quest3" ]] || { echo "Use --input-source pico|quest3" >&2; exit 2; }
if (( DRY )); then
    echo "[three-point] SIM ONLY; trained TELEOP encoder (267 -> 64), no arm IK/body trackers."
    echo "[three-point] source=$SOURCE models=$MODELS"
    echo "[three-point] stack: pose.onnx + token.onnx + teleop.onnx (same checkpoint)"
    echo "[three-point] planner :5656 -> TELEOP -> tokens :5574; debug :5659; commands :5663"
    exit 0
fi
if [[ -n "$CHECKPOINT" ]] && (( ! PREPARE )); then
    echo "--checkpoint requires --prepare-models (exports the matched set before launch)." >&2; exit 2
fi
if (( PREPARE )) || [[ ! -f "$MODELS/pose.onnx" || ! -f "$MODELS/token.onnx" || ! -f "$MODELS/teleop.onnx" ]]; then
    PREP_ARGS=(--out-dir "$MODELS")
    [[ -n "$CHECKPOINT" ]] && PREP_ARGS+=(--checkpoint "$CHECKPOINT")
    "$PY" gear_sonic/scripts/prepare_three_point_vr.py "${PREP_ARGS[@]}"
fi
(( PREPARE )) && exit 0
"$PY" gear_sonic/scripts/onnx_provenance.py check --pose "$MODELS/pose.onnx" \
    --token "$MODELS/token.onnx" --teleop-tokenizer "$MODELS/teleop.onnx"
export SIMSTACK_SONIC="$MODELS/pose.onnx" SIMSTACK_TOKEN="$MODELS/token.onnx"
export SIMSTACK_TOKENIZER="$MODELS/teleop.onnx" SIMSTACK_DUAL_MODEL=""
export X2_THREE_POINT_VR=1 X2_WHOLE_BODY_TELEOP=1 SIMSTACK_NO_TOKEN_SERVICE=1
# A VR-only session must not acquire a second source of stick commands.
export SIMSTACK_NO_PAD=1
cleanup() { "$REPO/gear_sonic/scripts/simstack_local.sh" --stop; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$REPO/gear_sonic/scripts/simstack_local.sh"
if [[ "$SOURCE" == "pico" ]]; then
    # Reuse the headset service startup, without the SMPL sender's port gate.
    THREE_POINT_MODELS="$MODELS" "$REPO/gear_sonic/scripts/run_pico_teleop.sh" \
        --three-point-vr "${PASS[@]}"
else
    "$PY" gear_sonic/scripts/three_point_vr.py --tokenizer "$MODELS/teleop.onnx" \
        --input-source quest3 "${PASS[@]}"
fi
