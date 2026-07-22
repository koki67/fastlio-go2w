#!/usr/bin/env bash
# Run and evaluate the fixed XT16 LiDAR-header offset candidates.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<'EOF'
Usage:
  scripts/offline/sweep_xt16_time_offset.sh BAG --output DIR [options]

Options:
  --start-offset SEC  Shared bag start offset (default: 20)
  --duration SEC      Optional shared smoke interval
  --rate RATE         Shared playback rate (default: 1.0)
  --domain-id ID      First ROS domain ID; increments per run (default: 80)
  --config YAML       Optional compatible XT16 FAST-LIO config
  -h, --help          Show this help

Runs -10, -5, 0, +5, and +10 ms against the same source interval, preserves
every run directory, and writes offset_comparison.json plus CSV.
EOF
}

die() {
    echo "Error: $*" >&2
    exit 2
}

BAG=""
OUTPUT=""
START_OFFSET="20"
DURATION=""
RATE="1.0"
BASE_DOMAIN_ID="80"
CONFIG=""

while [ "$#" -gt 0 ]; do
    case "$1" in
        --output) OUTPUT="${2:?Error: --output requires a value}"; shift 2 ;;
        --start-offset) START_OFFSET="${2:?Error: --start-offset requires a value}"; shift 2 ;;
        --duration) DURATION="${2:?Error: --duration requires a value}"; shift 2 ;;
        --rate) RATE="${2:?Error: --rate requires a value}"; shift 2 ;;
        --domain-id) BASE_DOMAIN_ID="${2:?Error: --domain-id requires a value}"; shift 2 ;;
        --config) CONFIG="${2:?Error: --config requires a value}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        -*) die "unknown option: $1" ;;
        *)
            [ -z "$BAG" ] || die "only one bag path may be supplied"
            BAG="$1"
            shift
            ;;
    esac
done

[ -n "$BAG" ] || die "a bag directory is required"
[ -n "$OUTPUT" ] || die "--output is required"
[[ "$BASE_DOMAIN_ID" =~ ^[0-9]+$ ]] || die "--domain-id must be non-negative"
BAG="$(realpath "$BAG")"
OUTPUT="$(realpath -m "$OUTPUT")"
[ -d "$BAG" ] || die "bag directory not found: $BAG"
if [ -d "$OUTPUT" ] && [ -n "$(find "$OUTPUT" -mindepth 1 -print -quit)" ]; then
    die "output directory is not empty: $OUTPUT"
fi
mkdir -p "$OUTPUT"

OFFSETS=(-0.010 -0.005 0.000 0.005 0.010)
LABELS=(neg010ms neg005ms zero pos005ms pos010ms)
RUN_DIRS=()
FAILED_RUNS=0

for index in "${!OFFSETS[@]}"; do
    run_dir="$OUTPUT/${LABELS[$index]}"
    RUN_DIRS+=("$run_dir")
    command=(
        bash "$SCRIPT_DIR/run_fastlio_offline.sh" "$BAG"
        --sensor xt16
        --lidar-time-offset-sec "${OFFSETS[$index]}"
        --start-offset "$START_OFFSET"
        --rate "$RATE"
        --domain-id "$((BASE_DOMAIN_ID + index))"
        --output "$run_dir"
    )
    if [ -n "$DURATION" ]; then
        command+=(--duration "$DURATION")
    fi
    if [ -n "$CONFIG" ]; then
        command+=(--config "$CONFIG")
    fi
    {
        printf 'run:'
        printf ' %q' "${command[@]}"
        printf '\n'
    } >> "$OUTPUT/sweep_commands.log"
    echo "Running XT16 offset ${OFFSETS[$index]} s -> $run_dir"
    if ! "${command[@]}"; then
        FAILED_RUNS=$((FAILED_RUNS + 1))
        echo "Candidate ${OFFSETS[$index]} s failed; retaining artifacts for selection." >&2
    fi
done

python3 "$SCRIPT_DIR/select_xt16_time_offset.py" \
    "${RUN_DIRS[@]}" --output "$OUTPUT/offset_comparison.json"

if [ "$FAILED_RUNS" -gt 0 ]; then
    echo "$FAILED_RUNS candidate run(s) failed and were disqualified." >&2
fi
echo "Sweep complete: $OUTPUT/offset_comparison.json"
