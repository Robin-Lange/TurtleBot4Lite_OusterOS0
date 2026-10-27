#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGS_DIR="${SCRIPT_DIR}/../logs"
mkdir -p "${LOGS_DIR}"

WARMUP="${1:-30}"
DURATION="${2:-120}"

echo "=========================================================="
echo "STARTING FULL STEP 2 SUSTAINED ACQUISITION CAMPAIGN"
echo "Warmup per run: ${WARMUP}s | Measurement per run: ${DURATION}s"
echo "Conditions: A1 (D455 alone), A2 (RPLIDAR alone), A3 (Both concurrent)"
echo "3 trials per condition"
echo "=========================================================="

for cond in D455_alone RPLIDAR_alone Both_concurrent; do
    for run in 1 2 3; do
        echo ""
        echo ">>> [$(date -u +%T)] Starting ${cond} Trial ${run}/3..."
        "${SCRIPT_DIR}/run_acquisition_trials.sh" "${cond}" "${run}" "${WARMUP}" "${DURATION}"
        echo ">>> [$(date -u +%T)] Completed ${cond} Trial ${run}/3. Cooling down 5s..."
        sleep 5
    done
done

echo ""
echo "=========================================================="
echo "ALL STEP 2 ACQUISITION RUNS COMPLETED SUCCESSFULLY"
echo "=========================================================="
