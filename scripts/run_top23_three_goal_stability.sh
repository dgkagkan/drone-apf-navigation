#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

SOURCE_DIRECTORY="${1:-$REPO_ROOT/optuna_results/apf_three_goal}"
OUTPUT_DIRECTORY="${2:-$REPO_ROOT/optuna_results/apf_top23_three_goal_stability}"

export APF_BENCHMARK_STUDY="${APF_BENCHMARK_STUDY:-apf_top23_three_goal_stability}"
export APF_MISSION_PROFILE=three_goal
export APF_GOAL_X=750.0
export APF_GOAL_Y=0.0
export APF_CRUISE_ALTITUDE=15.0
export APF_GOAL_2_X=0.0
export APF_GOAL_2_Y=0.0
export APF_GOAL_2_ALTITUDE=15.0
export APF_GOAL_3_X=750.0
export APF_GOAL_3_Y=15.0
export APF_GOAL_3_ALTITUDE=15.0
export APF_INTERMEDIATE_GOAL_TOLERANCE=25.0

exec "$SCRIPT_DIR/run_top10_stability.sh" \
    "$SOURCE_DIRECTORY" \
    apf_three_goal \
    23 \
    15 \
    "$OUTPUT_DIRECTORY"
