#!/usr/bin/env bash
# Build (Release) and run all benchmarks, writing JSON results to <out_dir>.
#
#   benchmarks/run.sh <out_dir> [build_dir]
#
# Typical before/after workflow:
#   git checkout master      && benchmarks/run.sh results/base
#   git checkout my-branch   && benchmarks/run.sh results/new
#   benchmarks/compare.py results/base results/new
set -euo pipefail

OUT="${1:?usage: run.sh <out_dir> [build_dir]}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${2:-$ROOT/build-bench}"
REPS="${BT_BENCH_REPS:-5}"
MIN_TIME="${BT_BENCH_MIN_TIME:-0.2s}"

cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DBTCPP_BUILD_BENCHMARKS=ON -DBUILD_TESTING=OFF -DBTCPP_EXAMPLES=OFF \
  -DBTCPP_BUILD_TOOLS=OFF -DBTCPP_GROOT_INTERFACE=OFF -DBTCPP_SQLITE_LOGGING=OFF \
  >/dev/null
cmake --build "$BUILD" --parallel --target bt_bench bt_membench bt_latency

mkdir -p "$OUT"

# Pin to one core when possible: removes most scheduler noise.
PIN=()
if command -v taskset >/dev/null 2>&1; then
  PIN=(taskset -c "${BT_BENCH_CPU:-0}")
fi

"${PIN[@]}" "$BUILD/benchmarks/bt_bench" \
  --benchmark_repetitions="$REPS" \
  --benchmark_min_time="$MIN_TIME" \
  --benchmark_report_aggregates_only=true \
  --benchmark_out="$OUT/time.json" --benchmark_out_format=json \
  --benchmark_format=console

"$BUILD/benchmarks/bt_membench" --json="$OUT/memory.json"

mkdir -p "$OUT/latency_csv"
"${PIN[@]}" "$BUILD/benchmarks/bt_latency" \
  --json="$OUT/latency.json" --csv-dir="$OUT/latency_csv" ${BT_LATENCY_ARGS:-}

{
  echo "commit: $(git -C "$ROOT" rev-parse --short HEAD) $(git -C "$ROOT" status --porcelain | grep -q . && echo '(dirty)')"
  echo "compiler: $(grep -m1 CMAKE_CXX_COMPILER: "$BUILD/CMakeCache.txt" | cut -d= -f2)"
  echo "date: $(date -u +%FT%TZ)"
} > "$OUT/info.txt"
echo "results written to $OUT"
