# BehaviorTree.CPP benchmarks

Two executables, both disabled by default (`-DBTCPP_BUILD_BENCHMARKS=ON`):

| Target | What it measures |
|--------|------------------|
| `bt_bench` | Time (Google Benchmark, fetched automatically if not installed) |
| `bt_membench` | Heap footprint per node and heap allocations per steady-state tick |

`bt_membench` replaces the global `operator new/delete`, so it counts every C++
allocation made by the library. It reports `0` allocations per tick for the
cases that are allocation-free, which makes regressions obvious.

## Time benchmarks (`bench_*.cpp`)

| File | Scenarios |
|------|-----------|
| `bench_tick.cpp` | Sequence / Fallback / Parallel / ReactiveSequence / decorators / deep nesting; script pre-conditions; cost of tick-monitor, pre/post-tick and status-change callbacks |
| `bench_ports.cpp` | `getInput` (blackboard, literal, default, string->int, string, through SubTree remapping), `setOutput` |
| `bench_blackboard.cpp` | `Blackboard` get/set, parent lookup, `Any` copy/cast, script parse+execute |
| `bench_load.cpp` | Factory construction, XML parse + tree creation/destruction, subtrees, first tick |

Tick benchmarks report `items_per_second` = nodes ticked per second, so trees of
different size are comparable. Every benchmark checks the status of its first
tick and aborts (`SkipWithError`) if the tree does not behave as intended, so a
number can never silently measure an early-failing tree.

## Workflow

```bash
# baseline
git checkout master && benchmarks/run.sh results/base
# change
git checkout my-branch && benchmarks/run.sh results/new
benchmarks/compare.py results/base results/new            # exit code 1 if anything is worse
```

`run.sh` configures a Release build in `build-bench/` (Groot2 and SQLite are
off, they are irrelevant here), pins to CPU 0 when `taskset` exists, and runs
each benchmark 5 times, comparing medians.

Environment knobs: `BT_BENCH_REPS`, `BT_BENCH_MIN_TIME`, `BT_BENCH_CPU`.

Manual use:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBTCPP_BUILD_BENCHMARKS=ON
cmake --build build --target bt_bench bt_membench
./build/benchmarks/bt_bench --benchmark_filter=Port
./build/benchmarks/bt_membench
```

## Getting trustworthy numbers

- Always Release. Compare results from the **same machine and compiler**.
- Measured on a 4-vCPU shared VM with the default settings: two runs of the
  *same* commit differ by up to ~10% for tick benchmarks and ~25% for tree
  loading (allocation heavy), hence `compare.py` flags changes above 10% by
  default (`--threshold`). Timing noise can be tens of percent on such hosts. Run
  `compare.py` on two runs of the *same* commit first to see your noise floor,
  and only trust changes above it. Allocation counts in `memory.json` are
  deterministic and are not affected by noise.
- Close other workloads, disable CPU frequency scaling if you can.
