# BehaviorTree.CPP benchmarks

Three executables, all disabled by default (`-DBTCPP_BUILD_BENCHMARKS=ON`):

| Target | What it measures |
|--------|------------------|
| `bt_bench` | Average time of isolated operations (Google Benchmark, fetched automatically if not installed) |
| `bt_latency` | Distribution of the time of single ticks, with cold caches, periodic ticking and a fragmented heap |
| `bt_membench` | Heap footprint per node and heap allocations per steady-state tick |

`bt_bench` ticks a small tree back-to-back: the tree stays in L1/L2 and its
numbers are a **best case**, useful to compare code changes on a specific path.
`bt_latency` is closer to what a robot sees and should be the reference for
"how long does a tick take".

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

## Latency benchmark (`bench_latency.cpp`)

Trees: `Seq100` (100 trivial leaves), `Ports100` (100 blackboard reads) and
`Robot2k` (100 SubTree instances, ~2100 nodes with ports, scripts, decorators,
Parallel and reactive nodes).

Conditions, applied before each timed tick (never timed themselves):

| Condition | Meaning |
|-----------|---------|
| `hot` | back-to-back ticks, same as `bt_bench` |
| `evict` | write `--evict-mb` (default 32 MB) to push the tree out of the caches |
| `periodic` | sleep `--period-us` (default 1 ms), like a control loop |
| `frag` | like `evict`, but the tree is built after fragmenting the heap, so nodes are scattered in memory as in a long-running process |

For every scenario it prints p50 / p90 / p99 / p99.9 / max and a log-scale
histogram. Each scenario runs `--rounds` times on a fresh tree; the reported
statistics are medians across rounds and `*_spread_pct` tells how much p50/p99
moved between rounds: if it is large, the number is not trustworthy.
`--csv-dir` dumps every sample for plotting.

Choose `--evict-mb` larger than the last-level cache of the machine to measure
a fully cold tick (DRAM). On a Raspberry Pi 4 (1 MB L2, no L3) the default
already does that. On big x86 servers the L3 can be hundreds of MB.

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
- For `bt_latency`, only p50 is reproducible: on the same VM, p50 moved by ~3%
  between runs of the same commit (~16% in `periodic` mode), p90 and mean by up
  to 50%, the maximum by up to 400%. `compare.py` therefore gates only on p50
  and prints the other statistics as information. Tails are meaningful only on
  a quiet, dedicated machine (ideally the robot itself, with an isolated core).
- When comparing two commits, interleave runs (base, new, base, new) to cancel
  slow drifts of the host.
- Close other workloads, disable CPU frequency scaling if you can.
