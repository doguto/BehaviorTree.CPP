# BehaviorTree.CPP benchmarks

Four executables, all disabled by default (`-DBTCPP_BUILD_BENCHMARKS=ON`):

| Target | What it measures |
|--------|------------------|
| `bt_bench` | Average time of isolated operations (Google Benchmark, fetched automatically if not installed) |
| `bt_latency` | Distribution of the time of single ticks, with cold caches, periodic ticking and a fragmented heap |
| `bt_xml_latency` | Same distribution, for **your own application's XML trees**, with stub nodes |
| `bt_membench` | Heap footprint per node and heap allocations per steady-state tick |

`bt_bench` ticks a small tree back-to-back: the tree stays in L1/L2 and its
numbers are a **best case**, useful to compare code changes on a specific path.
`bt_latency` is closer to what a robot sees and should be the reference for
"how long does a tick take".

`bt_membench` replaces the global `operator new/delete`, so it counts every C++
allocation made by the library. The bytes held by a tree come from `mallinfo2()`
(the whole heap); the per-node values are the difference between a small and a
large tree, so that the granularity of the allocators is spread over thousands
of nodes. It reports `0` allocations per tick for the
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
`--csv-dir` dumps every sample for plotting. `--tick-once` times
`Tree::tickOnce()`, which most applications call, instead of
`Tree::tickExactlyOnce()`.

Choose `--evict-mb` larger than the last-level cache of the machine to measure
a fully cold tick (DRAM). On a Raspberry Pi 4 (1 MB L2, no L3) the default
already does that. On big x86 servers the L3 can be hundreds of MB.

## Your application's trees (`bench_xml_latency.cpp`)

```bash
bt_xml_latency --xml=subtrees.xml,main.xml --main=MainTree,OtherTree \
               --period-us=500 [--set=key=value,...] [--groot2] \
               [--writer-keys=k1,k2 --writer-hz=1000] [latency options]
```

The XML files are registered in the given order (`<include>` is followed).
Every node that is not built into BT.CPP is replaced by a stub with the type and
ports declared in the `<TreeNodesModel>` of the files (Groot2 writes it; or
generate it from your real factory with `BT::writeTreeNodesModelXML()` and pass
it with `--models`):

- Action: reads its inputs and writes its outputs back on every tick, stays
  RUNNING `--running-ticks` ticks (default 1), then SUCCESS.
- Condition: touches its ports, SUCCESS. Decorator: touches its ports, returns
  its child's status.
- `Delay` / `Sleep` wait `--delay-scale` ticks per millisecond (default 0.1)
  instead of wall-clock time, so the number of samples spent waiting does not
  depend on the tick speed. Delays used as timeouts still last longer than
  the action stubs. `--real-delays` keeps the real nodes.

Keys referenced by ports and scripts are seeded with 0. Data-dependent branches
(Switch, Precondition, loop bounds) may need other values to go anywhere
interesting: `--set=line_id=1,plan_size=3`. Use `--trace=N` to print which
leaves are RUNNING over N ticks and check how far the stubbed application gets.

The `<TreeNodesModel>` usually has no port types, while real nodes declare them
(`InputPort<double>`): `--typed-ports` gives each stub port the type guessed from
the literals of the trees (number -> `double`, `true`/`false` -> `bool`, other
text -> `std::string`; ports given only blackboard keys stay generic), so that
literals are converted the way a real application converts them.

The creation time of each tree is printed with its shape.

`--groot2` attaches a Groot2Publisher (it subscribes to every node's status
changes) and `--writer-keys` adds a thread writing keys of the root blackboard
like a ROS topic monitor: use them if your deployed application does that.

What it measures is the library overhead on the real tree shape. The work of
the real nodes (ROS calls, computation) is not included, and the proportion of
"waiting" ticks vs "transition" ticks depends on `--running-ticks` and
`--delay-scale`: look at the whole distribution (p90/p99 are the busy ticks),
not only at p50.

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
