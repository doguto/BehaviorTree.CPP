#!/usr/bin/env python3
"""Compare two benchmark result sets produced by run.sh.

    compare.py <base> <new> [--threshold PCT]

<base>/<new> are result directories (time.json, memory.json, latency.json)
or single JSON files.
Lower is better for every metric. Uses only the standard library.
"""
import argparse
import json
import os
import sys


def load_time(path):
    """name -> cpu_time (ns), preferring the median aggregate."""
    with open(path) as f:
        data = json.load(f)
    out = {}
    for b in data.get("benchmarks", []):
        if b.get("error_occurred"):
            continue
        name = b.get("run_name", b["name"])
        agg = b.get("aggregate_name")
        if agg is not None and agg != "median":
            continue
        unit = {"ns": 1, "us": 1e3, "ms": 1e6, "s": 1e9}[b.get("time_unit", "ns")]
        out[name] = b["cpu_time"] * unit
    return out


def load_memory(path):
    with open(path) as f:
        data = json.load(f)
    out = {}
    for r in data["results"]:
        for k, v in r.items():
            if k not in ("name", "nodes"):
                out[f'{r["name"]} [{k}]'] = v
    return out


def load_latency(path):
    """Only the statistics: the *_spread_pct values describe noise, not speed."""
    return {k: v for k, v in load_memory(path).items() if "spread" not in k}


def collect(p):
    if os.path.isdir(p):
        files = [("time.json", load_time), ("memory.json", load_memory),
                 ("latency.json", load_latency)]
        return tuple(fn(os.path.join(p, f)) if os.path.exists(os.path.join(p, f)) else {}
                     for f, fn in files)
    d = json.load(open(p))
    if "benchmarks" in d:
        return load_time(p), {}, {}
    if "context" in d:
        return {}, {}, load_latency(p)
    return {}, load_memory(p), {}


def fmt_time(ns):
    for unit, scale in (("s", 1e9), ("ms", 1e6), ("us", 1e3)):
        if ns >= scale:
            return f"{ns / scale:.2f} {unit}"
    return f"{ns:.1f} ns"


# Only the median of the latency distribution is stable between runs (measured:
# ~3% between two runs of the same commit, while p90/mean moved up to 50% and the
# max up to 400%). The other statistics are shown but never count as regressions.
def latency_gate(name):
    return "[p50_ns]" in name


def table(title, base, new, fmt, threshold, gate=lambda name: True):
    names = [n for n in base if n in new]
    if not names:
        return 0
    print(f"\n{title}")
    width = max(len(n) for n in names)
    print(f"{'':{width}}  {'base':>12}  {'new':>12}  {'change':>9}")
    regress = 0
    for n in names:
        b, v = base[n], new[n]
        if b == 0 and v == 0:
            continue
        pct = (v - b) / b * 100 if b else float("inf")
        mark = ""
        if not gate(n):
            mark = "  (info)" if abs(pct) > threshold else ""
        elif pct > threshold:
            mark, regress = "  SLOWER" if title.startswith("Time") else "  WORSE", regress + 1
        elif pct < -threshold:
            mark = "  better"
        print(f"{n:{width}}  {fmt(b):>12}  {fmt(v):>12}  {pct:+8.1f}%{mark}")
    return regress


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("base")
    ap.add_argument("new")
    ap.add_argument("--threshold", type=float, default=10.0,
                    help="percent change to flag (default 10)")
    a = ap.parse_args()
    bt, bm, bl = collect(a.base)
    nt, nm, nl = collect(a.new)
    r = table("Time (cpu, median)", bt, nt, fmt_time, a.threshold)
    r += table("Memory / allocations", bm, nm, lambda x: f"{x:,.1f}", a.threshold)
    r += table("Time: per-tick latency (wall)", bl, nl, fmt_time, a.threshold,
               latency_gate)
    print(f"\n{r} metric(s) worse than +{a.threshold}%")
    return 1 if r else 0


if __name__ == "__main__":
    sys.exit(main())
