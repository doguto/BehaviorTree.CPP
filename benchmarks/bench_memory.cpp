// Memory / allocation report. Global operator new/delete are instrumented, so
// every C++ allocation made by the library (and by this program) is counted.
//
// Reports, per scenario:
//   - bytes and allocations needed to build a tree, normalised per node
//   - heap allocations performed by a steady-state tick (ideally 0)
//
// Usage: bt_membench [--json=FILE]
#include "bench_common.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <new>
#include <vector>

#include <malloc.h>

namespace
{
std::atomic<int64_t> g_live_bytes{ 0 };
std::atomic<int64_t> g_allocs{ 0 };
std::atomic<int64_t> g_total_bytes{ 0 };

inline void* track(void* p)
{
  if(p == nullptr)
  {
    throw std::bad_alloc();
  }
  const auto sz = static_cast<int64_t>(malloc_usable_size(p));
  g_live_bytes.fetch_add(sz, std::memory_order_relaxed);
  g_total_bytes.fetch_add(sz, std::memory_order_relaxed);
  g_allocs.fetch_add(1, std::memory_order_relaxed);
  return p;
}

inline void untrack(void* p) noexcept
{
  if(p != nullptr)
  {
    g_live_bytes.fetch_sub(static_cast<int64_t>(malloc_usable_size(p)),
                           std::memory_order_relaxed);
    std::free(p);
  }
}
}  // namespace

void* operator new(std::size_t n)
{
  return track(std::malloc(n));
}
void* operator new[](std::size_t n)
{
  return track(std::malloc(n));
}
void* operator new(std::size_t n, const std::nothrow_t&) noexcept
{
  void* p = std::malloc(n);
  if(p)
  {
    track(p);
  }
  return p;
}
void* operator new(std::size_t n, std::align_val_t a)
{
  return track(
      std::aligned_alloc(static_cast<size_t>(a), (n + static_cast<size_t>(a) - 1) &
                                                     ~(static_cast<size_t>(a) - 1)));
}
void operator delete(void* p) noexcept
{
  untrack(p);
}
void operator delete[](void* p) noexcept
{
  untrack(p);
}
void operator delete(void* p, std::size_t) noexcept
{
  untrack(p);
}
void operator delete[](void* p, std::size_t) noexcept
{
  untrack(p);
}
void operator delete(void* p, std::align_val_t) noexcept
{
  untrack(p);
}
void operator delete(void* p, std::size_t, std::align_val_t) noexcept
{
  untrack(p);
}

namespace
{
using namespace bench;

struct Snapshot
{
  int64_t live, allocs, total;
};
Snapshot snap()
{
  return { g_live_bytes.load(), g_allocs.load(), g_total_bytes.load() };
}

struct Result
{
  std::string name;
  std::vector<std::pair<std::string, double>> metrics;
};
std::vector<Result> g_results;

void add(const std::string& name, std::vector<std::pair<std::string, double>> m)
{
  g_results.push_back({ name, std::move(m) });
}

// Footprint of a loaded tree, per node (live bytes still held after creation,
// and total allocation count during creation, minus the empty-factory baseline).
void footprint(const std::string& name, const std::string& xml, int nodes,
               const std::function<void(BT::Blackboard&)>& setup = {})
{
  BT::BehaviorTreeFactory f;
  registerBenchNodes(f);
  auto bb = BT::Blackboard::create();
  if(setup)
  {
    setup(*bb);
  }
  const auto before = snap();
  auto tree = std::make_unique<BT::Tree>(f.createTreeFromText(xml, bb));
  const auto after = snap();
  const double live = double(after.live - before.live);
  const double allocs = double(after.allocs - before.allocs);
  const double peak_total = double(after.total - before.total);
  add(name, { { "nodes", double(nodes) },
              { "live_bytes", live },
              { "live_bytes_per_node", live / nodes },
              { "allocs", allocs },
              { "allocs_per_node", allocs / nodes },
              { "alloc_bytes_total", peak_total } });
  tree.reset();
}

// Allocations performed by one steady-state tick, after warm-up.
void tickAllocs(const std::string& name, const std::string& xml,
                const std::function<void(BT::Blackboard&)>& setup = {})
{
  BT::BehaviorTreeFactory f;
  registerBenchNodes(f);
  auto bb = BT::Blackboard::create();
  if(setup)
  {
    setup(*bb);
  }
  auto tree = f.createTreeFromText(xml, bb);
  for(int i = 0; i < 10; i++)
  {
    tree.tickExactlyOnce();
  }
  constexpr int kTicks = 100;
  const auto before = snap();
  for(int i = 0; i < kTicks; i++)
  {
    tree.tickExactlyOnce();
  }
  const auto after = snap();
  add(name, { { "allocs_per_tick", double(after.allocs - before.allocs) / kTicks },
              { "bytes_per_tick", double(after.total - before.total) / kTicks } });
}

void writeJson(const std::string& path)
{
  std::ofstream out(path);
  out << "{\n  \"results\": [\n";
  for(size_t i = 0; i < g_results.size(); i++)
  {
    out << "    {\"name\": \"" << g_results[i].name << "\"";
    for(const auto& [k, v] : g_results[i].metrics)
    {
      out << ", \"" << k << "\": " << v;
    }
    out << "}" << (i + 1 < g_results.size() ? "," : "") << "\n";
  }
  out << "  ]\n}\n";
}

void printTable()
{
  for(const auto& r : g_results)
  {
    std::printf("%-44s", r.name.c_str());
    for(const auto& [k, v] : r.metrics)
    {
      std::printf("  %s=%.1f", k.c_str(), v);
    }
    std::printf("\n");
  }
}
}  // namespace

int main(int argc, char** argv)
{
  std::string json_path;
  for(int i = 1; i < argc; i++)
  {
    if(std::strncmp(argv[i], "--json=", 7) == 0)
    {
      json_path = argv[i] + 7;
    }
  }

  // ---- factory ----
  {
    const auto before = snap();
    auto f = std::make_unique<BT::BehaviorTreeFactory>();
    const auto after = snap();
    add("Factory/construct", { { "live_bytes", double(after.live - before.live) },
                               { "allocs", double(after.allocs - before.allocs) } });
  }

  // ---- tree footprint ----
  const int kN = 1000;
  footprint("Footprint/Sequence_of_Ok", repeatLeaf("Sequence", "<Ok/>", kN), kN + 1);
  footprint("Footprint/Sequence_of_ReadInt_bb",
            repeatLeaf("Sequence", "<ReadInt value=\"{x}\"/>", kN), kN + 1,
            [](BT::Blackboard& bb) { bb.set("x", 1); });
  footprint("Footprint/Sequence_of_ReadInt_literal",
            repeatLeaf("Sequence", "<ReadInt value=\"42\"/>", kN), kN + 1);
  footprint("Footprint/Sequence_of_PreCondition",
            repeatLeaf("Sequence", "<Ok _skipIf=\"x &gt; 100\"/>", kN), kN + 1,
            [](BT::Blackboard& bb) { bb.set("x", 1); });
  footprint("Footprint/Nested_Sequence_depth64", nested("Sequence", 64), 65);

  // ---- steady-state tick allocations ----
  tickAllocs("Tick/Sequence_of_Ok", repeatLeaf("Sequence", "<Ok/>", 100));
  tickAllocs("Tick/ReadInt_bb", repeatLeaf("Sequence", "<ReadInt value=\"{x}\"/>", 100),
             [](BT::Blackboard& bb) { bb.set("x", 1); });
  tickAllocs("Tick/ReadInt_literal",
             repeatLeaf("Sequence", "<ReadInt value=\"42\"/>", 100));
  tickAllocs("Tick/ReadInt_default", repeatLeaf("Sequence", "<ReadIntDefault/>", 100));
  tickAllocs("Tick/ReadString_bb",
             repeatLeaf("Sequence", "<ReadString value=\"{s}\"/>", 100),
             [](BT::Blackboard& bb) { bb.set("s", std::string(64, 'x')); });
  tickAllocs("Tick/WriteInt_bb", repeatLeaf("Sequence", "<WriteInt value=\"{x}\"/>", 100),
             [](BT::Blackboard& bb) { bb.set("x", 1); });
  tickAllocs("Tick/PreCondition",
             repeatLeaf("Sequence", "<Ok _skipIf=\"x &gt; 100\"/>", 100),
             [](BT::Blackboard& bb) { bb.set("x", 1); });
  tickAllocs("Tick/ReactiveSequence_Running",
             repeatLeaf("ReactiveSequence", "<Ok/>", 100, "<Running/>"));

  printTable();
  if(!json_path.empty())
  {
    writeJson(json_path);
    std::printf("\nwrote %s\n", json_path.c_str());
  }
  return 0;
}
