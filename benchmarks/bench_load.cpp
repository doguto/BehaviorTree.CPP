// Tree construction / destruction, XML parsing and factory setup.
#include "bench_common.h"

#include <benchmark/benchmark.h>

using namespace bench;

namespace
{

void BM_Factory_Construct(benchmark::State& state)
{
  for(auto _ : state)
  {
    BT::BehaviorTreeFactory f;
    benchmark::DoNotOptimize(f);
  }
}

// Parse XML + instantiate + destroy a flat tree of N leaves.
void BM_Load_CreateDestroy(benchmark::State& state)
{
  const int n = state.range(0);
  const auto xml = repeatLeaf("Sequence", "<Ok/>", n);
  BT::BehaviorTreeFactory f;
  registerBenchNodes(f);
  for(auto _ : state)
  {
    auto tree = f.createTreeFromText(xml);
    benchmark::DoNotOptimize(tree);
  }
  state.SetItemsProcessed(state.iterations() * (n + 1));
}

// Same but with ports on every node (remapping, blackboard entries).
void BM_Load_CreateDestroy_Ports(benchmark::State& state)
{
  const int n = state.range(0);
  const auto xml = repeatLeaf("Sequence", "<ReadInt value=\"{x}\"/>", n);
  BT::BehaviorTreeFactory f;
  registerBenchNodes(f);
  for(auto _ : state)
  {
    auto bb = BT::Blackboard::create();
    bb->set("x", 1);
    auto tree = f.createTreeFromText(xml, bb);
    benchmark::DoNotOptimize(tree);
  }
  state.SetItemsProcessed(state.iterations() * (n + 1));
}

// K SubTree instances of a 10-leaf tree.
void BM_Load_Subtrees(benchmark::State& state)
{
  const int k = state.range(0);
  std::string subs;
  for(int i = 0; i < k; i++)
  {
    subs += "<SubTree ID=\"Sub\"/>";
  }
  const std::string xml = "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">"
                          "<BehaviorTree ID=\"MainTree\"><Sequence>" +
                          subs + "</Sequence></BehaviorTree><BehaviorTree ID=\"Sub\">" +
                          "<Sequence><Ok/><Ok/><Ok/><Ok/><Ok/><Ok/><Ok/><Ok/><Ok/><Ok/></"
                          "Sequence>"
                          "</BehaviorTree></root>";
  BT::BehaviorTreeFactory f;
  registerBenchNodes(f);
  for(auto _ : state)
  {
    auto tree = f.createTreeFromText(xml);
    benchmark::DoNotOptimize(tree);
  }
  state.SetItemsProcessed(state.iterations() * k * 11);
}

// Only destruction cost (creation excluded from timing).
void BM_Load_DestroyOnly(benchmark::State& state)
{
  const int n = state.range(0);
  const auto xml = repeatLeaf("Sequence", "<Ok/>", n);
  BT::BehaviorTreeFactory f;
  registerBenchNodes(f);
  for(auto _ : state)
  {
    state.PauseTiming();
    auto tree = std::make_unique<BT::Tree>(f.createTreeFromText(xml));
    state.ResumeTiming();
    tree.reset();
  }
  state.SetItemsProcessed(state.iterations() * (n + 1));
}

// First tick right after creation (tree initialization path).
void BM_Load_FirstTick(benchmark::State& state)
{
  const int n = state.range(0);
  const auto xml = repeatLeaf("Sequence", "<Ok/>", n);
  BT::BehaviorTreeFactory f;
  registerBenchNodes(f);
  for(auto _ : state)
  {
    state.PauseTiming();
    auto tree = std::make_unique<BT::Tree>(f.createTreeFromText(xml));
    state.ResumeTiming();
    benchmark::DoNotOptimize(tree->tickExactlyOnce());
    state.PauseTiming();
    tree.reset();
    state.ResumeTiming();
  }
}

}  // namespace

BENCHMARK(BM_Factory_Construct);
BENCHMARK(BM_Load_CreateDestroy)
    ->Arg(10)
    ->Arg(100)
    ->Arg(1000)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_Load_CreateDestroy_Ports)
    ->Arg(100)
    ->Arg(1000)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_Load_Subtrees)->Arg(10)->Arg(100)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_Load_DestroyOnly)->Arg(100)->Arg(1000)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_Load_FirstTick)->Arg(100)->Unit(benchmark::kMicrosecond);
