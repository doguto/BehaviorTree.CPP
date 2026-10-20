// Tick-throughput benchmarks. Every iteration is one Tree::tickExactlyOnce().
// items_processed counts nodes in the tree, so "items/s" is comparable across sizes.
#include "bench_common.h"

#include <benchmark/benchmark.h>

using namespace bench;

namespace
{

void runTicks(benchmark::State& state, BT::Tree& tree, int64_t nodes_per_tick,
              BT::NodeStatus expected = BT::NodeStatus::SUCCESS)
{
  if(!checkFirstTick(state, tree, expected))
  {
    return;
  }
  for(auto _ : state)
  {
    benchmark::DoNotOptimize(tree.tickExactlyOnce());
  }
  state.SetItemsProcessed(state.iterations() * nodes_per_tick);
}

void BM_Tick_Sequence(benchmark::State& state)
{
  const int n = state.range(0);
  BT::BehaviorTreeFactory f;
  auto tree = makeTree(f, repeatLeaf("Sequence", "<Ok/>", n));
  runTicks(state, tree, n + 1);
}

void BM_Tick_Fallback(benchmark::State& state)
{
  const int n = state.range(0);
  BT::BehaviorTreeFactory f;
  auto tree = makeTree(f, repeatLeaf("Fallback", "<Fail/>", n, "<Ok/>"));
  runTicks(state, tree, n + 2);
}

void BM_Tick_Nested(benchmark::State& state)
{
  const int d = state.range(0);
  BT::BehaviorTreeFactory f;
  auto tree = makeTree(f, nested("Sequence", d));
  runTicks(state, tree, d + 1);
}

void BM_Tick_Decorators(benchmark::State& state)
{
  const int d = state.range(0);  // number of Inverter pairs (result stays SUCCESS)
  BT::BehaviorTreeFactory f;
  auto tree = makeTree(f, nested("Inverter", d * 2));
  runTicks(state, tree, d * 2 + 1);
}

// Tree stays RUNNING: ReactiveSequence re-ticks all the N preceding children.
void BM_Tick_ReactiveSequence_Running(benchmark::State& state)
{
  const int n = state.range(0);
  BT::BehaviorTreeFactory f;
  auto tree = makeTree(f, repeatLeaf("ReactiveSequence", "<Ok/>", n, "<Running/>"));
  runTicks(state, tree, n + 2, BT::NodeStatus::RUNNING);
}

void BM_Tick_Parallel(benchmark::State& state)
{
  const int n = state.range(0);
  BT::BehaviorTreeFactory f;
  auto tree = makeTree(f, repeatLeaf("Parallel", "<Ok/>", n));
  runTicks(state, tree, n + 1);
}

// Nodes with a script pre-condition evaluated on every tick.
void BM_Tick_PreCondition(benchmark::State& state)
{
  const int n = state.range(0);
  BT::BehaviorTreeFactory f;
  auto bb = BT::Blackboard::create();
  bb->set("x", 10);
  auto tree = makeTree(f, repeatLeaf("Sequence", "<Ok _skipIf=\"x &gt; 100\"/>", n), bb);
  runTicks(state, tree, n + 1);
}

// Cost of installed instrumentation callbacks (monitor / pre / post tick).
enum class Hook
{
  Monitor,
  PreTick,
  PostTick
};

template <Hook H>
void BM_Tick_WithCallback(benchmark::State& state)
{
  const int n = state.range(0);
  BT::BehaviorTreeFactory f;
  auto tree = makeTree(f, repeatLeaf("Sequence", "<Ok/>", n));
  int64_t counter = 0;
  tree.applyVisitor([&](BT::TreeNode* node) {
    if constexpr(H == Hook::Monitor)
    {
      node->setTickMonitorCallback([&counter](BT::TreeNode&, BT::NodeStatus,
                                              std::chrono::microseconds) { counter++; });
    }
    else if constexpr(H == Hook::PreTick)
    {
      node->setPreTickFunction([&counter](BT::TreeNode&) {
        counter++;
        return BT::NodeStatus::IDLE;
      });
    }
    else
    {
      node->setPostTickFunction([&counter](BT::TreeNode&, BT::NodeStatus s) {
        counter++;
        return s;
      });
    }
  });
  runTicks(state, tree, n + 1);
  benchmark::DoNotOptimize(counter);
}

// A subscriber on every node's status changes (what loggers/observers do).
void BM_Tick_StatusSubscribers(benchmark::State& state)
{
  const int n = state.range(0);
  BT::BehaviorTreeFactory f;
  auto tree = makeTree(f, repeatLeaf("Sequence", "<Ok/>", n));
  int64_t counter = 0;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subs;
  tree.applyVisitor([&](BT::TreeNode* node) {
    subs.push_back(node->subscribeToStatusChange(
        [&counter](BT::TimePoint, const BT::TreeNode&, BT::NodeStatus, BT::NodeStatus) {
          counter++;
        }));
  });
  runTicks(state, tree, n + 1);
  benchmark::DoNotOptimize(counter);
}

}  // namespace

BENCHMARK(BM_Tick_Sequence)->Arg(1)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_Tick_Fallback)->Arg(10)->Arg(100)->Arg(1000);
BENCHMARK(BM_Tick_Nested)->Arg(4)->Arg(16)->Arg(64);
BENCHMARK(BM_Tick_Decorators)->Arg(2)->Arg(16);
BENCHMARK(BM_Tick_ReactiveSequence_Running)->Arg(10)->Arg(100);
BENCHMARK(BM_Tick_Parallel)->Arg(10)->Arg(100);
BENCHMARK(BM_Tick_PreCondition)->Arg(10)->Arg(100);
BENCHMARK_TEMPLATE(BM_Tick_WithCallback, Hook::Monitor)->Arg(100);
BENCHMARK_TEMPLATE(BM_Tick_WithCallback, Hook::PreTick)->Arg(100);
BENCHMARK_TEMPLATE(BM_Tick_WithCallback, Hook::PostTick)->Arg(100);
BENCHMARK(BM_Tick_StatusSubscribers)->Arg(100);
