// getInput / setOutput cost, measured through real nodes in a Sequence of N leaves.
#include "bench_common.h"

#include <benchmark/benchmark.h>

using namespace bench;

namespace
{

void runPortBench(benchmark::State& state, const std::string& leaf,
                  const std::function<void(BT::Blackboard&)>& setup)
{
  const int n = state.range(0);
  BT::BehaviorTreeFactory f;
  auto bb = BT::Blackboard::create();
  setup(*bb);
  auto tree = makeTree(f, repeatLeaf("Sequence", leaf, n), bb);
  if(!checkFirstTick(state, tree))
  {
    return;
  }
  for(auto _ : state)
  {
    benchmark::DoNotOptimize(tree.tickExactlyOnce());
  }
  // items = port accesses
  state.SetItemsProcessed(state.iterations() * n);
}

void BM_Port_GetInt_Blackboard(benchmark::State& state)
{
  runPortBench(state, "<ReadInt value=\"{x}\"/>",
               [](BT::Blackboard& bb) { bb.set("x", 42); });
}

void BM_Port_GetInt_Literal(benchmark::State& state)
{
  runPortBench(state, "<ReadInt value=\"42\"/>", [](BT::Blackboard&) {});
}

void BM_Port_GetInt_Default(benchmark::State& state)
{
  runPortBench(state, "<ReadIntDefault/>", [](BT::Blackboard&) {});
}

// Blackboard holds the string "42", port wants int -> parse on each read.
void BM_Port_GetInt_FromStringEntry(benchmark::State& state)
{
  runPortBench(state, "<ReadInt value=\"{x}\"/>",
               [](BT::Blackboard& bb) { bb.set("x", std::string("42")); });
}

void BM_Port_GetString_Blackboard(benchmark::State& state)
{
  runPortBench(state, "<ReadString value=\"{s}\"/>", [](BT::Blackboard& bb) {
    bb.set("s", std::string("a reasonably long string value, beyond any SSO buffer"));
  });
}

void BM_Port_GetString_Literal(benchmark::State& state)
{
  runPortBench(state,
               "<ReadString value=\"a literal string value, beyond any SSO buf\"/>",
               [](BT::Blackboard&) {});
}

void BM_Port_SetInt(benchmark::State& state)
{
  runPortBench(state, "<WriteInt value=\"{x}\"/>",
               [](BT::Blackboard& bb) { bb.set("x", 1); });
}

// Entry lives in the parent blackboard (SubTree with remapping) -> extra lookup hops.
void BM_Port_GetInt_ThroughSubtree(benchmark::State& state)
{
  const int n = state.range(0);
  std::string leaves;
  for(int i = 0; i < n; i++)
  {
    leaves += "<ReadInt value=\"{in}\"/>";
  }
  const std::string xml = "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">"
                          "<BehaviorTree ID=\"MainTree\"><SubTree ID=\"Sub\" "
                          "in=\"{x}\"/></BehaviorTree>"
                          "<BehaviorTree ID=\"Sub\"><Sequence>" +
                          leaves + "</Sequence></BehaviorTree></root>";
  BT::BehaviorTreeFactory f;
  auto bb = BT::Blackboard::create();
  bb->set("x", 42);
  auto tree = makeTree(f, xml, bb);
  if(!checkFirstTick(state, tree))
  {
    return;
  }
  for(auto _ : state)
  {
    benchmark::DoNotOptimize(tree.tickExactlyOnce());
  }
  state.SetItemsProcessed(state.iterations() * n);
}

}  // namespace

BENCHMARK(BM_Port_GetInt_Blackboard)->Arg(100);
BENCHMARK(BM_Port_GetInt_Literal)->Arg(100);
BENCHMARK(BM_Port_GetInt_Default)->Arg(100);
BENCHMARK(BM_Port_GetInt_FromStringEntry)->Arg(100);
BENCHMARK(BM_Port_GetString_Blackboard)->Arg(100);
BENCHMARK(BM_Port_GetString_Literal)->Arg(100);
BENCHMARK(BM_Port_SetInt)->Arg(100);
BENCHMARK(BM_Port_GetInt_ThroughSubtree)->Arg(100);
