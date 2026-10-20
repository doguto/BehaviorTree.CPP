// Direct Blackboard / Any micro-benchmarks (no tree involved).
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/scripting/script_parser.hpp"

#include <string>
#include <vector>

#include <benchmark/benchmark.h>

namespace
{

std::vector<std::string> makeKeys(int n)
{
  std::vector<std::string> keys;
  keys.reserve(n);
  for(int i = 0; i < n; i++)
  {
    keys.push_back("some_entry_key_" + std::to_string(i));
  }
  return keys;
}

void BM_Blackboard_GetInt(benchmark::State& state)
{
  const auto keys = makeKeys(state.range(0));
  auto bb = BT::Blackboard::create();
  for(const auto& k : keys)
  {
    bb->set(k, 1);
  }
  size_t i = 0;
  for(auto _ : state)
  {
    benchmark::DoNotOptimize(bb->get<int>(keys[i]));
    i = (i + 1) % keys.size();
  }
}

void BM_Blackboard_SetInt_Existing(benchmark::State& state)
{
  const auto keys = makeKeys(state.range(0));
  auto bb = BT::Blackboard::create();
  for(const auto& k : keys)
  {
    bb->set(k, 1);
  }
  size_t i = 0;
  int v = 0;
  for(auto _ : state)
  {
    bb->set(keys[i], v++);
    i = (i + 1) % keys.size();
  }
}

void BM_Blackboard_SetString_Existing(benchmark::State& state)
{
  auto bb = BT::Blackboard::create();
  const std::string value(64, 'x');
  bb->set("s", value);
  for(auto _ : state)
  {
    bb->set("s", value);
  }
}

// Blackboard creation + N new entries (what a tree load does per subtree).
void BM_Blackboard_CreateAndFill(benchmark::State& state)
{
  const auto keys = makeKeys(state.range(0));
  for(auto _ : state)
  {
    auto bb = BT::Blackboard::create();
    for(const auto& k : keys)
    {
      bb->set(k, 1);
    }
    benchmark::DoNotOptimize(bb);
  }
  state.SetItemsProcessed(state.iterations() * keys.size());
}

// Reading from a child blackboard whose entry lives in the parent (depth hops).
void BM_Blackboard_GetFromParent(benchmark::State& state)
{
  auto root = BT::Blackboard::create();
  root->set("x", 1);
  auto bb = root;
  for(int i = 0; i < state.range(0); i++)
  {
    bb = BT::Blackboard::create(bb);
    bb->enableAutoRemapping(true);
  }
  for(auto _ : state)
  {
    benchmark::DoNotOptimize(bb->get<int>("x"));
  }
}

void BM_Any_CopyInt(benchmark::State& state)
{
  BT::Any a(42);
  for(auto _ : state)
  {
    BT::Any b(a);
    benchmark::DoNotOptimize(b);
  }
}

void BM_Any_CopyString(benchmark::State& state)
{
  BT::Any a(std::string(64, 'x'));
  for(auto _ : state)
  {
    BT::Any b(a);
    benchmark::DoNotOptimize(b);
  }
}

void BM_Any_CastInt(benchmark::State& state)
{
  BT::Any a(42);
  for(auto _ : state)
  {
    benchmark::DoNotOptimize(a.cast<int>());
  }
}

void BM_Any_CastIntToDouble(benchmark::State& state)
{
  BT::Any a(42);
  for(auto _ : state)
  {
    benchmark::DoNotOptimize(a.cast<double>());
  }
}

void BM_Script_ParseAndExecute(benchmark::State& state)
{
  auto bb = BT::Blackboard::create();
  bb->set("x", 10);
  BT::Ast::Environment env = { bb, {} };
  for(auto _ : state)
  {
    benchmark::DoNotOptimize(BT::ParseScriptAndExecute(env, "x > 5 && x < 100"));
  }
}

void BM_Script_ExecutePreParsed(benchmark::State& state)
{
  auto bb = BT::Blackboard::create();
  bb->set("x", 10);
  BT::Ast::Environment env = { bb, {} };
  auto fn = BT::ParseScript("x > 5 && x < 100");
  for(auto _ : state)
  {
    benchmark::DoNotOptimize(fn.value()(env));
  }
}

}  // namespace

BENCHMARK(BM_Blackboard_GetInt)->Arg(1)->Arg(100)->Arg(10000);
BENCHMARK(BM_Blackboard_SetInt_Existing)->Arg(1)->Arg(100)->Arg(10000);
BENCHMARK(BM_Blackboard_SetString_Existing);
BENCHMARK(BM_Blackboard_CreateAndFill)->Arg(10)->Arg(1000);
BENCHMARK(BM_Blackboard_GetFromParent)->Arg(1);
BENCHMARK(BM_Any_CopyInt);
BENCHMARK(BM_Any_CopyString);
BENCHMARK(BM_Any_CastInt);
BENCHMARK(BM_Any_CastIntToDouble);
BENCHMARK(BM_Script_ParseAndExecute);
BENCHMARK(BM_Script_ExecutePreParsed);
