// Per-tick latency distribution of synthetic trees under realistic conditions.
//
// bt_bench measures the average of back-to-back ticks of a small tree: the tree
// stays in L1/L2 and the number is a best case. A robot control loop instead
// ticks once per period while other code runs in between, so caches are cold
// and the tail latency matters as much as the median.
// See latency_harness.h for the conditions (hot, evict, periodic, frag).
//
// Usage: bt_latency [options]
#include "bench_common.h"
#include "latency_harness.h"

using namespace bench;

int main(int argc, char** argv)
{
  LatencyOptions opt;
  for(int i = 1; i < argc; i++)
  {
    if(!opt.parse(argv[i]))
    {
      std::fprintf(stderr, "unknown argument: %s\noptions:\n%s", argv[i],
                   LatencyOptions::usage);
      return 1;
    }
  }

  struct TreeSpec
  {
    std::string name;
    std::string xml;
    std::function<void(BT::Blackboard&)> setup;
  };
  const std::vector<TreeSpec> trees = {
    { "Seq100", repeatLeaf("Sequence", "<Ok/>", 100), [](BT::Blackboard&) {} },
    { "Ports100", repeatLeaf("Sequence", "<ReadInt value=\"{x}\"/>", 100),
      [](BT::Blackboard& bb) { bb.set("x", 1); } },
    { "Robot2k", robotLikeTree(100), setupRobotBlackboard },
  };

  LatencyRunner runner(opt);
  for(const auto& spec : trees)
  {
    auto make = [&spec]() {
      struct Holder
      {
        BT::BehaviorTreeFactory factory;
        BT::Tree tree;
      };
      auto h = std::make_shared<Holder>();
      registerBenchNodes(h->factory);
      auto bb = BT::Blackboard::create();
      spec.setup(*bb);
      h->tree = h->factory.createTreeFromText(spec.xml, bb);
      return TreeInstance{ h, &h->tree };
    };
    auto check = [](BT::NodeStatus s) -> std::string {
      return s == BT::NodeStatus::SUCCESS ? "" : "tree did not return SUCCESS";
    };
    runner.run(spec.name, make, check);
  }
  return runner.finish();
}
