// Shared helpers for the benchmarks: custom nodes and XML tree generators.
#pragma once

#include "behaviortree_cpp/bt_factory.h"

#include <sstream>
#include <string>

namespace bench
{

// Leaf that returns a fixed status without touching ports.
class FixedStatus : public BT::SyncActionNode
{
public:
  FixedStatus(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}
  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

class AlwaysFail : public FixedStatus
{
public:
  using FixedStatus::FixedStatus;
  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::FAILURE;
  }
};

// Stateful leaf that stays RUNNING forever (until halted).
class Running : public BT::StatefulActionNode
{
public:
  Running(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
  {}
  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus onStart() override
  {
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override
  {}
};

// Leaf that reads one input port of type T.
template <typename T>
class ReadPort : public BT::SyncActionNode
{
public:
  ReadPort(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<T>("value") };
  }
  BT::NodeStatus tick() override
  {
    T v{};
    if(!getInput("value", v))
    {
      return BT::NodeStatus::FAILURE;
    }
    benchmark_sink = sizeof(v);
    return BT::NodeStatus::SUCCESS;
  }
  static inline volatile size_t benchmark_sink = 0;
};

// Same as ReadPort<int> but the port declares a default value.
class ReadPortDefault : public BT::SyncActionNode
{
public:
  ReadPortDefault(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("value", 7, "") };
  }
  BT::NodeStatus tick() override
  {
    int v = 0;
    return getInput("value", v) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

template <typename T>
class WritePort : public BT::SyncActionNode
{
public:
  WritePort(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}
  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<T>("value") };
  }
  BT::NodeStatus tick() override
  {
    return setOutput("value", T{}) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

inline void registerBenchNodes(BT::BehaviorTreeFactory& f)
{
  f.registerNodeType<FixedStatus>("Ok");
  f.registerNodeType<AlwaysFail>("Fail");
  f.registerNodeType<Running>("Running");
  f.registerNodeType<ReadPort<int>>("ReadInt");
  f.registerNodeType<ReadPort<std::string>>("ReadString");
  f.registerNodeType<ReadPortDefault>("ReadIntDefault");
  f.registerNodeType<WritePort<int>>("WriteInt");
}

// <BehaviorTree ID="MainTree"> <control> N x leaf </control> </BehaviorTree>
inline std::string wrapTree(const std::string& body)
{
  return "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">"
         "<BehaviorTree ID=\"MainTree\">" +
         body + "</BehaviorTree></root>";
}

inline std::string repeatLeaf(const std::string& control, const std::string& leaf, int n,
                              const std::string& tail = "")
{
  std::ostringstream s;
  s << "<" << control << ">";
  for(int i = 0; i < n; i++)
  {
    s << leaf;
  }
  s << tail << "</" << control << ">";
  return wrapTree(s.str());
}

// Control nodes nested `depth` levels deep, with one leaf at the bottom.
inline std::string nested(const std::string& control, int depth,
                          const std::string& leaf = "<Ok/>")
{
  std::ostringstream s;
  for(int i = 0; i < depth; i++)
  {
    s << "<" << control << ">";
  }
  s << leaf;
  for(int i = 0; i < depth; i++)
  {
    s << "</" << control << ">";
  }
  return wrapTree(s.str());
}

// A larger tree shaped like a real robot application: `tasks` SubTree instances
// (own blackboard, remapped ports), each mixing control nodes, decorators,
// port reads/writes and script pre-conditions. Every tick returns SUCCESS and
// visits all ~21 nodes of each task.
inline std::string robotLikeTree(int tasks)
{
  std::ostringstream s;
  s << "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">"
       "<BehaviorTree ID=\"MainTree\"><Sequence>";
  for(int i = 0; i < tasks; i++)
  {
    s << "<SubTree ID=\"Task\" x=\"{x}\" s=\"{s}\"/>";
  }
  s << "</Sequence></BehaviorTree>"
       "<BehaviorTree ID=\"Task\"><Sequence>"
       "<Fallback><Fail/><Inverter><Ok/></Inverter><ReadInt value=\"{x}\"/></Fallback>"
       "<ReadString value=\"{s}\"/>"
       "<WriteInt value=\"{y}\"/>"
       "<Sequence><Ok/><ReadInt value=\"{y}\"/><Ok _skipIf=\"x &gt; 1000\"/></Sequence>"
       "<ForceSuccess><Fail/></ForceSuccess>"
       "<Parallel><Ok/><Ok/><ReadInt value=\"5\"/></Parallel>"
       "<ReactiveSequence><Ok/><ReadInt value=\"{x}\"/></ReactiveSequence>"
       "</Sequence></BehaviorTree></root>";
  return s.str();
}

inline void setupRobotBlackboard(BT::Blackboard& bb)
{
  bb.set("x", 1);
  bb.set("s", std::string("a string value long enough to need a heap buffer"));
}

inline BT::Tree makeTree(BT::BehaviorTreeFactory& f, const std::string& xml,
                         BT::Blackboard::Ptr bb = {})
{
  registerBenchNodes(f);
  return f.createTreeFromText(xml, bb ? bb : BT::Blackboard::create());
}

// Ticks once and aborts the benchmark if the tree does not finish with `expected`.
// Guards against numbers that silently measure an early-failing tree.
template <typename State>
inline bool checkFirstTick(State& state, BT::Tree& tree,
                           BT::NodeStatus expected = BT::NodeStatus::SUCCESS)
{
  const auto status = tree.tickExactlyOnce();
  if(status != expected)
  {
    state.SkipWithError(("unexpected tree status: " + BT::toStr(status)).c_str());
    return false;
  }
  return true;
}

}  // namespace bench
