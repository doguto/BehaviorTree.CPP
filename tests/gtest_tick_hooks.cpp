// Behavior of the optional per-node hooks: injected tick callbacks,
// status-change subscribers and waitValidStatus().
// executeTick()/setStatus() skip work when these are unused, so make sure
// they still work when they are set late, replaced or removed.
#include "behaviortree_cpp/bt_factory.h"

#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>

using namespace BT;

namespace
{
const char* kSingleLeafXML = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Sequence>
      <AlwaysSuccess name="leaf"/>
    </Sequence>
  </BehaviorTree>
</root>)";

TreeNode* findNode(Tree& tree, const std::string& name)
{
  TreeNode* found = nullptr;
  tree.applyVisitor([&](TreeNode* node) {
    if(node->name() == name)
    {
      found = node;
    }
  });
  return found;
}

class KeepRunning : public StatefulActionNode
{
public:
  KeepRunning(const std::string& name, const NodeConfig& config)
    : StatefulActionNode(name, config)
  {}
  static PortsList providedPorts()
  {
    return {};
  }
  NodeStatus onStart() override
  {
    return NodeStatus::RUNNING;
  }
  NodeStatus onRunning() override
  {
    return NodeStatus::RUNNING;
  }
  void onHalted() override
  {}
};
}  // namespace

TEST(TickHooks, CallbacksInstalledAfterFirstTick)
{
  BehaviorTreeFactory factory;
  auto tree = factory.createTreeFromText(kSingleLeafXML);
  auto* leaf = findNode(tree, "leaf");
  ASSERT_NE(leaf, nullptr);

  // first tick without any callback
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);

  int pre = 0;
  int post = 0;
  int monitor = 0;
  leaf->setPreTickFunction([&](TreeNode&) {
    pre++;
    return NodeStatus::IDLE;
  });
  leaf->setPostTickFunction([&](TreeNode&, NodeStatus s) {
    post++;
    return s;
  });
  leaf->setTickMonitorCallback([&](TreeNode&, NodeStatus s, std::chrono::microseconds d) {
    EXPECT_EQ(s, NodeStatus::SUCCESS);
    EXPECT_GE(d.count(), 0);
    monitor++;
  });

  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(pre, 1);
  EXPECT_EQ(post, 1);
  EXPECT_EQ(monitor, 1);

  // Removing only some callbacks keeps the remaining ones active
  leaf->setPreTickFunction({});
  leaf->setTickMonitorCallback({});
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(pre, 1);
  EXPECT_EQ(post, 2);
  EXPECT_EQ(monitor, 1);

  // Removing all of them
  leaf->setPostTickFunction({});
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(post, 2);
}

TEST(TickHooks, PreTickSubstitutionAfterFirstTick)
{
  BehaviorTreeFactory factory;
  auto tree = factory.createTreeFromText(kSingleLeafXML);
  auto* leaf = findNode(tree, "leaf");
  ASSERT_NE(leaf, nullptr);
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);

  int monitor = 0;
  leaf->setTickMonitorCallback(
      [&](TreeNode&, NodeStatus, std::chrono::microseconds) { monitor++; });
  leaf->setPreTickFunction([](TreeNode&) { return NodeStatus::FAILURE; });
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::FAILURE);
  // tick() was substituted, so the monitor is not called
  EXPECT_EQ(monitor, 0);
}

TEST(TickHooks, StatusChangeSubscriber)
{
  BehaviorTreeFactory factory;
  auto tree = factory.createTreeFromText(kSingleLeafXML);
  auto* leaf = findNode(tree, "leaf");
  ASSERT_NE(leaf, nullptr);
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);

  std::vector<std::pair<NodeStatus, NodeStatus>> transitions;
  auto sub = leaf->subscribeToStatusChange(
      [&](TimePoint stamp, const TreeNode&, NodeStatus prev, NodeStatus next) {
        EXPECT_GT(stamp.time_since_epoch().count(), 0);
        transitions.emplace_back(prev, next);
      });

  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  ASSERT_EQ(transitions.size(), 2u);
  EXPECT_EQ(transitions[0], std::make_pair(NodeStatus::IDLE, NodeStatus::SUCCESS));
  EXPECT_EQ(transitions[1], std::make_pair(NodeStatus::SUCCESS, NodeStatus::IDLE));

  // An expired subscriber is not called anymore
  sub.reset();
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(transitions.size(), 2u);

  // and a new one can be added afterwards
  int count = 0;
  auto sub2 = leaf->subscribeToStatusChange(
      [&](TimePoint, const TreeNode&, NodeStatus, NodeStatus) { count++; });
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(count, 2);
}

TEST(TickHooks, WaitValidStatusIsWokenUpByTick)
{
  BehaviorTreeFactory factory;
  factory.registerNodeType<KeepRunning>("KeepRunning");
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Sequence>
      <KeepRunning name="leaf"/>
    </Sequence>
  </BehaviorTree>
</root>)");
  auto* leaf = findNode(tree, "leaf");
  ASSERT_NE(leaf, nullptr);
  ASSERT_EQ(leaf->status(), NodeStatus::IDLE);

  // Several threads block until the node leaves IDLE
  std::vector<std::future<NodeStatus>> waiters;
  for(int i = 0; i < 3; i++)
  {
    waiters.push_back(
        std::async(std::launch::async, [leaf] { return leaf->waitValidStatus(); }));
  }
  // give them time to actually block on the condition variable
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  for(auto& w : waiters)
  {
    ASSERT_EQ(w.wait_for(std::chrono::milliseconds(0)), std::future_status::timeout);
  }

  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);
  for(auto& w : waiters)
  {
    ASSERT_EQ(w.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_EQ(w.get(), NodeStatus::RUNNING);
  }

  // Once nobody waits anymore, status changes still work normally
  tree.haltTree();
  EXPECT_EQ(leaf->status(), NodeStatus::IDLE);
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(leaf->waitValidStatus(), NodeStatus::RUNNING);
  tree.haltTree();
}
