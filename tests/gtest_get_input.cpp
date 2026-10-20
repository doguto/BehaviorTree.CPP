// Value paths of TreeNode::getInput(): literal, default, blackboard entry of the
// same type, of a different type, stored as string, and "{=}" remapping.
#include "behaviortree_cpp/bt_factory.h"

#include <atomic>
#include <thread>

#include <gtest/gtest.h>

using namespace BT;

namespace
{
struct Pose
{
  double x = 0;
  double y = 0;
  std::string frame;
};

// Reads port "in" on every tick and stores the result in *out.
template <typename T>
class Reader : public SyncActionNode
{
public:
  Reader(const std::string& name, const NodeConfig& config, T* out)
    : SyncActionNode(name, config), out_(out)
  {}
  static PortsList providedPorts()
  {
    return { InputPort<T>("in") };
  }
  NodeStatus tick() override
  {
    // destination is intentionally reused: it must be overwritten every time
    if(auto res = getInput("in", *out_); !res)
    {
      throw RuntimeError(res.error());
    }
    return NodeStatus::SUCCESS;
  }

private:
  T* out_;
};

template <typename T>
class ReaderWithDefault : public Reader<T>
{
public:
  using Reader<T>::Reader;
  static PortsList providedPorts();
};

template <>
PortsList ReaderWithDefault<std::string>::providedPorts()
{
  return { InputPort<std::string>("in", "default text", "") };
}
template <>
PortsList ReaderWithDefault<float>::providedPorts()
{
  return { InputPort<float>("in", 2.5f, "") };
}

std::string xmlWithLeaf(const std::string& leaf)
{
  return R"(<root BTCPP_format="4"><BehaviorTree ID="Main">)" + leaf +
         "</BehaviorTree></root>";
}

template <typename T, template <typename> class Node = Reader>
T readOnce(const std::string& leaf_attrs, const std::function<void(Blackboard&)>& setup,
           T initial = T{})
{
  BehaviorTreeFactory factory;
  T out = initial;
  factory.registerNodeType<Node<T>>("Read", &out);
  auto bb = Blackboard::create();
  setup(*bb);
  auto tree = factory.createTreeFromText(xmlWithLeaf("<Read " + leaf_attrs + "/>"), bb);
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  return out;
}

const std::string kLong = "a string long enough to not fit any small-string buffer";
}  // namespace

TEST(GetInput, Literal)
{
  EXPECT_EQ(readOnce<int>(R"(in="42")", [](Blackboard&) {}), 42);
  EXPECT_FLOAT_EQ(readOnce<float>(R"(in="1.5")", [](Blackboard&) {}), 1.5f);
  EXPECT_EQ(readOnce<std::string>(
                "in=\"" + kLong + "\"", [](Blackboard&) {}, "old"),
            kLong);
}

TEST(GetInput, DefaultValue)
{
  EXPECT_EQ((readOnce<std::string, ReaderWithDefault>(
                "", [](Blackboard&) {}, "old")),
            "default text");
  EXPECT_FLOAT_EQ((readOnce<float, ReaderWithDefault>("", [](Blackboard&) {})), 2.5f);
}

TEST(GetInput, BlackboardSameType)
{
  EXPECT_EQ(readOnce<int>(R"(in="{v}")", [](Blackboard& bb) { bb.set("v", 7); }), 7);
  EXPECT_FLOAT_EQ(
      readOnce<float>(R"(in="{v}")", [](Blackboard& bb) { bb.set("v", 0.25f); }), 0.25f);
  EXPECT_EQ(readOnce<std::string>(R"(in="{v}")",
                                  [](Blackboard& bb) { bb.set("v", kLong); }, "old"),
            kLong);

  const auto pose = readOnce<Pose>(R"(in="{v}")", [](Blackboard& bb) {
    bb.set("v", Pose{ 1, 2, kLong });
  });
  EXPECT_EQ(pose.x, 1);
  EXPECT_EQ(pose.y, 2);
  EXPECT_EQ(pose.frame, kLong);
}

TEST(GetInput, BlackboardConvertedType)
{
  // stored as string, read as number
  EXPECT_EQ(readOnce<int>(R"(in="{v}")",
                          [](Blackboard& bb) { bb.set("v", std::string("123")); }),
            123);
  EXPECT_FLOAT_EQ(
      readOnce<float>(R"(in="{v}")",
                      [](Blackboard& bb) { bb.set("v", std::string("0.5")); }),
      0.5f);
}

TEST(GetInput, RemapToSameName)
{
  EXPECT_EQ(readOnce<int>(R"(in="{=}")", [](Blackboard& bb) { bb.set("in", 99); }), 99);
}

TEST(GetInput, SeesUpdatedValuesAcrossTicks)
{
  BehaviorTreeFactory factory;
  std::string out;
  factory.registerNodeType<Reader<std::string>>("Read", &out);
  auto bb = Blackboard::create();
  bb->set("v", kLong);
  auto tree = factory.createTreeFromText(xmlWithLeaf(R"(<Read in="{v}"/>)"), bb);

  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(out, kLong);
  bb->set("v", std::string("short"));
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(out, "short");
  bb->set("v", kLong + kLong);
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(out, kLong + kLong);
}

TEST(GetInput, WrongTypeFails)
{
  BehaviorTreeFactory factory;
  int out = 0;
  factory.registerNodeType<Reader<int>>("Read", &out);
  auto bb = Blackboard::create();
  bb->set("v", std::string("not a number"));
  auto tree = factory.createTreeFromText(xmlWithLeaf(R"(<Read in="{v}"/>)"), bb);
  EXPECT_ANY_THROW(tree.tickExactlyOnce());
}

//------------------------------------------------------------------
// The blackboard entry of a port is cached by the node (TreeNode::getPortEntry).
// These tests change the blackboard between reads, and check that getInput()
// always behaves as if the entry was looked up again.

namespace
{
// Reads port "in" (an int) each tick; -1 if it can't be read.
class EntryReader : public SyncActionNode
{
public:
  EntryReader(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config)
  {}
  static PortsList providedPorts()
  {
    return { InputPort<int>("in") };
  }
  NodeStatus tick() override
  {
    auto res = getInput<int>("in");
    value = res ? res.value() : -1;
    return NodeStatus::SUCCESS;
  }
  void setBlackboard(Blackboard::Ptr bb)
  {
    config().blackboard = std::move(bb);
  }
  void remap(const std::string& port, const std::string& value)
  {
    modifyPortsRemapping({ { port, value } });
  }
  int value = 0;
};

struct EntryReaderTree
{
  BehaviorTreeFactory factory;
  Blackboard::Ptr bb = Blackboard::create();
  Tree tree;
  EntryReader* reader = nullptr;

  explicit EntryReaderTree(const std::string& xml)
  {
    factory.registerNodeType<EntryReader>("Read");
    tree = factory.createTreeFromText(xml, bb);
    tree.applyVisitor([&](TreeNode* node) {
      if(auto r = dynamic_cast<EntryReader*>(node))
      {
        reader = r;
      }
    });
  }
  int read()
  {
    EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
    return reader->value;
  }
};

const char* kReadX = R"(
<root BTCPP_format="4"><BehaviorTree ID="Main"><Read in="{x}"/></BehaviorTree></root>)";

const char* kReadXInSubtree = R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main"><SubTree ID="Sub" in="{x}"/></BehaviorTree>
  <BehaviorTree ID="Sub"><Read in="{in}"/></BehaviorTree>
</root>)";

const char* kReadXInAutoremapSubtree = R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main"><SubTree ID="Sub" _autoremap="true"/></BehaviorTree>
  <BehaviorTree ID="Sub"><Read in="{x}"/></BehaviorTree>
</root>)";
}  // namespace

TEST(GetInputCache, EntryUnsetAndCreatedAgain)
{
  EntryReaderTree t(kReadX);
  t.bb->set("x", 1);
  EXPECT_EQ(t.read(), 1);
  t.bb->unset("x");
  EXPECT_EQ(t.read(), -1);
  t.bb->set("x", 2);
  EXPECT_EQ(t.read(), 2);
}

TEST(GetInputCache, EntryCreatedAfterFirstRead)
{
  EntryReaderTree t(kReadX);
  t.bb->unset("x");  // created empty by the tree, from the port type
  EXPECT_EQ(t.read(), -1);
  t.bb->set("x", 3);
  EXPECT_EQ(t.read(), 3);
}

TEST(GetInputCache, PortRemappedToAnotherKey)
{
  EntryReaderTree t(kReadX);
  t.bb->set("x", 1);
  t.bb->set("y", 10);
  EXPECT_EQ(t.read(), 1);
  t.reader->remap("in", "{y}");
  EXPECT_EQ(t.read(), 10);
  t.reader->remap("in", "{x}");
  EXPECT_EQ(t.read(), 1);
}

TEST(GetInputCache, NodeMovedToAnotherBlackboard)
{
  EntryReaderTree t(kReadX);
  t.bb->set("x", 1);
  EXPECT_EQ(t.read(), 1);
  for(int i = 0; i < 10; i++)
  {
    // new blackboards may be allocated where a previous one was
    auto other = Blackboard::create();
    other->set("x", 100 + i);
    t.reader->setBlackboard(other);
    EXPECT_EQ(t.read(), 100 + i);
  }
  t.reader->setBlackboard(t.bb);
  EXPECT_EQ(t.read(), 1);
}

TEST(GetInputCache, EntryOfParentBlackboard)
{
  for(const char* xml : { kReadXInSubtree, kReadXInAutoremapSubtree })
  {
    EntryReaderTree t(xml);
    t.bb->set("x", 1);
    EXPECT_EQ(t.read(), 1);
    t.bb->set("x", 2);
    EXPECT_EQ(t.read(), 2);
    t.bb->unset("x");
    EXPECT_EQ(t.read(), -1);
    t.bb->set("x", 3);
    EXPECT_EQ(t.read(), 3);
  }
}

TEST(GetInputCache, BlackboardRestore)
{
  EntryReaderTree t(kReadXInSubtree);
  t.bb->set("x", 1);
  EXPECT_EQ(t.read(), 1);
  auto backup = BlackboardBackup(t.tree);
  t.bb->set("x", 2);
  EXPECT_EQ(t.read(), 2);
  BlackboardRestore(backup, t.tree);
  EXPECT_EQ(t.read(), 1);
}

TEST(GetInputCache, ConcurrentWriterAndUnset)
{
  // A thread keeps replacing the entry while the tree reads it: every read
  // must return a value that was written (or -1 while it doesn't exist).
  EntryReaderTree t(kReadXInAutoremapSubtree);
  t.bb->set("x", 0);
  std::atomic<bool> stop{ false };
  std::thread writer([&] {
    int i = 0;
    while(!stop)
    {
      t.bb->unset("x");
      t.bb->set("x", (i++ % 100));
    }
  });
  for(int i = 0; i < 20000; i++)
  {
    const int v = t.read();
    ASSERT_TRUE(v >= -1 && v < 100) << v;
  }
  stop = true;
  writer.join();
  t.bb->set("x", 42);
  EXPECT_EQ(t.read(), 42);
}

//------------------------------------------------------------------
// Literal values of built-in types are converted once and cached

namespace
{
// A type whose conversion has a side effect: it must not be cached
struct Counted
{
  int value = 0;
};
int g_counted_conversions = 0;
}  // namespace

namespace BT
{
template <>
inline Counted convertFromString<Counted>(StringView str)
{
  g_counted_conversions++;
  return Counted{ convertFromString<int>(str) };
}
}  // namespace BT

namespace
{
class MultiTypeReader : public SyncActionNode
{
public:
  MultiTypeReader(const std::string& name, const NodeConfig& config)
    : SyncActionNode(name, config)
  {}
  static PortsList providedPorts()
  {
    return { InputPort<int>("in"), InputPort<Counted>("counted") };
  }
  NodeStatus tick() override
  {
    auto i = getInput<int>("in");
    int_value = i ? i.value() : -1;
    if(read_double)
    {
      auto d = getInput<double>("in");
      double_value = d ? d.value() : -1;
    }
    auto c = getInput<Counted>("counted");
    counted_value = c ? c.value().value : -1;
    return NodeStatus::SUCCESS;
  }
  void remap(const std::string& port, const std::string& value)
  {
    modifyPortsRemapping({ { port, value } });
  }
  // reading the same port with two types alternates the cached type
  bool read_double = true;
  int int_value = 0;
  double double_value = 0;
  int counted_value = 0;
};

struct MultiTypeTree
{
  BehaviorTreeFactory factory;
  Blackboard::Ptr bb = Blackboard::create();
  Tree tree;
  MultiTypeReader* reader = nullptr;

  explicit MultiTypeTree(const std::string& attributes)
  {
    factory.registerNodeType<MultiTypeReader>("Read");
    tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main">)"
                                      "<Read " +
                                          attributes + "/></BehaviorTree></root>",
                                      bb);
    tree.applyVisitor([&](TreeNode* node) {
      if(auto r = dynamic_cast<MultiTypeReader*>(node))
      {
        reader = r;
      }
    });
  }
  void tick()
  {
    ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  }
};
}  // namespace

TEST(GetInputCache, LiteralReadAsSeveralTypes)
{
  MultiTypeTree t(R"(in="7" counted="1")");
  for(int i = 0; i < 3; i++)
  {
    t.tick();
    EXPECT_EQ(t.reader->int_value, 7);
    EXPECT_EQ(t.reader->double_value, 7.0);
  }
}

TEST(GetInputCache, LiteralChanged)
{
  MultiTypeTree t(R"(in="7" counted="1")");
  t.reader->read_double = false;
  t.tick();
  t.tick();
  EXPECT_EQ(t.reader->int_value, 7);
  t.reader->remap("in", "8");
  t.tick();
  EXPECT_EQ(t.reader->int_value, 8);
  // literal -> blackboard -> literal
  t.bb->set("x", 9);
  t.reader->remap("in", "{x}");
  t.tick();
  EXPECT_EQ(t.reader->int_value, 9);
  t.reader->remap("in", "7");
  t.tick();
  EXPECT_EQ(t.reader->int_value, 7);
}

TEST(GetInputCache, InvalidLiteralFailsEveryTime)
{
  // invalid literals are rejected when the tree is created: change it afterwards
  MultiTypeTree t(R"(in="7" counted="1")");
  t.reader->read_double = false;
  t.tick();
  t.tick();
  EXPECT_EQ(t.reader->int_value, 7);
  t.reader->remap("in", "seven");
  for(int i = 0; i < 3; i++)
  {
    t.tick();
    EXPECT_EQ(t.reader->int_value, -1);
  }
  t.reader->remap("in", "7");
  t.tick();
  EXPECT_EQ(t.reader->int_value, 7);
}

TEST(GetInputCache, CustomConversionNotCached)
{
  MultiTypeTree t(R"(in="7" counted="5")");
  g_counted_conversions = 0;
  for(int i = 0; i < 3; i++)
  {
    t.tick();
    EXPECT_EQ(t.reader->counted_value, 5);
  }
  EXPECT_EQ(g_counted_conversions, 3);
}
