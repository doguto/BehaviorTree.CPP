// The input ports of a node are resolved once (literals converted to the declared
// type, blackboard keys extracted): getInput() must behave as if they were
// parsed at every call.
#include "behaviortree_cpp/bt_factory.h"

#include <atomic>
#include <future>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using namespace BT;

namespace
{
struct Values
{
  int i = 0;
  double d = 0;
  bool b = false;
  std::string s;
  int i_from_double_port = 0;
  std::string s_from_int_port;
  double d_from_bb = 0;
  std::string error;
};

class ReadAll : public SyncActionNode
{
public:
  ReadAll(const std::string& name, const NodeConfig& config, Values* out)
    : SyncActionNode(name, config), out_(out)
  {}
  static PortsList providedPorts()
  {
    return { InputPort<int>("i"),
             InputPort<double>("d"),
             InputPort<bool>("b"),
             InputPort<std::string>("s"),
             InputPort<double>("d2"),
             InputPort<int>("i2"),
             InputPort<double>("from_bb"),
             InputPort<int>("bad"),
             InputPort<int>("with_default", 7, "") };
  }
  NodeStatus tick() override
  {
    *out_ = Values{};
    (void)getInput("i", out_->i);
    (void)getInput("d", out_->d);
    (void)getInput("b", out_->b);
    (void)getInput("s", out_->s);
    (void)getInput("d2", out_->i_from_double_port);
    (void)getInput("i2", out_->s_from_int_port);
    (void)getInput("from_bb", out_->d_from_bb);
    int unused = 0;
    if(auto res = getInput("bad", unused); !res)
    {
      out_->error = res.error();
    }
    return NodeStatus::SUCCESS;
  }

  // what a node may do before it is ticked
  void setPort(const std::string& key, const std::string& value)
  {
    mutableConfig().input_ports[key] = value;
  }

private:
  Values* out_;
};

const char* kXML = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <ReadAll name="reader" i="42" d="0.25" b="true" s="hello" d2="3" i2="12"
             from_bb="{x}"/>
  </BehaviorTree>
</root>)";
}  // namespace

TEST(PortTable, LiteralsAndBlackboard)
{
  BehaviorTreeFactory factory;
  Values values;
  factory.registerNodeType<ReadAll>("ReadAll", &values);
  auto bb = Blackboard::create();
  bb->set("x", 1.5);
  auto tree = factory.createTreeFromText(kXML, bb);

  for(int tick = 0; tick < 3; tick++)
  {
    ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
    EXPECT_EQ(values.i, 42);
    EXPECT_DOUBLE_EQ(values.d, 0.25);
    EXPECT_TRUE(values.b);
    EXPECT_EQ(values.s, "hello");
    // read with another type than the declared one
    EXPECT_EQ(values.i_from_double_port, 3);
    EXPECT_EQ(values.s_from_int_port, "12");
    // the blackboard is read at every tick
    EXPECT_DOUBLE_EQ(values.d_from_bb, 1.5 + tick);
    // "bad" is not in the XML
    EXPECT_FALSE(values.error.empty());
    bb->set("x", 2.5 + tick);
  }
}

TEST(PortTable, MutableConfigResolvesAgain)
{
  BehaviorTreeFactory factory;
  Values values;
  factory.registerNodeType<ReadAll>("ReadAll", &values);
  auto bb = Blackboard::create();
  bb->set("x", 1.5);
  bb->set("y", 9.0);
  bb->set("text", std::string("from the blackboard"));
  auto tree = factory.createTreeFromText(kXML, bb);
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(values.i, 42);

  ReadAll* reader = nullptr;
  tree.applyVisitor([&](TreeNode* node) {
    if(auto* r = dynamic_cast<ReadAll*>(node))
    {
      reader = r;
    }
  });
  ASSERT_NE(reader, nullptr);
  reader->setPort("i", "-7");
  reader->setPort("from_bb", "{y}");
  reader->setPort("s", "{text}");  // literal -> blackboard
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(values.i, -7);
  EXPECT_DOUBLE_EQ(values.d_from_bb, 9.0);
  EXPECT_EQ(values.s, "from the blackboard");
}

TEST(PortTable, DefaultValueNotInXml)
{
  BehaviorTreeFactory factory;
  int value = 0;
  factory.registerSimpleAction("ReadDefault",
                               [&](TreeNode& node) {
                                 value = node.getInput<int>("with_default").value();
                                 return NodeStatus::SUCCESS;
                               },
                               { InputPort<int>("with_default", 7, "") });
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4"><BehaviorTree ID="Main"><ReadDefault/></BehaviorTree></root>)");
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(value, 7);
}

TEST(PortTable, InvalidLiteral)
{
  // The factory rejects invalid literals when the tree is created; a node created
  // directly gets the error from getInput(), as before.
  NodeConfig config;
  config.input_ports["bad"] = "not_a_number";
  Values values;
  ReadAll node("reader", config, &values);
  int value = 0;
  auto res = node.getInput("bad", value);
  ASSERT_FALSE(res);
  EXPECT_NE(res.error().find("not_a_number"), std::string::npos);
}

TEST(PortTable, ConcurrentFirstRead)
{
  // A node created without a Tree resolves its ports at the first getInput():
  // several threads may do it at the same time.
  for(int round = 0; round < 50; round++)
  {
    NodeConfig config;
    config.input_ports["i"] = "42";
    config.input_ports["s"] = "text";
    config.input_ports["from_bb"] = "{x}";
    config.blackboard = Blackboard::create();
    config.blackboard->set("x", 2.0);
    Values values;
    ReadAll node("reader", config, &values);

    std::vector<std::future<bool>> readers;
    for(int t = 0; t < 4; t++)
    {
      readers.push_back(std::async(std::launch::async, [&node] {
        int i = 0;
        std::string s;
        double x = 0;
        return node.getInput("i", i).has_value() && i == 42 &&
               node.getInput("s", s).has_value() && s == "text" &&
               node.getInput("from_bb", x).has_value() && x == 2.0;
      }));
    }
    for(auto& r : readers)
    {
      EXPECT_TRUE(r.get());
    }
  }
}

namespace
{
// Reads "in" into *out, writes "out" from *to_write, at every tick
class Copier : public SyncActionNode
{
public:
  Copier(const std::string& name, const NodeConfig& config, int* out, int* to_write)
    : SyncActionNode(name, config), out_(out), to_write_(to_write)
  {}
  static PortsList providedPorts()
  {
    return { InputPort<int>("in"), OutputPort<int>("out") };
  }
  NodeStatus tick() override
  {
    *out_ = getInput<int>("in").value();
    if(auto res = setOutput("out", *to_write_); !res)
    {
      throw RuntimeError(res.error());
    }
    return NodeStatus::SUCCESS;
  }

private:
  int* out_;
  int* to_write_;
};
}  // namespace

TEST(PortTable, BoundEntriesFollowTheBlackboards)
{
  BehaviorTreeFactory factory;
  int read = 0;
  int to_write = 1;
  factory.registerNodeType<Copier>("Copier", &read, &to_write);
  auto root_bb = Blackboard::create();
  root_bb->set("x", 10);
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <SubTree ID="Sub" _autoremap="true"/>
  </BehaviorTree>
  <BehaviorTree ID="Sub">
    <Copier in="{x}" out="{y}"/>
  </BehaviorTree>
</root>)",
                                         root_bb);
  auto sub_bb = tree.subtrees.at(1)->blackboard;

  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(read, 10);
  EXPECT_EQ(root_bb->get<int>("y"), 1);  // created in the root (autoremap)

  // values change, entries don't
  root_bb->set("x", 11);
  to_write = 2;
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(read, 11);
  EXPECT_EQ(root_bb->get<int>("y"), 2);

  // removed and created again: the new entries are used
  root_bb->unset("x");
  root_bb->unset("y");
  root_bb->set("x", 12);
  to_write = 3;
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(read, 12);
  EXPECT_EQ(root_bb->get<int>("y"), 3);

  // a private entry of the subtree hides the one of the root... ("_" keys are
  // never remapped: use a manual remapping instead)
  sub_bb->addSubtreeRemapping("x", "z");
  root_bb->set("z", 42);
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(read, 42);

  // ...and the missing entry is reported
  root_bb->unset("z");
  EXPECT_ANY_THROW((void)tree.tickExactlyOnce());
}

TEST(PortTable, BoundOutputKeepsTheTypeChecks)
{
  BehaviorTreeFactory factory;
  int read = 0;
  int to_write = 7;
  factory.registerNodeType<Copier>("Copier", &read, &to_write);
  auto bb = Blackboard::create();
  bb->set("x", 1);
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4"><BehaviorTree ID="Main">
  <Copier in="{x}" out="{y}"/>
</BehaviorTree></root>)",
                                         bb);
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<int>("y"), 7);
  // the entry now holds another type: writing an int is refused, as before
  bb->unset("y");
  bb->set("y", std::string("text"));
  bb->unset("y");
  bb->set("y", std::vector<int>{ 1 });
  EXPECT_ANY_THROW((void)tree.tickExactlyOnce());
}

TEST(PortTable, ScriptVariablesFollowTheBlackboard)
{
  BehaviorTreeFactory factory;
  auto bb = Blackboard::create();
  bb->set("x", 1);
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4"><BehaviorTree ID="Main">
  <ScriptCondition code="x == 1"/>
</BehaviorTree></root>)",
                                         bb);
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  bb->set("x", 2);
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::FAILURE);
  bb->unset("x");
  EXPECT_ANY_THROW((void)tree.tickExactlyOnce());
  bb->set("x", 1);
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
}

TEST(PortTable, ConcurrentReadsAndStructureChanges)
{
  NodeConfig config;
  config.input_ports["in"] = "{x}";
  config.output_ports["out"] = "{y}";
  config.blackboard = Blackboard::create();
  config.blackboard->set("x", 5);
  int read = 0;
  int to_write = 1;
  Copier node("copier", config, &read, &to_write);

  std::atomic_bool stop = false;
  // another thread changes the structure of the blackboard meanwhile
  auto changer = std::async(std::launch::async, [&] {
    int i = 0;
    while(!stop)
    {
      config.blackboard->set("other_" + std::to_string(i++ % 50), i);
      config.blackboard->unset("other_" + std::to_string((i + 25) % 50));
    }
  });
  std::vector<std::future<bool>> readers;
  for(int t = 0; t < 3; t++)
  {
    readers.push_back(std::async(std::launch::async, [&node] {
      for(int i = 0; i < 2000; i++)
      {
        int value = 0;
        if(!node.getInput("in", value) || value != 5)
        {
          return false;
        }
      }
      return true;
    }));
  }
  for(auto& r : readers)
  {
    EXPECT_TRUE(r.get());
  }
  stop = true;
  changer.get();
}
