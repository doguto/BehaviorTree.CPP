// The input ports of a node are resolved once (literals converted to the declared
// type, blackboard keys extracted): getInput() must behave as if they were
// parsed at every call.
#include "behaviortree_cpp/bt_factory.h"

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
