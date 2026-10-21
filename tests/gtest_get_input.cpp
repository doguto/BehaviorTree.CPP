// Value paths of TreeNode::getInput(): literal, default, blackboard entry of the
// same type, of a different type, stored as string, and "{=}" remapping.
#include "behaviortree_cpp/bt_factory.h"

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

TEST(GetInput, BlackboardKeyOf)
{
  EXPECT_EQ(TreeNode::blackboardKeyOf("port", "{key}"), StringView("key"));
  EXPECT_EQ(TreeNode::blackboardKeyOf("port", "{=}"), StringView("port"));
  EXPECT_EQ(TreeNode::blackboardKeyOf("port", "="), StringView("port"));
  EXPECT_EQ(TreeNode::blackboardKeyOf("port", "{@key}"), StringView("@key"));
  EXPECT_FALSE(TreeNode::blackboardKeyOf("port", "42"));
  EXPECT_FALSE(TreeNode::blackboardKeyOf("port", "text"));
  EXPECT_FALSE(TreeNode::blackboardKeyOf("port", ""));
  // same answer as getRemappedKey()
  for(const char* value : { "{key}", "{=}", "=", "42", "" })
  {
    auto key = TreeNode::blackboardKeyOf("port", value);
    auto remapped = TreeNode::getRemappedKey("port", value);
    ASSERT_EQ(key.has_value(), remapped.has_value()) << value;
    if(key)
    {
      EXPECT_EQ(*key, *remapped);
    }
  }
}
