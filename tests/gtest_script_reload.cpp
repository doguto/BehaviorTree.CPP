// Precondition, Script and ScriptCondition re-read their script (and
// Precondition its "else" port) at every tick: a script given through the
// blackboard can change between ticks. A literal one is not copied again.
#include "behaviortree_cpp/bt_factory.h"

#include <gtest/gtest.h>

using namespace BT;

namespace
{
Tree createTree(BehaviorTreeFactory& factory, const std::string& body, Blackboard::Ptr bb)
{
  return factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main">)" +
                                        body + "</BehaviorTree></root>",
                                    bb);
}
}  // namespace

TEST(ScriptReload, LiteralScriptsAcrossTicks)
{
  BehaviorTreeFactory factory;
  auto bb = Blackboard::create();
  bb->set("x", 0);
  auto tree = createTree(factory, R"(
    <Sequence>
      <Script code="x := x + 1"/>
      <Precondition if="x &lt; 3" else="FAILURE">
        <ScriptCondition code="x &gt; 0"/>
      </Precondition>
    </Sequence>)",
                         bb);
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);  // x=1
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);  // x=2
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::FAILURE);  // x=3
  EXPECT_EQ(bb->get<int>("x"), 3);
}

TEST(ScriptReload, ScriptsFromBlackboardChange)
{
  BehaviorTreeFactory factory;
  auto bb = Blackboard::create();
  bb->set("x", 0);
  bb->set("code", std::string("x := 10"));
  bb->set("cond", std::string("x == 10"));
  bb->set("else_status", std::string("SUCCESS"));
  auto tree = createTree(factory, R"(
    <Sequence>
      <Script code="{code}"/>
      <Precondition if="{cond}" else="{else_status}">
        <AlwaysFailure/>
      </Precondition>
    </Sequence>)",
                         bb);
  // cond true: the child (AlwaysFailure) is ticked
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::FAILURE);
  EXPECT_EQ(bb->get<int>("x"), 10);

  // new scripts and else status: cond false, else=SUCCESS
  bb->set("code", std::string("x := 20"));
  bb->set("cond", std::string("x == 10"));
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<int>("x"), 20);

  bb->set("else_status", std::string("FAILURE"));
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::FAILURE);

  bb->set("cond", std::string("x == 20"));
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::FAILURE);  // AlwaysFailure child
}

TEST(ScriptReload, ElseLiteralAcrossTicks)
{
  BehaviorTreeFactory factory;
  auto bb = Blackboard::create();
  bb->set("x", 0);
  for(const char* status : { "SUCCESS", "FAILURE" })
  {
    auto tree = createTree(factory,
                           std::string(R"(<Precondition if="x == 1" else=")") + status +
                               R"("><AlwaysSuccess/></Precondition>)",
                           bb);
    for(int i = 0; i < 3; i++)
    {
      EXPECT_EQ(toStr(tree.tickExactlyOnce()), status);
    }
  }
}
