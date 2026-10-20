// Blackboard::set() of a key that lives in a parent blackboard (remapping or
// autoremapping) writes the parent's entry, with the same type rules as
// creating it would apply.
#include "behaviortree_cpp/blackboard.h"

#include <gtest/gtest.h>

using namespace BT;

namespace
{
struct Chain
{
  Blackboard::Ptr root = Blackboard::create();
  Blackboard::Ptr remapped = Blackboard::create(root);
  Blackboard::Ptr autoremapped = Blackboard::create(root);
  Chain()
  {
    remapped->addSubtreeRemapping("in", "x");
    autoremapped->enableAutoRemapping(true);
  }
};
}  // namespace

TEST(BlackboardRemappedSet, WritesParentEntry)
{
  Chain c;
  c.root->set("x", 1);
  const auto seq = c.root->getEntry("x")->sequence_id;
  c.remapped->set("in", 2);
  EXPECT_EQ(c.root->get<int>("x"), 2);
  c.autoremapped->set("x", 3);
  EXPECT_EQ(c.root->get<int>("x"), 3);
  EXPECT_EQ(c.root->getEntry("x")->sequence_id, seq + 2);
  // nothing created locally
  EXPECT_TRUE(c.remapped->getKeys().empty());
  EXPECT_TRUE(c.autoremapped->getKeys().empty());
}

TEST(BlackboardRemappedSet, TypeMismatchThrows)
{
  Chain c;
  c.root->set("x", 1);
  EXPECT_THROW(c.remapped->set("in", std::vector<int>{ 1 }), LogicError);
  EXPECT_THROW(c.autoremapped->set("x", std::vector<int>{ 1 }), LogicError);
  try
  {
    c.autoremapped->set("x", std::vector<int>{ 1 });
  }
  catch(const LogicError& err)
  {
    EXPECT_NE(std::string(err.what())
                  .find("once declared, the type of a port shall not "
                        "change"),
              std::string::npos)
        << err.what();
  }
  EXPECT_EQ(c.root->get<int>("x"), 1);
}

TEST(BlackboardRemappedSet, StringIntoTypedEntry)
{
  Chain c;
  c.root->set("x", 1);
  // a string is not strongly typed: no error, the value is replaced
  EXPECT_NO_THROW(c.autoremapped->set("x", std::string("42")));
  EXPECT_EQ(c.root->get<int>("x"), 42);
}

TEST(BlackboardRemappedSet, UntypedEntry)
{
  Chain c;
  c.root->createEntry("x", PortInfo(PortDirection::INOUT));  // AnyTypeAllowed
  EXPECT_NO_THROW(c.autoremapped->set("x", 5));
  EXPECT_EQ(c.root->get<int>("x"), 5);
}

TEST(BlackboardRemappedSet, MissingEntryIsCreatedInParent)
{
  Chain c;
  c.autoremapped->set("y", 7);
  EXPECT_EQ(c.root->get<int>("y"), 7);
  c.remapped->set("in", 8);
  EXPECT_EQ(c.root->get<int>("x"), 8);
}

TEST(BlackboardRemappedSet, PrivateKeysStayLocal)
{
  Chain c;
  c.root->set("_p", 1);
  c.autoremapped->set("_p", 2);
  EXPECT_EQ(c.root->get<int>("_p"), 1);
  EXPECT_EQ(c.autoremapped->get<int>("_p"), 2);
}
