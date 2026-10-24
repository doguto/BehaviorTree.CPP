#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/symbol_table.h"

#include <gtest/gtest.h>

using namespace BT;

namespace
{

void RegisterNodes(BehaviorTreeFactory& factory)
{
  auto ok = [](TreeNode&) { return NodeStatus::SUCCESS; };
  factory.registerSimpleAction("WriteInt", ok, { OutputPort<int>("value") });
  factory.registerSimpleAction("ReadInt", ok, { InputPort<int>("value") });
  factory.registerSimpleAction("ReadDouble", ok, { InputPort<double>("value") });
  factory.registerSimpleAction("ReadAny", ok, { InputPort("value") });
  factory.registerSimpleAction("ReadDefault", ok,
                               { InputPort<int>("value", "{counter}", "") });
}

const Symbol* FindSymbol(const SymbolTable& table, const std::string& qualified_name)
{
  for(int i = 0; i < int(table.symbols.size()); i++)
  {
    if(table.qualifiedName(i) == qualified_name)
    {
      return &table.symbols[i];
    }
  }
  return nullptr;
}

const SymbolRef* FindRef(const Symbol& symbol, const std::string& node_path,
                         const std::string& source)
{
  for(const auto& ref : symbol.refs)
  {
    if(ref.node_path == node_path && ref.source == source)
    {
      return &ref;
    }
  }
  return nullptr;
}

int CountIssues(const SymbolTable& table, SymbolIssue::Kind kind)
{
  int count = 0;
  for(const auto& issue : table.issues)
  {
    count += (issue.kind == kind) ? 1 : 0;
  }
  return count;
}

}  // namespace

TEST(SymbolTable, PortsInSameScope)
{
  static const char* xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Sequence>
      <WriteInt name="writer" value="{x}"/>
      <ReadInt name="reader" value="{x}"/>
    </Sequence>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  auto tree = factory.createTreeFromText(xml);
  auto table = buildSymbolTable(tree);

  ASSERT_EQ(table.scopes.size(), 1u);
  ASSERT_EQ(table.symbols.size(), 1u);
  const auto& x = table.symbols[0];
  EXPECT_EQ(x.name, "x");
  EXPECT_EQ(x.scope, 0);
  EXPECT_TRUE(x.exists);
  EXPECT_FALSE(x.has_value);
  EXPECT_EQ(x.type.type(), typeid(int));
  ASSERT_NE(FindRef(x, "writer", "value"), nullptr);
  ASSERT_NE(FindRef(x, "reader", "value"), nullptr);
  EXPECT_EQ(FindRef(x, "writer", "value")->access, SYMBOL_WRITE);
  EXPECT_EQ(FindRef(x, "reader", "value")->access, SYMBOL_READ);
  EXPECT_TRUE(table.issues.empty()) << toString(table);
}

TEST(SymbolTable, RemappingChainResolvesToRoot)
{
  static const char* xml = R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <WriteInt name="writer" value="{pos}"/>
      <SubTree ID="A" name="a" target="{pos}"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="A">
    <SubTree ID="B" name="b" goal="{target}"/>
  </BehaviorTree>
  <BehaviorTree ID="B">
    <ReadInt name="reader" value="{goal}"/>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  factory.registerBehaviorTreeFromText(xml);
  auto tree2 = factory.createTree("Main");
  auto table = buildSymbolTable(tree2);

  ASSERT_EQ(table.scopes.size(), 3u);
  ASSERT_EQ(table.symbols.size(), 1u) << toString(table);
  const auto* pos = FindSymbol(table, "pos");
  ASSERT_NE(pos, nullptr);
  EXPECT_EQ(pos->scope, 0);
  const auto* reader = FindRef(*pos, "a/b/reader", "value");
  ASSERT_NE(reader, nullptr);
  EXPECT_EQ(reader->key, "goal");

  const int scope_b = table.scopeOf(tree2.subtrees[2]->blackboard.get());
  ASSERT_GE(scope_b, 0);
  EXPECT_EQ(table.find(scope_b, "goal"), 0);
  EXPECT_TRUE(table.issues.empty()) << toString(table);
}

TEST(SymbolTable, AutoremapAndPrivateKeys)
{
  static const char* xml = R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <SubTree ID="A" name="a" _autoremap="true"/>
      <ReadInt name="reader" value="{count}"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="A">
    <Sequence>
      <Script name="init" code="count := 1; _tmp := 2"/>
      <ReadAny name="tmp_reader" value="{_tmp}"/>
    </Sequence>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  factory.registerBehaviorTreeFromText(xml);
  auto tree = factory.createTree("Main");
  auto table = buildSymbolTable(tree);

  const auto* count = FindSymbol(table, "count");
  ASSERT_NE(count, nullptr) << toString(table);
  EXPECT_EQ(count->scope, 0);
  ASSERT_NE(FindRef(*count, "a/init", "code"), nullptr);
  EXPECT_EQ(FindRef(*count, "a/init", "code")->access, SYMBOL_CREATE);
  EXPECT_TRUE(FindRef(*count, "a/init", "code")->from_script);
  ASSERT_NE(FindRef(*count, "reader", "value"), nullptr);

  // private keys are not autoremapped
  const auto* tmp = FindSymbol(table, "a/_tmp");
  ASSERT_NE(tmp, nullptr) << toString(table);
  EXPECT_EQ(tmp->refs.size(), 2u);
  EXPECT_EQ(FindSymbol(table, "_tmp"), nullptr);
  EXPECT_TRUE(table.issues.empty()) << toString(table);
}

TEST(SymbolTable, RootPrefix)
{
  static const char* xml = R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <Script name="init" code="x := 1"/>
      <SubTree ID="A" name="a"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="A">
    <ReadInt name="reader" value="{@x}"/>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  factory.registerBehaviorTreeFromText(xml);
  auto tree = factory.createTree("Main");
  auto table = buildSymbolTable(tree);

  ASSERT_EQ(table.symbols.size(), 1u) << toString(table);
  const auto* x = FindSymbol(table, "x");
  ASSERT_NE(x, nullptr);
  ASSERT_NE(FindRef(*x, "a/reader", "value"), nullptr);
  EXPECT_EQ(FindRef(*x, "a/reader", "value")->key, "@x");
  EXPECT_TRUE(table.issues.empty()) << toString(table);
}

TEST(SymbolTable, ScriptAccessAndEnums)
{
  enum Color
  {
    RED = 1
  };
  static const char* xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Script name="s" code="a := b + 1; c += 2; d = RED; e := (a > 1) ? b : 0"/>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  factory.registerScriptingEnums<Color>();
  auto tree = factory.createTreeFromText(xml);
  auto table = buildSymbolTable(tree, { {}, false });

  EXPECT_EQ(FindSymbol(table, "RED"), nullptr) << toString(table);
  ASSERT_NE(FindSymbol(table, "a"), nullptr);
  EXPECT_EQ(FindSymbol(table, "a")->access(), SYMBOL_CREATE | SYMBOL_READ);
  ASSERT_NE(FindSymbol(table, "b"), nullptr);
  EXPECT_EQ(FindSymbol(table, "b")->access(), SYMBOL_READ);
  ASSERT_NE(FindSymbol(table, "c"), nullptr);
  EXPECT_EQ(FindSymbol(table, "c")->access(), SYMBOL_READ | SYMBOL_WRITE);
  ASSERT_NE(FindSymbol(table, "d"), nullptr);
  EXPECT_EQ(FindSymbol(table, "d")->access(), SYMBOL_WRITE);
  ASSERT_NE(FindSymbol(table, "e"), nullptr);

  // b is never written; c and d are assigned with "+=" / "=" but never created
  EXPECT_EQ(CountIssues(table, SymbolIssue::READ_NEVER_WRITTEN), 1) << toString(table);
  EXPECT_EQ(CountIssues(table, SymbolIssue::ASSIGN_WITHOUT_CREATE), 2) << toString(table);
  EXPECT_EQ(CountIssues(table, SymbolIssue::WRITTEN_NEVER_READ), 0);
}

TEST(SymbolTable, PreAndPostConditions)
{
  static const char* xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Sequence>
      <Script name="init" code="flag := true"/>
      <AlwaysSuccess name="act" _skipIf="flag == false" _post="n := 3"/>
    </Sequence>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  auto tree = factory.createTreeFromText(xml);
  auto table = buildSymbolTable(tree);

  const auto* flag = FindSymbol(table, "flag");
  ASSERT_NE(flag, nullptr);
  ASSERT_NE(FindRef(*flag, "act", "_skipIf"), nullptr) << toString(table);
  EXPECT_EQ(FindRef(*flag, "act", "_skipIf")->access, SYMBOL_READ);
  const auto* n = FindSymbol(table, "n");
  ASSERT_NE(n, nullptr);
  ASSERT_NE(FindRef(*n, "act", "_post"), nullptr) << toString(table);
  EXPECT_EQ(FindRef(*n, "act", "_post")->access, SYMBOL_CREATE);
  EXPECT_EQ(CountIssues(table, SymbolIssue::WRITTEN_NEVER_READ), 1);
}

TEST(SymbolTable, SubTreeLiteralHasValue)
{
  static const char* xml = R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <SubTree ID="A" name="a" speed="1.5"/>
  </BehaviorTree>
  <BehaviorTree ID="A">
    <ReadDouble name="reader" value="{speed}"/>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  factory.registerBehaviorTreeFromText(xml);
  auto tree = factory.createTree("Main");
  auto table = buildSymbolTable(tree);

  const auto* speed = FindSymbol(table, "a/speed");
  ASSERT_NE(speed, nullptr) << toString(table);
  EXPECT_TRUE(speed->has_value);
  EXPECT_TRUE(table.issues.empty()) << toString(table);
}

TEST(SymbolTable, ExternalKeys)
{
  static const char* xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <ReadInt name="reader" value="{battery}"/>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  auto tree = factory.createTreeFromText(xml);

  auto table = buildSymbolTable(tree);
  EXPECT_EQ(CountIssues(table, SymbolIssue::READ_NEVER_WRITTEN), 1);

  auto declared = buildSymbolTable(tree, { { "@battery" }, true });
  EXPECT_TRUE(declared.issues.empty()) << toString(declared);
  ASSERT_EQ(declared.symbols.size(), 1u);
  EXPECT_TRUE(declared.symbols[0].external);

  // a value set before ticking counts as written
  tree.rootBlackboard()->set("battery", 12);
  EXPECT_TRUE(buildSymbolTable(tree).issues.empty());
}

TEST(SymbolTable, DefaultPortValue)
{
  static const char* xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Sequence>
      <WriteInt name="writer" value="{counter}"/>
      <ReadDefault name="reader"/>
    </Sequence>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  auto tree = factory.createTreeFromText(xml);
  auto table = buildSymbolTable(tree);

  const auto* counter = FindSymbol(table, "counter");
  ASSERT_NE(counter, nullptr) << toString(table);
  EXPECT_NE(FindRef(*counter, "reader", "value"), nullptr) << toString(table);
}

TEST(SymbolTable, KeyNamePorts)
{
  static const char* xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Sequence>
      <SetBlackboard name="set" value="3" output_key="k"/>
      <SkipUnlessUpdated name="watch" entry="{k}">
        <AlwaysSuccess/>
      </SkipUnlessUpdated>
      <UnsetBlackboard name="unset" key="k"/>
    </Sequence>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  auto tree = factory.createTreeFromText(xml);
  auto table = buildSymbolTable(tree);

  ASSERT_EQ(table.symbols.size(), 1u) << toString(table);
  const auto& k = table.symbols[0];
  EXPECT_EQ(k.name, "k");
  EXPECT_EQ(FindRef(k, "set", "output_key")->access, SYMBOL_CREATE);
  EXPECT_EQ(FindRef(k, "watch", "entry")->access, SYMBOL_READ);
  EXPECT_EQ(FindRef(k, "unset", "key")->access, SYMBOL_UNSET);
  EXPECT_TRUE(table.issues.empty()) << toString(table);
}

TEST(SymbolTable, ToStringMentionsEverything)
{
  static const char* xml = R"(
<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence>
      <WriteInt name="writer" value="{pos}"/>
      <SubTree ID="A" name="a" target="{pos}"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="A">
    <ReadInt name="reader" value="{target}"/>
  </BehaviorTree>
</root>)";
  BehaviorTreeFactory factory;
  RegisterNodes(factory);
  factory.registerBehaviorTreeFromText(xml);
  auto tree = factory.createTree("Main");
  const auto text = toString(buildSymbolTable(tree));

  EXPECT_NE(text.find("target->{pos}"), std::string::npos) << text;
  EXPECT_NE(text.find("a/reader [value]  as \"target\""), std::string::npos) << text;
  EXPECT_NE(text.find("Issues:\n  none"), std::string::npos) << text;
}
