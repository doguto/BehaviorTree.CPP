#include "behaviortree_cpp/symbol_table.h"

#include "behaviortree_cpp/actions/script_condition.h"
#include "behaviortree_cpp/actions/script_node.h"
#include "behaviortree_cpp/actions/set_blackboard_node.h"
#include "behaviortree_cpp/actions/unset_blackboard_node.h"
#include "behaviortree_cpp/actions/updated_action.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/script_precondition.h"
#include "behaviortree_cpp/decorators/subtree_node.h"
#include "behaviortree_cpp/decorators/updated_decorator.h"
#include "behaviortree_cpp/scripting/operators.hpp"

#include <algorithm>
#include <sstream>

namespace BT
{

namespace
{

bool IsPrivateKey(const std::string& key)
{
  return !key.empty() && key.front() == '_';
}

std::string SymbolKey(int scope, const std::string& name)
{
  return std::to_string(scope) + ':' + name;
}

std::string AccessStr(uint8_t access)
{
  std::string out;
  if(access & SYMBOL_READ)
  {
    out += 'R';
  }
  if(access & SYMBOL_WRITE)
  {
    out += 'W';
  }
  if(access & SYMBOL_CREATE)
  {
    out += 'C';
  }
  if(access & SYMBOL_UNSET)
  {
    out += 'U';
  }
  return out;
}

struct ResolvedKey
{
  int scope;
  std::string name;
};

// Same rules as Blackboard::getEntry() and Blackboard::createEntryImpl():
// "@" goes to the root, then the local entry, then the SubTree remapping,
// then _autoremap (except private keys).
ResolvedKey Resolve(const std::vector<SymbolScope>& scopes, int scope, std::string key)
{
  while(true)
  {
    if(StartWith(key, '@'))
    {
      while(scopes[scope].parent >= 0)
      {
        scope = scopes[scope].parent;
      }
      key.erase(0, 1);
      continue;
    }
    const auto& sc = scopes[scope];
    if(sc.local_keys.count(key) != 0)
    {
      return { scope, key };
    }
    if(sc.parent >= 0)
    {
      auto remap_it = sc.remap.find(key);
      if(remap_it != sc.remap.end())
      {
        scope = sc.parent;
        key = remap_it->second;
        continue;
      }
      if(sc.autoremap && !IsPrivateKey(key))
      {
        scope = sc.parent;
        continue;
      }
    }
    return { scope, key };
  }
}

class Builder
{
public:
  Builder(const Tree& tree, const SymbolTableOptions& options)
    : tree_(tree), options_(options)
  {}

  SymbolTable build()
  {
    buildScopes();
    for(const auto& subtree : tree_.subtrees)
    {
      for(const auto& node : subtree->nodes)
      {
        collectNode(*node);
      }
    }
    for(const auto& key : options_.external_keys)
    {
      symbols_[symbolIndex(root_scope_, key)].external = true;
    }
    finalizeSymbols();
    findIssues();
    return std::move(table_);
  }

private:
  const Tree& tree_;
  const SymbolTableOptions& options_;
  SymbolTable table_;
  std::unordered_map<std::string, int> symbol_index_;
  std::vector<Symbol>& symbols_ = table_.symbols;
  int root_scope_ = 0;

  int addScope(Blackboard* bb, std::string path, std::string tree_ID, bool in_tree)
  {
    SymbolScope scope;
    scope.path = std::move(path);
    scope.tree_ID = std::move(tree_ID);
    scope.in_tree = in_tree;
    scope.autoremap = bb->autoRemappingEnabled();
    scope.remap = bb->subtreeRemappings();
    scope.blackboard = bb;
    for(const auto& key : bb->getKeys())
    {
      scope.local_keys.emplace(key);
    }
    table_.scopes.push_back(std::move(scope));
    return int(table_.scopes.size()) - 1;
  }

  void buildScopes()
  {
    for(const auto& subtree : tree_.subtrees)
    {
      if(table_.scopeOf(subtree->blackboard.get()) < 0)
      {
        addScope(subtree->blackboard.get(), subtree->instance_name, subtree->tree_ID,
                 true);
      }
    }
    // Link the parents. The user may give a root blackboard that has a parent:
    // add the scopes outside the tree too.
    for(size_t i = 0; i < table_.scopes.size(); i++)
    {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
      auto* bb = const_cast<Blackboard*>(table_.scopes[i].blackboard);
      auto parent = bb->parent();
      if(!parent)
      {
        continue;
      }
      int parent_index = table_.scopeOf(parent.get());
      if(parent_index < 0)
      {
        parent_index = addScope(parent.get(), "<external>", {}, false);
      }
      table_.scopes[i].parent = parent_index;
    }
    if(!tree_.subtrees.empty())
    {
      root_scope_ = table_.scopeOf(tree_.subtrees.front()->blackboard.get());
    }
  }

  int symbolIndex(int scope, const std::string& key)
  {
    auto resolved = Resolve(table_.scopes, scope, key);
    auto id = SymbolKey(resolved.scope, resolved.name);
    auto it = symbol_index_.find(id);
    if(it != symbol_index_.end())
    {
      return it->second;
    }
    Symbol symbol;
    symbol.name = resolved.name;
    symbol.scope = resolved.scope;
    symbols_.push_back(std::move(symbol));
    const int index = int(symbols_.size()) - 1;
    symbol_index_.emplace(id, index);
    return index;
  }

  void addRef(int scope, const TreeNode& node, const std::string& source,
              const std::string& key, uint8_t access, bool from_script,
              const TypeInfo& type = {})
  {
    if(key.empty())
    {
      return;
    }
    auto& refs = symbols_[symbolIndex(scope, key)].refs;
    for(auto& ref : refs)
    {
      if(ref.node_uid == node.UID() && ref.source == source && ref.key == key)
      {
        ref.access |= access;
        return;
      }
    }
    SymbolRef ref;
    ref.node_uid = node.UID();
    ref.node_path = node.fullPath();
    ref.source = source;
    ref.key = key;
    ref.access = access;
    ref.from_script = from_script;
    ref.type = type;
    refs.push_back(std::move(ref));
  }

  void visitExpr(const Ast::ExprBase* expr, int scope, const TreeNode& node,
                 const std::string& source)
  {
    if(expr == nullptr)
    {
      return;
    }
    if(const auto* name = dynamic_cast<const Ast::ExprName*>(expr))
    {
      // enums are looked up before the variables
      const auto& enums = node.config().enums;
      if(!enums || enums->count(name->name) == 0)
      {
        addRef(scope, node, source, name->name, SYMBOL_READ, true);
      }
    }
    else if(const auto* assign = dynamic_cast<const Ast::ExprAssignment*>(expr))
    {
      if(const auto* lhs = dynamic_cast<const Ast::ExprName*>(assign->lhs.get()))
      {
        uint8_t access = SYMBOL_WRITE;
        if(assign->op == Ast::ExprAssignment::assign_create)
        {
          access = SYMBOL_CREATE;
        }
        else if(assign->op != Ast::ExprAssignment::assign_existing)
        {
          access = SYMBOL_READ | SYMBOL_WRITE;
        }
        addRef(scope, node, source, lhs->name, access, true);
      }
      visitExpr(assign->rhs.get(), scope, node, source);
    }
    else if(const auto* unary = dynamic_cast<const Ast::ExprUnaryArithmetic*>(expr))
    {
      visitExpr(unary->rhs.get(), scope, node, source);
    }
    else if(const auto* binary = dynamic_cast<const Ast::ExprBinaryArithmetic*>(expr))
    {
      visitExpr(binary->lhs.get(), scope, node, source);
      visitExpr(binary->rhs.get(), scope, node, source);
    }
    else if(const auto* cmp = dynamic_cast<const Ast::ExprComparison*>(expr))
    {
      for(const auto& operand : cmp->operands)
      {
        visitExpr(operand.get(), scope, node, source);
      }
    }
    else if(const auto* if_expr = dynamic_cast<const Ast::ExprIf*>(expr))
    {
      visitExpr(if_expr->condition.get(), scope, node, source);
      visitExpr(if_expr->then.get(), scope, node, source);
      visitExpr(if_expr->else_.get(), scope, node, source);
    }
  }

  void addScript(int scope, const TreeNode& node, const std::string& source,
                 const std::string& script)
  {
    if(script.empty())
    {
      return;
    }
    try
    {
      for(const auto& expr : Scripting::parseStatements(script))
      {
        visitExpr(expr.get(), scope, node, source);
      }
    }
    catch(const std::exception& ex)
    {
      SymbolIssue issue;
      issue.severity = SymbolIssue::ERROR;
      issue.kind = SymbolIssue::SCRIPT_PARSE_ERROR;
      issue.message = StrCat(node.fullPath(), " [", source, "]: can't parse \"", script,
                             "\": ", ex.what());
      table_.issues.push_back(std::move(issue));
    }
  }

  static bool isScriptPort(const TreeNode& node, const std::string& port)
  {
    if(port == "code")
    {
      return dynamic_cast<const ScriptNode*>(&node) != nullptr ||
             dynamic_cast<const ScriptCondition*>(&node) != nullptr;
    }
    if(port == "if")
    {
      return dynamic_cast<const PreconditionNode*>(&node) != nullptr;
    }
    return false;
  }

  // Ports whose literal value is the name of a blackboard key.
  static uint8_t keyNamePortAccess(const TreeNode& node, const std::string& port)
  {
    if(port == "key" && dynamic_cast<const UnsetBlackboardNode*>(&node) != nullptr)
    {
      return SYMBOL_UNSET;
    }
    if(port == "output_key" && dynamic_cast<const SetBlackboardNode*>(&node) != nullptr)
    {
      return SYMBOL_CREATE;
    }
    return 0;
  }

  static bool isEntryUpdatedPort(const TreeNode& node, const std::string& port)
  {
    return port == "entry" &&
           (dynamic_cast<const EntryUpdatedAction*>(&node) != nullptr ||
            dynamic_cast<const EntryUpdatedDecorator*>(&node) != nullptr);
  }

  void collectPort(int scope, const TreeNode& node, const std::string& port,
                   const std::string& value)
  {
    StringView stripped;
    const bool is_pointer = TreeNode::isBlackboardPointer(value, &stripped);

    if(isEntryUpdatedPort(node, port))
    {
      addRef(scope, node, port, is_pointer ? std::string(stripped) : value, SYMBOL_READ,
             false);
      return;
    }
    if(!is_pointer)
    {
      if(isScriptPort(node, port))
      {
        addScript(scope, node, port, value);
      }
      else if(auto access = keyNamePortAccess(node, port))
      {
        addRef(scope, node, port, value, access, false);
      }
      return;
    }
    if(isScriptPort(node, port) || keyNamePortAccess(node, port) != 0)
    {
      // the value of the entry is a script or a key name: only the entry is read here
      addRef(scope, node, port, std::string(stripped), SYMBOL_READ, false);
      return;
    }

    auto key = TreeNode::getRemappedKey(port, value);
    if(!key)
    {
      return;
    }
    PortDirection direction = PortDirection::INPUT;
    TypeInfo type;
    if(const auto* manifest = node.config().manifest)
    {
      auto it = manifest->ports.find(port);
      if(it != manifest->ports.end())
      {
        direction = it->second.direction();
        type = it->second;
      }
    }
    uint8_t access = 0;
    if(direction != PortDirection::OUTPUT)
    {
      access |= SYMBOL_READ;
    }
    if(direction != PortDirection::INPUT)
    {
      access |= SYMBOL_WRITE;
    }
    addRef(scope, node, port, std::string(key.value()), access, false, type);
  }

  void collectNode(const TreeNode& node)
  {
    const auto& config = node.config();
    const int scope = table_.scopeOf(config.blackboard.get());
    if(scope < 0)
    {
      return;
    }
    // the remapping of a SubTree is part of the scope of its children
    if(dynamic_cast<const SubTreeNode*>(&node) == nullptr)
    {
      // INOUT ports are in both lists, with the same value
      std::map<std::string, std::string> ports(config.input_ports.begin(),
                                               config.input_ports.end());
      ports.insert(config.output_ports.begin(), config.output_ports.end());
      for(const auto& [port, value] : ports)
      {
        collectPort(scope, node, port, value);
      }
    }
    for(const auto& [cond, script] : config.pre_conditions)
    {
      addScript(scope, node, toStr(cond), script);
    }
    for(const auto& [cond, script] : config.post_conditions)
    {
      addScript(scope, node, toStr(cond), script);
    }
  }

  void finalizeSymbols()
  {
    for(auto& symbol : symbols_)
    {
      const auto* bb = table_.scopes[symbol.scope].blackboard;
      if(auto entry = bb->getLocalEntry(symbol.name); entry != nullptr)
      {
        symbol.exists = true;
        symbol.has_value = !entry->value.empty();
        if(entry->info.isStronglyTyped())
        {
          symbol.type = entry->info;
        }
      }
      if(!symbol.type.isStronglyTyped())
      {
        for(const auto& ref : symbol.refs)
        {
          if(ref.type.isStronglyTyped())
          {
            symbol.type = ref.type;
            break;
          }
        }
      }
    }

    // sort by scope, then by name, for a stable output
    std::vector<int> order(symbols_.size());
    for(size_t i = 0; i < order.size(); i++)
    {
      order[i] = int(i);
    }
    std::sort(order.begin(), order.end(), [this](int a, int b) {
      return std::tie(symbols_[a].scope, symbols_[a].name) <
             std::tie(symbols_[b].scope, symbols_[b].name);
    });
    std::vector<Symbol> sorted;
    sorted.reserve(symbols_.size());
    for(int index : order)
    {
      sorted.push_back(std::move(symbols_[index]));
    }
    symbols_ = std::move(sorted);
  }

  static std::string firstRef(const Symbol& symbol, uint8_t access)
  {
    for(const auto& ref : symbol.refs)
    {
      if(ref.access & access)
      {
        return StrCat(ref.node_path, " [", ref.source, "]");
      }
    }
    return {};
  }

  void addIssue(SymbolIssue::Severity severity, SymbolIssue::Kind kind, int symbol,
                std::string message)
  {
    SymbolIssue issue;
    issue.severity = severity;
    issue.kind = kind;
    issue.symbol = symbol;
    issue.message = StrCat(table_.qualifiedName(symbol), ": ", message);
    table_.issues.push_back(std::move(issue));
  }

  void findIssues()
  {
    for(int i = 0; i < int(symbols_.size()); i++)
    {
      const auto& symbol = symbols_[i];
      if(symbol.external)
      {
        continue;
      }
      const uint8_t access = symbol.access();
      const bool written = (access & (SYMBOL_WRITE | SYMBOL_CREATE)) != 0;

      if((access & SYMBOL_READ) && !written && !symbol.has_value)
      {
        addIssue(SymbolIssue::WARNING, SymbolIssue::READ_NEVER_WRITTEN, i,
                 StrCat("read by ", firstRef(symbol, SYMBOL_READ),
                        ", but nothing in the tree writes it"));
      }
      bool script_assign = false;
      for(const auto& ref : symbol.refs)
      {
        script_assign |= ref.from_script && (ref.access & SYMBOL_WRITE);
      }
      if(script_assign && !symbol.exists && !(access & SYMBOL_CREATE))
      {
        addIssue(SymbolIssue::ERROR, SymbolIssue::ASSIGN_WITHOUT_CREATE, i,
                 StrCat("assigned by ", firstRef(symbol, SYMBOL_WRITE),
                        ", but nothing creates it (use \":=\")"));
      }
      if(options_.report_unread && written && !(access & SYMBOL_READ))
      {
        addIssue(SymbolIssue::INFO, SymbolIssue::WRITTEN_NEVER_READ, i,
                 StrCat("written by ", firstRef(symbol, SYMBOL_WRITE | SYMBOL_CREATE),
                        ", but nothing in the tree reads it"));
      }
    }
  }
};

}  // namespace

uint8_t Symbol::access() const
{
  uint8_t out = 0;
  for(const auto& ref : refs)
  {
    out |= ref.access;
  }
  return out;
}

int SymbolTable::scopeOf(const Blackboard* blackboard) const
{
  for(size_t i = 0; i < scopes.size(); i++)
  {
    if(scopes[i].blackboard == blackboard)
    {
      return int(i);
    }
  }
  return -1;
}

int SymbolTable::find(int scope, const std::string& key) const
{
  if(scope < 0 || scope >= int(scopes.size()))
  {
    return -1;
  }
  auto resolved = Resolve(scopes, scope, key);
  for(size_t i = 0; i < symbols.size(); i++)
  {
    if(symbols[i].scope == resolved.scope && symbols[i].name == resolved.name)
    {
      return int(i);
    }
  }
  return -1;
}

std::string SymbolTable::qualifiedName(int symbol) const
{
  const auto& sym = symbols.at(symbol);
  const auto& path = scopes.at(sym.scope).path;
  return path.empty() ? sym.name : StrCat(path, "/", sym.name);
}

SymbolTable buildSymbolTable(const Tree& tree, const SymbolTableOptions& options)
{
  return Builder(tree, options).build();
}

std::string toString(const SymbolTable& table)
{
  std::ostringstream out;
  out << "Scopes:\n";
  for(size_t i = 0; i < table.scopes.size(); i++)
  {
    const auto& scope = table.scopes[i];
    out << "  #" << i << "  " << (scope.path.empty() ? "(root)" : scope.path);
    if(!scope.tree_ID.empty())
    {
      out << "  tree=" << scope.tree_ID;
    }
    if(scope.parent >= 0)
    {
      out << "  parent=#" << scope.parent;
    }
    if(scope.autoremap)
    {
      out << "  _autoremap";
    }
    std::map<std::string, std::string> remap(scope.remap.begin(), scope.remap.end());
    for(const auto& [internal, external] : remap)
    {
      out << "  " << internal << "->{" << external << "}";
    }
    out << "\n";
  }

  out << "Symbols:\n";
  for(size_t i = 0; i < table.symbols.size(); i++)
  {
    const auto& symbol = table.symbols[i];
    out << "  " << table.qualifiedName(int(i))
        << "  type=" << (symbol.type.isStronglyTyped() ? symbol.type.typeName() : "?");
    if(symbol.exists)
    {
      out << (symbol.has_value ? "  [has value]" : "  [exists]");
    }
    if(symbol.external)
    {
      out << "  [external]";
    }
    out << "\n";
    for(const auto& ref : symbol.refs)
    {
      out << "    " << AccessStr(ref.access);
      for(size_t pad = AccessStr(ref.access).size(); pad < 3; pad++)
      {
        out << ' ';
      }
      out << "  " << ref.node_path << " [" << ref.source
          << (ref.from_script ? ", script" : "") << "]";
      if(ref.key != symbol.name)
      {
        out << "  as \"" << ref.key << "\"";
      }
      out << "\n";
    }
  }

  out << "Issues:\n";
  if(table.issues.empty())
  {
    out << "  none\n";
  }
  for(const auto& issue : table.issues)
  {
    static const char* severity[] = { "INFO", "WARNING", "ERROR" };
    out << "  " << severity[issue.severity] << "  " << issue.message << "\n";
  }
  return out.str();
}

}  // namespace BT
