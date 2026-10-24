#pragma once

#include "behaviortree_cpp/blackboard.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace BT
{

class Tree;

/**
 * @brief One blackboard of the tree: the root one, or the one created for
 * each instance of a SubTree.
 */
struct SymbolScope
{
  // Path of the SubTree instance, empty for the root of the tree.
  std::string path;
  std::string tree_ID;
  // Index of the parent scope, -1 for the outermost one.
  int parent = -1;
  bool autoremap = false;
  // false when the blackboard is a parent of the root one, given by the user.
  bool in_tree = true;
  // internal key -> key in the parent scope
  std::unordered_map<std::string, std::string> remap;
  // Entries stored in this blackboard when the table was built.
  std::unordered_set<std::string> local_keys;
  // Only used to identify the scope; not dereferenced after buildSymbolTable().
  const Blackboard* blackboard = nullptr;
};

/// Bit flags describing how a node uses a symbol.
enum SymbolAccess : uint8_t
{
  SYMBOL_READ = 1,
  // setOutput(), or a script assignment that requires an existing entry ("=", "+=", ...)
  SYMBOL_WRITE = 2,
  // script assignment that creates the entry (":=")
  SYMBOL_CREATE = 4,
  // UnsetBlackboard
  SYMBOL_UNSET = 8,
};

/// A place in the tree where a symbol is used.
struct SymbolRef
{
  uint16_t node_uid = 0;
  std::string node_path;
  // Port name, or name of the pre/post condition ("_successIf", ...)
  std::string source;
  // Key as written in the node, before remapping.
  std::string key;
  uint8_t access = 0;
  bool from_script = false;
  // Type declared by the port. Not strongly typed for scripts.
  TypeInfo type;
};

/**
 * @brief A blackboard entry that exists, or may exist, once the tree runs.
 *
 * Every reference to the same entry (after following SubTree remapping,
 * _autoremap and "@") is collected in the same symbol.
 */
struct Symbol
{
  std::string name;
  // Index of the scope that owns the entry.
  int scope = -1;
  std::vector<SymbolRef> refs;
  // The entry exists after the creation of the tree.
  bool exists = false;
  // The entry holds a value after the creation of the tree
  // (SubTree literal, or set by the user before createTree).
  bool has_value = false;
  // Declared with SymbolTableOptions::external_keys.
  bool external = false;
  // Type of the existing entry or of the first strongly typed port.
  TypeInfo type;

  [[nodiscard]] uint8_t access() const;
};

struct SymbolIssue
{
  enum Severity
  {
    INFO,
    WARNING,
    ERROR
  };
  enum Kind
  {
    // Read by some node, but nothing in the tree writes it.
    READ_NEVER_WRITTEN,
    // Assigned with "=" in a script, but nothing creates the entry.
    ASSIGN_WITHOUT_CREATE,
    // Written, but nothing in the tree reads it.
    WRITTEN_NEVER_READ,
    // A script could not be parsed.
    SCRIPT_PARSE_ERROR,
  };
  Severity severity = WARNING;
  Kind kind = READ_NEVER_WRITTEN;
  // Index in SymbolTable::symbols, -1 if not related to a symbol.
  int symbol = -1;
  std::string message;
};

struct SymbolTableOptions
{
  // Keys set from outside the tree (for instance by main() through
  // tree.rootBlackboard()). Relative to the root scope; "@" is accepted.
  std::vector<std::string> external_keys;
  // Report WRITTEN_NEVER_READ (the value may be read by C++ code).
  bool report_unread = true;
};

struct SymbolTable
{
  std::vector<SymbolScope> scopes;
  std::vector<Symbol> symbols;
  std::vector<SymbolIssue> issues;

  /// Index of the symbol that `key`, used in `scope`, refers to. -1 if unknown.
  [[nodiscard]] int find(int scope, const std::string& key) const;

  /// Index of the scope of a blackboard. -1 if the blackboard is not part of the tree.
  [[nodiscard]] int scopeOf(const Blackboard* blackboard) const;

  /// Name of the symbol, qualified with the path of its scope.
  [[nodiscard]] std::string qualifiedName(int symbol) const;
};

/**
 * @brief Collect all blackboard entries used by the tree, and resolve each
 * reference to the entry it will use at runtime.
 *
 * The tree is not modified. References come from:
 *  - ports remapped to "{key}" / "{=}", including default values;
 *  - scripts: Script, ScriptCondition, Precondition and pre/post conditions;
 *  - the "key" of UnsetBlackboard and the "entry" of the EntryUpdated nodes.
 *
 * Keys created by C++ code with Blackboard::set() are not visible here.
 */
[[nodiscard]] SymbolTable buildSymbolTable(const Tree& tree,
                                           const SymbolTableOptions& options = {});

/// Human readable dump of the table and its issues.
[[nodiscard]] std::string toString(const SymbolTable& table);

}  // namespace BT
