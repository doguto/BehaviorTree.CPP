// Per-tick latency distribution of a real application tree, without its nodes.
//
// Loads the application's XML files, and replaces every node that is not built
// into BT.CPP with a stub having the same type and ports, as described by the
// <TreeNodesModel> found in those files (Groot2 writes it; it can also be
// generated with BT::writeTreeNodesModelXML(factory) and passed with --models).
// This measures the cost of the library itself on the real tree shape: the real
// nodes' own work (ROS, I/O, computation) is not included.
//
// Stubs:
//   Action     reads all its input ports (literals as number, bool or string,
//              guessed from the text) and writes back all its output ports,
//              then stays RUNNING for --running-ticks ticks, then SUCCESS.
//              Ports are touched on every tick, as a polling node would do.
//   Condition  touches its ports, returns SUCCESS.
//   Decorator  touches its ports, returns the status of its child.
// So, ticking repeatedly walks through the whole application, like a mission.
//
// Every blackboard key referenced by a port or a script is seeded with 0 (or a
// value of the entry's type), since the real nodes that would write them are
// gone. Branches that depend on data (Switch, Precondition...) may need other
// values to walk the whole application: give them with --set=key=value,...
// (strings are converted to the type of the entry when it has one). Optionally, a thread keeps writing some keys (--writer-keys,
// --writer-hz) like a ROS topic monitor, and --groot2 attaches a
// Groot2Publisher, as a deployed application may do.
//
// Usage: bt_xml_latency --xml=a.xml,b.xml --main=TreeID [--models=m.xml,...]
//                       [--running-ticks=N] [--writer-keys=k1,k2] [--writer-hz=N]
//                       [--set=key=value,...] [--delay-scale=F] [--real-delays]
//                       [--shared-timer] [--groot2[=PORT]] [--trace=N]
//                       [latency options]
// --trace=N only ticks N times, printing the RUNNING leaves when they change.
// --real-delays keeps the built-in Delay/Sleep nodes. Otherwise they wait
// --delay-scale ticks per millisecond (default 0.1, i.e. 2000 ms -> 200 ticks).
// --shared-timer: BehaviorTreeFactory::useSharedTimerThread() (with --real-delays).
#include "latency_harness.h"

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/xml_parsing.h"
#ifdef BENCH_WITH_GROOT2
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#endif

#include <atomic>
#include <filesystem>
#include <regex>
#include <set>

#include <tinyxml2.h>

using namespace bench;
namespace fs = std::filesystem;

namespace
{
struct StubSettings
{
  int running_ticks = 1;
  bool real_delays = false;
  bool shared_timer = false;
  // Delay/Sleep stubs wait this many ticks per millisecond of their duration
  double delay_scale = 0.1;

  int delayTicks(unsigned msec) const
  {
    return std::max(1, int(std::lround(msec * delay_scale)));
  }
};

// Base of the stubs: touch every port like a real node would, i.e. read the
// inputs and write the outputs back.
template <typename Base>
class PortToucher : public Base
{
public:
  using Base::Base;

protected:
  void touchPorts()
  {
    const auto& config = this->config();
    if(!inputs_initialized_)
    {
      // The real node would read a literal with its own type: guess it from
      // the text, once (the cost of guessing is not part of the measure).
      for(const auto& [name, value] : config.input_ports)
      {
        inputs_.emplace_back(name, kindOf(value));
      }
      inputs_initialized_ = true;
    }
    for(const auto& [name, kind] : inputs_)
    {
      switch(kind)
      {
        case Kind::Blackboard: {
          BT::Any any;
          (void)this->getInput(name, any);
          break;
        }
        case Kind::Number: {
          double number = 0;
          (void)this->getInput(name, number);
          break;
        }
        case Kind::Bool: {
          bool flag = false;
          (void)this->getInput(name, flag);
          break;
        }
        case Kind::String: {
          std::string str;
          (void)this->getInput(name, str);
          break;
        }
      }
    }
    for(const auto& [name, value] : config.output_ports)
    {
      if(!config.blackboard)
      {
        break;
      }
      BT::StringView key = value;
      if(value == "{=}" || value == "=")
      {
        key = name;
      }
      else if(!BT::TreeNode::isBlackboardPointer(value, &key))
      {
        continue;
      }
      BT::Any current;
      if(auto entry = config.blackboard->getEntry(std::string(key)))
      {
        std::scoped_lock lk(entry->entry_mutex);
        current = entry->value;
      }
      if(!current.empty())
      {
        (void)this->setOutput(name, current);
      }
    }
  }

private:
  enum class Kind
  {
    Blackboard,
    Number,
    Bool,
    String
  };

  static Kind kindOf(const std::string& value)
  {
    if(BT::TreeNode::isBlackboardPointer(value) || value == "{=}" || value == "=")
    {
      return Kind::Blackboard;
    }
    if(value == "true" || value == "false")
    {
      return Kind::Bool;
    }
    char* end = nullptr;
    (void)std::strtod(value.c_str(), &end);
    if(!value.empty() && *end == '\0')
    {
      return Kind::Number;
    }
    return Kind::String;
  }

  std::vector<std::pair<std::string, Kind>> inputs_;
  bool inputs_initialized_ = false;
};

class StubAction : public PortToucher<BT::StatefulActionNode>
{
public:
  StubAction(const std::string& name, const BT::NodeConfig& config,
             const StubSettings& settings)
    : PortToucher(name, config), settings_(settings)
  {}
  BT::NodeStatus onStart() override
  {
    touchPorts();
    remaining_ = settings_.running_ticks;
    return remaining_ > 0 ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  BT::NodeStatus onRunning() override
  {
    touchPorts();
    return --remaining_ > 0 ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  void onHalted() override
  {}

private:
  const StubSettings& settings_;
  int remaining_ = 0;
};

class StubCondition : public PortToucher<BT::ConditionNode>
{
public:
  StubCondition(const std::string& name, const BT::NodeConfig& config)
    : PortToucher(name, config)
  {}
  BT::NodeStatus tick() override
  {
    touchPorts();
    return BT::NodeStatus::SUCCESS;
  }
};

class StubDecorator : public PortToucher<BT::DecoratorNode>
{
public:
  StubDecorator(const std::string& name, const BT::NodeConfig& config)
    : PortToucher(name, config)
  {}
  BT::NodeStatus tick() override
  {
    touchPorts();
    const auto status = child()->executeTick();
    if(BT::isStatusCompleted(status))
    {
      resetChild();
    }
    return status;
  }
};

// Replaces the built-in Delay: waits a number of ticks proportional to delay_msec
// instead of wall-clock time, then ticks its child. Delays are often used as
// timeouts (e.g. in a Parallel with a polling condition): keeping them longer
// than the action stubs preserves the logic of the tree.
class StubDelay : public PortToucher<BT::DecoratorNode>
{
public:
  StubDelay(const std::string& name, const BT::NodeConfig& config,
            const StubSettings& settings)
    : PortToucher(name, config), settings_(settings)
  {}
  BT::NodeStatus tick() override
  {
    if(!started_)
    {
      unsigned msec = 0;
      (void)getInput("delay_msec", msec);
      remaining_ = settings_.delayTicks(msec);
      started_ = true;
    }
    if(remaining_ > 0)
    {
      remaining_--;
      return BT::NodeStatus::RUNNING;
    }
    const auto status = child()->executeTick();
    if(BT::isStatusCompleted(status))
    {
      started_ = false;
      resetChild();
    }
    return status;
  }
  void halt() override
  {
    started_ = false;
    DecoratorNode::halt();
  }

private:
  const StubSettings& settings_;
  bool started_ = false;
  int remaining_ = 0;
};

// Replaces the built-in Sleep, see StubDelay.
class StubSleep : public PortToucher<BT::StatefulActionNode>
{
public:
  StubSleep(const std::string& name, const BT::NodeConfig& config,
            const StubSettings& settings)
    : PortToucher(name, config), settings_(settings)
  {}
  BT::NodeStatus onStart() override
  {
    unsigned msec = 0;
    (void)getInput("msec", msec);
    remaining_ = settings_.delayTicks(msec);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    return --remaining_ > 0 ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  void onHalted() override
  {}

private:
  const StubSettings& settings_;
  int remaining_ = 0;
};

struct ParsedFiles
{
  std::map<std::string, BT::TreeNodeManifest> models;
  // blackboard keys referenced by ports or scripts, per BehaviorTree ID
  std::map<std::string, std::set<std::string>> keys;
  // keys assigned by a script (`key := ...`): the script creates them with the
  // right type, so they are not seeded
  std::set<std::string> assigned;
};

const std::set<std::string> kScriptAttributes = {
  "code",   "if",         "_skipIf",    "_failureIf", "_successIf",
  "_while", "_onSuccess", "_onFailure", "_post",      "_onHalted",
};

void collectKeys(const tinyxml2::XMLElement* el, std::set<std::string>& keys,
                 std::set<std::string>& assigned)
{
  static const std::regex identifier(R"([A-Za-z_][A-Za-z0-9_]*)");
  static const std::regex assignment(R"(([A-Za-z_][A-Za-z0-9_]*)\s*:=)");
  static const std::regex quoted(R"('[^']*')");
  for(auto attr = el->FirstAttribute(); attr != nullptr; attr = attr->Next())
  {
    const std::string name = attr->Name();
    const std::string value = attr->Value();
    BT::StringView stripped;
    if(BT::TreeNode::isBlackboardPointer(value, &stripped))
    {
      keys.insert(std::string(stripped));
    }
    else if(kScriptAttributes.count(name) != 0)
    {
      const std::string code = std::regex_replace(value, quoted, "");
      for(auto it = std::sregex_iterator(code.begin(), code.end(), assignment);
          it != std::sregex_iterator(); ++it)
      {
        assigned.insert((*it)[1].str());
      }
      for(auto it = std::sregex_iterator(code.begin(), code.end(), identifier);
          it != std::sregex_iterator(); ++it)
      {
        const std::string id = it->str();
        if(id != "true" && id != "false")
        {
          keys.insert(id);
        }
      }
    }
  }
  for(auto child = el->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    collectKeys(child, keys, assigned);
  }
}

void parseFile(const fs::path& path, ParsedFiles& out, std::set<fs::path>& visited)
{
  const auto canonical = fs::weakly_canonical(path);
  if(!visited.insert(canonical).second)
  {
    return;
  }
  tinyxml2::XMLDocument doc;
  if(doc.LoadFile(canonical.string().c_str()) != tinyxml2::XML_SUCCESS)
  {
    throw BT::RuntimeError("cannot parse ", canonical.string());
  }
  const auto* root = doc.RootElement();
  for(auto el = root->FirstChildElement(); el; el = el->NextSiblingElement())
  {
    const std::string tag = el->Name();
    if(tag == "include" && el->Attribute("path"))
    {
      parseFile(canonical.parent_path() / el->Attribute("path"), out, visited);
    }
    else if(tag == "BehaviorTree" && el->Attribute("ID"))
    {
      collectKeys(el, out.keys[el->Attribute("ID")], out.assigned);
    }
    else if(tag == "TreeNodesModel")
    {
      for(auto n = el->FirstChildElement(); n; n = n->NextSiblingElement())
      {
        if(!n->Attribute("ID"))
        {
          continue;
        }
        BT::TreeNodeManifest manifest;
        manifest.type = BT::convertFromString<BT::NodeType>(n->Name());
        manifest.registration_ID = n->Attribute("ID");
        for(auto p = n->FirstChildElement(); p; p = p->NextSiblingElement())
        {
          const std::string kind = p->Name();
          const auto dir = kind == "input_port"  ? BT::PortDirection::INPUT :
                           kind == "output_port" ? BT::PortDirection::OUTPUT :
                                                   BT::PortDirection::INOUT;
          if(p->Attribute("name"))
          {
            manifest.ports.emplace(p->Attribute("name"), BT::PortInfo(dir));
          }
        }
        out.models[manifest.registration_ID] = std::move(manifest);
      }
    }
  }
}

// Built-in nodes that wait for wall-clock time. Unless --real-delays is given they
// are replaced by StubDelay / StubSleep, which wait a number of ticks instead:
// with wall-clock time, the number of (trivial) ticks spent waiting would depend
// on the speed of the tick itself, not on the tree.
const std::set<std::string> kTimedBuiltins = { "Delay", "Sleep" };

// Registers a stub for every modeled node that the factory does not know.
size_t registerStubs(BT::BehaviorTreeFactory& factory, const ParsedFiles& parsed,
                     const StubSettings& settings)
{
  std::map<std::string, BT::TreeNodeManifest> to_stub = parsed.models;
  factory.useSharedTimerThread(settings.shared_timer);
  if(!settings.real_delays)
  {
    // built-ins can't be unregistered: register the stub under another ID and
    // substitute it when the tree is created
    for(const auto& id : kTimedBuiltins)
    {
      if(auto it = factory.manifests().find(id); it != factory.manifests().end())
      {
        auto manifest = it->second;
        manifest.registration_ID = "BenchStub_" + id;
        to_stub[manifest.registration_ID] = manifest;
        factory.addSubstitutionRule(id, manifest.registration_ID);
      }
    }
  }
  size_t count = 0;
  for(const auto& [id, manifest] : to_stub)
  {
    if(factory.manifests().count(id) != 0)
    {
      continue;
    }
    BT::NodeBuilder builder;
    if(id == "BenchStub_Delay")
    {
      builder = [&settings](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<StubDelay>(name, config, settings);
      };
      factory.registerBuilder(manifest, builder);
      count++;
      continue;
    }
    if(id == "BenchStub_Sleep")
    {
      builder = [&settings](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<StubSleep>(name, config, settings);
      };
      factory.registerBuilder(manifest, builder);
      count++;
      continue;
    }
    switch(manifest.type)
    {
      case BT::NodeType::ACTION:
        builder = [&settings](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<StubAction>(name, config, settings);
        };
        break;
      case BT::NodeType::CONDITION:
        builder = [](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<StubCondition>(name, config);
        };
        break;
      case BT::NodeType::DECORATOR:
        builder = [](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<StubDecorator>(name, config);
        };
        break;
      case BT::NodeType::SUBTREE:
        continue;
      default:
        std::fprintf(stderr, "warning: no stub for node [%s] of type %s\n", id.c_str(),
                     BT::toStr(manifest.type).c_str());
        continue;
    }
    factory.registerBuilder(manifest, builder);
    count++;
  }
  return count;
}

// Value given with --set, for an entry that has no type yet: number, bool or string.
BT::Any parseLiteral(const std::string& text)
{
  if(text == "true" || text == "false")
  {
    return BT::Any(text == "true");
  }
  char* end = nullptr;
  const long long i = std::strtoll(text.c_str(), &end, 10);
  if(!text.empty() && *end == '\0')
  {
    return BT::Any(int64_t(i));
  }
  const double d = std::strtod(text.c_str(), &end);
  if(!text.empty() && *end == '\0')
  {
    return BT::Any(d);
  }
  return BT::Any(text);
}

// A value of the same type as the entry, parsed from `text`, or empty.
BT::Any valueLike(const BT::Blackboard::Entry& entry, const std::string& text)
{
  if(!entry.info.isStronglyTyped() || entry.info.type() == typeid(BT::Any))
  {
    return parseLiteral(text);
  }
  // Entries created by Blackboard::set(key, Any) carry a converter for BT::Any,
  // not for their actual type: build common types directly.
  const auto& type = entry.info.type();
  if(type == typeid(int64_t) || type == typeid(int) || type == typeid(int32_t))
  {
    return BT::Any(int64_t(std::stoll(text)));
  }
  if(type == typeid(double) || type == typeid(float))
  {
    return BT::Any(std::stod(text));
  }
  if(type == typeid(bool))
  {
    return BT::Any(text == "1" || text == "true");
  }
  if(type == typeid(std::string))
  {
    return BT::Any(text);
  }
  try
  {
    return entry.info.parseString(text);
  }
  catch(const std::exception&)
  {
    return {};
  }
}

// Seeds every key referenced by each subtree: with the value given by --set if
// any, otherwise 0 (converted to the type of the entry, when it has one).
void seedBlackboards(BT::Tree& tree, const ParsedFiles& parsed,
                     const std::map<std::string, std::string>& overrides)
{
  for(const auto& subtree : tree.subtrees)
  {
    auto it = parsed.keys.find(subtree->tree_ID);
    if(it == parsed.keys.end())
    {
      continue;
    }
    auto& bb = *subtree->blackboard;
    for(const auto& key : it->second)
    {
      const auto ov = overrides.find(key);
      if(ov == overrides.end() && parsed.assigned.count(key) != 0)
      {
        continue;
      }
      const std::string text = ov != overrides.end() ? ov->second : "0";
      auto entry = bb.getEntry(key);
      if(!entry)
      {
        bb.set(key, parseLiteral(text));
        continue;
      }
      BT::Any value;
      {
        std::scoped_lock lk(entry->entry_mutex);
        // already set: by the application, or by us through another subtree
        // sharing the same entry (remapping)
        if(!entry->value.empty())
        {
          continue;
        }
        value =
            entry->info.isStronglyTyped() ? valueLike(*entry, text) : parseLiteral(text);
      }
      if(!value.empty())
      {
        bb.set(key, value);
      }
    }
  }
}

// Writes some keys of the root blackboard at a fixed rate, from another thread.
class BlackboardWriter
{
public:
  BlackboardWriter(BT::Blackboard::Ptr bb, const std::vector<std::string>& keys, int hz)
  {
    for(const auto& key : keys)
    {
      auto entry = bb->getEntry(key);
      if(!entry)
      {
        bb->set(key, int64_t(0));
        entry = bb->getEntry(key);
      }
      std::scoped_lock lk(entry->entry_mutex);
      Values v{ key, valueLike(*entry, "0"), valueLike(*entry, "1") };
      if(!v.zero.empty() && !v.one.empty())
      {
        values_.push_back(std::move(v));
      }
    }
    thread_ = std::thread([this, bb, hz] {
      const auto period = std::chrono::nanoseconds(1000000000LL / std::max(1, hz));
      auto next = Clock::now();
      bool flip = false;
      while(!stop_)
      {
        for(const auto& v : values_)
        {
          bb->set(v.key, flip ? v.one : v.zero);
        }
        flip = !flip;
        next += period;
        std::this_thread::sleep_until(next);
      }
    });
  }
  ~BlackboardWriter()
  {
    stop_ = true;
    thread_.join();
  }

private:
  struct Values
  {
    std::string key;
    BT::Any zero;
    BT::Any one;
  };
  std::vector<Values> values_;
  std::atomic<bool> stop_{ false };
  std::thread thread_;
};

struct XmlOptions
{
  std::vector<std::string> xml_files;
  std::vector<std::string> model_files;
  std::vector<std::string> main_trees;
  StubSettings stubs;
  std::vector<std::string> writer_keys;
  int writer_hz = 1000;
  int groot2_port = 0;
  int trace_ticks = 0;
  std::map<std::string, std::string> overrides;
};

// Prints which leaves are RUNNING each time it changes: shows how the stubbed
// application progresses (and where it gets stuck, if it does).
void traceTicks(BT::Tree& tree, int ticks)
{
  std::string previous;
  int same = 0;
  for(int i = 0; i < ticks; i++)
  {
    const auto status = tree.tickExactlyOnce();
    std::string running;
    tree.applyVisitor([&](BT::TreeNode* node) {
      if(node->type() != BT::NodeType::CONTROL &&
         node->type() != BT::NodeType::DECORATOR &&
         node->type() != BT::NodeType::SUBTREE &&
         node->status() == BT::NodeStatus::RUNNING)
      {
        running += " " + node->fullPath() + " (" + node->registrationName() + ")";
      }
    });
    if(running == previous)
    {
      same++;
      continue;
    }
    if(same > 0)
    {
      std::printf("        ... same for %d more ticks\n", same);
    }
    same = 0;
    previous = running;
    std::printf("tick %5d %-8s running:%s\n", i, BT::toStr(status).c_str(),
                running.empty() ? " -" : running.c_str());
  }
  if(same > 0)
  {
    std::printf("        ... same for %d more ticks\n", same);
  }
}

std::vector<std::string> splitList(const std::string& s)
{
  std::vector<std::string> out;
  for(const auto& part : BT::splitString(s, ','))
  {
    if(!part.empty())
    {
      out.emplace_back(part);
    }
  }
  return out;
}
}  // namespace

int main(int argc, char** argv)
{
  LatencyOptions opt;
  XmlOptions xopt;
  for(int i = 1; i < argc; i++)
  {
    const std::string a = argv[i];
    auto value = [&](const char* prefix) -> const char* {
      const size_t n = std::strlen(prefix);
      return a.compare(0, n, prefix) == 0 ? argv[i] + n : nullptr;
    };
    if(opt.parse(a))
      continue;
    if(auto v = value("--xml="))
      xopt.xml_files = splitList(v);
    else if(auto v = value("--models="))
      xopt.model_files = splitList(v);
    else if(auto v = value("--main="))
      xopt.main_trees = splitList(v);
    else if(auto v = value("--running-ticks="))
      xopt.stubs.running_ticks = std::max(0, std::atoi(v));
    else if(auto v = value("--writer-keys="))
      xopt.writer_keys = splitList(v);
    else if(auto v = value("--writer-hz="))
      xopt.writer_hz = std::max(1, std::atoi(v));
    else if(a == "--groot2")
      xopt.groot2_port = 1667;
    else if(auto v = value("--set="))
    {
      for(const auto& kv : splitList(v))
      {
        const auto eq = kv.find('=');
        if(eq == std::string::npos)
        {
          std::fprintf(stderr, "--set expects key=value, got [%s]\n", kv.c_str());
          return 1;
        }
        xopt.overrides[kv.substr(0, eq)] = kv.substr(eq + 1);
      }
    }
    else if(auto v = value("--delay-scale="))
      xopt.stubs.delay_scale = std::atof(v);
    else if(a == "--real-delays")
      xopt.stubs.real_delays = true;
    else if(a == "--shared-timer")
      xopt.stubs.shared_timer = true;
    else if(auto v = value("--trace="))
      xopt.trace_ticks = std::atoi(v);
    else if(auto v = value("--groot2="))
      xopt.groot2_port = std::atoi(v);
    else
    {
      std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
      return 1;
    }
  }
  if(xopt.xml_files.empty() || xopt.main_trees.empty())
  {
    std::fprintf(stderr,
                 "usage: bt_xml_latency --xml=a.xml,b.xml --main=TreeID "
                 "[--models=m.xml] [--running-ticks=N] [--writer-keys=k1,k2] "
                 "[--writer-hz=N] [--groot2[=PORT]]\nlatency options:\n%s",
                 LatencyOptions::usage);
    return 1;
  }
#ifndef BENCH_WITH_GROOT2
  if(xopt.groot2_port != 0)
  {
    std::fprintf(stderr, "--groot2: library built without BTCPP_GROOT_INTERFACE\n");
    return 1;
  }
#endif

  ParsedFiles parsed;
  std::set<fs::path> visited;
  for(const auto& f : xopt.xml_files)
  {
    parseFile(f, parsed, visited);
  }
  for(const auto& f : xopt.model_files)
  {
    parseFile(f, parsed, visited);
  }

  // Report the shape of the trees once
  for(const auto& main_tree : xopt.main_trees)
  {
    BT::BehaviorTreeFactory factory;
    const size_t stubs = registerStubs(factory, parsed, xopt.stubs);
    for(const auto& f : xopt.xml_files)
    {
      factory.registerBehaviorTreeFromFile(f);
    }
    const auto t1 = std::chrono::steady_clock::now();
    auto tree = factory.createTree(main_tree);
    const auto t2 = std::chrono::steady_clock::now();
    size_t nodes = 0;
    tree.applyVisitor([&](BT::TreeNode*) { nodes++; });
    std::printf("tree [%s]: %zu nodes, %zu subtree instances, %zu stub types, "
                "created in %.2f ms\n",
                main_tree.c_str(), nodes, tree.subtrees.size(), stubs,
                std::chrono::duration<double, std::milli>(t2 - t1).count());
    if(xopt.trace_ticks > 0)
    {
      seedBlackboards(tree, parsed, xopt.overrides);
      traceTicks(tree, xopt.trace_ticks);
    }
  }
  if(xopt.trace_ticks > 0)
  {
    return 0;
  }

  LatencyRunner runner(opt);
  runner.addContext("running_ticks", std::to_string(xopt.stubs.running_ticks));
  runner.addContext("delay_scale", xopt.stubs.real_delays ?
                                       "real" :
                                       std::to_string(xopt.stubs.delay_scale));
  runner.addContext("writer_hz",
                    xopt.writer_keys.empty() ? "0" : std::to_string(xopt.writer_hz));
  runner.addContext("groot2", xopt.groot2_port ? "1" : "0");

  for(const auto& main_tree : xopt.main_trees)
  {
    auto make = [&]() {
      struct Holder
      {
        BT::BehaviorTreeFactory factory;
        BT::Tree tree;
        std::unique_ptr<BlackboardWriter> writer;
#ifdef BENCH_WITH_GROOT2
        std::unique_ptr<BT::Groot2Publisher> groot2;
#endif
        ~Holder()
        {
          writer.reset();  // stop writing before the blackboard goes away
        }
      };
      auto h = std::make_shared<Holder>();
      registerStubs(h->factory, parsed, xopt.stubs);
      for(const auto& f : xopt.xml_files)
      {
        h->factory.registerBehaviorTreeFromFile(f);
      }
      h->tree = h->factory.createTree(main_tree);
      seedBlackboards(h->tree, parsed, xopt.overrides);
      if(!xopt.writer_keys.empty())
      {
        h->writer = std::make_unique<BlackboardWriter>(h->tree.rootBlackboard(),
                                                       xopt.writer_keys, xopt.writer_hz);
      }
#ifdef BENCH_WITH_GROOT2
      if(xopt.groot2_port != 0)
      {
        h->groot2 = std::make_unique<BT::Groot2Publisher>(h->tree, xopt.groot2_port);
      }
#endif
      return TreeInstance{ h, &h->tree };
    };
    // A real application may return any status: all of them are valid.
    auto check = [](BT::NodeStatus) { return std::string(); };
    runner.run(main_tree, make, check);
  }
  return runner.finish();
}
