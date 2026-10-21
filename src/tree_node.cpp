/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018-2025 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "behaviortree_cpp/tree_node.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <optional>
#include <vector>

namespace BT
{

namespace
{
// Allocator for the data of the nodes (TreeNode::PImpl and its condition scripts).
//
// The nodes of a tree are created one after the other and ticked in the same
// (depth-first) order. Allocating their data next to each other keeps a tree
// compact in memory whatever the state of the heap: in a long-running process
// the general allocator scatters them into the holes left by previous
// allocations, and a tick then misses the cache on every node.
//
// Memory comes in chunks of kChunkSize bytes. Each thread fills its own current
// chunk with 64-byte aligned slots. The last 8 bytes of a slot point back to
// the chunk, found with the (sized) delete. A chunk is freed when its last
// object is destroyed (by any thread) and no thread is filling it anymore.
// The space of destroyed objects is not reused before that: fine for nodes,
// which live as long as their tree.
class NodeDataArena
{
public:
  static constexpr size_t kChunkSize = 64 * 1024;
  static constexpr size_t kAlign = 64;

  static void* allocate(size_t size)
  {
    const size_t slot = slotSize(size);
    Chunk*& current = threadChunk().chunk;
    if(current == nullptr || current->next + slot > current->end)
    {
      Chunk* fresh = newChunk(slot);
      release(current);  // this thread does not fill it anymore
      current = fresh;
    }
    char* ptr = current->next;
    current->next += slot;
    backPointer(ptr, size) = current;
    current->refs.fetch_add(1, std::memory_order_relaxed);
    return ptr;
  }

  // `size` must be the one given to allocate()
  static void deallocate(void* ptr, size_t size) noexcept
  {
    if(ptr != nullptr)
    {
      release(backPointer(static_cast<char*>(ptr), size));
    }
  }

private:
  struct Chunk
  {
    // live objects, +1 while a thread fills the chunk
    std::atomic<size_t> refs{ 1 };
    // free space. Only accessed by the thread filling the chunk.
    char* next = nullptr;
    char* end = nullptr;
  };

  static constexpr size_t slotSize(size_t size)
  {
    return (size + sizeof(Chunk*) + kAlign - 1) & ~(kAlign - 1);
  }

  static Chunk*& backPointer(char* ptr, size_t size)
  {
    return *reinterpret_cast<Chunk**>(ptr + slotSize(size) - sizeof(Chunk*));
  }

  static Chunk* newChunk(size_t min_slot)
  {
    const size_t bytes = std::max(kChunkSize, min_slot + sizeof(Chunk) + kAlign);
    void* mem = std::malloc(bytes);
    if(mem == nullptr)
    {
      throw std::bad_alloc();
    }
    auto* chunk = new(mem) Chunk;
    const auto first = reinterpret_cast<std::uintptr_t>(chunk + 1);
    chunk->next =
        reinterpret_cast<char*>((first + kAlign - 1) & ~std::uintptr_t(kAlign - 1));
    chunk->end = static_cast<char*>(mem) + bytes;
    return chunk;
  }

  static void release(Chunk* chunk) noexcept
  {
    if(chunk != nullptr && chunk->refs.fetch_sub(1, std::memory_order_acq_rel) == 1)
    {
      chunk->~Chunk();
      std::free(chunk);
    }
  }

  struct ThreadChunk
  {
    Chunk* chunk = nullptr;
    ~ThreadChunk()
    {
      release(chunk);
    }
  };

  static ThreadChunk& threadChunk()
  {
    thread_local ThreadChunk thread_chunk;
    return thread_chunk;
  }
};

// Base class: allocate the derived type with NodeDataArena.
// The derived type must be final, so that delete gets the allocated size.
struct ArenaAllocated
{
  static void* operator new(std::size_t size)
  {
    return NodeDataArena::allocate(size);
  }
  static void* operator new(std::size_t size, std::align_val_t align)
  {
    if(static_cast<size_t>(align) > NodeDataArena::kAlign)
    {
      throw std::bad_alloc();
    }
    return NodeDataArena::allocate(size);
  }
  static void operator delete(void* ptr, std::size_t size) noexcept
  {
    NodeDataArena::deallocate(ptr, size);
  }
  static void operator delete(void* ptr, std::size_t size, std::align_val_t) noexcept
  {
    NodeDataArena::deallocate(ptr, size);
  }
};
}  // namespace

// Fields are ordered by access frequency: the first block is read or written by
// every tick and fits in two cache lines (the struct is cache-line aligned), so
// a tick touches as few lines as possible when the tree is not in the cache.
// It is allocated next to the data of the nodes created before and after it.
struct alignas(64) TreeNode::PImpl final : ArenaAllocated
{
  PImpl(std::string name, NodeConfig config)
    : config(std::move(config)), name(std::move(name))
  {}

  //---------- hot: used by every tick ----------
  mutable std::mutex state_mutex;

  NodeStatus status = NodeStatus::IDLE;

  // True when at least one injected callback is set. Written while holding
  // callback_injection_mutex; lets executeTick() skip that mutex otherwise.
  std::atomic<bool> has_injected_callbacks{ false };

  // Number of threads blocked in waitValidStatus(). Guarded by state_mutex,
  // it lets setStatus()/resetStatus() skip notify_all() when nobody waits.
  int status_waiters = 0;

  StatusChangeSignal state_change_signal;

  std::shared_ptr<WakeUpSignal> wake_up;

  // Pre/post condition scripts. Allocated only when requested by
  // preConditionsScripts() / postConditionsScripts() (the only way to set them):
  // most nodes have none, and executeTick() then skips them entirely.
  struct ConditionScripts final : ArenaAllocated
  {
    PreScripts pre;
    PostScripts post;
  };
  std::unique_ptr<ConditionScripts> scripts;

  //---------- used by ports and introspection ----------
  NodeConfig config;

  const std::string name;

  std::string registration_ID;

  //---------- rarely used: allocated on first use ----------

  // Created by the first waitValidStatus(), while holding state_mutex, and never
  // destroyed before the node: a thread that saw status_waiters > 0 under
  // state_mutex can use it.
  std::unique_ptr<std::condition_variable> state_condition_variable;

  struct InjectedCallbacks
  {
    PreTickCallback pre_tick;
    PostTickCallback post_tick;
    TickMonitorCallback tick_monitor;
  };
  // Guarded by callback_injection_mutex
  std::unique_ptr<InjectedCallbacks> callbacks;

  std::mutex callback_injection_mutex;

  // Call while holding callback_injection_mutex
  InjectedCallbacks& injectedCallbacks()
  {
    if(!callbacks)
    {
      callbacks = std::make_unique<InjectedCallbacks>();
    }
    return *callbacks;
  }

  // Call while holding callback_injection_mutex
  void updateInjectedCallbacksFlag()
  {
    has_injected_callbacks.store(
        callbacks &&
            (callbacks->pre_tick || callbacks->post_tick || callbacks->tick_monitor),
        std::memory_order_release);
  }

  ConditionScripts& conditionScripts()
  {
    if(!scripts)
    {
      scripts = std::make_unique<ConditionScripts>();
    }
    return *scripts;
  }

  // Called after a status change, outside state_mutex.
  // The clock is read only if there is a subscriber to receive it.
  void notifyStatusChange(TreeNode& node, NodeStatus prev_status, NodeStatus new_status,
                          bool has_waiters)
  {
    if(has_waiters)
    {
      state_condition_variable->notify_all();
    }
    if(!state_change_signal.empty())
    {
      state_change_signal.notify(std::chrono::high_resolution_clock::now(), node,
                                 prev_status, new_status);
    }
  }
};

void* TreeNode::operator new(std::size_t size)
{
  return NodeDataArena::allocate(size);
}

void* TreeNode::operator new(std::size_t size, std::align_val_t align)
{
  if(static_cast<size_t>(align) > NodeDataArena::kAlign)
  {
    throw std::bad_alloc();
  }
  return NodeDataArena::allocate(size);
}

void* TreeNode::operator new(std::size_t size, const std::nothrow_t&) noexcept
{
  try
  {
    return NodeDataArena::allocate(size);
  }
  catch(...)
  {
    return nullptr;
  }
}

void TreeNode::operator delete(void* ptr, std::size_t size) noexcept
{
  NodeDataArena::deallocate(ptr, size);
}

void TreeNode::operator delete(void* ptr, std::size_t size, std::align_val_t) noexcept
{
  NodeDataArena::deallocate(ptr, size);
}

TreeNode::TreeNode(std::string name, NodeConfig config)
  : _p(new PImpl(std::move(name), std::move(config)))
{}

TreeNode::TreeNode(TreeNode&& other) noexcept : _p(std::move(other._p))
{}

TreeNode& TreeNode::operator=(TreeNode&& other) noexcept
{
  this->_p = std::move(other._p);
  return *this;
}

TreeNode::~TreeNode() = default;

NodeStatus TreeNode::executeTick()
{
  auto new_status = _p->status;
  PreTickCallback pre_tick;
  PostTickCallback post_tick;
  TickMonitorCallback monitor_tick;
  if(_p->has_injected_callbacks.load(std::memory_order_acquire))
  {
    const std::scoped_lock lk(_p->callback_injection_mutex);
    if(const auto& callbacks = _p->callbacks)
    {
      pre_tick = callbacks->pre_tick;
      post_tick = callbacks->post_tick;
      monitor_tick = callbacks->tick_monitor;
    }
  }

  const bool has_condition_scripts = static_cast<bool>(_p->scripts);

  // a pre-condition may return the new status.
  // In this case it override the actual tick()
  std::optional<NodeStatus> precond_status;
  if(has_condition_scripts)
  {
    if(auto precond = checkPreConditions())
    {
      precond_status = precond.value();
    }
  }
  if(precond_status)
  {
    new_status = *precond_status;
  }
  else
  {
    // injected pre-callback
    bool substituted = false;
    if(pre_tick && !isStatusCompleted(_p->status))
    {
      auto override_status = pre_tick(*this);
      if(isStatusCompleted(override_status))
      {
        // don't execute the actual tick()
        substituted = true;
        new_status = override_status;
      }
    }

    // Call the ACTUAL tick
    if(!substituted)
    {
      using namespace std::chrono;
      // The duration is only needed by the tick monitor: don't read the clock otherwise.
      const bool timed = static_cast<bool>(monitor_tick);
      steady_clock::time_point t1;
      if(timed)
      {
        // Use atomic_thread_fence to prevent compiler reordering of time measurements.
        // See issue #861 for details.
        t1 = steady_clock::now();
        std::atomic_thread_fence(std::memory_order_seq_cst);
      }
      try
      {
        new_status = tick();
      }
      catch(const NodeExecutionError&)
      {
        // Already wrapped by a child node, re-throw as-is to preserve original info
        throw;
      }
      catch(const std::exception& ex)
      {
        // Wrap the exception with this node's context
        throw NodeExecutionError({ name(), fullPath(), registrationName() }, ex.what());
      }
      if(timed)
      {
        std::atomic_thread_fence(std::memory_order_seq_cst);
        const auto t2 = steady_clock::now();
        monitor_tick(*this, new_status, duration_cast<microseconds>(t2 - t1));
      }
    }
  }

  // injected post callback
  if(has_condition_scripts && isStatusCompleted(new_status))
  {
    checkPostConditions(new_status);
  }

  if(post_tick)
  {
    auto override_status = post_tick(*this, new_status);
    if(isStatusCompleted(override_status))
    {
      new_status = override_status;
    }
  }

  // preserve the IDLE state if skipped, but communicate SKIPPED to parent
  if(new_status != NodeStatus::SKIPPED)
  {
    setStatus(new_status);
  }
  return new_status;
}

void TreeNode::haltNode()
{
  halt();

  if(!_p->scripts)
  {
    return;
  }
  const auto& parse_executor = _p->scripts->post[size_t(PostCond::ON_HALTED)];
  if(parse_executor)
  {
    Ast::Environment env = { config().blackboard, config().enums };
    parse_executor(env);
  }
}

void TreeNode::setStatus(NodeStatus new_status)
{
  if(new_status == NodeStatus::IDLE)
  {
    throw RuntimeError("Node [", name(),
                       "]: you are not allowed to set manually the status to IDLE. "
                       "If you know what you are doing (?) use resetStatus() instead.");
  }

  NodeStatus prev_status = NodeStatus::IDLE;
  bool has_waiters = false;
  {
    const std::unique_lock<std::mutex> UniqueLock(_p->state_mutex);
    prev_status = _p->status;
    _p->status = new_status;
    has_waiters = _p->status_waiters > 0;
  }
  if(prev_status != new_status)
  {
    _p->notifyStatusChange(*this, prev_status, new_status, has_waiters);
  }
}

TreeNode::PreScripts& TreeNode::preConditionsScripts()
{
  return _p->conditionScripts().pre;
}

TreeNode::PostScripts& TreeNode::postConditionsScripts()
{
  return _p->conditionScripts().post;
}

Expected<NodeStatus> TreeNode::checkPreConditions()
{
  if(!_p->scripts)
  {
    return nonstd::make_unexpected("");  // no precondition
  }
  // Built only when a script exists: copying its shared_ptrs is not free.
  std::optional<Ast::Environment> env_storage;
  auto env = [&]() -> Ast::Environment& {
    if(!env_storage)
    {
      env_storage.emplace(Ast::Environment{ config().blackboard, config().enums });
    }
    return *env_storage;
  };

  // Check pre-conditions in order: FAILURE_IF, SUCCESS_IF, SKIP_IF, WHILE_TRUE.
  // IMPORTANT: _failureIf, _successIf, and _skipIf are evaluated ONLY when the
  // node is IDLE or SKIPPED. They are NOT re-evaluated while the node is RUNNING.
  for(size_t index = 0; index < size_t(PreCond::COUNT_); index++)
  {
    const auto& parse_executor = _p->scripts->pre[index];
    if(!parse_executor)
    {
      continue;
    }

    const auto preID = static_cast<PreCond>(index);

    // _failureIf, _successIf, _skipIf: only checked when IDLE or SKIPPED
    // _while: checked here AND also when RUNNING (see below)
    if(_p->status == NodeStatus::IDLE || _p->status == NodeStatus::SKIPPED)
    {
      // what to do if the condition is true
      if(parse_executor(env()).cast<bool>())
      {
        if(preID == PreCond::FAILURE_IF)
        {
          return NodeStatus::FAILURE;
        }
        if(preID == PreCond::SUCCESS_IF)
        {
          return NodeStatus::SUCCESS;
        }
        if(preID == PreCond::SKIP_IF)
        {
          return NodeStatus::SKIPPED;
        }
      }
      // if the conditions is false
      else if(preID == PreCond::WHILE_TRUE)
      {
        return NodeStatus::SKIPPED;
      }
    }
    else if(_p->status == NodeStatus::RUNNING && preID == PreCond::WHILE_TRUE)
    {
      // _while is the ONLY precondition checked while RUNNING.
      // If the condition becomes false, halt the node and return SKIPPED.
      if(!parse_executor(env()).cast<bool>())
      {
        haltNode();
        return NodeStatus::SKIPPED;
      }
    }
  }
  return nonstd::make_unexpected("");  // no precondition
}

void TreeNode::checkPostConditions(NodeStatus status)
{
  if(!_p->scripts)
  {
    return;
  }
  auto ExecuteScript = [this](const PostCond& cond) {
    const auto& parse_executor = _p->scripts->post[size_t(cond)];
    if(parse_executor)
    {
      Ast::Environment env = { config().blackboard, config().enums };
      parse_executor(env);
    }
  };

  if(status == NodeStatus::SUCCESS)
  {
    ExecuteScript(PostCond::ON_SUCCESS);
  }
  else if(status == NodeStatus::FAILURE)
  {
    ExecuteScript(PostCond::ON_FAILURE);
  }
  ExecuteScript(PostCond::ALWAYS);
}

void TreeNode::resetStatus()
{
  NodeStatus prev_status = NodeStatus::IDLE;
  bool has_waiters = false;
  {
    const std::unique_lock<std::mutex> lock(_p->state_mutex);
    prev_status = _p->status;
    _p->status = NodeStatus::IDLE;
    has_waiters = _p->status_waiters > 0;
  }

  if(prev_status != NodeStatus::IDLE)
  {
    _p->notifyStatusChange(*this, prev_status, NodeStatus::IDLE, has_waiters);
  }
}

NodeStatus TreeNode::status() const
{
  const std::lock_guard<std::mutex> lock(_p->state_mutex);
  return _p->status;
}

NodeStatus TreeNode::waitValidStatus()
{
  std::unique_lock<std::mutex> lock(_p->state_mutex);

  if(!_p->state_condition_variable)
  {
    _p->state_condition_variable = std::make_unique<std::condition_variable>();
  }
  _p->status_waiters++;
  while(isHalted())
  {
    _p->state_condition_variable->wait(lock);
  }
  _p->status_waiters--;
  return _p->status;
}

const std::string& TreeNode::name() const
{
  return _p->name;
}

bool TreeNode::isHalted() const
{
  return _p->status == NodeStatus::IDLE;
}

TreeNode::StatusChangeSubscriber
TreeNode::subscribeToStatusChange(TreeNode::StatusChangeCallback callback)
{
  return _p->state_change_signal.subscribe(std::move(callback));
}

void TreeNode::setPreTickFunction(PreTickCallback callback)
{
  const std::unique_lock lk(_p->callback_injection_mutex);
  _p->injectedCallbacks().pre_tick = std::move(callback);
  _p->updateInjectedCallbacksFlag();
}

void TreeNode::setPostTickFunction(PostTickCallback callback)
{
  const std::unique_lock lk(_p->callback_injection_mutex);
  _p->injectedCallbacks().post_tick = std::move(callback);
  _p->updateInjectedCallbacksFlag();
}

void TreeNode::setTickMonitorCallback(TickMonitorCallback callback)
{
  const std::unique_lock lk(_p->callback_injection_mutex);
  _p->injectedCallbacks().tick_monitor = std::move(callback);
  _p->updateInjectedCallbacksFlag();
}

uint16_t TreeNode::UID() const
{
  return _p->config.uid;
}

const std::string& TreeNode::fullPath() const
{
  return _p->config.path;
}

const std::string& TreeNode::registrationName() const
{
  return _p->registration_ID;
}

const NodeConfig& TreeNode::config() const
{
  return _p->config;
}

NodeConfig& TreeNode::config()
{
  return _p->config;
}

StringView TreeNode::getRawPortValue(const std::string& key) const
{
  auto remap_it = _p->config.input_ports.find(key);
  if(remap_it == _p->config.input_ports.end())
  {
    remap_it = _p->config.output_ports.find(key);
    if(remap_it == _p->config.output_ports.end())
    {
      throw std::logic_error(StrCat("[", key, "] not found"));
    }
  }
  return remap_it->second;
}

bool TreeNode::isBlackboardPointer(StringView str, StringView* stripped_pointer)
{
  if(str.size() < 3)
  {
    return false;
  }
  // strip leading and following spaces
  size_t front_index = 0;
  size_t last_index = str.size() - 1;
  while(str[front_index] == ' ' && front_index <= last_index)
  {
    front_index++;
  }
  while(str[last_index] == ' ' && front_index <= last_index)
  {
    last_index--;
  }
  const auto size = (last_index - front_index) + 1;
  auto valid = size >= 3 && str[front_index] == '{' && str[last_index] == '}';
  if(valid && stripped_pointer != nullptr)
  {
    *stripped_pointer = StringView(&str[front_index + 1], size - 2);
  }
  return valid;
}

StringView TreeNode::stripBlackboardPointer(StringView str)
{
  StringView out;
  if(isBlackboardPointer(str, &out))
  {
    return out;
  }
  return {};
}

std::optional<StringView> TreeNode::blackboardKeyOf(StringView port_name,
                                                    StringView port_value)
{
  if(port_value == "{=}" || port_value == "=")
  {
    return port_name;
  }
  StringView stripped;
  if(isBlackboardPointer(port_value, &stripped))
  {
    return stripped;
  }
  return std::nullopt;
}

Expected<StringView> TreeNode::getRemappedKey(StringView port_name,
                                              StringView remapped_port)
{
  if(auto key = blackboardKeyOf(port_name, remapped_port))
  {
    return *key;
  }
  return nonstd::make_unexpected("Not a blackboard pointer");
}

void TreeNode::emitWakeUpSignal()
{
  if(_p->wake_up)
  {
    _p->wake_up->emitSignal();
  }
}

bool TreeNode::requiresWakeUp() const
{
  return bool(_p->wake_up);
}

void TreeNode::setRegistrationID(StringView ID)
{
  _p->registration_ID.assign(ID.data(), ID.size());
}

void TreeNode::setWakeUpInstance(std::shared_ptr<WakeUpSignal> instance)
{
  _p->wake_up = instance;
}

void TreeNode::modifyPortsRemapping(const PortsRemapping& new_remapping)
{
  for(const auto& new_it : new_remapping)
  {
    auto it = _p->config.input_ports.find(new_it.first);
    if(it != _p->config.input_ports.end())
    {
      it->second = new_it.second;
    }
    it = _p->config.output_ports.find(new_it.first);
    if(it != _p->config.output_ports.end())
    {
      it->second = new_it.second;
    }
  }
}

template <>
std::string toStr<PreCond>(const PreCond& cond)
{
  if(cond < PreCond::COUNT_)
  {
    return BT::PreCondNames[static_cast<size_t>(cond)];
  }
  return "Undefined";
}

template <>
std::string toStr<PostCond>(const PostCond& cond)
{
  if(cond < BT::PostCond::COUNT_)
  {
    return BT::PostCondNames[static_cast<size_t>(cond)];
  }
  return "Undefined";
}

AnyPtrLocked BT::TreeNode::getLockedPortContent(const std::string& key)
{
  if(auto remapped_key = getRemappedKey(key, getRawPortValue(key)))
  {
    const auto bb_key = std::string(*remapped_key);
    auto result = _p->config.blackboard->getAnyLocked(bb_key);
    if(!result && _p->config.manifest != nullptr)
    {
      // Entry doesn't exist yet. Create it using the port's type info
      // from the manifest so that getLockedPortContent works even when
      // the port is not explicitly declared in XML. Issue #942.
      auto port_it = _p->config.manifest->ports.find(key);
      if(port_it != _p->config.manifest->ports.end())
      {
        _p->config.blackboard->createEntry(bb_key, port_it->second);
        result = _p->config.blackboard->getAnyLocked(bb_key);
      }
    }
    return result;
  }
  return {};
}

}  // namespace BT
