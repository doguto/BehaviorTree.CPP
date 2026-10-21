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

#include <array>
#include <atomic>
#include <cstring>
#include <optional>
#include <vector>

namespace BT
{

struct TreeNode::PImpl
{
  PImpl(std::string name, NodeConfig config)
    : name(std::move(name)), config(std::move(config))
  {}

  const std::string name;

  NodeStatus status = NodeStatus::IDLE;

  std::condition_variable state_condition_variable;

  mutable std::mutex state_mutex;

  // Number of threads blocked in waitValidStatus(). Guarded by state_mutex,
  // it lets setStatus()/resetStatus() skip notify_all() when nobody waits.
  int status_waiters = 0;

  StatusChangeSignal state_change_signal;

  // Called after a status change, outside state_mutex.
  // The clock is read only if there is a subscriber to receive it.
  void notifyStatusChange(TreeNode& node, NodeStatus prev_status, NodeStatus new_status,
                          bool has_waiters)
  {
    if(has_waiters)
    {
      state_condition_variable.notify_all();
    }
    if(!state_change_signal.empty())
    {
      state_change_signal.notify(std::chrono::high_resolution_clock::now(), node,
                                 prev_status, new_status);
    }
  }

  NodeConfig config;

  std::string registration_ID;

  PreTickCallback pre_tick_callback;
  PostTickCallback post_tick_callback;
  TickMonitorCallback tick_monitor_callback;

  std::mutex callback_injection_mutex;

  // True when at least one injected callback is set. Written while holding
  // callback_injection_mutex; lets executeTick() skip that mutex otherwise.
  std::atomic<bool> has_injected_callbacks{ false };

  void updateInjectedCallbacksFlag()
  {
    has_injected_callbacks.store(pre_tick_callback || post_tick_callback ||
                                     tick_monitor_callback,
                                 std::memory_order_release);
  }

  std::shared_ptr<WakeUpSignal> wake_up;

  std::array<ScriptFunction, size_t(PreCond::COUNT_)> pre_parsed;
  std::array<ScriptFunction, size_t(PostCond::COUNT_)> post_parsed;
};

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
    pre_tick = _p->pre_tick_callback;
    post_tick = _p->post_tick_callback;
    monitor_tick = _p->tick_monitor_callback;
  }

  // a pre-condition may return the new status.
  // In this case it override the actual tick()
  if(auto precond = checkPreConditions())
  {
    new_status = precond.value();
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
  if(isStatusCompleted(new_status))
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

  const auto& parse_executor = _p->post_parsed[size_t(PostCond::ON_HALTED)];
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
  return _p->pre_parsed;
}

TreeNode::PostScripts& TreeNode::postConditionsScripts()
{
  return _p->post_parsed;
}

Expected<NodeStatus> TreeNode::checkPreConditions()
{
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
    const auto& parse_executor = _p->pre_parsed[index];
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
  auto ExecuteScript = [this](const PostCond& cond) {
    const auto& parse_executor = _p->post_parsed[size_t(cond)];
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

  _p->status_waiters++;
  while(isHalted())
  {
    _p->state_condition_variable.wait(lock);
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
  _p->pre_tick_callback = std::move(callback);
  _p->updateInjectedCallbacksFlag();
}

void TreeNode::setPostTickFunction(PostTickCallback callback)
{
  const std::unique_lock lk(_p->callback_injection_mutex);
  _p->post_tick_callback = std::move(callback);
  _p->updateInjectedCallbacksFlag();
}

void TreeNode::setTickMonitorCallback(TickMonitorCallback callback)
{
  const std::unique_lock lk(_p->callback_injection_mutex);
  _p->tick_monitor_callback = std::move(callback);
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

Expected<StringView> TreeNode::getRemappedKey(StringView port_name,
                                              StringView remapped_port)
{
  if(remapped_port == "{=}" || remapped_port == "=")
  {
    return { port_name };
  }
  StringView stripped;
  if(isBlackboardPointer(remapped_port, &stripped))
  {
    return { stripped };
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
