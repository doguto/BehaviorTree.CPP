#pragma once

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/utils/shared_timer_queue.h"
#include "behaviortree_cpp/utils/timer_queue.h"

#include <atomic>

namespace BT
{
/**
 * @brief Sleep for a certain amount of time.
 * Consider also using the decorator <Delay/>
 *
 * <Sleep msec="5000"/>
 */
class SleepNode : public StatefulActionNode
{
public:
  SleepNode(const std::string& name, const NodeConfig& config);

  /// Uses `shared_timer`, when not null, instead of starting its own timer
  /// thread (see BehaviorTreeFactory::useSharedTimerThread()).
  SleepNode(const std::string& name, const NodeConfig& config,
            std::shared_ptr<SharedTimerQueue> shared_timer);

  ~SleepNode() override
  {
    halt();
  }

  SleepNode(const SleepNode&) = delete;
  SleepNode& operator=(const SleepNode&) = delete;
  SleepNode(SleepNode&&) = delete;
  SleepNode& operator=(SleepNode&&) = delete;

  NodeStatus onStart() override;

  NodeStatus onRunning() override;

  void onHalted() override;

  static PortsList providedPorts()
  {
    return { InputPort<unsigned>("msec") };
  }

private:
  // Its own timer thread (default), or the shared one
  std::unique_ptr<TimerQueue<>> timer_;
  std::shared_ptr<SharedTimerQueue> shared_timer_;
  uint64_t timer_id_ = 0;

  std::atomic_bool timer_waiting_ = false;
  std::mutex delay_mutex_;
};

}  // namespace BT
