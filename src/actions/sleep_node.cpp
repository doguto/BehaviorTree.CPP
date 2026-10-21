#include "behaviortree_cpp/actions/sleep_node.h"

namespace BT
{

SleepNode::SleepNode(const std::string& name, const NodeConfig& config)
  : SleepNode(name, config, nullptr)
{}

SleepNode::SleepNode(const std::string& name, const NodeConfig& config,
                     std::shared_ptr<SharedTimerQueue> shared_timer)
  : StatefulActionNode(name, config)
  , timer_(shared_timer ? nullptr : std::make_unique<TimerQueue<>>())
  , shared_timer_(std::move(shared_timer))
  , timer_waiting_(false)
{}

NodeStatus SleepNode::onStart()
{
  unsigned msec = 0;
  if(!getInput("msec", msec))
  {
    throw RuntimeError("Missing parameter [msec] in SleepNode");
  }

  if(msec <= 0)
  {
    return NodeStatus::SUCCESS;
  }

  setStatus(NodeStatus::RUNNING);

  timer_waiting_ = true;

  if(shared_timer_)
  {
    timer_id_ = shared_timer_->add(std::chrono::milliseconds(msec), [this]() {
      const std::unique_lock<std::mutex> lk(delay_mutex_);
      emitWakeUpSignal();
      timer_waiting_ = false;
    });
  }
  else
  {
    timer_id_ = timer_->add(std::chrono::milliseconds(msec), [this](bool aborted) {
      const std::unique_lock<std::mutex> lk(delay_mutex_);
      if(!aborted)
      {
        emitWakeUpSignal();
      }
      timer_waiting_ = false;
    });
  }

  return NodeStatus::RUNNING;
}

NodeStatus SleepNode::onRunning()
{
  return timer_waiting_ ? NodeStatus::RUNNING : NodeStatus::SUCCESS;
}

void SleepNode::onHalted()
{
  timer_waiting_ = false;
  if(shared_timer_)
  {
    // the callback is not running anymore when cancel() returns
    shared_timer_->cancel(timer_id_);
  }
  else
  {
    timer_->cancel(timer_id_);
  }
}

}  // namespace BT
