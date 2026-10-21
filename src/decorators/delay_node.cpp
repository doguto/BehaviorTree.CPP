/*  Contributed by Indraneel on 26/04/2020
 */

#include "behaviortree_cpp/decorators/delay_node.h"

namespace BT
{
DelayNode::DelayNode(const std::string& name, unsigned milliseconds)
  : DecoratorNode(name, {})
  , timer_(std::make_unique<TimerQueue<>>())
  , timer_id_(0)
  , msec_(milliseconds)
  , read_parameter_from_ports_(false)
{
  setRegistrationID("Delay");
}

DelayNode::DelayNode(const std::string& name, const NodeConfig& config)
  : DelayNode(name, config, nullptr)
{}

DelayNode::DelayNode(const std::string& name, const NodeConfig& config,
                     std::shared_ptr<SharedTimerQueue> shared_timer)
  : DecoratorNode(name, config)
  , timer_(shared_timer ? nullptr : std::make_unique<TimerQueue<>>())
  , shared_timer_(std::move(shared_timer))
  , timer_id_(0)
  , msec_(0)
  , read_parameter_from_ports_(true)
{}

void DelayNode::halt()
{
  delay_started_ = false;
  if(shared_timer_)
  {
    // the callback is not running anymore when cancel() returns
    shared_timer_->cancel(timer_id_);
  }
  else
  {
    timer_->cancelAll();
  }
  DecoratorNode::halt();
}

NodeStatus DelayNode::tick()
{
  if(read_parameter_from_ports_)
  {
    if(!getInput("delay_msec", msec_))
    {
      throw RuntimeError("Missing parameter [delay_msec] in DelayNode");
    }
  }

  if(!delay_started_)
  {
    delay_complete_ = false;
    delay_aborted_ = false;
    delay_started_ = true;
    setStatus(NodeStatus::RUNNING);

    if(shared_timer_)
    {
      timer_id_ = shared_timer_->add(std::chrono::milliseconds(msec_), [this]() {
        const std::unique_lock<std::mutex> lk(delay_mutex_);
        delay_complete_ = true;
        emitWakeUpSignal();
      });
    }
    else
    {
      timer_id_ = timer_->add(std::chrono::milliseconds(msec_), [this](bool aborted) {
        const std::unique_lock<std::mutex> lk(delay_mutex_);
        delay_complete_ = (!aborted);
        if(!aborted)
        {
          emitWakeUpSignal();
        }
      });
    }
  }

  const std::unique_lock<std::mutex> lk(delay_mutex_);

  if(delay_aborted_)
  {
    delay_aborted_ = false;
    delay_started_ = false;
    return NodeStatus::FAILURE;
  }
  if(delay_complete_)
  {
    const NodeStatus child_status = child()->executeTick();
    if(isStatusCompleted(child_status))
    {
      delay_started_ = false;
      delay_aborted_ = false;
      resetChild();
    }
    return child_status;
  }
  return NodeStatus::RUNNING;
}

}  // namespace BT
