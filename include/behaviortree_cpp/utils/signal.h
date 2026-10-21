#ifndef SIMPLE_SIGNAL_H
#define SIMPLE_SIGNAL_H

#include <functional>
#include <memory>
#include <vector>

namespace BT
{
/**
 * Super simple Signal/Slop implementation, AKA "Observable pattern".
 * The subscriber is active until it goes out of scope or Subscriber::reset() is called.
 */
template <typename... CallableArgs>
class Signal
{
public:
  using CallableFunction = std::function<void(CallableArgs...)>;
  using Subscriber = std::shared_ptr<CallableFunction>;

  void notify(CallableArgs... args)
  {
    for(size_t i = 0; i < subscribers_.size();)
    {
      if(auto sub = subscribers_[i].lock())
      {
        (*sub)(args...);
        i++;
      }
      else
      {
        subscribers_.erase(subscribers_.begin() + i);
      }
    }
  }

  /// True if nobody subscribed. Expired subscribers count until the next notify().
  [[nodiscard]] bool empty() const
  {
    return subscribers_.empty();
  }

  Subscriber subscribe(CallableFunction func)
  {
    Subscriber sub = std::make_shared<CallableFunction>(std::move(func));
    subscribers_.emplace_back(sub);
    return sub;
  }

private:
  std::vector<std::weak_ptr<CallableFunction>> subscribers_;
};
}  // namespace BT

#endif  // SIMPLE_SIGNAL_H
