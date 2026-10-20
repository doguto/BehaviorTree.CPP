#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>

namespace BT
{

/**
 * @brief A single thread running the timers of many nodes
 * (see BehaviorTreeFactory::useSharedTimerThread()).
 *
 * Differences with TimerQueue, which starts one thread per instance:
 * - a handler is invoked only when its timer expires, never when it is
 *   cancelled;
 * - cancel() returns once the handler can't be running anymore, so a node can
 *   cancel its timer in its destructor and be destroyed safely.
 *
 * Handlers run one at a time on the timer thread: they must be short and must
 * not wait for other timers.
 */
class SharedTimerQueue
{
public:
  using Clock = std::chrono::steady_clock;

  SharedTimerQueue() : thread_([this] { run(); })
  {}

  /// Pending timers are dropped. Must not be called by a handler.
  ~SharedTimerQueue()
  {
    {
      const std::scoped_lock lk(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    thread_.join();
  }

  SharedTimerQueue(const SharedTimerQueue&) = delete;
  SharedTimerQueue& operator=(const SharedTimerQueue&) = delete;
  SharedTimerQueue(SharedTimerQueue&&) = delete;
  SharedTimerQueue& operator=(SharedTimerQueue&&) = delete;

  /// Invoke `handler` in the timer thread after `delay`. Returns its id (never 0).
  uint64_t add(std::chrono::milliseconds delay, std::function<void()> handler)
  {
    const auto deadline = Clock::now() + delay;
    bool earliest = false;
    uint64_t id = 0;
    {
      const std::scoped_lock lk(mutex_);
      id = ++last_id_;
      earliest = queue_.empty() || deadline < queue_.begin()->first.first;
      queue_.emplace(Key{ deadline, id }, std::move(handler));
      deadlines_.emplace(id, deadline);
    }
    if(earliest)
    {
      cv_.notify_one();
    }
    return id;
  }

  /// After this call, the handler of `id` is not running and won't be invoked
  /// (if a handler cancels itself, it is still running when this returns).
  /// Unknown or already expired ids are ignored.
  void cancel(uint64_t id)
  {
    if(id == 0)
    {
      return;
    }
    std::unique_lock lk(mutex_);
    if(auto it = deadlines_.find(id); it != deadlines_.end())
    {
      queue_.erase(Key{ it->second, id });
      deadlines_.erase(it);
      return;
    }
    if(std::this_thread::get_id() != thread_.get_id())
    {
      done_cv_.wait(lk, [&] { return running_ != id; });
    }
  }

  /// Number of timers not expired nor cancelled yet.
  [[nodiscard]] size_t pending() const
  {
    const std::scoped_lock lk(mutex_);
    return queue_.size();
  }

private:
  using Key = std::pair<Clock::time_point, uint64_t>;

  void run()
  {
    std::unique_lock lk(mutex_);
    while(!stop_)
    {
      if(queue_.empty())
      {
        cv_.wait(lk);
        continue;
      }
      auto it = queue_.begin();
      if(Clock::now() < it->first.first)
      {
        cv_.wait_until(lk, it->first.first);
        continue;
      }
      running_ = it->first.second;
      std::function<void()> handler = std::move(it->second);
      deadlines_.erase(running_);
      queue_.erase(it);

      lk.unlock();
      handler();
      handler = nullptr;
      lk.lock();

      running_ = 0;
      done_cv_.notify_all();
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable cv_;       // new earliest timer, or stop
  std::condition_variable done_cv_;  // a handler returned
  std::map<Key, std::function<void()>> queue_;
  std::unordered_map<uint64_t, Clock::time_point> deadlines_;
  uint64_t last_id_ = 0;
  uint64_t running_ = 0;
  bool stop_ = false;
  // last: the thread starts once all the other members are constructed
  std::thread thread_;
};

}  // namespace BT
