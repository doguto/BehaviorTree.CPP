// SharedTimerQueue, and the Delay / Sleep nodes of a factory with
// useSharedTimerThread().
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_timer_queue.h"

#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using namespace BT;
using namespace std::chrono_literals;

namespace
{
// Waits until `pred` is true, at most 5 seconds.
template <typename Pred>
bool eventually(Pred pred)
{
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while(!pred())
  {
    if(std::chrono::steady_clock::now() > deadline)
    {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

#ifdef __linux__
int threadCount()
{
  std::ifstream status("/proc/self/status");
  std::string line;
  while(std::getline(status, line))
  {
    if(line.rfind("Threads:", 0) == 0)
    {
      return std::stoi(line.substr(8));
    }
  }
  return -1;
}
#endif

std::string manyDelays(int count, int msec)
{
  std::string xml = R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><Parallel>)";
  for(int i = 0; i < count; i++)
  {
    xml += "<Delay delay_msec=\"" + std::to_string(msec) + "\"><AlwaysSuccess/></Delay>";
  }
  xml += "</Parallel></BehaviorTree></root>";
  return xml;
}
}  // namespace

TEST(SharedTimerQueue, ExpiresInOrder)
{
  SharedTimerQueue queue;
  std::mutex mutex;
  std::vector<int> order;
  auto record = [&](int i) {
    return [&, i] {
      const std::scoped_lock lk(mutex);
      order.push_back(i);
    };
  };
  queue.add(30ms, record(3));
  queue.add(10ms, record(1));
  queue.add(20ms, record(2));
  ASSERT_TRUE(eventually([&] {
    const std::scoped_lock lk(mutex);
    return order.size() == 3;
  }));
  EXPECT_EQ(order, (std::vector<int>{ 1, 2, 3 }));
  EXPECT_EQ(queue.pending(), 0u);
}

TEST(SharedTimerQueue, CancelledHandlerIsNotInvoked)
{
  SharedTimerQueue queue;
  std::atomic_int calls = 0;
  const auto id = queue.add(20ms, [&] { calls++; });
  queue.add(40ms, [&] { calls += 10; });
  queue.cancel(id);
  queue.cancel(id);   // twice: ignored
  queue.cancel(0);    // never a valid id
  queue.cancel(999);  // unknown
  ASSERT_TRUE(eventually([&] { return calls != 0; }));
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(calls, 10);
}

TEST(SharedTimerQueue, CancelWaitsForRunningHandler)
{
  SharedTimerQueue queue;
  std::atomic_bool started = false;
  std::atomic_bool finished = false;
  const auto id = queue.add(0ms, [&] {
    started = true;
    std::this_thread::sleep_for(50ms);
    finished = true;
  });
  ASSERT_TRUE(eventually([&] { return started.load(); }));
  queue.cancel(id);
  EXPECT_TRUE(finished);
}

TEST(SharedTimerQueue, HandlerCanCancelItselfAndOthers)
{
  SharedTimerQueue queue;
  std::atomic_int calls = 0;
  std::atomic<uint64_t> self = 0;
  const auto other = queue.add(50ms, [&] { calls += 10; });
  std::atomic_bool done = false;
  self = queue.add(10ms, [&] {
    queue.cancel(self);  // must not wait for itself
    queue.cancel(other);
    calls++;
    done = true;
  });
  ASSERT_TRUE(eventually([&] { return done.load(); }));
  std::this_thread::sleep_for(60ms);
  EXPECT_EQ(calls, 1);
}

TEST(SharedTimerQueue, DestroyedWithPendingTimers)
{
  std::atomic_int calls = 0;
  {
    SharedTimerQueue queue;
    queue.add(10s, [&] { calls++; });
  }
  EXPECT_EQ(calls, 0);
}

TEST(SharedTimer, DelayAndSleep)
{
  BehaviorTreeFactory factory;
  factory.useSharedTimerThread();
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Sequence>
      <Delay delay_msec="30"><AlwaysSuccess/></Delay>
      <Sleep msec="30"/>
      <Delay delay_msec="10"><AlwaysSuccess/></Delay>
    </Sequence>
  </BehaviorTree>
</root>)");
  const auto t1 = std::chrono::steady_clock::now();
  EXPECT_EQ(tree.tickWhileRunning(1ms), NodeStatus::SUCCESS);
  EXPECT_GE(std::chrono::steady_clock::now() - t1, 70ms);

  // and again, after the tree went back to IDLE
  const auto t2 = std::chrono::steady_clock::now();
  EXPECT_EQ(tree.tickWhileRunning(1ms), NodeStatus::SUCCESS);
  EXPECT_GE(std::chrono::steady_clock::now() - t2, 70ms);
}

TEST(SharedTimer, WakesUpTickWhileRunning)
{
  // tickWhileRunning() sleeps until a node emits the wake-up signal: the shared
  // timer must emit it, otherwise each Delay would take the whole sleep time.
  BehaviorTreeFactory factory;
  factory.useSharedTimerThread();
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Sequence>
      <Delay delay_msec="20"><AlwaysSuccess/></Delay>
      <Sleep msec="20"/>
    </Sequence>
  </BehaviorTree>
</root>)");
  const auto t1 = std::chrono::steady_clock::now();
  EXPECT_EQ(tree.tickWhileRunning(5s), NodeStatus::SUCCESS);
  EXPECT_LT(std::chrono::steady_clock::now() - t1, 2s);
}

TEST(SharedTimer, HaltAndRestart)
{
  BehaviorTreeFactory factory;
  factory.useSharedTimerThread();
  auto tree = factory.createTreeFromText(R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Main">
    <Parallel success_count="2">
      <Delay delay_msec="20"><AlwaysSuccess/></Delay>
      <Sleep msec="20"/>
    </Parallel>
  </BehaviorTree>
</root>)");
  for(int i = 0; i < 20; i++)
  {
    // halted before (or around) the timer expiration...
    ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(i % 3 == 0 ? 20 : 1));
    tree.haltTree();
  }
  // ...a late callback of a cancelled timer must not complete the new one
  ASSERT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);
  std::this_thread::sleep_for(5ms);
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickWhileRunning(1ms), NodeStatus::SUCCESS);
}

TEST(SharedTimer, DestroyTreeWhileRunning)
{
  BehaviorTreeFactory factory;
  factory.useSharedTimerThread();
  for(int i = 0; i < 20; i++)
  {
    auto tree = factory.createTreeFromText(manyDelays(10, i % 3));
    (void)tree.tickExactlyOnce();
    if(i % 2 == 0)
    {
      std::this_thread::sleep_for(1ms);
    }
  }
}

#ifdef __linux__
TEST(SharedTimer, OneThreadForAllTheDelays)
{
  const int before = threadCount();
  {
    BehaviorTreeFactory factory;
    auto tree = factory.createTreeFromText(manyDelays(20, 10));
    // default: one thread per Delay
    EXPECT_GE(threadCount() - before, 20);
  }
  ASSERT_TRUE(eventually([&] { return threadCount() == before; }));
  {
    BehaviorTreeFactory factory;
    factory.useSharedTimerThread();
    auto tree1 = factory.createTreeFromText(manyDelays(20, 10));
    auto tree2 = factory.createTreeFromText(manyDelays(20, 10));
    EXPECT_EQ(threadCount() - before, 1);
    EXPECT_EQ(tree1.tickWhileRunning(1ms), NodeStatus::SUCCESS);
    EXPECT_EQ(tree2.tickWhileRunning(1ms), NodeStatus::SUCCESS);
  }
  // the shared thread stops with the last node using it
  EXPECT_EQ(threadCount(), before);
}
#endif
