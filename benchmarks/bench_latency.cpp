// Per-tick latency distribution under realistic conditions.
//
// bt_bench measures the average of back-to-back ticks of a small tree: the tree
// stays in L1/L2 and the number is a best case. A robot control loop instead
// ticks once per period while other code runs in between, so caches are cold
// and the tail latency matters as much as the median.
//
// Each scenario combines a tree with a condition:
//   hot        back-to-back ticks (same as bt_bench, for reference)
//   evict      before each tick, write a buffer (--evict-mb) to evict caches
//   periodic   sleep --period-us before each tick, like a control loop
//   frag       like evict, but the tree is built in a fragmented heap
// Only the tick itself is timed. Every scenario runs --rounds times (fresh tree
// each time); the reported value of each statistic is the median across rounds
// and *_spread_pct shows how much p50 / p99 varied between rounds.
//
// Usage: bt_latency [--json=FILE] [--csv-dir=DIR] [--samples=N] [--rounds=N]
//                   [--evict-mb=N] [--period-us=N] [--filter=SUBSTR] [--no-hist]
#include "bench_common.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include <unistd.h>

using namespace bench;
using Clock = std::chrono::steady_clock;

namespace
{
struct Options
{
  std::string json_path;
  std::string csv_dir;
  std::string filter;
  int samples = 2000;
  int rounds = 3;
  size_t evict_mb = 32;
  int period_us = 1000;
  bool histogram = true;
};

enum class Mode
{
  Hot,
  Evict,
  Periodic,
  Fragmented
};

const char* modeName(Mode m)
{
  switch(m)
  {
    case Mode::Hot:
      return "hot";
    case Mode::Evict:
      return "evict";
    case Mode::Periodic:
      return "periodic";
    case Mode::Fragmented:
      return "frag";
  }
  return "";
}

struct TreeSpec
{
  std::string name;
  std::string xml;
  std::function<void(BT::Blackboard&)> setup;
};

// Writes one byte per cache line of a large buffer: evicts the tree from caches
// smaller than the buffer (and dirties the lines, so they must be written back).
class CacheEvictor
{
public:
  explicit CacheEvictor(size_t bytes) : buffer_(bytes, 1)
  {}
  void run()
  {
    for(size_t i = 0; i < buffer_.size(); i += 64)
    {
      buffer_[i]++;
    }
    sink_ = sink_ + buffer_[buffer_.size() / 2];
  }

private:
  std::vector<unsigned char> buffer_;
  volatile unsigned sink_ = 0;
};

// Simulates a long-running process: many live allocations of random size with
// holes in between, so the tree nodes are scattered instead of contiguous.
class HeapFragmenter
{
public:
  HeapFragmenter()
  {
    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> size(16, 1024);
    std::vector<std::unique_ptr<char[]>> all;
    all.reserve(200000);
    for(int i = 0; i < 200000; i++)
    {
      all.emplace_back(new char[size(rng)]);
    }
    // keep every other block alive
    for(size_t i = 0; i < all.size(); i += 2)
    {
      kept_.push_back(std::move(all[i]));
    }
  }

private:
  std::vector<std::unique_ptr<char[]>> kept_;
};

struct Stats
{
  double p50, p90, p99, p999, max, mean;
};

Stats computeStats(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  auto pct = [&](double p) {
    const size_t idx = std::min(v.size() - 1, size_t(std::ceil(p * v.size())) - 1);
    return v[idx];
  };
  double sum = 0;
  for(double x : v)
  {
    sum += x;
  }
  return { pct(0.50), pct(0.90), pct(0.99), pct(0.999), v.back(), sum / v.size() };
}

double median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return n % 2 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

double spreadPct(const std::vector<double>& v)
{
  const auto [mn, mx] = std::minmax_element(v.begin(), v.end());
  return (*mx - *mn) / median(v) * 100.0;
}

double timerOverheadNs()
{
  std::vector<double> d(20000);
  for(auto& x : d)
  {
    const auto t1 = Clock::now();
    const auto t2 = Clock::now();
    x = std::chrono::duration<double, std::nano>(t2 - t1).count();
  }
  return median(d);
}

std::string formatNs(double ns)
{
  char buf[32];
  if(ns >= 1e6)
  {
    std::snprintf(buf, sizeof(buf), "%.2f ms", ns / 1e6);
  }
  else if(ns >= 1e3)
  {
    std::snprintf(buf, sizeof(buf), "%.2f us", ns / 1e3);
  }
  else
  {
    std::snprintf(buf, sizeof(buf), "%.0f ns", ns);
  }
  return buf;
}

// Log-scale histogram, 4 buckets per octave.
void printHistogram(const std::vector<double>& samples)
{
  const double step = std::pow(2.0, 0.25);
  const auto [mn, mx] = std::minmax_element(samples.begin(), samples.end());
  const int first = int(std::floor(std::log(*mn) / std::log(step)));
  const int last = int(std::floor(std::log(*mx) / std::log(step)));
  std::vector<size_t> counts(last - first + 1, 0);
  for(double s : samples)
  {
    counts[int(std::floor(std::log(s) / std::log(step))) - first]++;
  }
  const size_t peak = *std::max_element(counts.begin(), counts.end());
  for(size_t i = 0; i < counts.size(); i++)
  {
    if(counts[i] == 0)
    {
      continue;
    }
    const double lo = std::pow(step, first + int(i));
    const int bar = std::max<int>(1, int(50.0 * counts[i] / peak));
    std::printf("    %10s | %-50s %6.2f%%\n", formatNs(lo).c_str(),
                std::string(bar, '#').c_str(), 100.0 * counts[i] / samples.size());
  }
}

struct Result
{
  std::string name;
  std::vector<std::pair<std::string, double>> metrics;
};

std::vector<double> runRound(const TreeSpec& spec, Mode mode, const Options& opt,
                             CacheEvictor& evictor, std::string& error)
{
  std::unique_ptr<HeapFragmenter> fragmenter;
  if(mode == Mode::Fragmented)
  {
    fragmenter = std::make_unique<HeapFragmenter>();
  }
  BT::BehaviorTreeFactory factory;
  registerBenchNodes(factory);
  auto bb = BT::Blackboard::create();
  spec.setup(*bb);
  auto tree = factory.createTreeFromText(spec.xml, bb);

  if(tree.tickExactlyOnce() != BT::NodeStatus::SUCCESS)
  {
    error = "tree did not return SUCCESS";
    return {};
  }
  if(mode == Mode::Hot)
  {
    for(int i = 0; i < 200; i++)
    {
      tree.tickExactlyOnce();
    }
  }

  std::vector<double> samples;
  samples.reserve(opt.samples);
  for(int i = 0; i < opt.samples; i++)
  {
    if(mode == Mode::Evict || mode == Mode::Fragmented)
    {
      evictor.run();
    }
    else if(mode == Mode::Periodic)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(opt.period_us));
    }
    const auto t1 = Clock::now();
    const auto status = tree.tickExactlyOnce();
    const auto t2 = Clock::now();
    if(status != BT::NodeStatus::SUCCESS)
    {
      error = "tree did not return SUCCESS";
      return {};
    }
    samples.push_back(std::chrono::duration<double, std::nano>(t2 - t1).count());
  }
  return samples;
}

void writeJson(const std::string& path, const std::vector<Result>& results,
               const std::vector<std::pair<std::string, std::string>>& context)
{
  std::ofstream out(path);
  out << "{\n  \"context\": {";
  for(size_t i = 0; i < context.size(); i++)
  {
    out << (i ? ", " : "") << "\"" << context[i].first << "\": \"" << context[i].second
        << "\"";
  }
  out << "},\n  \"results\": [\n";
  for(size_t i = 0; i < results.size(); i++)
  {
    out << "    {\"name\": \"" << results[i].name << "\"";
    for(const auto& [k, v] : results[i].metrics)
    {
      out << ", \"" << k << "\": " << v;
    }
    out << "}" << (i + 1 < results.size() ? "," : "") << "\n";
  }
  out << "  ]\n}\n";
}

Options parseArgs(int argc, char** argv)
{
  Options opt;
  for(int i = 1; i < argc; i++)
  {
    const std::string a = argv[i];
    auto value = [&](const char* prefix) -> const char* {
      const size_t n = std::strlen(prefix);
      return a.compare(0, n, prefix) == 0 ? argv[i] + n : nullptr;
    };
    if(auto v = value("--json="))
      opt.json_path = v;
    else if(auto v = value("--csv-dir="))
      opt.csv_dir = v;
    else if(auto v = value("--filter="))
      opt.filter = v;
    else if(auto v = value("--samples="))
      opt.samples = std::max(1, std::atoi(v));
    else if(auto v = value("--rounds="))
      opt.rounds = std::max(1, std::atoi(v));
    else if(auto v = value("--evict-mb="))
      opt.evict_mb = std::max(1, std::atoi(v));
    else if(auto v = value("--period-us="))
      opt.period_us = std::max(0, std::atoi(v));
    else if(a == "--no-hist")
      opt.histogram = false;
    else
    {
      std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
      std::exit(1);
    }
  }
  return opt;
}
}  // namespace

int main(int argc, char** argv)
{
  const Options opt = parseArgs(argc, argv);

  const std::vector<TreeSpec> trees = {
    { "Seq100", repeatLeaf("Sequence", "<Ok/>", 100), [](BT::Blackboard&) {} },
    { "Ports100", repeatLeaf("Sequence", "<ReadInt value=\"{x}\"/>", 100),
      [](BT::Blackboard& bb) { bb.set("x", 1); } },
    { "Robot2k", robotLikeTree(100), setupRobotBlackboard },
  };
  const Mode modes[] = { Mode::Hot, Mode::Evict, Mode::Periodic, Mode::Fragmented };

  const double timer_ns = timerOverheadNs();
  const std::vector<std::pair<std::string, std::string>> context = {
    { "samples", std::to_string(opt.samples) },
    { "rounds", std::to_string(opt.rounds) },
    { "evict_mb", std::to_string(opt.evict_mb) },
    { "period_us", std::to_string(opt.period_us) },
    { "timer_overhead_ns", std::to_string(timer_ns) },
    { "l2_bytes", std::to_string(sysconf(_SC_LEVEL2_CACHE_SIZE)) },
    { "l3_bytes", std::to_string(sysconf(_SC_LEVEL3_CACHE_SIZE)) },
  };
  std::printf("samples/round=%d rounds=%d evict=%zuMB period=%dus timer_overhead=%.0fns\n"
              "(the timer overhead is included in every sample)\n",
              opt.samples, opt.rounds, opt.evict_mb, opt.period_us, timer_ns);

  CacheEvictor evictor(opt.evict_mb << 20);
  std::vector<Result> results;
  int failures = 0;

  for(const auto& tree : trees)
  {
    for(const Mode mode : modes)
    {
      const std::string name = tree.name + "/" + modeName(mode);
      if(!opt.filter.empty() && name.find(opt.filter) == std::string::npos)
      {
        continue;
      }
      std::vector<Stats> per_round;
      std::vector<double> all;
      std::string error;
      for(int r = 0; r < opt.rounds && error.empty(); r++)
      {
        auto samples = runRound(tree, mode, opt, evictor, error);
        if(!error.empty())
        {
          break;
        }
        per_round.push_back(computeStats(samples));
        all.insert(all.end(), samples.begin(), samples.end());
      }
      if(!error.empty())
      {
        std::printf("\n%s: ERROR %s\n", name.c_str(), error.c_str());
        failures++;
        continue;
      }

      auto med = [&](double Stats::*field) {
        std::vector<double> v;
        for(const auto& s : per_round)
        {
          v.push_back(s.*field);
        }
        return v;
      };
      const Result res{ name,
                        { { "p50_ns", median(med(&Stats::p50)) },
                          { "p90_ns", median(med(&Stats::p90)) },
                          { "p99_ns", median(med(&Stats::p99)) },
                          { "p999_ns", median(med(&Stats::p999)) },
                          { "max_ns", median(med(&Stats::max)) },
                          { "mean_ns", median(med(&Stats::mean)) },
                          { "p50_spread_pct", spreadPct(med(&Stats::p50)) },
                          { "p99_spread_pct", spreadPct(med(&Stats::p99)) } } };
      results.push_back(res);

      const auto& m = res.metrics;
      std::printf("\n%-20s p50=%-10s p90=%-10s p99=%-10s p99.9=%-10s max=%-10s "
                  "(round spread p50 %.1f%%, p99 %.1f%%)\n",
                  name.c_str(), formatNs(m[0].second).c_str(),
                  formatNs(m[1].second).c_str(), formatNs(m[2].second).c_str(),
                  formatNs(m[3].second).c_str(), formatNs(m[4].second).c_str(),
                  m[6].second, m[7].second);
      if(opt.histogram)
      {
        printHistogram(all);
      }
      if(!opt.csv_dir.empty())
      {
        std::string file = name;
        std::replace(file.begin(), file.end(), '/', '_');
        std::ofstream csv(opt.csv_dir + "/" + file + ".csv");
        csv << "round,sample,ns\n";
        for(size_t i = 0; i < all.size(); i++)
        {
          csv << i / opt.samples << "," << i % opt.samples << "," << all[i] << "\n";
        }
      }
    }
  }

  if(!opt.json_path.empty())
  {
    writeJson(opt.json_path, results, context);
    std::printf("\nwrote %s\n", opt.json_path.c_str());
  }
  return failures == 0 ? 0 : 1;
}
