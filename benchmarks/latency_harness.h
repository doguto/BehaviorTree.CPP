// Shared harness for the per-tick latency tools (bt_latency, bt_xml_latency).
//
// A scenario is a tree plus a condition applied before each timed tick:
//   hot        back-to-back ticks
//   evict      write a buffer (--evict-mb) to evict the tree from the caches
//   periodic   sleep --period-us, like a control loop
//   frag       like evict, but the tree is built after fragmenting the heap
// Only the tick itself is timed. Every scenario runs --rounds times (fresh tree
// each time); each statistic is the median across rounds and *_spread_pct shows
// how much p50 / p99 moved between rounds.
#pragma once

#include "behaviortree_cpp/bt_factory.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace bench
{
using Clock = std::chrono::steady_clock;

enum class Mode
{
  Hot,
  Evict,
  Periodic,
  Fragmented
};

inline const char* modeName(Mode m)
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

struct LatencyOptions
{
  std::string json_path;
  std::string csv_dir;
  std::string filter;
  int samples = 2000;
  int rounds = 3;
  size_t evict_mb = 32;
  int period_us = 1000;
  bool histogram = true;
  std::vector<Mode> modes = { Mode::Hot, Mode::Evict, Mode::Periodic, Mode::Fragmented };

  static constexpr const char* usage = "  --json=FILE --csv-dir=DIR --samples=N "
                                       "--rounds=N --evict-mb=N\n"
                                       "  --period-us=N --filter=SUBSTR "
                                       "--modes=hot,evict,periodic,frag --no-hist\n";

  // Returns false if `arg` is not one of the common options.
  bool parse(const std::string& arg)
  {
    auto value = [&](const char* prefix) -> const char* {
      const size_t n = std::strlen(prefix);
      return arg.compare(0, n, prefix) == 0 ? arg.c_str() + n : nullptr;
    };
    if(auto v = value("--json="))
      json_path = v;
    else if(auto v = value("--csv-dir="))
      csv_dir = v;
    else if(auto v = value("--filter="))
      filter = v;
    else if(auto v = value("--samples="))
      samples = std::max(1, std::atoi(v));
    else if(auto v = value("--rounds="))
      rounds = std::max(1, std::atoi(v));
    else if(auto v = value("--evict-mb="))
      evict_mb = std::max(1, std::atoi(v));
    else if(auto v = value("--period-us="))
      period_us = std::max(0, std::atoi(v));
    else if(auto v = value("--modes="))
    {
      modes.clear();
      for(const auto& m : BT::splitString(v, ','))
      {
        for(Mode mode : { Mode::Hot, Mode::Evict, Mode::Periodic, Mode::Fragmented })
        {
          if(m == modeName(mode))
          {
            modes.push_back(mode);
          }
        }
      }
    }
    else if(arg == "--no-hist")
      histogram = false;
    else
      return false;
    return true;
  }
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

inline Stats computeStats(std::vector<double> v)
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

inline double median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return n % 2 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

inline double spreadPct(const std::vector<double>& v)
{
  const auto [mn, mx] = std::minmax_element(v.begin(), v.end());
  return (*mx - *mn) / median(v) * 100.0;
}

inline double timerOverheadNs()
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

inline std::string formatNs(double ns)
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
inline void printHistogram(const std::vector<double>& samples)
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

// A tree ready to be ticked, plus whatever must outlive it (factory, threads...).
struct TreeInstance
{
  std::shared_ptr<void> keepalive;
  BT::Tree* tree = nullptr;
};

// Builds the tree of one round. Called after the heap was fragmented, if requested.
using TreeMaker = std::function<TreeInstance()>;

// Returns an error message if a tick returned a status that the scenario does not
// expect (for synthetic trees, anything but SUCCESS).
using StatusCheck = std::function<std::string(BT::NodeStatus)>;

class LatencyRunner
{
public:
  using Context = std::vector<std::pair<std::string, std::string>>;

  explicit LatencyRunner(LatencyOptions opt)
    : opt_(std::move(opt)), evictor_(opt_.evict_mb << 20), timer_ns_(timerOverheadNs())
  {
    context_ = {
      { "samples", std::to_string(opt_.samples) },
      { "rounds", std::to_string(opt_.rounds) },
      { "evict_mb", std::to_string(opt_.evict_mb) },
      { "period_us", std::to_string(opt_.period_us) },
      { "timer_overhead_ns", std::to_string(timer_ns_) },
      { "l2_bytes", std::to_string(sysconf(_SC_LEVEL2_CACHE_SIZE)) },
      { "l3_bytes", std::to_string(sysconf(_SC_LEVEL3_CACHE_SIZE)) },
    };
    std::printf("samples/round=%d rounds=%d evict=%zuMB period=%dus "
                "timer_overhead=%.0fns\n(the timer overhead is included in every "
                "sample)\n",
                opt_.samples, opt_.rounds, opt_.evict_mb, opt_.period_us, timer_ns_);
  }

  const LatencyOptions& options() const
  {
    return opt_;
  }

  void addContext(const std::string& key, const std::string& value)
  {
    context_.emplace_back(key, value);
  }

  // Runs `tree_name` under every selected mode.
  void run(const std::string& tree_name, const TreeMaker& make, const StatusCheck& check)
  {
    for(const Mode mode : opt_.modes)
    {
      const std::string name = tree_name + "/" + modeName(mode);
      if(!opt_.filter.empty() && name.find(opt_.filter) == std::string::npos)
      {
        continue;
      }
      runScenario(name, mode, make, check);
    }
  }

  // Writes the JSON file (if requested). Returns the process exit code.
  int finish()
  {
    if(!opt_.json_path.empty())
    {
      writeJson();
      std::printf("\nwrote %s\n", opt_.json_path.c_str());
    }
    return failures_ == 0 ? 0 : 1;
  }

private:
  struct Result
  {
    std::string name;
    std::vector<std::pair<std::string, double>> metrics;
  };

  void runScenario(const std::string& name, Mode mode, const TreeMaker& make,
                   const StatusCheck& check)
  {
    std::vector<Stats> per_round;
    std::vector<double> all;
    std::map<BT::NodeStatus, size_t> statuses;
    std::string error;
    for(int r = 0; r < opt_.rounds && error.empty(); r++)
    {
      auto samples = runRound(mode, make, check, statuses, error);
      if(error.empty())
      {
        per_round.push_back(computeStats(samples));
        all.insert(all.end(), samples.begin(), samples.end());
      }
    }
    if(!error.empty())
    {
      std::printf("\n%s: ERROR %s\n", name.c_str(), error.c_str());
      failures_++;
      return;
    }

    auto field = [&](double Stats::*f) {
      std::vector<double> v;
      for(const auto& s : per_round)
      {
        v.push_back(s.*f);
      }
      return v;
    };
    Result res{ name,
                { { "p50_ns", median(field(&Stats::p50)) },
                  { "p90_ns", median(field(&Stats::p90)) },
                  { "p99_ns", median(field(&Stats::p99)) },
                  { "p999_ns", median(field(&Stats::p999)) },
                  { "max_ns", median(field(&Stats::max)) },
                  { "mean_ns", median(field(&Stats::mean)) },
                  { "p50_spread_pct", spreadPct(field(&Stats::p50)) },
                  { "p99_spread_pct", spreadPct(field(&Stats::p99)) } } };

    const auto& m = res.metrics;
    std::printf("\n%-24s p50=%-10s p90=%-10s p99=%-10s p99.9=%-10s max=%-10s "
                "(round spread p50 %.1f%%, p99 %.1f%%)\n",
                name.c_str(), formatNs(m[0].second).c_str(),
                formatNs(m[1].second).c_str(), formatNs(m[2].second).c_str(),
                formatNs(m[3].second).c_str(), formatNs(m[4].second).c_str(), m[6].second,
                m[7].second);
    if(statuses.size() > 1 || statuses.count(BT::NodeStatus::SUCCESS) == 0)
    {
      std::printf("    tick results:");
      for(const auto& [status, count] : statuses)
      {
        std::printf(" %s=%.1f%%", BT::toStr(status).c_str(), 100.0 * count / all.size());
      }
      std::printf("\n");
    }
    if(opt_.histogram)
    {
      printHistogram(all);
    }
    if(!opt_.csv_dir.empty())
    {
      std::string file = name;
      std::replace(file.begin(), file.end(), '/', '_');
      std::ofstream csv(opt_.csv_dir + "/" + file + ".csv");
      csv << "round,sample,ns\n";
      for(size_t i = 0; i < all.size(); i++)
      {
        csv << i / opt_.samples << "," << i % opt_.samples << "," << all[i] << "\n";
      }
    }
    results_.push_back(std::move(res));
  }

  std::vector<double> runRound(Mode mode, const TreeMaker& make, const StatusCheck& check,
                               std::map<BT::NodeStatus, size_t>& statuses,
                               std::string& error)
  {
    std::unique_ptr<HeapFragmenter> fragmenter;
    if(mode == Mode::Fragmented)
    {
      fragmenter = std::make_unique<HeapFragmenter>();
    }
    try
    {
      return timeTicks(mode, make, check, statuses, error);
    }
    catch(const std::exception& ex)
    {
      error = std::string("exception: ") + ex.what();
      return {};
    }
  }

  std::vector<double> timeTicks(Mode mode, const TreeMaker& make,
                                const StatusCheck& check,
                                std::map<BT::NodeStatus, size_t>& statuses,
                                std::string& error)
  {
    TreeInstance instance = make();
    BT::Tree& tree = *instance.tree;

    // first tick (tree initialization) is not part of the statistics
    if(error = check(tree.tickExactlyOnce()); !error.empty())
    {
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
    samples.reserve(opt_.samples);
    for(int i = 0; i < opt_.samples; i++)
    {
      if(mode == Mode::Evict || mode == Mode::Fragmented)
      {
        evictor_.run();
      }
      else if(mode == Mode::Periodic)
      {
        std::this_thread::sleep_for(std::chrono::microseconds(opt_.period_us));
      }
      const auto t1 = Clock::now();
      const auto status = tree.tickExactlyOnce();
      const auto t2 = Clock::now();
      if(error = check(status); !error.empty())
      {
        return {};
      }
      statuses[status]++;
      samples.push_back(std::chrono::duration<double, std::nano>(t2 - t1).count());
    }
    return samples;
  }

  void writeJson() const
  {
    std::ofstream out(opt_.json_path);
    out << "{\n  \"context\": {";
    for(size_t i = 0; i < context_.size(); i++)
    {
      out << (i ? ", " : "") << "\"" << context_[i].first << "\": \""
          << context_[i].second << "\"";
    }
    out << "},\n  \"results\": [\n";
    for(size_t i = 0; i < results_.size(); i++)
    {
      out << "    {\"name\": \"" << results_[i].name << "\"";
      for(const auto& [k, v] : results_[i].metrics)
      {
        out << ", \"" << k << "\": " << v;
      }
      out << "}" << (i + 1 < results_.size() ? "," : "") << "\n";
    }
    out << "  ]\n}\n";
  }

  LatencyOptions opt_;
  CacheEvictor evictor_;
  double timer_ns_;
  Context context_;
  std::vector<Result> results_;
  int failures_ = 0;
};

}  // namespace bench
