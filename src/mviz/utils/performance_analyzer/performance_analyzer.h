#ifndef PERFORMANCE_ANALYZER_H_
#define PERFORMANCE_ANALYZER_H_

#include <gperftools/heap-profiler.h>
#include <gperftools/profiler.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
namespace utils::profile {
// 这个类封装了性能分析的功能
class PerformanceAnalyzer {
  // 使用智能指针和Lambda表达式定义类型别名
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Duration = std::chrono::duration<double, std::milli>;
  using Callback = std::function<void()>;
  using ProfilerPtr = std::shared_ptr<void>;
  using ThisPtr = std::shared_ptr<PerformanceAnalyzer>;

 private:
  TimePoint start_time_;
  std::thread keyboard_thread_;
  std::mutex mutex_;

  static PerformanceAnalyzer* this_instance_;  // 为了静态函数能够调用成员函数

 public:
  PerformanceAnalyzer();

  ~PerformanceAnalyzer();

  void RegisterSignalHandler();

  static void keyboardListener(int signal);
  std::condition_variable cond_;
  bool stop_flag_;

  void WaitForStopSignal();
  static ProfilerPtr CPUProfileAnalyzer(const std::string& file_name);


  static ProfilerPtr MemoryProfileAnalysis(const std::string& file_name);
  static ProfilerPtr TimeProfileAnalysis(const std::string& file_name);

  // 测量函数执行时间
  template <typename F, typename... Args>
  void MeasureTime(F&& func, Args&&... args) {
    start_time_ = Clock::now();
    std::invoke(std::forward<F>(func), std::forward<Args>(args)...);
    TimePoint end_time = Clock::now();
    Duration duration = end_time - start_time_;
    std::cout << "Duration: " << duration.count() << " ms\n";
  }

  // auto ProfilerPtr make_cpu_profiler = [](const std::string& filename) {
  //   ProfilerStart(filename.c_str());
  //   ProfilerRegisterThread();
  //   return std::shared_ptr<void>(nullptr, [](void*) { ProfilerStop(); });
  // };
};  // namespace utils::profileclassPerformanceAnalyzer

}  // namespace utils::profile
#endif