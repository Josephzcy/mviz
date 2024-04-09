// #include "performance_analyzer/performance_analyzer.h"

// #include "performance_analyzer.h"
#include "performance_analyzer/performance_analyzer.h"

namespace utils::profile {

PerformanceAnalyzer::PerformanceAnalyzer() : stop_flag_(false) {
  this_instance_ = this;
  // keyboard_thread_ = std::thread(&PerformanceAnalyzer::keyboardListener, this);
}
PerformanceAnalyzer::~PerformanceAnalyzer() {
  if (keyboard_thread_.joinable()) {
    keyboard_thread_.join();
  }
}

void PerformanceAnalyzer::RegisterSignalHandler() {
  // 注册信号量处理程序
  std::signal(SIGINT, keyboardListener);
  std::cout << "Signal handler registered." << std::endl;
}

void PerformanceAnalyzer::WaitForStopSignal() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stop_flag_) {
    cond_.wait(lock);
    ProfilerStop();
    HeapProfilerStop();
    // std::exit(signal);
  }
}

void PerformanceAnalyzer::keyboardListener(int signal) {
  std::cout << "Received signal: " << signal << std::endl;
  this_instance_->stop_flag_ = true;
  this_instance_->cond_.notify_all();
  // std::exit(signal);
}

PerformanceAnalyzer::ProfilerPtr PerformanceAnalyzer::CPUProfileAnalyzer(const std::string& file_name) {
  ProfilerStart(file_name.c_str());
  ProfilerRegisterThread();
  return ProfilerPtr(nullptr, [](void*) { ProfilerStop(); });

  
};

PerformanceAnalyzer::ProfilerPtr PerformanceAnalyzer::MemoryProfileAnalysis(const std::string& file_name) {
  HeapProfilerStart(file_name.c_str());
  
  return ProfilerPtr(nullptr, [](void*) { HeapProfilerStop(); });
};

PerformanceAnalyzer::ProfilerPtr PerformanceAnalyzer::TimeProfileAnalysis(const std::string& file_name) {
  TimePoint start_time = Clock::now();
  return ProfilerPtr(nullptr, [&start_time](void*) {
    TimePoint end_time = Clock::now();
    Duration duration = end_time - start_time;
    std::cout << "Duration: " << duration.count() << " ms\n";
  });
}
PerformanceAnalyzer* PerformanceAnalyzer::this_instance_ = nullptr;  // 不要加static ，否则就是定义新变量

}  // namespace utils