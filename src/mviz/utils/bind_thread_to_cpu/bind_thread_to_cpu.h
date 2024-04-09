#ifndef BIND_THREAD_TO_CPU_H_
#define BIND_THREAD_TO_CPU_H_
#include <mutex>
#include <thread>
#include <vector>
namespace utils {
class BindThreadToCpu final {
  using ThreadVecPtr = std::vector<std::unique_ptr<std::thread>>;

 private:
  std::mutex mutex_;
  ThreadVecPtr thread_vec_ptr;
  std::vector<int> thread_vec_para;

 public:
  BindThreadToCpu(/* args */) = default;
  ~BindThreadToCpu();

 public:
  void BindThreadToCpuFunc(const int cpu_index);

 public:
  void start();
};
}  // namespace utils

#endif