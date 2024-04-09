#include "bind_thread_to_cpu/bind_thread_to_cpu.h"

#include <pthread.h>
#include <sched.h>

#include <iostream>
namespace utils {
BindThreadToCpu::~BindThreadToCpu() {
  for (const auto& pThread : thread_vec_ptr) {
    pThread->joinable();
  }
}
void BindThreadToCpu::BindThreadToCpuFunc(const int cpu_core_index) {
  // todo::需要确保cpu_core_index 是有效的
  std::cout << "[Info:] thread start : " << cpu_core_index << std::endl;
  std::cout << "[Info:] thread id : " << thread_vec_ptr[cpu_core_index]->get_id() << std::endl;

  // 绑定对应的核，执行相应的线程
  std::unique_lock<std::mutex> lock(mutex_);
  pthread_t native_handle = pthread_self();
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  CPU_SET(cpu_core_index, &cpu_set);

  int ret = pthread_setaffinity_np(native_handle, sizeof(cpu_set), &cpu_set);

  if (ret != 0) {
    // 处理绑定失败的情况
    std::cout << "[Info:] cpu_core_index  " << cpu_core_index << std::endl;
    std::cerr << "Failed to bind thread to CPU core:" << cpu_core_index << std::endl;
    // ...
    return;
  }
  lock.unlock();
  while (true) {  // todo::full_cpu 线程
    // std::cout << "[Info:] cpu_core_index  " << cpu_core_index << std::endl;
    static thread_local int count = 0;
    count++;
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void BindThreadToCpu::start() {
  // 获取硬件支持的cpu线程数,按需要启动线程
  int thread_num = std::thread::hardware_concurrency();
  std::cout << "[Info:] thread_num:  " << thread_num << std::endl;

  // question1 :由于线程可能在循环结束之前开始执行，循环变量i的值可能已经被修改，导致线程访问到无效的值
  // question2 :本次循环结束，但是本次循环的线程还未完全起来，导致引用失效，所以这里最好拷贝
  for (int i = 0; i < thread_num; i++) {
    thread_vec_ptr.emplace_back(new std::thread(&BindThreadToCpu::BindThreadToCpuFunc, this, i));
    if (i ==2) break;
  }

  for (const auto& pThread : thread_vec_ptr) {
    pThread->join();
  }
}

}  // namespace utils
