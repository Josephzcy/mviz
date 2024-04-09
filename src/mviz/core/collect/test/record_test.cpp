//
// Created by nick on 23-2-13.
//
#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include "glog/logging.h"
#include "mviz_collect_inf.h"
#include "planning.pb.h"
#include "vehicle_control.pb.h"

#ifdef USED_CTRL_STOP
#include <csignal>
#endif

int mode = 0;  //'0':hil ; '1':oncar
using uOnApaptFunc = std::function<int()>;
uOnApaptFunc mviz_collect_init_void = std::bind(mviz_collect_init, std::ref(mode));
typedef std::map<char, uOnApaptFunc> ZxMvizTestCmdFuncMap;
ZxMvizTestCmdFuncMap cmdFuncMap{
    {'i', mviz_collect_init_void}, {'s', mviz_collect_start}, {'p', mviz_collect_stop}, {'r', mviz_collect_release}};

class Test1 {
 public:
  Test1() = default;
  ~Test1() = default;
  uint64_t timestamp() {
    LOG(INFO) << "test1 timestamp";
    return 11;
  }
  void ParseFromString(std::string &) { LOG(INFO) << "test1 ParseFromString"; }
  uint64_t tick() {
    LOG(INFO) << "test1 tick";
    return 12;
  }
};

class Test2 {
 public:
  Test2() = default;
  ~Test2() = default;
  uint64_t timestamp() {
    LOG(INFO) << "test2 timestamp";
    return 21;
  }
  void ParseFromString(std::string &) { LOG(INFO) << "test2 ParseFromString"; }
  uint64_t tick() {
    LOG(INFO) << "test2 tick";
    return 22;
  }
};

class TestA {
 public:
  void m_print() { std::cout << "TEST .....A" << std::endl; }
};

#ifdef USED_CTRL_STOP
static bool kStopMain = false;

void SingleHandler(int sig) {
  std::cout << ("\nleaving args_main\n") << std::endl;
  kStopMain = true;
}
#endif

int main(int argc, char *argv[]) {
  int ret = 0;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_stderrthreshold = 0;

  if (argc <= 1) {
    LOG(INFO) << "Please input a number: 0 : hil, 1 : oncar";
    return -1;
  } else if ((1 == std::string(argv[1]).size()) && (argv[1][0] >= '0' && argv[1][0] <= '1')) {
    mode = argv[1][0] - '0';
    std::cout << "mode=" << mode;
  } else {
    LOG(INFO) << "Please input right parameter!";
    return -1;
  }

#ifdef USED_CTRL_STOP
  signal(SIGINT, SingleHandler);
  signal(SIGTERM, SingleHandler);
#endif

  // minieye::Planning* ptrObj = (minieye::Planning*)ClassFactory::getInstance().getClassByName("TestA");
  // ptrObj->set_tick(12345677);
  // Test1* ptrObj2 = (Test1*)ClassFactory::getInstance().getClassByName("Test5");
  // ptrObj2->timestamp();
  // Test2* ptrObj3 = (Test2*)ClassFactory::getInstance().getClassByName("Test2");
  // ptrObj3->timestamp();
  /*std::map<std::string , PTRCreateObject>::iterator iter;
  for (iter = ClassFactory::getInstance().get_map().begin();
       iter != ClassFactory::getInstance().get_map().end();
       iter++) {
    std::cout << iter->first << std::endl;
    Test1* ptrObj2 = (Test1*)iter->second();
    std::cout << ptrObj2->timestamp()<< std::endl;

  }
  */

  char input_key;
#if 1
  while (std::cout << "input cmd: i|s|p|r\n", std::cin >> input_key) {
#ifdef USED_CTRL_STOP
    if (kStopMain) {
      return 0;
    }
#endif
    std::cout << input_key << std::endl;
    if (input_key == 'e') break;

    if (cmdFuncMap.find(input_key) != cmdFuncMap.end()) ret = cmdFuncMap[input_key]();
  }
  LOG(INFO) << "test was finished" << std::endl;
#endif

  return 0;
}