#include <iostream>

#include "pystring.h"

using namespace std;
using namespace pystring;

void test() {
  std::string path = "~/root/home/test.txt";
  std::string path1 = "test";
  cout << "basename:" << os::path::basename(path) << std::endl;      // test.txt
  cout << "dirname:" << os::path::dirname(path) << std::endl;        // ~/root/home
  cout << "join:" << os::path::join(path, path1) << std::endl;       // ~/root/home/test.txt/test
  cout << "join:" << os::path::join("path1", "path2") << std::endl;  // path1/path2
}

int main() {
  test();
  return 0;
}