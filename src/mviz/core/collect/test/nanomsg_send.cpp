#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

int main() {
  int sock = nn_socket(AF_SP, NN_PUB);
  if (sock < 0) {
    std::cerr << "Failed to create socket: " << nn_strerror(nn_errno()) << std::endl;
    return 1;
  }

  int bind = nn_bind(sock, "tcp://*:2080");
  if (bind < 0) {
    std::cerr << "Failed to bind: " << nn_strerror(nn_errno()) << std::endl;
    nn_close(sock);
    return 1;
  }



  int count = 0;
  int messagesPerSecond = 100000;
  int microsecondsPerMessage = 1000000 / messagesPerSecond;

  while (true) {
    auto start = std::chrono::high_resolution_clock::now();

    std::string msg = "Hello, subscribers:! " + std::to_string(count++);

    const char* message = msg.c_str();

    std::cout << "message:"
              << "Hello, subscribers:!" + std::to_string(count) << std::endl;


    int bytes = nn_send(sock, message, strlen(message) + 1, 0);

    if (bytes < 0) {
      std::cerr << "Failed to send topic: " << nn_strerror(nn_errno()) << std::endl;
      nn_close(sock);
      return 1;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    auto sleepTime = microsecondsPerMessage - duration.count();

    if (sleepTime > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleepTime));
    }
  }

  nn_close(sock);
  return 0;
}
