#include "utils.h"
#include <pthread.h>
#include <signal.h>
#include <thread>
namespace concurrent_data_structures {

void sleep_ignore_signals(const std::chrono::seconds &duration) {
  auto now = std::chrono::high_resolution_clock::now();
  const auto sleep_to = now + duration;
  while (now < sleep_to) {
    std::this_thread::sleep_until(sleep_to);
    now = std::chrono::high_resolution_clock::now();
  }
}
}
