#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

extern "C" uint64_t fsim_micros() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
extern "C" int fsim_delay_us(uint64_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
  return 0;
}

int main() {
  auto mutex = xSemaphoreCreateMutex();
  assert(xQueuePeek(mutex, nullptr, 0) == pdTRUE);
  assert(xQueuePeek(mutex, nullptr, 0) == pdTRUE); // Peeking must not consume it.
  assert(xSemaphoreTake(mutex, 0) == pdTRUE);
  assert(xQueuePeek(mutex, nullptr, 0) == pdFALSE);
  assert(uxQueueMessagesWaiting(mutex) == 0);
  std::thread release([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    xSemaphoreGive(mutex);
  });
  assert(xQueuePeek(mutex, nullptr, 200) == pdTRUE);
  release.join();
  assert(uxQueueMessagesWaiting(mutex) == 1);
  assert(xSemaphoreTake(mutex, 0) == pdTRUE);
  assert(xQueuePeek(mutex, nullptr, 2) == pdFALSE);
  vSemaphoreDelete(mutex);

  auto recursive = xSemaphoreCreateRecursiveMutex();
  assert(xSemaphoreTakeRecursive(recursive, 0) == pdTRUE);
  assert(xSemaphoreTakeRecursive(recursive, 0) == pdTRUE);
  assert(xSemaphoreGiveRecursive(recursive) == pdTRUE);
  assert(xQueuePeek(recursive, nullptr, 0) == pdFALSE);
  assert(xSemaphoreGiveRecursive(recursive) == pdTRUE);
  assert(xQueuePeek(recursive, nullptr, 0) == pdTRUE);
  vSemaphoreDelete(recursive);

  auto queue = xQueueCreate(1, sizeof(int));
  int sent = 42, received = 0;
  assert(xQueuePeek(queue, &received, 0) == pdFALSE);
  assert(xQueueSend(queue, &sent, 0) == pdTRUE);
  assert(xQueuePeek(queue, &received, 0) == pdTRUE && received == sent);
  assert(uxQueueMessagesWaiting(queue) == 1);
  assert(xQueueReceive(queue, &received, 0) == pdTRUE && received == sent);
  vQueueDelete(queue);
  std::cout << "Mutex peek availability, non-consumption, wait/timeout, recursion and item queues passed.\n";
}
