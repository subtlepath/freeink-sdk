// FreeInk simulator — FreeRTOS port on host threads.
//
// Tasks are std::threads; queues, semaphores and event groups are ordinary
// host primitives. The one thing that is deliberately *not* ordinary is time:
// every blocking wait is measured against the simulated clock through
// fsim_delay_us(), so pausing or stepping the machine freezes firmware tasks
// along with the main loop. A background task that kept running while the
// machine was paused would change the screen under a capture.

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <atomic>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint64_t kWaitSliceUs = 100;

// Sleeps in slices on the simulated clock until `pred` holds or the simulated
// deadline passes. `mutex` is released across each slice so another task can
// make the predicate true.
template <typename Pred>
bool waitFor(std::mutex& mutex, TickType_t ticks_to_wait, Pred pred) {
  std::unique_lock<std::mutex> lock(mutex);
  if (pred()) return true;
  if (ticks_to_wait == 0) return false;
  const bool forever = ticks_to_wait == portMAX_DELAY;
  const uint64_t deadline = fsim_micros() + static_cast<uint64_t>(ticks_to_wait) * 1000ULL;
  while (!pred()) {
    if (!forever && fsim_micros() >= deadline) return false;
    lock.unlock();
    if (fsim_delay_us(kWaitSliceUs) != 0) return false;  // machine tearing down
    lock.lock();
  }
  return true;
}

struct Task {
  std::thread thread;
  std::string name;
  std::atomic<bool> stop{false};
  std::atomic<uint32_t> notifications{0};
  UBaseType_t priority = 1;
};

struct Queue {
  std::mutex mutex;
  std::deque<std::vector<uint8_t>> items;
  size_t capacity = 0;
  size_t itemSize = 0;
};

struct Semaphore {
  std::mutex mutex;
  int64_t count = 0;
  int64_t maxCount = 1;
  bool recursive = false;
  std::thread::id owner{};
  int64_t recursionDepth = 0;
};

struct EventGroup {
  std::mutex mutex;
  EventBits_t bits = 0;
};

// A task that deletes itself unwinds out of its thread function by throwing
// this, so destructors run rather than the thread being torn down mid-frame.
struct TaskExit {};

thread_local Task* t_currentTask = nullptr;

}  // namespace

// ── Tasks ────────────────────────────────────────────────────────────────────
extern "C" BaseType_t xTaskCreate(TaskFunction_t fn, const char* name, uint32_t, void* arg, UBaseType_t priority,
                                  TaskHandle_t* out) {
  if (!fn) return pdFAIL;
  auto* task = new Task();
  task->name = name ? name : "task";
  task->priority = priority;
  task->thread = std::thread([task, fn, arg]() {
    t_currentTask = task;
    try {
      fn(arg);
    } catch (const TaskExit&) {
      // vTaskDelete(NULL) from inside the task.
    }
  });
  if (out) *out = task;
  return pdPASS;
}

extern "C" BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name, uint32_t stack, void* arg,
                                              UBaseType_t priority, TaskHandle_t* out, BaseType_t) {
  return xTaskCreate(fn, name, stack, arg, priority, out);
}

// Static creation still allocates here: the caller's stack and TCB buffers have
// no meaning for a host thread. MemoryManager's pool accounting already ran by
// the time it calls this, so its PSRAM/internal split is still reported.
extern "C" TaskHandle_t xTaskCreateStatic(TaskFunction_t fn, const char* name, uint32_t stack, void* arg,
                                          UBaseType_t priority, StackType_t*, StaticTask_t*) {
  TaskHandle_t handle = nullptr;
  return xTaskCreate(fn, name, stack, arg, priority, &handle) == pdPASS ? handle : nullptr;
}

extern "C" TaskHandle_t xTaskCreateStaticPinnedToCore(TaskFunction_t fn, const char* name, uint32_t stack, void* arg,
                                                      UBaseType_t priority, StackType_t* stackBuf, StaticTask_t* tcb,
                                                      BaseType_t) {
  return xTaskCreateStatic(fn, name, stack, arg, priority, stackBuf, tcb);
}

extern "C" void vTaskDelete(TaskHandle_t handle) {
  auto* task = static_cast<Task*>(handle);
  if (!task || task == t_currentTask) {
    if (t_currentTask) t_currentTask->stop = true;
    throw TaskExit{};
  }
  task->stop = true;
  if (task->thread.joinable()) task->thread.join();
  delete task;
}

extern "C" void vTaskDelay(TickType_t ticks) { fsim_delay_us(static_cast<uint64_t>(ticks) * 1000ULL); }

extern "C" void vTaskDelayUntil(TickType_t* previous_wake, TickType_t increment) {
  if (!previous_wake) return;
  const TickType_t target = *previous_wake + increment;
  const uint64_t nowMs = fsim_micros() / 1000ULL;
  if (target > nowMs) fsim_delay_us((target - nowMs) * 1000ULL);
  *previous_wake = target;
}

extern "C" TickType_t xTaskGetTickCount(void) { return static_cast<TickType_t>(fsim_micros() / 1000ULL); }
extern "C" TaskHandle_t xTaskGetCurrentTaskHandle(void) { return t_currentTask; }
extern "C" const char* pcTaskGetName(TaskHandle_t handle) {
  auto* task = static_cast<Task*>(handle ? handle : t_currentTask);
  return task ? task->name.c_str() : "main";
}
// Host threads have no measurable FreeRTOS stack; report the nominal headroom
// rather than a zero that would read as "about to overflow".
extern "C" UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t) { return 2048; }
extern "C" UBaseType_t uxTaskPriorityGet(TaskHandle_t handle) {
  auto* task = static_cast<Task*>(handle ? handle : t_currentTask);
  return task ? task->priority : 1;
}
extern "C" void vTaskPrioritySet(TaskHandle_t handle, UBaseType_t priority) {
  auto* task = static_cast<Task*>(handle ? handle : t_currentTask);
  if (task) task->priority = priority;
}
extern "C" void vTaskSuspend(TaskHandle_t) {}
extern "C" void vTaskResume(TaskHandle_t) {}

extern "C" BaseType_t xTaskNotifyGive(TaskHandle_t handle) {
  auto* task = static_cast<Task*>(handle);
  if (!task) return pdFAIL;
  ++task->notifications;
  return pdPASS;
}
extern "C" void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t* woken) {
  xTaskNotifyGive(handle);
  if (woken) *woken = pdFALSE;
}
extern "C" uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t ticks_to_wait) {
  Task* task = t_currentTask;
  if (!task) return 0;
  const bool forever = ticks_to_wait == portMAX_DELAY;
  const uint64_t deadline = fsim_micros() + static_cast<uint64_t>(ticks_to_wait) * 1000ULL;
  while (task->notifications.load() == 0) {
    if (ticks_to_wait == 0 || (!forever && fsim_micros() >= deadline)) return 0;
    if (fsim_delay_us(kWaitSliceUs) != 0) return 0;
  }
  if (clear_on_exit) {
    const uint32_t v = task->notifications.exchange(0);
    return v;
  }
  return task->notifications--;
}

// ── Queues ───────────────────────────────────────────────────────────────────
extern "C" QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size) {
  auto* q = new Queue();
  q->capacity = length;
  q->itemSize = item_size;
  return q;
}
extern "C" QueueHandle_t xQueueCreateStatic(UBaseType_t length, UBaseType_t item_size, uint8_t*, StaticQueue_t*) {
  return xQueueCreate(length, item_size);
}
extern "C" void vQueueDelete(QueueHandle_t handle) { delete static_cast<Queue*>(handle); }

namespace {
BaseType_t queueSend(QueueHandle_t handle, const void* item, TickType_t ticks_to_wait, bool front) {
  auto* q = static_cast<Queue*>(handle);
  if (!q || !item) return errQUEUE_FULL;
  if (!waitFor(q->mutex, ticks_to_wait, [q] { return q->items.size() < q->capacity; })) return errQUEUE_FULL;
  std::lock_guard<std::mutex> lock(q->mutex);
  std::vector<uint8_t> copy(q->itemSize);
  memcpy(copy.data(), item, q->itemSize);
  if (front) {
    q->items.push_front(std::move(copy));
  } else {
    q->items.push_back(std::move(copy));
  }
  return pdPASS;
}
}  // namespace

extern "C" BaseType_t xQueueSend(QueueHandle_t q, const void* item, TickType_t ticks) {
  return queueSend(q, item, ticks, false);
}
extern "C" BaseType_t xQueueSendToBack(QueueHandle_t q, const void* item, TickType_t ticks) {
  return queueSend(q, item, ticks, false);
}
extern "C" BaseType_t xQueueSendToFront(QueueHandle_t q, const void* item, TickType_t ticks) {
  return queueSend(q, item, ticks, true);
}
extern "C" BaseType_t xQueueSendFromISR(QueueHandle_t q, const void* item, BaseType_t* woken) {
  if (woken) *woken = pdFALSE;
  return queueSend(q, item, 0, false);
}

extern "C" BaseType_t xQueueReceive(QueueHandle_t handle, void* out, TickType_t ticks_to_wait) {
  auto* q = static_cast<Queue*>(handle);
  if (!q) return pdFAIL;
  if (!waitFor(q->mutex, ticks_to_wait, [q] { return !q->items.empty(); })) return pdFAIL;
  std::lock_guard<std::mutex> lock(q->mutex);
  if (q->items.empty()) return pdFAIL;
  if (out) memcpy(out, q->items.front().data(), q->itemSize);
  q->items.pop_front();
  return pdPASS;
}

extern "C" BaseType_t xQueuePeek(QueueHandle_t handle, void* out, TickType_t ticks_to_wait) {
  auto* q = static_cast<Queue*>(handle);
  if (!q) return pdFAIL;
  if (!waitFor(q->mutex, ticks_to_wait, [q] { return !q->items.empty(); })) return pdFAIL;
  std::lock_guard<std::mutex> lock(q->mutex);
  if (q->items.empty()) return pdFAIL;
  if (out) memcpy(out, q->items.front().data(), q->itemSize);
  return pdPASS;
}

extern "C" BaseType_t xQueueReset(QueueHandle_t handle) {
  auto* q = static_cast<Queue*>(handle);
  if (!q) return pdFAIL;
  std::lock_guard<std::mutex> lock(q->mutex);
  q->items.clear();
  return pdPASS;
}

extern "C" UBaseType_t uxQueueMessagesWaiting(QueueHandle_t handle) {
  auto* q = static_cast<Queue*>(handle);
  if (!q) return 0;
  std::lock_guard<std::mutex> lock(q->mutex);
  return static_cast<UBaseType_t>(q->items.size());
}

extern "C" UBaseType_t uxQueueSpacesAvailable(QueueHandle_t handle) {
  auto* q = static_cast<Queue*>(handle);
  if (!q) return 0;
  std::lock_guard<std::mutex> lock(q->mutex);
  return static_cast<UBaseType_t>(q->capacity - q->items.size());
}

// ── Semaphores and mutexes ───────────────────────────────────────────────────
extern "C" SemaphoreHandle_t xSemaphoreCreateBinary(void) {
  auto* s = new Semaphore();
  s->count = 0;
  s->maxCount = 1;
  return s;
}
extern "C" SemaphoreHandle_t xSemaphoreCreateMutex(void) {
  auto* s = new Semaphore();
  s->count = 1;
  s->maxCount = 1;
  return s;
}
extern "C" SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void) {
  auto* s = new Semaphore();
  s->count = 1;
  s->maxCount = 1;
  s->recursive = true;
  return s;
}
extern "C" SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t max_count, UBaseType_t initial_count) {
  auto* s = new Semaphore();
  s->count = initial_count;
  s->maxCount = max_count;
  return s;
}
extern "C" SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t*) { return xSemaphoreCreateBinary(); }
extern "C" SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t*) { return xSemaphoreCreateMutex(); }
extern "C" void vSemaphoreDelete(SemaphoreHandle_t handle) { delete static_cast<Semaphore*>(handle); }

extern "C" BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t ticks_to_wait) {
  auto* s = static_cast<Semaphore*>(handle);
  if (!s) return pdFAIL;
  if (!waitFor(s->mutex, ticks_to_wait, [s] { return s->count > 0; })) return pdFAIL;
  std::lock_guard<std::mutex> lock(s->mutex);
  if (s->count <= 0) return pdFAIL;
  --s->count;
  s->owner = std::this_thread::get_id();
  return pdPASS;
}

extern "C" BaseType_t xSemaphoreGive(SemaphoreHandle_t handle) {
  auto* s = static_cast<Semaphore*>(handle);
  if (!s) return pdFAIL;
  std::lock_guard<std::mutex> lock(s->mutex);
  if (s->count >= s->maxCount) return pdFAIL;
  ++s->count;
  return pdPASS;
}

extern "C" BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t handle, TickType_t ticks_to_wait) {
  auto* s = static_cast<Semaphore*>(handle);
  if (!s) return pdFAIL;
  {
    std::lock_guard<std::mutex> lock(s->mutex);
    if (s->recursionDepth > 0 && s->owner == std::this_thread::get_id()) {
      ++s->recursionDepth;
      return pdPASS;
    }
  }
  if (xSemaphoreTake(handle, ticks_to_wait) != pdPASS) return pdFAIL;
  std::lock_guard<std::mutex> lock(s->mutex);
  s->recursionDepth = 1;
  return pdPASS;
}

extern "C" BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t handle) {
  auto* s = static_cast<Semaphore*>(handle);
  if (!s) return pdFAIL;
  {
    std::lock_guard<std::mutex> lock(s->mutex);
    if (s->recursionDepth > 1) {
      --s->recursionDepth;
      return pdPASS;
    }
    s->recursionDepth = 0;
  }
  return xSemaphoreGive(handle);
}

// Callable from the daemon's virtual-interrupt thread: this is how the panel
// model's BUSY completion edge wakes EpdBus::waitRefreshComplete().
extern "C" BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t handle, BaseType_t* woken) {
  if (woken) *woken = pdFALSE;
  return xSemaphoreGive(handle);
}
extern "C" BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t handle, BaseType_t* woken) {
  if (woken) *woken = pdFALSE;
  return xSemaphoreTake(handle, 0);
}

// ── Event groups ─────────────────────────────────────────────────────────────
extern "C" EventGroupHandle_t xEventGroupCreate(void) { return new EventGroup(); }
extern "C" void vEventGroupDelete(EventGroupHandle_t handle) { delete static_cast<EventGroup*>(handle); }

extern "C" EventBits_t xEventGroupSetBits(EventGroupHandle_t handle, EventBits_t bits) {
  auto* g = static_cast<EventGroup*>(handle);
  if (!g) return 0;
  std::lock_guard<std::mutex> lock(g->mutex);
  g->bits |= bits;
  return g->bits;
}
extern "C" EventBits_t xEventGroupClearBits(EventGroupHandle_t handle, EventBits_t bits) {
  auto* g = static_cast<EventGroup*>(handle);
  if (!g) return 0;
  std::lock_guard<std::mutex> lock(g->mutex);
  const EventBits_t before = g->bits;
  g->bits &= ~bits;
  return before;
}
extern "C" EventBits_t xEventGroupGetBits(EventGroupHandle_t handle) {
  auto* g = static_cast<EventGroup*>(handle);
  if (!g) return 0;
  std::lock_guard<std::mutex> lock(g->mutex);
  return g->bits;
}
extern "C" EventBits_t xEventGroupWaitBits(EventGroupHandle_t handle, EventBits_t bits, BaseType_t clear_on_exit,
                                           BaseType_t wait_for_all, TickType_t ticks_to_wait) {
  auto* g = static_cast<EventGroup*>(handle);
  if (!g) return 0;
  waitFor(g->mutex, ticks_to_wait, [g, bits, wait_for_all] {
    return wait_for_all ? ((g->bits & bits) == bits) : ((g->bits & bits) != 0);
  });
  std::lock_guard<std::mutex> lock(g->mutex);
  const EventBits_t result = g->bits;
  if (clear_on_exit) g->bits &= ~bits;
  return result;
}

// ── Software timers ──────────────────────────────────────────────────────────
namespace {
struct Timer {
  std::string name;
  TickType_t period = 0;
  bool autoReload = false;
  void* id = nullptr;
  TimerCallbackFunction_t callback = nullptr;
  std::thread thread;
  std::atomic<bool> running{false};
};
}  // namespace

extern "C" TimerHandle_t xTimerCreate(const char* name, TickType_t period, BaseType_t auto_reload, void* id,
                                      TimerCallbackFunction_t cb) {
  auto* t = new Timer();
  t->name = name ? name : "timer";
  t->period = period;
  t->autoReload = auto_reload != pdFALSE;
  t->id = id;
  t->callback = cb;
  return t;
}

extern "C" BaseType_t xTimerStart(TimerHandle_t handle, TickType_t) {
  auto* t = static_cast<Timer*>(handle);
  if (!t || t->running.exchange(true)) return pdFAIL;
  t->thread = std::thread([t]() {
    do {
      if (fsim_delay_us(static_cast<uint64_t>(t->period) * 1000ULL) != 0) break;
      if (!t->running.load()) break;
      if (t->callback) t->callback(t);
    } while (t->autoReload && t->running.load());
    t->running = false;
  });
  return pdPASS;
}

extern "C" BaseType_t xTimerStop(TimerHandle_t handle, TickType_t) {
  auto* t = static_cast<Timer*>(handle);
  if (!t) return pdFAIL;
  t->running = false;
  if (t->thread.joinable()) t->thread.join();
  return pdPASS;
}

extern "C" BaseType_t xTimerDelete(TimerHandle_t handle, TickType_t block) {
  auto* t = static_cast<Timer*>(handle);
  if (!t) return pdFAIL;
  xTimerStop(handle, block);
  delete t;
  return pdPASS;
}

extern "C" void* pvTimerGetTimerID(TimerHandle_t handle) {
  auto* t = static_cast<Timer*>(handle);
  return t ? t->id : nullptr;
}
