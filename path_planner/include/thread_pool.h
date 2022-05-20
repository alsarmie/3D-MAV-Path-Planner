#ifndef PATH_PLANNER_THREAD_POOL_H
#define PATH_PLANNER_THREAD_POOL_H
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
namespace globalPlanner::RRT {
class ThreadPool {
public:
  // Constructor
  // Creating a thread pool with the number of workers specified.
  /**
   *
   * @param workers_ Number of worker threads to create.
   */
  explicit ThreadPool(int workers_) : shutdown(false) {
    workers.reserve(workers_);
    for (int n = 0; n < workers_; n++)
      workers.emplace_back([this, n] { startThread(n); });
    std::cout << "Thread pool created with " << workers_ << " workers"
              << std::endl;
  }
  // Destructor
  // The destructor of the class. It is setting the shutdown flag to true and
  // notifying all the threads that the pool is shutting down.
  ~ThreadPool() {
    {
      std::lock_guard lck(threadMtx);
      shutdown = true;
      cv.notify_all(); // send message to all active threads.
      std::cout << "Thread pool destroyed " << std::endl;
    }
    for (auto &worker : workers)
      worker.join();
  }
  // Main member function to place a task on queue for an available thread to
  // take.
  /**
   *
   * @param functor function object to execute in a worker thread.
   */
  void workOn(std::function<void(void)> functor) noexcept {
    std::unique_lock<std::mutex> lck(threadMtx);
    tasks.emplace(std::move(functor));
    cv.notify_one();
  }

protected:
  /**
   *
   * @param n Worker number/ID.
   */
  void startThread(int n) noexcept {
    std::function<void(void)>
        task; // To handle th receiving function to execute.
    // thread will be alive until shutdown flag is set by
    // destructor.
    while (true) {
      { // local scope for lock
        std::unique_lock<std::mutex> lck(threadMtx);
        if (!shutdown.load())
          cv.wait(lck, [this] { return !tasks.empty(); });
        if (!tasks.empty()) {
          task = std::move(tasks.front());
          tasks.pop();
        } else {
          return;
        }
      }
      // Execute task
      task();
    }
  }

  std::mutex threadMtx;
  std::atomic<bool> shutdown;
  std::condition_variable cv;
  std::vector<std::thread> workers;
  std::queue<std::function<void(void)>> tasks;
};
} // namespace globalPlanner::RRT
#endif // PATH_PLANNER_THREAD_POOL_H