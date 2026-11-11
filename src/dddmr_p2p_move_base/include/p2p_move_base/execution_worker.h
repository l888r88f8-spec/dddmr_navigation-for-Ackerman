#ifndef P2P_MOVE_BASE_EXECUTION_WORKER_H
#define P2P_MOVE_BASE_EXECUTION_WORKER_H

#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>

namespace p2p_move_base
{
/**
 * Owns a single worker thread and executes at most one submitted task at a time.
 *
 * submit() replaces an already pending task; the replaced task's on_replaced
 * callback is invoked outside the worker lock so callers can terminate the
 * discarded work explicitly; its ordering relative to the replacement task is
 * not guaranteed. stop() is idempotent, wakes the worker, prevents new
 * submissions, and joins the worker outside the task lock. A stop() called by
 * the worker task itself only requests termination and returns immediately;
 * reaping is left to an external stop()/destructor.
 */
class ExecutionWorker
{
public:
  using ExceptionHandler = std::function<void(std::exception_ptr)>;

  struct Task
  {
    std::function<void()> run;
    std::function<void()> on_replaced;
    ExceptionHandler on_failed;
  };

  explicit ExecutionWorker(ExceptionHandler unexpected_handler = {});
  ~ExecutionWorker();

  ExecutionWorker(const ExecutionWorker &) = delete;
  ExecutionWorker & operator=(const ExecutionWorker &) = delete;

  bool submit(Task task);
  void stop();
  bool is_stopping() const;

private:
  void run();
  void notifyFailure(const Task & task, std::exception_ptr error) noexcept;
  void notifyReplaced(const Task & task) noexcept;

  mutable std::mutex mutex_;
  std::mutex join_mutex_;
  std::condition_variable wake_cv_;
  ExceptionHandler unexpected_handler_;
  Task pending_;
  bool has_pending_ = false;
  bool stop_requested_ = false;
  bool stop_complete_ = false;
  bool executing_ = false;
  std::thread worker_;
  std::thread::id worker_id_;
};
}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE_EXECUTION_WORKER_H
