#include <p2p_move_base/execution_worker.h>

#include <utility>

namespace p2p_move_base
{
ExecutionWorker::ExecutionWorker(ExceptionHandler unexpected_handler)
: unexpected_handler_(std::move(unexpected_handler))
{
  worker_ = std::thread(&ExecutionWorker::run, this);
  worker_id_ = worker_.get_id();
}

ExecutionWorker::~ExecutionWorker()
{
  stop();
}

bool ExecutionWorker::submit(Task task)
{
  if(!task.run){
    return false;
  }

  Task replaced;
  bool has_replaced = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stop_requested_){
      return false;
    }
    if(has_pending_){
      replaced = std::move(pending_);
      has_replaced = true;
    }
    pending_ = std::move(task);
    has_pending_ = true;
  }

  // Terminate the discarded pending task outside the lock. Ordering relative
  // to the replacement task is not guaranteed; callers that need strict
  // terminate-before-submit ordering must do it themselves.
  if(has_replaced){
    notifyReplaced(replaced);
  }
  wake_cv_.notify_one();
  return true;
}

void ExecutionWorker::stop()
{
  Task discarded;
  bool has_discarded = false;
  const bool self_stop = (worker_id_ == std::this_thread::get_id());

  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
    if(has_pending_){
      discarded = std::move(pending_);
      has_pending_ = false;
      has_discarded = true;
    }
  }

  if(has_discarded){
    notifyReplaced(discarded);
  }
  wake_cv_.notify_all();

  if(self_stop){
    // The calling task must return to run() so the worker can exit. Joining
    // here, or waiting for an external join, would deadlock.
    return;
  }

  // External callers serialize reaping. worker_ may already have been moved
  // out by an earlier caller, so stop_complete_ is only published after that
  // thread has actually been joined.
  std::lock_guard<std::mutex> join_lock(join_mutex_);
  std::thread thread_to_join;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stop_complete_){
      return;
    }
    if(worker_.joinable()){
      thread_to_join = std::move(worker_);
    }
  }

  if(thread_to_join.joinable()){
    thread_to_join.join();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_complete_ = true;
  }
}

bool ExecutionWorker::is_stopping() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stop_requested_;
}

void ExecutionWorker::run()
{
  for(;;){
    Task task;
    bool execute_task = true;

    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_cv_.wait(lock, [this]() {return stop_requested_ || has_pending_;});

      if(stop_requested_){
        if(!has_pending_){
          executing_ = false;
          return;
        }
        task = std::move(pending_);
        has_pending_ = false;
        execute_task = false;
      }
      else{
        task = std::move(pending_);
        has_pending_ = false;
        executing_ = true;
      }
    }

    if(execute_task){
      try{
        if(task.run){
          task.run();
        }
      }
      catch(...){
        notifyFailure(task, std::current_exception());
      }
    }
    else{
      notifyReplaced(task);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      executing_ = false;
    }
  }
}

void ExecutionWorker::notifyFailure(const Task & task, std::exception_ptr error) noexcept
{
  if(task.on_failed){
    try{
      task.on_failed(error);
      return;
    }
    catch(...){
      error = std::current_exception();
    }
  }

  if(unexpected_handler_){
    try{
      unexpected_handler_(error);
    }
    catch(...){
    }
  }
}

void ExecutionWorker::notifyReplaced(const Task & task) noexcept
{
  if(!task.on_replaced){
    return;
  }

  try{
    task.on_replaced();
  }
  catch(...){
    if(unexpected_handler_){
      try{
        unexpected_handler_(std::current_exception());
      }
      catch(...){
      }
    }
  }
}
}  // namespace p2p_move_base
