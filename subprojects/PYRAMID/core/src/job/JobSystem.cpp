#include <job/JobSystem.h>
#include <stdexcept> // For std::runtime_error
#include <iostream> // For potential error logging

namespace pyramid {
namespace core {
namespace job {

  JobSystem::JobSystem(size_t num_threads) : stop_(false), active_jobs_(0) {
    size_t thread_count = num_threads;
    if (thread_count == 0) {
      thread_count = std::thread::hardware_concurrency();
      // hardware_concurrency might return 0, handle this case
      if (thread_count == 0) {
        thread_count = 1; // Default to at least one thread
      }
    }

    workers_.reserve(thread_count);
    for(size_t i = 0; i < thread_count; ++i) {
      workers_.emplace_back([this] { this->workerLoop(); });
    }
  }

  JobSystem::~JobSystem() {
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      stop_ = true; // Signal threads to stop
    }
    condition_.notify_all(); // Wake up all waiting threads

    // Join all worker threads
    for(std::thread &worker: workers_) {
      if(worker.joinable()) {
        worker.join();
      }
    }
    // Note: waitIdle() is implicitly called by waiting for threads to join
    // if the worker loop correctly handles the stop flag and remaining jobs.
  }

  void JobSystem::enqueue(Job job) {
    if (!job) return; // Optional: ignore null jobs
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      if(stop_) {
        throw std::runtime_error("enqueue on stopped JobSystem");
      }
      job_queue_.push(std::move(job));
      // Increment active jobs count *before* notifying
      // Needs careful consideration with waitIdle logic
      // Let's increment inside the worker loop for now before execution
    } // lock released
    condition_.notify_one(); // Notify one waiting thread
  }
  
  void JobSystem::waitIdle() {
    std::unique_lock<std::mutex> lock(idle_mutex_);
    idle_condition_.wait(lock, [this] {
      // Wait until the queue is empty AND there are no jobs being executed
      std::lock_guard<std::mutex> queue_lock(queue_mutex_); // Need to lock queue too for check
      return job_queue_.empty() && active_jobs_ == 0;
    });
  }

  void JobSystem::workerLoop() {
    while (true) {
      Job job;
      {
        // Wait until there is a job or stop is signaled
        std::unique_lock<std::mutex> lock(queue_mutex_);
        condition_.wait(lock, [this]{ return stop_ || !job_queue_.empty(); });

        // If stop is signaled and queue is empty, exit the loop
        if (stop_ && job_queue_.empty()) {
          return; 
        }

        // Dequeue a job
        job = std::move(job_queue_.front());
        job_queue_.pop();
        
        // Increment active jobs count now that we've taken one
        active_jobs_++; 
      } // Release the lock before executing the job
      
      // Notify waitIdle potentially (though this is before job execution)
      // It might be better to notify *after* job completion. 
      // idle_condition_.notify_all(); // Potential notification point

      try {
        // Execute the job
        job(); 
      } catch (const std::exception& e) {
        // Consider logging the exception
        std::cerr << "JobSystem worker caught exception: " << e.what() << std::endl;
      } catch (...) {
        std::cerr << "JobSystem worker caught unknown exception." << std::endl;
      }
      
      // Decrement active jobs count after execution
      active_jobs_--;
      
      // Notify threads waiting in waitIdle that a job finished
      // Lock is needed only for the notification check and notify_all
      {
        std::lock_guard<std::mutex> idle_lock(idle_mutex_);
        // Check condition again before notifying unnecessarily
        std::lock_guard<std::mutex> queue_lock(queue_mutex_); 
        if(job_queue_.empty() && active_jobs_ == 0) {
          idle_condition_.notify_all();
        }
      } 
    }
  }

} // namespace job
} // namespace core
} // namespace pyramid


