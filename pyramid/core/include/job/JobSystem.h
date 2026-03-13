#pragma once

#include <vector>
#include <thread>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <future> // For std::packaged_task

namespace pyramid {
namespace core {
  namespace job {

  /// \brief Defines the type for a job to be executed by the JobSystem
  using Job = std::function<void()>;

  /// \brief Manages a pool of worker threads to execute jobs concurrently
  /// Jobs are processed in FIFO order.
  class JobSystem {
  public:
  /// \brief Constructs the JobSystem and starts the worker threads
  /// \param num_threads Number of worker threads. Defaults to hardware concurrency
  explicit JobSystem(size_t num_threads = 0);

  /// \brief Destructor. Signals workers to stop, waits for pending jobs, and joins threads
  ~JobSystem();

  // Prevent copying and moving
  JobSystem(const JobSystem&) = delete;
  JobSystem& operator=(const JobSystem&) = delete;
  JobSystem(JobSystem&&) = delete;
  JobSystem& operator=(JobSystem&&) = delete;

  /// \brief Enqueues a job for execution by a worker thread
  /// This operation is thread-safe.
  /// \param job The job (std::function<void()>) to enqueue
  void enqueue(Job job);
  
   /// \brief Enqueues a job and returns a future to get its result
   /// \tparam F Function type
   /// \tparam Args Argument types
   /// \param f The function or callable object
   /// \param args Arguments to pass to the function
   /// \return A std::future representing the asynchronous result
   template<class F, class... Args>
   auto enqueueTask(F&& f, Args&&... args) 
     -> std::future<typename std::result_of<F(Args...)>::type>;

  /// \brief Waits until the job queue is empty and all currently executing jobs are finished
  /// Note: New jobs could be enqueued immediately after this returns.
  void waitIdle();

  private:
  /// \brief The function executed by each worker thread
  void workerLoop();

  std::vector<std::thread> workers_;
  std::queue<Job> job_queue_;
  std::mutex queue_mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_{false};
  
  std::atomic<size_t> active_jobs_{0};
  std::mutex idle_mutex_;
  std::condition_variable idle_condition_;
  };

  // --- Template Implementation --- 
  template<class F, class... Args>
  auto JobSystem::enqueueTask(F&& f, Args&&... args) 
  -> std::future<typename std::result_of<F(Args...)>::type>
  {
  using return_type = typename std::result_of<F(Args...)>::type;

  // Create a packaged_task which wraps the function and its arguments
  auto task = std::make_shared< std::packaged_task<return_type()> >(
    std::bind(std::forward<F>(f), std::forward<Args>(args)...)
  );
    
  // Get the future associated with the packaged_task
  std::future<return_type> res = task->get_future();
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);

    // Don't allow enqueueing after stopping the pool
    if(stop_) {
    throw std::runtime_error("enqueue on stopped JobSystem");
    }

    // Enqueue the packaged_task as a void() lambda
    job_queue_.emplace([task](){ (*task)(); });
  }
  condition_.notify_one(); // Notify a worker thread
  return res;
  }


  } // namespace job
} // namespace core
} // namespace pyramid 


