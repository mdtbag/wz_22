#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

template<typename T>
class SafeQueue {
private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_producer_; // 生产者条件变量
    std::condition_variable cond_consumer_; // 消费者条件变量
    bool is_running_ = true;
    size_t max_size_ = 10; // 队列最大缓存10帧

public:
    SafeQueue(size_t max_size = 10) : max_size_(max_size) {}

    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        is_running_ = false;
        cond_producer_.notify_all();
        cond_consumer_.notify_all();
    }

    // 入队：队列满时阻塞等待，不丢帧（关键修改）
    bool push(const T& data) {
        std::unique_lock<std::mutex> lock(mutex_);
        // 队列满且运行中，阻塞等待消费者消费
        cond_producer_.wait(lock, [this]() {
            return !is_running_ || queue_.size() < max_size_;
        });

        if (!is_running_) return false;

        queue_.push(data);
        cond_consumer_.notify_one(); // 通知消费者有新帧
        return true;
    }

    // 出队：空时阻塞等待
    bool pop(T& data, int timeout_ms = 500) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cond_consumer_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() {
            return !is_running_ || !queue_.empty();
        })) {
            return false;
        }

        if (!is_running_ || queue_.empty()) return false;

        data = queue_.front();
        queue_.pop();
        cond_producer_.notify_one(); // 通知生产者有空位
        return true;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) queue_.pop();
    }
};

#endif // SAFE_QUEUE_H