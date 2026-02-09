// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#include "parallel.h"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <atomic>
#include <thread>

class ThreadPool {
public:
    ThreadPool(size_t num_threads = std::thread::hardware_concurrency()) : pendingTask_(0), noPendingTask_(true) {
        for (size_t i = 0; i < num_threads; ++i) {
            threads_.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex_);
                        cv_.wait(lock, [this] {
                            return !tasks_.empty() || stop_;
                        });
                        if (stop_ && tasks_.empty()) {
                            return;
                        }
                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }
                    task();
                    { 
                        std::unique_lock<std::mutex> lock(queue_mutex_);
                        --pendingTask_;
                        if (pendingTask_ == 0) {
                            noPendingTask_ = true;
                            cvPending_.notify_one();
                        }
                    }
                }
            });
        }
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& thread : threads_) {
            thread.join();
        }
    }

    void enqueue(std::function<void()> task) {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            tasks_.emplace(std::move(task));
            ++pendingTask_;
            noPendingTask_ = false;
        }
        cv_.notify_one();
    }

    void sync() {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        cvPending_.wait(lock, [this]{ return noPendingTask_; });
    }

    size_t threadCount() {
        return threads_.size();
    }

private:
    std::vector<std::thread> threads_;
    std::queue<std::function<void()> > tasks_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    bool stop_ = false;
    size_t pendingTask_;
    bool noPendingTask_ = true;
    std::condition_variable cvPending_;
};

static ThreadPool &getThreadPool() {
    static ThreadPool pool;
    return pool;
}

void Parallel::For(size_t start, size_t stop, std::function<void(size_t)> foo) {
    auto &pool = getThreadPool();
    size_t N = pool.threadCount();
    size_t grain = (stop-start)/N + ((stop-start)%N ? 1 : 0);
    for (uint32_t t = 0; t < N; ++t) {
        uint32_t begin = start+t*grain, end = std::min(start+(t+1)*grain, stop);
        auto task = [begin, end, &foo]() {
            for (uint32_t i = begin; i < end; ++i) {
                foo(i);
            }
        };
        pool.enqueue(task);
    }
    pool.sync();
}

bool Parallel::ForAny(size_t start, size_t stop, std::function<bool(size_t)> foo) {
    auto &pool = getThreadPool();
    size_t N = pool.threadCount();
    size_t grain = (stop-start)/N + ((stop-start)%N ? 1 : 0);
    std::vector<uint8_t> booleans(N);

    for (uint32_t t = 0; t < N; ++t) {
        uint32_t begin = start+t*grain, end = std::min(start+(t+1)*grain, stop);
        auto task = [begin, end, t, &foo, &booleans]() {
            bool res = false;
            for (uint32_t i = begin; i < end; ++i) {
                res |= foo(i);
            }
            booleans[t] = res;
        };
        pool.enqueue(task);
    }
    pool.sync();

    bool result = false;
    for (auto b : booleans) {
        result |= b;
    }
    return result;
}

std::array<bool, 2> Parallel::ForAny2(size_t start, size_t stop, std::function<std::array<bool, 2>(size_t)> foo) {
    auto &pool = getThreadPool();
    size_t N = pool.threadCount();
    size_t grain = (stop-start)/N + ((stop-start)%N ? 1 : 0);
    std::vector<uint8_t> booleans0(N);
    std::vector<uint8_t> booleans1(N);

    for (uint32_t t = 0; t < N; ++t) {
        uint32_t begin = start+t*grain, end = std::min(start+(t+1)*grain, stop);
        auto task = [begin, end, t, &foo, &booleans0, &booleans1]() {
            bool res0 = false;
            bool res1 = false;
            for (uint32_t i = begin; i < end; ++i) {
                auto val = foo(i);
                res0 |= val[0];
                res1 |= val[1];
            }
            booleans0[t] = res0;
            booleans1[t] = res1;

        };
        pool.enqueue(task);
    }
    pool.sync();

    std::array<bool, 2> result{false, false};
    for (uint32_t i = 0; i < N; ++i) {
        result[0] |= booleans0[i];
        result[1] |= booleans1[i];
    }
    return result;
}

size_t Parallel::ArgMin(size_t start, size_t stop, std::function<float(size_t)> foo) {
    struct FS { float f; size_t s; };

    auto &pool = getThreadPool();
    size_t N = pool.threadCount();
    size_t grain = (stop-start)/N + ((stop-start)%N ? 1 : 0);
    std::vector<FS> values(N);

    for (uint32_t t = 0; t < N; ++t) {
        uint32_t begin = start+t*grain, end = std::min(start+(t+1)*grain, stop);
        auto task = [begin, end, t, &foo, &values]() {
            float val = std::numeric_limits<float>::infinity();
            size_t id = 0;
            for (uint32_t i = begin; i < end; ++i) {
                float newVal = foo(i);
                if (newVal < val) {
                    val = newVal;
                    id = i;
                }
            }
            values[t] = {val, id};
        };
        pool.enqueue(task);
    }
    pool.sync();

    FS result{std::numeric_limits<float>::infinity(), 0};
    for (auto v : values) {
        if (v.f < result.f) {
            result = v;
        }
    }

    return result.s;
}