#pragma once

#include <mutex>

struct StatisticsSnapshot
{
    int produced = 0;
    int processed = 0;
    int saved = 0;
    int corrupted = 0;
};

class Statistics
{
public:
    void onProduced();
    void onProcessed();
    void onSaved();
    void onCorrupted();
    StatisticsSnapshot snapshot() const;

private:
    // producer 和多个 worker 线程会同时读写这些计数，统一由 mutex_ 保护。
    // snapshot() 是 const 成员函数也要加锁，所以声明为 mutable。
    mutable std::mutex mutex_;
    int produced_ = 0;
    int processed_ = 0;
    int saved_ = 0;
    int corrupted_ = 0;
};
