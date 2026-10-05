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
    // 多个线程会同时读写计数，统一用 mutex_ 保护
    mutable std::mutex mutex_;
    int produced_ = 0;
    int processed_ = 0;
    int saved_ = 0;
    int corrupted_ = 0;
};
