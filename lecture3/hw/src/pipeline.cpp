#include "pipeline.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <opencv2/imgcodecs.hpp>

namespace
{
    std::mutex output_mutex;

    // A simple thread-safe log output helper function
    void logLine(std::ostream &output, const std::string &message)
    {
        std::lock_guard<std::mutex> lock(output_mutex);
        output << message << '\n';
    }
}

Pipeline::Pipeline(std::unique_ptr<FrameSource> source, PipelineConfig config)
    : source_(std::move(source)), config_(std::move(config))
{
    if (!source_)
    {
        throw std::invalid_argument("Pipeline requires a frame source");
    }
    if (config_.worker_count < 2)
    {
        throw std::invalid_argument("worker_count must be at least 2");
    }
}

Pipeline::~Pipeline()
{
    // RAII：析构时必须回收所有已启动的线程。销毁一个仍可 join 的 std::thread
    // 会直接调用 std::terminate，而且线程里还在用 this 的成员，会变成悬空访问。
    //
    // 正常情况下 producer 读完所有帧后会自己 close() 队列，worker 消费完剩余帧
    // 后退出，所以直接 wait() 就能把线程依次 join 掉。
    // 若 producer 从未启动（例如 start() 中途失败），就没人关闭队列，等待中的
    // worker 会永远卡在 pop() 里，所以这里先补一次 close()。close() 可重复调用。
    if (!producer_.joinable())
    {
        queue_.close();
    }
    wait();
}

void Pipeline::start()
{
    // 前置条件：start() 只能调用一次。再次调用会给仍可 join 的 producer_ 重新
    // 赋值（触发 std::terminate），或者让两批线程同时消费同一个队列。
    if (producer_.joinable() || !workers_.empty())
    {
        throw std::logic_error("Pipeline::start() may only be called once");
    }
    std::filesystem::create_directories(config_.output_directory);
    workers_.reserve(static_cast<std::size_t>(config_.worker_count));
    for (int i = 0; i < config_.worker_count; ++i)
    {
        workers_.emplace_back([this, i]
                              { workerLoop(i); });
    }
    producer_ = std::thread([this]
                            { producerLoop(); });
}

void Pipeline::wait()
{
    if (producer_.joinable())
    {
        producer_.join();
    }
    for (auto &worker : workers_)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
}

StatisticsSnapshot Pipeline::statistics() const
{
    return statistics_.snapshot();
}

void Pipeline::producerLoop()
{
    Frame frame;
    while (source_->next(frame))
    {
        statistics_.onProduced();
        logLine(std::cout, "[Producer] frame " + std::to_string(frame.id));

        // frame 马上会被下一次 next() 整体覆盖，不再需要原来的内容，
        // 所以用 std::move 把资源直接转交给队列，避免多复制一次 Frame。
        queue_.push(std::move(frame));
    }
    queue_.close();
}

void Pipeline::workerLoop(int worker_id)
{
    Frame frame;
    while (queue_.pop(frame))
    {
        if (config_.worker_delay_ms > 0)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.worker_delay_ms));
        }

        if (checksum(frame.image) != frame.expected_checksum)
        {
            statistics_.onCorrupted();
            logLine(std::cerr,
                    "[Worker " + std::to_string(worker_id) + "] ERROR: frame " +
                        std::to_string(frame.id) + " data changed before processing");
            continue;
        }

        logLine(std::cout,
                "[Worker " + std::to_string(worker_id) + "] processing frame " +
                    std::to_string(frame.id));
        const cv::Mat output = processor_.process(frame);
        statistics_.onProcessed();

        std::ostringstream filename;
        filename << std::setw(3) << std::setfill('0') << frame.id << ".jpg";
        const auto name = filename.str();
        if (cv::imwrite((config_.output_directory / name).string(), output))
        {
            statistics_.onSaved();
        }
    }
}
