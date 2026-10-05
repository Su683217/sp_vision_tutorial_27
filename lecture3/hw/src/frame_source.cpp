#include "frame_source.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <opencv2/imgcodecs.hpp>

namespace
{
    bool isImage(const std::filesystem::path &path)
    {
        const auto extension = path.extension().string();
        return extension == ".jpg" || extension == ".jpeg" ||
               extension == ".png" || extension == ".ppm";
    }
}

ImageSequenceSource::ImageSequenceSource(std::filesystem::path directory,
                                         int producer_delay_ms)
    : producer_delay_ms_(producer_delay_ms)
{
    if (!std::filesystem::is_directory(directory))
    {
        throw std::runtime_error("input directory does not exist: " + directory.string());
    }
    for (const auto &entry : std::filesystem::directory_iterator(directory))
    {
        if (entry.is_regular_file() && isImage(entry.path()))
        {
            paths_.push_back(entry.path());
        }
    }
    std::sort(paths_.begin(), paths_.end());
    if (paths_.empty())
    {
        throw std::runtime_error("input directory contains no images");
    }
}

bool ImageSequenceSource::next(Frame &frame)
{
    if (next_index_ >= paths_.size())
    {
        return false;
    }

    const cv::Mat raw = cv::imread(paths_[next_index_].string(), cv::IMREAD_COLOR);
    if (raw.empty())
    {
        throw std::runtime_error("failed to read: " + paths_[next_index_].string());
    }
    raw.copyTo(buffer_); // Simulates a camera-owned reusable buffer.

    frame.id = static_cast<int>(next_index_++);
    frame.expected_checksum = checksum(buffer_);

    // buffer_ 会在下一次 next() 时被复用（copyTo 直接写回同一块内存）。
    // 普通赋值只复制 Mat 头、和 buffer_ 共享像素，所以必须 clone() 出独立的一份，
    // 让这一帧真正拥有自己的像素数据。
    frame.image = buffer_.clone();

    if (producer_delay_ms_ > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(producer_delay_ms_));
    }
    return true;
}
