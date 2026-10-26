/***
    This file is part of snapcast
    Copyright (C) 2014-2024  Johannes Pohl

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

#pragma once

// local headers
#include "common/sample_format.hpp"

// 3rd party headers
#include <alsa/asoundlib.h>
#include <soxr.h>

// standard headers
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


namespace player
{

/// Second ALSA output that follows the main player sample by sample.
///
/// Meant for an echo canceller's reference input (e.g. the USB playback of a
/// microphone array): it gets exactly the samples the main output plays, with a
/// fixed time offset, instead of syncing to the server on its own. The main
/// player hands over each block together with the time its first sample is
/// heard. A worker thread writes the blocks to the reference device, converts
/// to its rate with soxr (fixed ratio, constant delay) and keeps the reference at
/// "main output time + offset" by dropping or repeating single input frames at the
/// rate a PI controller asks for. Two sound cards on different clocks are fine; on
/// the same clock nothing is dropped.
class RefOutput
{
public:
    struct Settings
    {
        std::string pcm;                             ///< ALSA device of the reference output
        uint32_t rate{16000};                        ///< sample rate of the reference device
        uint32_t channels{2};                        ///< channels of the reference device (mono mix on all)
        std::chrono::microseconds offset{0};         ///< reference is heard this much after the main output
        std::chrono::microseconds buffer{60000};     ///< ALSA buffer of the reference device
        std::chrono::microseconds max_error{20000};  ///< beyond this, realign hard
    };

    explicit RefOutput(Settings settings);
    ~RefOutput();

    void start();
    void stop();

    /// A block the main output has just written; its first frame is heard at @p play_time.
    void push(const char* data, size_t frames, const SampleFormat& format, std::chrono::steady_clock::time_point play_time);

private:
    using clock = std::chrono::steady_clock;

    struct Block
    {
        std::vector<float> samples;
        uint32_t rate;
        clock::time_point play_time;
    };

    void worker();
    bool openPcm();
    void closePcm();
    void resetResampler(uint32_t in_rate);
    void writeOutput(const std::vector<float>& mono, size_t frames);
    void writeSilence(size_t frames);
    void realign(clock::time_point now, snd_pcm_sframes_t delay);
    void logStats(clock::time_point now);

    Settings settings_;
    std::atomic<bool> active_{false};
    std::thread thread_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Block> queue_;

    // worker state
    snd_pcm_t* pcm_{nullptr};
    soxr_t soxr_{nullptr};
    uint32_t in_rate_{0};
    std::vector<float> in_;           ///< mono input not yet given to the resampler
    clock::time_point in_time_;       ///< main output play time of in_[0]
    bool aligned_{false};
    double integral_{0.0};
    double ratio_corr_{0.0};
    double err_avg_{0.0};
    double acc_{0.0};
    size_t dropped_{0};
    size_t repeated_{0};
    clock::time_point last_data_;
    clock::time_point last_stats_;
    size_t realigns_{0};
    size_t xruns_{0};
    std::vector<int16_t> out_buffer_;
};

} // namespace player
