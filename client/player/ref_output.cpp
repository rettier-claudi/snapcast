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

// prototype/interface header file
#include "ref_output.hpp"

// local headers
#include "common/aixlog.hpp"

// standard headers
#include <algorithm>
#include <cmath>


using namespace std::chrono_literals;
using namespace std;

namespace player
{

static constexpr auto LOG_TAG = "RefOutput";

namespace
{
// Rate control: error in seconds -> relative ratio correction. Time constant ~10 s,
// so a sound card 140 ppm off is caught within seconds and period jitter stays out.
constexpr double KP = 0.05;
constexpr double KI = 0.005;
constexpr double MAX_CORR = 1e-3;   // +-1000 ppm
constexpr double ERR_SMOOTH = 0.02; // EMA of the measured error per write
constexpr auto IDLE_CLOSE = 5s;     // like the main ALSA player: close after 5 s without data

double seconds(chrono::steady_clock::duration d)
{
    return chrono::duration<double>(d).count();
}
} // namespace


RefOutput::RefOutput(Settings settings) : settings_(std::move(settings))
{
}


RefOutput::~RefOutput()
{
    stop();
}


void RefOutput::start()
{
    if (active_)
        return;
    active_ = true;
    last_data_ = clock::now();
    last_stats_ = clock::now();
    thread_ = std::thread(&RefOutput::worker, this);
    LOG(INFO, LOG_TAG) << "Reference output on " << settings_.pcm << ", " << settings_.rate << " Hz, " << settings_.channels
                       << " ch, offset " << settings_.offset.count() / 1000. << " ms\n";
}


void RefOutput::stop()
{
    if (!active_)
        return;
    active_ = false;
    cv_.notify_all();
    if (thread_.joinable())
        thread_.join();
    closePcm();
    if (soxr_ != nullptr)
    {
        soxr_delete(soxr_);
        soxr_ = nullptr;
    }
}


void RefOutput::push(const char* data, size_t frames, const SampleFormat& format, clock::time_point play_time)
{
    if (!active_ || frames == 0)
        return;
    Block block;
    block.rate = format.rate();
    block.play_time = play_time;
    block.samples.resize(frames);
    const uint16_t ch = format.channels();
    const float scale = 1.f / ch;
    if (format.sampleSize() == 2)
    {
        const auto* in = reinterpret_cast<const int16_t*>(data);
        for (size_t f = 0; f < frames; ++f)
        {
            float sum = 0;
            for (uint16_t c = 0; c < ch; ++c)
                sum += in[f * ch + c];
            block.samples[f] = sum * scale / 32768.f;
        }
    }
    else if (format.sampleSize() == 4)
    {
        // 24 bit samples are carried in 32 bit, left aligned to bit 23 (snapcast convention)
        const auto* in = reinterpret_cast<const int32_t*>(data);
        const float full = (format.bits() == 24) ? 8388608.f : 2147483648.f;
        for (size_t f = 0; f < frames; ++f)
        {
            float sum = 0;
            for (uint16_t c = 0; c < ch; ++c)
                sum += static_cast<float>(in[f * ch + c]);
            block.samples[f] = sum * scale / full;
        }
    }
    else
    {
        static bool warned = false;
        if (!warned)
            LOG(WARNING, LOG_TAG) << "Unsupported sample size " << format.sampleSize() << ", reference stays silent\n";
        warned = true;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(std::move(block));
        // never hold more than ~2 s (main output paused without us noticing)
        while (queue_.size() > 400)
            queue_.pop_front();
    }
    cv_.notify_one();
}


bool RefOutput::openPcm()
{
    int err = snd_pcm_open(&pcm_, settings_.pcm.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0)
    {
        LOG(ERROR, LOG_TAG) << "Can't open " << settings_.pcm << ": " << snd_strerror(err) << "\n";
        pcm_ = nullptr;
        return false;
    }
    snd_pcm_hw_params_t* hw;
    snd_pcm_hw_params_alloca(&hw);
    snd_pcm_hw_params_any(pcm_, hw);
    unsigned int rate = settings_.rate;
    unsigned int buffer_time = static_cast<unsigned int>(settings_.buffer.count());
    unsigned int period_time = buffer_time / 4;
    if ((err = snd_pcm_hw_params_set_access(pcm_, hw, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0 ||
        (err = snd_pcm_hw_params_set_format(pcm_, hw, SND_PCM_FORMAT_S16_LE)) < 0 ||
        (err = snd_pcm_hw_params_set_channels(pcm_, hw, settings_.channels)) < 0 || (err = snd_pcm_hw_params_set_rate_near(pcm_, hw, &rate, nullptr)) < 0 ||
        (err = snd_pcm_hw_params_set_buffer_time_near(pcm_, hw, &buffer_time, nullptr)) < 0 ||
        (err = snd_pcm_hw_params_set_period_time_near(pcm_, hw, &period_time, nullptr)) < 0 || (err = snd_pcm_hw_params(pcm_, hw)) < 0)
    {
        LOG(ERROR, LOG_TAG) << "Can't set hw params on " << settings_.pcm << ": " << snd_strerror(err) << "\n";
        closePcm();
        return false;
    }
    if (rate != settings_.rate)
    {
        LOG(WARNING, LOG_TAG) << settings_.pcm << " runs at " << rate << " Hz instead of " << settings_.rate << "\n";
        settings_.rate = rate;
    }
    snd_pcm_prepare(pcm_);
    LOG(INFO, LOG_TAG) << "Opened " << settings_.pcm << ": " << rate << " Hz, buffer " << buffer_time << " us, period " << period_time << " us\n";
    aligned_ = false;
    return true;
}


void RefOutput::closePcm()
{
    if (pcm_ == nullptr)
        return;
    snd_pcm_drop(pcm_);
    snd_pcm_close(pcm_);
    pcm_ = nullptr;
    LOG(INFO, LOG_TAG) << "Closed " << settings_.pcm << "\n";
}


void RefOutput::resetResampler(uint32_t in_rate)
{
    if (soxr_ != nullptr)
        soxr_delete(soxr_);
    soxr_error_t error;
    soxr_quality_spec_t q = soxr_quality_spec(SOXR_HQ, SOXR_VR);
    soxr_io_spec_t io = soxr_io_spec(SOXR_FLOAT32_I, SOXR_FLOAT32_I);
    // variable rate: rates given as the maximum ratio range, the actual ratio is set per call
    soxr_ = soxr_create(in_rate, settings_.rate, 1, &error, &io, &q, nullptr);
    if (error != nullptr)
    {
        LOG(ERROR, LOG_TAG) << "soxr_create: " << error << "\n";
        soxr_ = nullptr;
        return;
    }
    in_rate_ = in_rate;
    soxr_set_io_ratio(soxr_, static_cast<double>(in_rate_) / settings_.rate, 0);
    integral_ = 0;
    ratio_corr_ = 0;
}


void RefOutput::writeOutput(const std::vector<float>& mono, size_t frames)
{
    out_buffer_.resize(frames * settings_.channels);
    for (size_t f = 0; f < frames; ++f)
    {
        float v = std::clamp(mono[f], -1.f, 1.f);
        auto s = static_cast<int16_t>(std::lrint(v * 32767.f));
        for (uint32_t c = 0; c < settings_.channels; ++c)
            out_buffer_[f * settings_.channels + c] = s;
    }
    size_t done = 0;
    while (done < frames)
    {
        snd_pcm_sframes_t n = snd_pcm_writei(pcm_, out_buffer_.data() + done * settings_.channels, frames - done);
        if (n == -EPIPE)
        {
            ++xruns_;
            snd_pcm_prepare(pcm_);
            aligned_ = false;
            return;
        }
        if (n < 0)
        {
            LOG(ERROR, LOG_TAG) << "write: " << snd_strerror(static_cast<int>(n)) << "\n";
            closePcm();
            return;
        }
        done += static_cast<size_t>(n);
    }
}


void RefOutput::writeSilence(size_t frames)
{
    std::vector<float> zero(frames, 0.f);
    writeOutput(zero, frames);
}


/// Drop or pad input so that the next frame written is heard at main time + offset.
void RefOutput::realign(clock::time_point now, snd_pcm_sframes_t delay)
{
    ++realigns_;
    if (soxr_ != nullptr)
    {
        soxr_clear(soxr_);
        soxr_set_io_ratio(soxr_, static_cast<double>(in_rate_) / settings_.rate, 0);
    }
    integral_ = 0;
    ratio_corr_ = 0;
    err_avg_ = 0;
    // the next written frame is heard at:
    auto t_out = now + chrono::duration_cast<clock::duration>(chrono::duration<double>(static_cast<double>(delay) / settings_.rate));
    // resampler group delay, in input time
    double group = (soxr_ != nullptr) ? soxr_delay(soxr_) / settings_.rate : 0.;
    // input sample that belongs there (main time of in_[0] is in_time_)
    double skip = (seconds(t_out - in_time_) - seconds(settings_.offset) + group) * in_rate_;
    if (skip >= 0)
    {
        auto n = std::min<size_t>(static_cast<size_t>(skip), in_.size());
        in_.erase(in_.begin(), in_.begin() + static_cast<long>(n));
        in_time_ += chrono::duration_cast<clock::duration>(chrono::duration<double>(static_cast<double>(n) / in_rate_));
        if (n < static_cast<size_t>(skip))
            return; // not enough input yet, try again with the next blocks
    }
    else
    {
        // input is ahead: pad the reference with silence
        auto pad = static_cast<size_t>(-skip * settings_.rate / in_rate_);
        writeSilence(std::min<size_t>(pad, settings_.rate)); // at most 1 s
    }
    aligned_ = true;
    LOG(INFO, LOG_TAG) << "Aligned (skip " << static_cast<long>(skip) << " input frames, delay " << delay << ")\n";
}


void RefOutput::logStats(clock::time_point now)
{
    if (now - last_stats_ < 60s)
        return;
    last_stats_ = now;
    LOG(INFO, LOG_TAG) << "Stats: error " << err_avg_ * 1e6 << " us, ratio " << ratio_corr_ * 1e6 << " ppm, realigns " << realigns_ << ", xruns " << xruns_
                       << "\n";
}


void RefOutput::worker()
{
    std::vector<float> out;
    while (active_)
    {
        // collect new blocks
        std::deque<Block> blocks;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, 10ms, [this] { return !queue_.empty() || !active_; });
            blocks.swap(queue_);
        }
        auto now = clock::now();
        for (auto& b : blocks)
        {
            if (soxr_ == nullptr || b.rate != in_rate_)
            {
                resetResampler(b.rate);
                in_.clear();
                aligned_ = false;
            }
            auto expected = in_time_ + chrono::duration_cast<clock::duration>(chrono::duration<double>(static_cast<double>(in_.size()) / in_rate_));
            if (in_.empty() || std::abs(seconds(b.play_time - expected)) > 0.002)
            {
                // gap or jump in the main output: restart the timeline at this block
                if (!in_.empty())
                    aligned_ = false;
                in_.clear();
                in_time_ = b.play_time;
            }
            in_.insert(in_.end(), b.samples.begin(), b.samples.end());
            last_data_ = now;
        }

        if (pcm_ == nullptr)
        {
            if (!in_.empty() && soxr_ != nullptr && !openPcm())
            {
                in_.clear();
                std::this_thread::sleep_for(1s);
            }
            if (pcm_ == nullptr)
                continue;
        }
        if (now - last_data_ > IDLE_CLOSE)
        {
            closePcm();
            in_.clear();
            continue;
        }

        snd_pcm_sframes_t avail = snd_pcm_avail_update(pcm_);
        snd_pcm_sframes_t delay = 0;
        if (avail == -EPIPE || snd_pcm_delay(pcm_, &delay) == -EPIPE)
        {
            ++xruns_;
            snd_pcm_prepare(pcm_);
            aligned_ = false;
            continue;
        }
        if (avail < 0)
        {
            LOG(ERROR, LOG_TAG) << "avail: " << snd_strerror(static_cast<int>(avail)) << "\n";
            closePcm();
            continue;
        }
        if (snd_pcm_state(pcm_) != SND_PCM_STATE_RUNNING)
            delay = 0;

        if (!aligned_)
        {
            if (in_.empty())
                continue;
            realign(now, delay);
            continue;
        }

        // How far off is the frame we write next?
        double group = soxr_delay(soxr_) / settings_.rate;
        double t_out = static_cast<double>(delay) / settings_.rate;              // from now until heard on the reference
        double t_main = seconds(in_time_ - now) - group + seconds(settings_.offset); // same frame, wanted time
        double err = t_out - t_main;                                             // > 0: reference late
        if (std::abs(err) > seconds(settings_.max_error))
        {
            LOG(INFO, LOG_TAG) << "Error " << err * 1000 << " ms, realigning\n";
            aligned_ = false;
            continue;
        }
        err_avg_ += ERR_SMOOTH * (err - err_avg_);
        integral_ = std::clamp(integral_ + err_avg_ * 0.01, -MAX_CORR / KI, MAX_CORR / KI);
        ratio_corr_ = std::clamp(KP * err_avg_ + KI * integral_, -MAX_CORR, MAX_CORR);
        // late -> eat input faster
        soxr_set_io_ratio(soxr_, static_cast<double>(in_rate_) / settings_.rate * (1.0 + ratio_corr_), 0);

        // keep the device about half full
        auto target = static_cast<snd_pcm_sframes_t>(settings_.rate * seconds(settings_.buffer) / 2);
        snd_pcm_sframes_t want = std::min(avail, std::max<snd_pcm_sframes_t>(0, target - delay));
        if (want <= 0)
        {
            snd_pcm_wait(pcm_, 10);
            logStats(now);
            continue;
        }
        if (in_.empty())
        {
            // main output delivered nothing (yet): keep the device running, realign once data is back
            writeSilence(static_cast<size_t>(want));
            aligned_ = false;
            continue;
        }
        out.resize(static_cast<size_t>(want));
        size_t idone = 0, odone = 0;
        // only as much input as this block needs, so little waits inside the resampler
        size_t in_len = std::min(in_.size(), static_cast<size_t>(want * (static_cast<double>(in_rate_) / settings_.rate) * (1.0 + ratio_corr_)) + 16);
        soxr_error_t e = soxr_process(soxr_, in_.data(), in_len, &idone, out.data(), out.size(), &odone);
        if (e != nullptr)
        {
            LOG(ERROR, LOG_TAG) << "soxr_process: " << e << "\n";
            aligned_ = false;
            continue;
        }
        in_.erase(in_.begin(), in_.begin() + static_cast<long>(idone));
        in_time_ += chrono::duration_cast<clock::duration>(chrono::duration<double>(static_cast<double>(idone) / in_rate_));
        if (odone > 0)
            writeOutput(out, odone);
        logStats(now);
    }
}

} // namespace player
