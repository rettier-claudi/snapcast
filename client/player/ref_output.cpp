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
// Rate control: error in seconds -> relative step correction. Time constant ~10 s: a card
// 140 ppm off is caught, the jitter of the delay readings stays out.
constexpr double KP = 0.05;
constexpr double KI = 0.005;
constexpr double MAX_CORR = 1e-3;   // +-1000 ppm
constexpr double ERR_SMOOTH = 0.02; // EMA of the measured error per write
constexpr auto IDLE_CLOSE = 5s;     // like the main ALSA player: close after 5 s without data

// Resampler: windowed sinc, centred on the output position (no delay, needs TAPS/2 input ahead)
constexpr int TAPS = 64;
constexpr int PHASES = 512;
constexpr int HALF = TAPS / 2;

double seconds(chrono::steady_clock::duration d)
{
    return chrono::duration<double>(d).count();
}

chrono::steady_clock::duration duration(double s)
{
    return chrono::duration_cast<chrono::steady_clock::duration>(chrono::duration<double>(s));
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
        // 24 bit samples are carried in 32 bit (snapcast convention)
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
        while (queue_.size() > 400) // never more than ~2 s
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
        if (in_rate_ != 0)
            resetInput(in_rate_);
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


/// New input rate: build the kernel (low pass just below the reference's Nyquist) and drop the input.
void RefOutput::resetInput(uint32_t in_rate)
{
    in_rate_ = in_rate;
    in_.clear();
    pos_ = HALF;
    aligned_ = false;
    integral_ = 0;
    ratio_corr_ = 0;
    // cutoff 0.45 * output rate (or 0.45 * input rate when upsampling), relative to the input rate
    double fc = 2.0 * 0.45 * std::min(settings_.rate, in_rate_) / in_rate_;
    kernel_.assign(static_cast<size_t>(PHASES) * TAPS, 0.f);
    for (int ph = 0; ph < PHASES; ++ph)
    {
        double frac = static_cast<double>(ph) / PHASES;
        double sum = 0;
        for (int k = 0; k < TAPS; ++k)
        {
            double x = (k - (HALF - 1)) - frac; // distance from the output position, in input frames
            double sinc = (std::abs(x) < 1e-9) ? 1.0 : std::sin(M_PI * fc * x) / (M_PI * fc * x);
            double t = x / HALF;                 // -1..1 over the span
            double w = (std::abs(t) >= 1.0) ? 0.0 : 0.42 + 0.5 * std::cos(M_PI * t) + 0.08 * std::cos(2 * M_PI * t);
            kernel_[static_cast<size_t>(ph * TAPS + k)] = static_cast<float>(fc * sinc * w);
            sum += fc * sinc * w;
        }
        for (int k = 0; k < TAPS; ++k) // unity gain at DC for every phase
            kernel_[static_cast<size_t>(ph * TAPS + k)] = static_cast<float>(kernel_[static_cast<size_t>(ph * TAPS + k)] / sum);
    }
    LOG(DEBUG, LOG_TAG) << "Resampler " << in_rate_ << " -> " << settings_.rate << " Hz, " << TAPS << " taps, cutoff " << fc * in_rate_ / 2 << " Hz\n";
}


/// Output value at input position pos (needs in_[pos - HALF + 1 .. pos + HALF]).
float RefOutput::sample(double pos) const
{
    auto i = static_cast<long>(std::floor(pos));
    double frac = pos - static_cast<double>(i);
    int ph = std::min(PHASES - 1, static_cast<int>(frac * PHASES));
    const float* h = &kernel_[static_cast<size_t>(ph * TAPS)];
    const float* x = &in_[static_cast<size_t>(i - (HALF - 1))];
    float y = 0;
    for (int k = 0; k < TAPS; ++k)
        y += x[k] * h[k];
    return y;
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


/// Set the input position so that the next frame written is heard at main time + offset.
void RefOutput::realign(clock::time_point now, snd_pcm_sframes_t delay)
{
    ++realigns_;
    // keep the learned rate (integral_): the clocks did not change, only the position did
    err_avg_ = 0;
    aligned_at_ = now;
    double t_out = static_cast<double>(delay) / settings_.rate; // next written frame heard in t_out
    double t_in = seconds(in_time_ - now);                      // in_[0] heard on the main output in t_in
    double p = (t_out - seconds(settings_.offset) - t_in) * in_rate_;
    LOG(DEBUG, LOG_TAG) << "realign: out in " << t_out * 1000 << " ms, in_[0] heard in " << t_in * 1000 << " ms, input " << in_.size() * 1000. / in_rate_
                        << " ms, position " << p << "\n";
    if (p < HALF)
    {
        // that frame is still in the future of our input: wait with silence
        auto pad = static_cast<size_t>(std::ceil((HALF - p) * settings_.rate / in_rate_));
        if (pad > settings_.rate)
            return; // more than 1 s ahead: wait for the queue instead
        writeSilence(pad);
        p += static_cast<double>(pad) * in_rate_ / settings_.rate;
    }
    if (p + HALF >= static_cast<double>(in_.size()))
    {
        // behind our input: keep only the history and try again with the next blocks
        if (in_.size() > static_cast<size_t>(TAPS))
        {
            size_t n = in_.size() - TAPS;
            in_.erase(in_.begin(), in_.begin() + static_cast<long>(n));
            in_time_ += duration(static_cast<double>(n) / in_rate_);
        }
        return;
    }
    pos_ = p;
    aligned_ = true;
    fine_pending_ = (delay == 0); // its real delay shows once it runs: align once more then
    LOG(INFO, LOG_TAG) << "Aligned (position " << p << ", delay " << delay << ")\n";
}


void RefOutput::logStats(clock::time_point now)
{
    if (now - last_stats_ < 60s)
        return;
    last_stats_ = now;
    LOG(INFO, LOG_TAG) << "Stats: error " << err_avg_ * 1e6 << " us, rate " << ratio_corr_ * 1e6 << " ppm, realigns " << realigns_ << ", xruns " << xruns_
                       << "\n";
}


void RefOutput::worker()
{
    std::vector<float> out;
    while (active_)
    {
        std::deque<Block> blocks;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, aligned_ ? 2ms : 10ms, [this] { return !queue_.empty() || !active_; });
            blocks.swap(queue_);
        }
        auto now = clock::now();
        for (auto& b : blocks)
        {
            if (b.rate != in_rate_)
                resetInput(b.rate);
            auto expected = in_time_ + duration(static_cast<double>(in_.size()) / in_rate_);
            double dev = seconds(b.play_time - expected);
            if (in_.empty() || std::abs(dev) > 0.015 || now - last_data_ > 100ms)
            {
                // start, gap or real jump in the main output: restart the timeline at this block
                if (!in_.empty())
                    LOG(DEBUG, LOG_TAG) << "Main output jumped by " << dev * 1000 << " ms\n";
                in_.assign(HALF, 0.f); // history for the filter
                in_time_ = b.play_time - duration(static_cast<double>(HALF) / in_rate_);
                aligned_ = false;
            }
            else
            {
                // the block times come from the main device's delay readings and jitter by a few ms:
                // follow them slowly (time constant ~2 s at 20 ms blocks)
                in_time_ += duration(0.01 * dev);
            }
            in_.insert(in_.end(), b.samples.begin(), b.samples.end());
            last_data_ = now;
        }

        if (pcm_ == nullptr)
        {
            if (in_.size() > static_cast<size_t>(TAPS) && !openPcm())
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
            if (in_.size() > static_cast<size_t>(TAPS))
                realign(now, delay);
            if (!aligned_)
                continue;
        }

        // How far off is the frame we write next?  > 0: reference late
        double t_out = static_cast<double>(delay) / settings_.rate;
        double t_want = seconds(in_time_ - now) + pos_ / in_rate_ + seconds(settings_.offset);
        double err = t_out - t_want;
        if (fine_pending_ && snd_pcm_state(pcm_) == SND_PCM_STATE_RUNNING && delay > 0)
        {
            fine_pending_ = false;
            if (std::abs(err) > 0.0003)
            {
                LOG(INFO, LOG_TAG) << "Device running, error " << err * 1000 << " ms, aligning once more\n";
                aligned_ = false;
                continue;
            }
        }
        if (std::abs(err) > seconds(settings_.max_error))
        {
            LOG(INFO, LOG_TAG) << "Error " << err * 1000 << " ms, realigning\n";
            aligned_ = false;
            continue;
        }
        // first 20 s after aligning: 10x faster, to learn the clock difference within seconds
        const double gain = (now - aligned_at_ < 20s) ? 10.0 : 1.0;
        err_avg_ += ERR_SMOOTH * (err - err_avg_);
        integral_ = std::clamp(integral_ + gain * err_avg_ * 0.01, -MAX_CORR / KI, MAX_CORR / KI);
        ratio_corr_ = std::clamp(gain * KP * err_avg_ + KI * integral_, -MAX_CORR, MAX_CORR);
        const double step = static_cast<double>(in_rate_) / settings_.rate * (1.0 + ratio_corr_);

        // keep the device three quarters full
        auto target = static_cast<snd_pcm_sframes_t>(settings_.rate * seconds(settings_.buffer) * 3 / 4);
        snd_pcm_sframes_t want = std::min(avail, std::max<snd_pcm_sframes_t>(0, target - delay));
        if (want <= 0)
        {
            snd_pcm_wait(pcm_, 2);
            logStats(now);
            continue;
        }
        out.resize(static_cast<size_t>(want));
        size_t n = 0;
        while (n < out.size() && pos_ + HALF < static_cast<double>(in_.size()))
        {
            out[n++] = sample(pos_);
            pos_ += step;
        }
        if (n == 0)
        {
            // main output delivered nothing (yet): keep the device running; realign only after a real gap
            writeSilence(static_cast<size_t>(want));
            if (now - last_data_ > 200ms)
                aligned_ = false;
            continue;
        }
        writeOutput(out, n);
        // drop consumed input, keep the filter history
        auto consumed = static_cast<long>(std::floor(pos_)) - HALF;
        if (consumed > 4096)
        {
            in_.erase(in_.begin(), in_.begin() + consumed);
            pos_ -= static_cast<double>(consumed);
            in_time_ += duration(static_cast<double>(consumed) / in_rate_);
        }
        logStats(now);
    }
}

} // namespace player
