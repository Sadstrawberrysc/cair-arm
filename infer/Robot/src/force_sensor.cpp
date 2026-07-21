#include "force_sensor.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace {

constexpr std::uint8_t kHeaderFirst = 0xaa;
constexpr std::uint8_t kHeaderSecond = 0x55;

bool BaudRateToTermios(unsigned int baud_rate, speed_t* speed) {
    if (speed == nullptr) {
        return false;
    }
    switch (baud_rate) {
        case 4800: *speed = B4800; return true;
        case 9600: *speed = B9600; return true;
        case 19200: *speed = B19200; return true;
        case 38400: *speed = B38400; return true;
        case 57600: *speed = B57600; return true;
        case 115200: *speed = B115200; return true;
#ifdef B230400
        case 230400: *speed = B230400; return true;
#endif
#ifdef B1000000
        case 1000000: *speed = B1000000; return true;
#endif
#ifdef B1152000
        case 1152000: *speed = B1152000; return true;
#endif
#ifdef B3000000
        case 3000000: *speed = B3000000; return true;
#endif
        default: return false;
    }
}

bool ConfigureSerialPort(int fd, unsigned int baud_rate, int* error_number) {
    speed_t speed = B0;
    if (!BaudRateToTermios(baud_rate, &speed)) {
        if (error_number != nullptr) {
            *error_number = EINVAL;
        }
        return false;
    }

    termios options{};
    if (::tcgetattr(fd, &options) != 0) {
        if (error_number != nullptr) {
            *error_number = errno;
        }
        return false;
    }

    ::cfmakeraw(&options);
    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    options.c_cflag |= CS8;
    options.c_cflag &= static_cast<tcflag_t>(~PARENB);
    options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (::cfsetispeed(&options, speed) != 0 ||
        ::cfsetospeed(&options, speed) != 0 ||
        ::tcsetattr(fd, TCSANOW, &options) != 0) {
        if (error_number != nullptr) {
            *error_number = errno;
        }
        return false;
    }

    // Discard bytes which predate this process. New bytes are resynchronised by
    // ForceSensorFrameParser, so a flush failure is not fatal.
    (void)::tcflush(fd, TCIFLUSH);
    return true;
}

int OpenConfiguredSerialPort(const std::string& device,
                             unsigned int baud_rate,
                             int* error_number) {
    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK |
                                             O_CLOEXEC);
    if (fd < 0) {
        if (error_number != nullptr) {
            *error_number = errno;
        }
        return -1;
    }
    if (::ioctl(fd, TIOCEXCL) != 0) {
        if (error_number != nullptr) {
            *error_number = errno;
        }
        (void)::close(fd);
        return -1;
    }
    if (!ConfigureSerialPort(fd, baud_rate, error_number)) {
        (void)::close(fd);
        return -1;
    }
    return fd;
}

std::string ErrorMessage(const std::string& operation, int error_number) {
    std::ostringstream stream;
    stream << operation;
    if (error_number != 0) {
        stream << ": " << std::strerror(error_number)
               << " (errno=" << error_number << ')';
    }
    return stream.str();
}

float DecodeLittleEndianFloat(const std::uint8_t* bytes) noexcept {
    static_assert(sizeof(float) == sizeof(std::uint32_t),
                  "force sensor protocol requires 32-bit float");
    const std::uint32_t bits =
        static_cast<std::uint32_t>(bytes[0]) |
        (static_cast<std::uint32_t>(bytes[1]) << 8U) |
        (static_cast<std::uint32_t>(bytes[2]) << 16U) |
        (static_cast<std::uint32_t>(bytes[3]) << 24U);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace

const char* ForceSensorIoStatusName(ForceSensorIoStatus status) noexcept {
    switch (status) {
        case ForceSensorIoStatus::kDisconnected: return "disconnected";
        case ForceSensorIoStatus::kOpening: return "opening";
        case ForceSensorIoStatus::kStreaming: return "streaming";
        case ForceSensorIoStatus::kStopped: return "stopped";
        case ForceSensorIoStatus::kIoError: return "io_error";
    }
    return "unknown";
}

const char* ForceSensorProtocolName(ForceSensorProtocol protocol) noexcept {
    switch (protocol) {
        case ForceSensorProtocol::kHaptronModbusRtu:
            return "haptron-modbus";
        case ForceSensorProtocol::kLegacyAa55Stream:
            return "legacy-aa55";
    }
    return "unknown";
}

bool WrenchSample::IsStale(std::chrono::milliseconds maximum_age) const
    noexcept {
    return IsStaleAt(std::chrono::steady_clock::now(), maximum_age);
}

bool WrenchSample::IsStaleAt(
    std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds maximum_age) const noexcept {
    if (!valid || io_error != 0 ||
        io_status != ForceSensorIoStatus::kStreaming ||
        monotonic_timestamp == std::chrono::steady_clock::time_point{}) {
        return true;
    }
    if (maximum_age.count() < 0 || now < monotonic_timestamp) {
        return true;
    }
    return now - monotonic_timestamp > maximum_age;
}

ForceSensorFrameParser::ForceSensorFrameParser(
    std::size_t maximum_buffer_bytes)
    : maximum_buffer_bytes_(
          std::max(maximum_buffer_bytes, ForceSensorFrameParser::kFrameSize)) {
    receive_buffer_.reserve(maximum_buffer_bytes_);
}

std::vector<WrenchSample> ForceSensorFrameParser::Feed(
    const std::uint8_t* data,
    std::size_t length) {
    return Feed(data, length, std::chrono::system_clock::now(),
                std::chrono::steady_clock::now());
}

std::vector<WrenchSample> ForceSensorFrameParser::Feed(
    const std::uint8_t* data,
    std::size_t length,
    std::chrono::system_clock::time_point timestamp,
    std::chrono::steady_clock::time_point monotonic_timestamp) {
    std::vector<WrenchSample> samples;
    if (length == 0) {
        UpdateBufferStatistics();
        return samples;
    }
    if (data == nullptr) {
        ++statistics_.buffer_overflow_events;
        statistics_.buffer_bytes_discarded += length;
        return samples;
    }

    statistics_.bytes_received += length;
    std::size_t offset = 0;
    while (offset < length) {
        // Parse between bounded appends. This keeps receive_buffer_ at or below
        // the configured cap even when Feed is handed a very large read.
        if (receive_buffer_.size() >= maximum_buffer_bytes_) {
            ParseAvailable(timestamp, monotonic_timestamp, &samples);
        }
        if (receive_buffer_.size() >= maximum_buffer_bytes_) {
            // Defensive fallback: ParseAvailable normally leaves fewer than 31
            // bytes. Preserve a possible first header byte while making room.
            const bool keep_header_prefix = receive_buffer_.back() == kHeaderFirst;
            const std::size_t keep = keep_header_prefix ? 1U : 0U;
            const std::size_t discarded = receive_buffer_.size() - keep;
            if (keep_header_prefix) {
                receive_buffer_[0] = receive_buffer_.back();
            }
            receive_buffer_.resize(keep);
            ++statistics_.buffer_overflow_events;
            statistics_.buffer_bytes_discarded += discarded;
            statistics_.noise_bytes_discarded += discarded;
        }

        const std::size_t available = maximum_buffer_bytes_ - receive_buffer_.size();
        const std::size_t append_count = std::min(available, length - offset);
        receive_buffer_.insert(receive_buffer_.end(), data + offset,
                               data + offset + append_count);
        offset += append_count;
        UpdateBufferStatistics();
        ParseAvailable(timestamp, monotonic_timestamp, &samples);
    }
    UpdateBufferStatistics();
    return samples;
}

void ForceSensorFrameParser::ParseAvailable(
    std::chrono::system_clock::time_point timestamp,
    std::chrono::steady_clock::time_point monotonic_timestamp,
    std::vector<WrenchSample>* samples) {
    if (samples == nullptr) {
        return;
    }

    const std::array<std::uint8_t, 2> header{{kHeaderFirst, kHeaderSecond}};
    while (!receive_buffer_.empty()) {
        const auto header_position =
            std::search(receive_buffer_.begin(), receive_buffer_.end(),
                        header.begin(), header.end());
        if (header_position == receive_buffer_.end()) {
            // Keep a trailing AA because the next Feed may begin with 55.
            const std::size_t keep =
                receive_buffer_.back() == kHeaderFirst ? 1U : 0U;
            const std::size_t discarded = receive_buffer_.size() - keep;
            if (keep != 0U) {
                receive_buffer_[0] = receive_buffer_.back();
            }
            receive_buffer_.resize(keep);
            statistics_.noise_bytes_discarded += discarded;
            break;
        }

        const std::size_t prefix_bytes = static_cast<std::size_t>(
            std::distance(receive_buffer_.begin(), header_position));
        if (prefix_bytes != 0U) {
            receive_buffer_.erase(receive_buffer_.begin(), header_position);
            statistics_.noise_bytes_discarded += prefix_bytes;
        }
        if (receive_buffer_.size() < kFrameSize) {
            break;
        }

        const std::uint8_t calculated_checksum = ComputeChecksum(
            receive_buffer_.data() + kPayloadOffset, kPayloadSize);
        if (calculated_checksum != receive_buffer_[kChecksumOffset]) {
            ++statistics_.checksum_errors;
            ++statistics_.noise_bytes_discarded;
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }

        WrenchSample sample;
        sample.sequence = next_sequence_;
        sample.timestamp = timestamp;
        sample.monotonic_timestamp = monotonic_timestamp;
        sample.checksum_valid = true;
        sample.io_status = ForceSensorIoStatus::kStreaming;
        bool finite = true;
        for (std::size_t index = 0; index < sample.wrench_si.size(); ++index) {
            const float value = DecodeLittleEndianFloat(
                receive_buffer_.data() + kPayloadOffset + index * sizeof(float));
            sample.wrench_si[index] = static_cast<double>(value);
            finite = finite && std::isfinite(value);
        }
        if (finite) {
            sample.valid = true;
            sample.stale = false;
            samples->push_back(sample);
            ++statistics_.valid_frames;
            ++next_sequence_;
        } else {
            ++statistics_.numeric_errors;
        }
        receive_buffer_.erase(receive_buffer_.begin(),
                              receive_buffer_.begin() + kFrameSize);
    }
    UpdateBufferStatistics();
}

void ForceSensorFrameParser::Reset() {
    receive_buffer_.clear();
    statistics_ = ForceSensorStatistics{};
    next_sequence_ = 1;
}

ForceSensorStatistics ForceSensorFrameParser::statistics() const noexcept {
    ForceSensorStatistics result = statistics_;
    result.buffered_bytes = receive_buffer_.size();
    return result;
}

std::size_t ForceSensorFrameParser::buffered_bytes() const noexcept {
    return receive_buffer_.size();
}

std::size_t ForceSensorFrameParser::maximum_buffer_bytes() const noexcept {
    return maximum_buffer_bytes_;
}

std::uint8_t ForceSensorFrameParser::ComputeChecksum(
    const std::uint8_t* data,
    std::size_t length) noexcept {
    std::uint8_t checksum = 0;
    if (data == nullptr) {
        return checksum;
    }
    for (std::size_t index = 0; index < length; ++index) {
        checksum = static_cast<std::uint8_t>(checksum + data[index]);
    }
    return checksum;
}

void ForceSensorFrameParser::UpdateBufferStatistics() noexcept {
    statistics_.buffered_bytes = receive_buffer_.size();
    statistics_.peak_buffered_bytes =
        std::max(statistics_.peak_buffered_bytes, receive_buffer_.size());
}

ForceSensorReader::ForceSensorReader(ForceSensorConfig config)
    : config_(std::move(config)),
      legacy_parser_(config_.maximum_buffer_bytes),
      modbus_parser_(config_.modbus_slave_address,
                     config_.maximum_buffer_bytes) {
    if (config_.device.empty()) {
        config_.device = "/dev/ttyUSB0";
    }
    if (config_.read_chunk_bytes == 0) {
        config_.read_chunk_bytes = 512;
    }
    if (config_.poll_timeout.count() <= 0) {
        config_.poll_timeout = std::chrono::milliseconds(50);
    }
    if (config_.query_period.count() <= 0) {
        config_.query_period = std::chrono::milliseconds(20);
    }
    if (config_.response_timeout.count() <= 0) {
        config_.response_timeout = std::chrono::milliseconds(50);
    }
    if (config_.modbus_slave_address == 0
        || config_.modbus_slave_address > 247) {
        config_.modbus_slave_address = kHaptronDefaultSlaveAddress;
        modbus_parser_ = HaptronModbusResponseParser(
            config_.modbus_slave_address, config_.maximum_buffer_bytes);
    }
    latest_sample_.io_status = ForceSensorIoStatus::kDisconnected;
    statistics_.io_status = ForceSensorIoStatus::kDisconnected;
}

ForceSensorReader::~ForceSensorReader() {
    Stop();
}

bool ForceSensorReader::Start() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (running_.load()) {
        return true;
    }
    if (worker_.joinable()) {
        worker_.join();
    }

    stop_requested_.store(false);
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        latest_sample_ = WrenchSample{};
        statistics_ = ForceSensorStatistics{};
        last_error_.clear();
    }
    PublishIoStatus(ForceSensorIoStatus::kOpening, 0, std::string{});
    int error_number = 0;
    const int fd =
        OpenConfiguredSerialPort(config_.device, config_.baud_rate, &error_number);
    if (fd < 0) {
        PublishIoStatus(ForceSensorIoStatus::kIoError, error_number,
                        ErrorMessage("open " + config_.device, error_number));
        return false;
    }

    legacy_parser_.Reset();
    modbus_parser_.Reset();
    serial_fd_.store(fd);
    running_.store(true);
    PublishIoStatus(ForceSensorIoStatus::kStreaming, 0, std::string{});
    try {
        worker_ = std::thread(&ForceSensorReader::WorkerLoop, this);
    } catch (...) {
        running_.store(false);
        ClosePort();
        PublishIoStatus(ForceSensorIoStatus::kIoError, EAGAIN,
                        "failed to start force sensor reader thread");
        return false;
    }
    return true;
}

void ForceSensorReader::Stop() noexcept {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    stop_requested_.store(true);
    if (worker_.joinable()) {
        worker_.join();
    }
    ClosePort();
    running_.store(false);
    // Do not erase the last valid observation; mark it stale/stopped so a
    // caller can diagnose what was last seen without treating it as live data.
    PublishIoStatus(ForceSensorIoStatus::kStopped, 0, std::string{});
}

bool ForceSensorReader::IsRunning() const noexcept {
    return running_.load();
}

bool ForceSensorReader::IsOpen() const noexcept {
    return serial_fd_.load() >= 0;
}

WrenchSample ForceSensorReader::LatestSample() const {
    WrenchSample result;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        result = latest_sample_;
    }
    result.stale = result.IsStale(config_.stale_after);
    return result;
}

ForceSensorStatistics ForceSensorReader::Statistics() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return statistics_;
}

std::string ForceSensorReader::LastError() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return last_error_;
}

const ForceSensorConfig& ForceSensorReader::config() const noexcept {
    return config_;
}

void ForceSensorReader::WorkerLoop() {
    if (config_.protocol == ForceSensorProtocol::kHaptronModbusRtu) {
        HaptronModbusWorkerLoop();
    } else {
        LegacyStreamWorkerLoop();
    }

    ClosePort();
    running_.store(false);
    if (stop_requested_.load()) {
        PublishIoStatus(ForceSensorIoStatus::kStopped, 0, std::string{});
    }
}

void ForceSensorReader::LegacyStreamWorkerLoop() {
    std::vector<std::uint8_t> read_buffer(config_.read_chunk_bytes);
    // Cap the poll duration so Stop() remains responsive even if a caller uses
    // an accidentally large timeout.
    const int poll_timeout_ms = static_cast<int>(std::min<std::int64_t>(
        std::max<std::int64_t>(1, config_.poll_timeout.count()), 100));

    while (!stop_requested_.load()) {
        const int fd = serial_fd_.load();
        if (fd < 0) {
            break;
        }
        pollfd descriptor{};
        descriptor.fd = fd;
        descriptor.events = POLLIN;
        const int poll_result = ::poll(&descriptor, 1, poll_timeout_ms);
        if (poll_result == 0) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ++statistics_.read_timeouts;
            continue;
        }
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            const int error_number = errno;
            PublishIoStatus(ForceSensorIoStatus::kIoError, error_number,
                            ErrorMessage("poll force sensor", error_number));
            break;
        }
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            PublishIoStatus(ForceSensorIoStatus::kIoError, EIO,
                            "force sensor serial device disconnected");
            break;
        }
        if ((descriptor.revents & POLLIN) == 0) {
            continue;
        }

        const ssize_t count = ::read(fd, read_buffer.data(), read_buffer.size());
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ++statistics_.read_calls;
        }
        if (count > 0) {
            const auto samples = legacy_parser_.Feed(
                read_buffer.data(), static_cast<std::size_t>(count));
            const ForceSensorStatistics parser_statistics =
                legacy_parser_.statistics();
            std::lock_guard<std::mutex> lock(state_mutex_);
            CopyParserStatistics(parser_statistics);
            for (const WrenchSample& sample : samples) {
                latest_sample_ = sample;
                latest_sample_.io_status = ForceSensorIoStatus::kStreaming;
                latest_sample_.io_error = 0;
                latest_sample_.stale = false;
            }
            continue;
        }
        if (count == 0) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            continue;
        }
        const int error_number = errno;
        PublishIoStatus(ForceSensorIoStatus::kIoError, error_number,
                        ErrorMessage("read force sensor", error_number));
        break;
    }

}

void ForceSensorReader::HaptronModbusWorkerLoop() {
    const auto request =
        BuildHaptronReadRequest(config_.modbus_slave_address);
    std::vector<std::uint8_t> read_buffer(config_.read_chunk_bytes);
    std::uint64_t next_sequence = 1;
    auto next_query_at = std::chrono::steady_clock::now();

    while (!stop_requested_.load()) {
        const int fd = serial_fd_.load();
        if (fd < 0) break;

        while (!stop_requested_.load()
               && std::chrono::steady_clock::now() < next_query_at) {
            const auto remaining = std::chrono::duration_cast<
                std::chrono::milliseconds>(next_query_at
                                            - std::chrono::steady_clock::now());
            const auto sleep_time = std::min(remaining,
                                              std::chrono::milliseconds(5));
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
        if (stop_requested_.load()) break;

        const auto query_started_at = std::chrono::steady_clock::now();
        const auto response_deadline =
            query_started_at + config_.response_timeout;
        std::size_t written = 0;
        while (written < request.size() && !stop_requested_.load()) {
            pollfd descriptor{};
            descriptor.fd = fd;
            descriptor.events = POLLOUT;
            const int poll_result = ::poll(&descriptor, 1, 10);
            if (poll_result < 0) {
                if (errno == EINTR) continue;
                const int error_number = errno;
                PublishIoStatus(ForceSensorIoStatus::kIoError, error_number,
                                ErrorMessage("poll Haptron request",
                                             error_number));
                return;
            }
            if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                PublishIoStatus(ForceSensorIoStatus::kIoError, EIO,
                                "Haptron serial device disconnected while writing");
                return;
            }
            if (poll_result == 0 || (descriptor.revents & POLLOUT) == 0) {
                if (std::chrono::steady_clock::now() >= response_deadline) {
                    PublishIoStatus(ForceSensorIoStatus::kIoError, ETIMEDOUT,
                                    "Haptron Modbus request write timeout");
                    return;
                }
                continue;
            }
            const ssize_t count = ::write(fd, request.data() + written,
                                           request.size() - written);
            if (count > 0) {
                written += static_cast<std::size_t>(count);
            } else if (count < 0 && errno != EINTR && errno != EAGAIN
                       && errno != EWOULDBLOCK) {
                const int error_number = errno;
                PublishIoStatus(ForceSensorIoStatus::kIoError, error_number,
                                ErrorMessage("write Haptron request",
                                             error_number));
                return;
            }
        }
        if (stop_requested_.load()) break;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ++statistics_.requests_sent;
        }

        bool response_received = false;
        while (!stop_requested_.load()
               && std::chrono::steady_clock::now() < response_deadline) {
            const auto remaining = std::chrono::duration_cast<
                std::chrono::milliseconds>(response_deadline
                                            - std::chrono::steady_clock::now());
            const int timeout_ms = static_cast<int>(std::max<std::int64_t>(
                1, std::min<std::int64_t>(10, remaining.count())));
            pollfd descriptor{};
            descriptor.fd = fd;
            descriptor.events = POLLIN;
            const int poll_result = ::poll(&descriptor, 1, timeout_ms);
            if (poll_result == 0) continue;
            if (poll_result < 0) {
                if (errno == EINTR) continue;
                const int error_number = errno;
                PublishIoStatus(ForceSensorIoStatus::kIoError, error_number,
                                ErrorMessage("poll Haptron response",
                                             error_number));
                return;
            }
            if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                PublishIoStatus(ForceSensorIoStatus::kIoError, EIO,
                                "Haptron serial device disconnected while reading");
                return;
            }
            if ((descriptor.revents & POLLIN) == 0) continue;

            const ssize_t count =
                ::read(fd, read_buffer.data(), read_buffer.size());
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ++statistics_.read_calls;
            }
            if (count > 0) {
                const auto results = modbus_parser_.Feed(
                    read_buffer.data(), static_cast<std::size_t>(count));
                const HaptronModbusStatistics parser_statistics =
                    modbus_parser_.statistics();
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    CopyModbusStatistics(parser_statistics);
                }
                for (const HaptronModbusResult& result : results) {
                    if (result.exception) {
                        PublishIoStatus(
                            ForceSensorIoStatus::kIoError, EPROTO,
                            "Haptron Modbus exception code="
                                + std::to_string(result.exception_code));
                        return;
                    }
                    if (!result.valid) continue;
                    WrenchSample sample;
                    sample.wrench_si = result.wrench_si;
                    sample.sequence = next_sequence++;
                    sample.timestamp = std::chrono::system_clock::now();
                    sample.monotonic_timestamp =
                        std::chrono::steady_clock::now();
                    sample.checksum_valid = true;
                    sample.valid = true;
                    sample.stale = false;
                    sample.io_status = ForceSensorIoStatus::kStreaming;
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    latest_sample_ = sample;
                    response_received = true;
                }
                if (response_received) break;
                continue;
            }
            if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK
                || errno == EINTR) {
                continue;
            }
            const int error_number = errno;
            PublishIoStatus(ForceSensorIoStatus::kIoError, error_number,
                            ErrorMessage("read Haptron response",
                                         error_number));
            return;
        }

        if (!response_received && !stop_requested_.load()) {
            // A partial or late reply must not be completed and timestamped as
            // the answer to the next request (Modbus RTU has no transaction
            // identifier). Discard only buffered/driver input; keep cumulative
            // parser statistics for diagnosis.
            modbus_parser_.DiscardBufferedData();
            (void)::tcflush(fd, TCIFLUSH);
            const HaptronModbusStatistics parser_statistics =
                modbus_parser_.statistics();
            std::lock_guard<std::mutex> lock(state_mutex_);
            CopyModbusStatistics(parser_statistics);
            ++statistics_.read_timeouts;
            ++statistics_.response_timeouts;
        }
        next_query_at = std::max(query_started_at + config_.query_period,
                                 std::chrono::steady_clock::now());
    }
}

void ForceSensorReader::PublishIoStatus(ForceSensorIoStatus status,
                                        int error_number,
                                        const std::string& message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    statistics_.io_status = status;
    latest_sample_.io_status = status;
    // A normal Stop after an I/O failure must not erase the errno which
    // explains why the worker exited. A subsequent Start resets the snapshot.
    if (status != ForceSensorIoStatus::kStopped || error_number != 0 ||
        latest_sample_.io_error == 0) {
        latest_sample_.io_error = error_number;
    }
    if (status != ForceSensorIoStatus::kStreaming) {
        latest_sample_.stale = true;
    }
    if (status == ForceSensorIoStatus::kIoError) {
        ++statistics_.io_errors;
    }
    if (!message.empty()) {
        last_error_ = message;
    } else if (status == ForceSensorIoStatus::kStreaming) {
        last_error_.clear();
    }
}

void ForceSensorReader::CopyParserStatistics(
    const ForceSensorStatistics& parser_statistics) {
    statistics_.bytes_received = parser_statistics.bytes_received;
    statistics_.valid_frames = parser_statistics.valid_frames;
    statistics_.checksum_errors = parser_statistics.checksum_errors;
    statistics_.numeric_errors = parser_statistics.numeric_errors;
    statistics_.noise_bytes_discarded =
        parser_statistics.noise_bytes_discarded;
    statistics_.buffer_overflow_events =
        parser_statistics.buffer_overflow_events;
    statistics_.buffer_bytes_discarded =
        parser_statistics.buffer_bytes_discarded;
    statistics_.buffered_bytes = parser_statistics.buffered_bytes;
    statistics_.peak_buffered_bytes = parser_statistics.peak_buffered_bytes;
}

void ForceSensorReader::CopyModbusStatistics(
    const HaptronModbusStatistics& parser_statistics) {
    statistics_.bytes_received = parser_statistics.bytes_received;
    statistics_.valid_frames = parser_statistics.valid_responses;
    statistics_.checksum_errors = parser_statistics.crc_errors;
    statistics_.numeric_errors = parser_statistics.numeric_errors;
    statistics_.noise_bytes_discarded =
        parser_statistics.noise_bytes_discarded;
    statistics_.buffer_overflow_events =
        parser_statistics.buffer_overflow_events;
    statistics_.buffered_bytes = parser_statistics.buffered_bytes;
    statistics_.peak_buffered_bytes = parser_statistics.peak_buffered_bytes;
    statistics_.exception_responses =
        parser_statistics.exception_responses;
    statistics_.protocol_errors = parser_statistics.protocol_errors;
}

void ForceSensorReader::ClosePort() noexcept {
    const int fd = serial_fd_.exchange(-1);
    if (fd >= 0) {
        (void)::close(fd);
    }
}

CLinuxSerial::CLinuxSerial() = default;

CLinuxSerial::CLinuxSerial(UINT portNo, UINT baudRate) {
    (void)InitPort(portNo, baudRate);
}

CLinuxSerial::CLinuxSerial(const std::string& device, UINT baudRate) {
    (void)InitPort(device, baudRate);
}

CLinuxSerial::~CLinuxSerial() {
    ClosePort();
}

void CLinuxSerial::ClosePort() noexcept {
    if (m_iSerialID >= 0) {
        (void)::close(m_iSerialID);
        m_iSerialID = -1;
    }
}

bool CLinuxSerial::IsOpen() const noexcept {
    return m_iSerialID >= 0;
}

bool CLinuxSerial::InitPort(UINT portNo, UINT baudRate) {
    return InitPort("/dev/ttyUSB" + std::to_string(portNo), baudRate);
}

bool CLinuxSerial::InitPort(const std::string& device, UINT baudRate) {
    ClosePort();
    parser_.Reset();
    int error_number = 0;
    const int fd = OpenConfiguredSerialPort(device, baudRate, &error_number);
    if (fd < 0) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_sample_.io_status = ForceSensorIoStatus::kIoError;
        latest_sample_.io_error = error_number;
        latest_sample_.stale = true;
        return false;
    }
    m_iSerialID = fd;
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_sample_ = WrenchSample{};
    latest_sample_.io_status = ForceSensorIoStatus::kStreaming;
    return true;
}

UINT CLinuxSerial::ReadData(UCHAR* data, UINT length) {
    if (!IsOpen() || data == nullptr || length == 0) {
        return 0;
    }
    const ssize_t result = ::read(m_iSerialID, data, length);
    return result > 0 ? static_cast<UINT>(result) : 0;
}

UINT CLinuxSerial::WriteData(UCHAR* data, UINT length) {
    if (!IsOpen() || data == nullptr || length == 0) {
        return 0;
    }
    const ssize_t result = ::write(m_iSerialID, data, length);
    return result > 0 ? static_cast<UINT>(result) : 0;
}

UINT CLinuxSerial::GetBytesInCom() {
    if (!IsOpen()) {
        return 0;
    }
    int available = 0;
    if (::ioctl(m_iSerialID, FIONREAD, &available) != 0 || available <= 0) {
        return 0;
    }
    return static_cast<UINT>(available);
}

unsigned char CLinuxSerial::CheckSum(unsigned char* buf, const int len) {
    return CheckSum(static_cast<const unsigned char*>(buf), len);
}

unsigned char CLinuxSerial::CheckSum(const unsigned char* buf,
                                     const int len) const {
    if (len <= 0) {
        return 0;
    }
    return ForceSensorFrameParser::ComputeChecksum(
        reinterpret_cast<const std::uint8_t*>(buf),
        static_cast<std::size_t>(len));
}

void CLinuxSerial::ProcessSensorData() {
    if (!IsOpen()) {
        return;
    }
    std::array<std::uint8_t, 512> bytes{};
    const UINT count = ReadData(bytes.data(), static_cast<UINT>(bytes.size()));
    if (count == 0) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto samples = parser_.Feed(bytes.data(), count);
    if (samples.empty()) {
        return;
    }

    const WrenchSample& sample = samples.back();
    latest_sample_ = sample;
    for (std::size_t index = 0; index < sample.wrench_si.size(); ++index) {
        sensor[index] = static_cast<float>(sample.wrench_si[index]);
    }
}

WrenchSample CLinuxSerial::LatestSample(
    std::chrono::milliseconds stale_after) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    WrenchSample result = latest_sample_;
    result.stale = result.IsStale(stale_after);
    return result;
}

ForceSensorStatistics CLinuxSerial::Statistics() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return parser_.statistics();
}

// Haptron Modbus RTU protocol implementation. Kept in the sensor translation
// unit so the runtime follows the original project's single sensor module.
#include "haptron_modbus.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {

float DecodeBigEndianFloat(const std::uint8_t* bytes) noexcept {
    static_assert(sizeof(float) == sizeof(std::uint32_t),
                  "Haptron protocol requires 32-bit float");
    const std::uint32_t bits =
        (static_cast<std::uint32_t>(bytes[0]) << 24U)
        | (static_cast<std::uint32_t>(bytes[1]) << 16U)
        | (static_cast<std::uint32_t>(bytes[2]) << 8U)
        | static_cast<std::uint32_t>(bytes[3]);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool HasValidCrc(const std::uint8_t* frame, std::size_t length) noexcept {
    if (frame == nullptr || length < 3) return false;
    const std::uint16_t crc = HaptronModbusCrc16(frame, length - 2);
    return frame[length - 2] == static_cast<std::uint8_t>(crc & 0xFFU)
           && frame[length - 1]
                  == static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
}

}  // namespace

std::uint16_t HaptronModbusCrc16(const std::uint8_t* data,
                                 std::size_t length) noexcept {
    if (data == nullptr && length != 0) return 0;
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc = static_cast<std::uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc = static_cast<std::uint16_t>(crc >> 1U);
            }
        }
    }
    return crc;
}

std::array<std::uint8_t, kHaptronRequestSize> BuildHaptronReadRequest(
    std::uint8_t slave_address) noexcept {
    std::array<std::uint8_t, kHaptronRequestSize> request{{
        slave_address,
        kHaptronReadInputRegisters,
        static_cast<std::uint8_t>((kHaptronStartRegister >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(kHaptronStartRegister & 0xFFU),
        static_cast<std::uint8_t>((kHaptronRegisterCount >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(kHaptronRegisterCount & 0xFFU),
        0,
        0,
    }};
    const std::uint16_t crc = HaptronModbusCrc16(request.data(), 6);
    request[6] = static_cast<std::uint8_t>(crc & 0xFFU);
    request[7] = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
    return request;
}

HaptronModbusResponseParser::HaptronModbusResponseParser(
    std::uint8_t slave_address,
    std::size_t maximum_buffer_bytes)
    : slave_address_(slave_address),
      maximum_buffer_bytes_(std::max(maximum_buffer_bytes,
                                     kHaptronResponseSize)) {
    receive_buffer_.reserve(maximum_buffer_bytes_);
}

std::vector<HaptronModbusResult> HaptronModbusResponseParser::Feed(
    const std::uint8_t* data,
    std::size_t length) {
    std::vector<HaptronModbusResult> results;
    if (data == nullptr || length == 0) return results;

    statistics_.bytes_received += length;
    if (length >= maximum_buffer_bytes_) {
        const bool overflowed = !receive_buffer_.empty()
                                || length > maximum_buffer_bytes_;
        statistics_.noise_bytes_discarded +=
            receive_buffer_.size() + (length - maximum_buffer_bytes_);
        receive_buffer_.assign(data + (length - maximum_buffer_bytes_),
                               data + length);
        if (overflowed) ++statistics_.buffer_overflow_events;
    } else {
        const std::size_t required = receive_buffer_.size() + length;
        if (required > maximum_buffer_bytes_) {
            const std::size_t discard = required - maximum_buffer_bytes_;
            statistics_.noise_bytes_discarded += discard;
            receive_buffer_.erase(receive_buffer_.begin(),
                                  receive_buffer_.begin()
                                      + static_cast<std::ptrdiff_t>(discard));
            ++statistics_.buffer_overflow_events;
        }
        receive_buffer_.insert(receive_buffer_.end(), data, data + length);
    }

    UpdateBufferStatistics();
    ParseAvailable(&results);
    UpdateBufferStatistics();
    return results;
}

void HaptronModbusResponseParser::Reset() {
    receive_buffer_.clear();
    statistics_ = HaptronModbusStatistics{};
}

void HaptronModbusResponseParser::DiscardBufferedData() noexcept {
    statistics_.noise_bytes_discarded += receive_buffer_.size();
    receive_buffer_.clear();
    UpdateBufferStatistics();
}

HaptronModbusStatistics HaptronModbusResponseParser::statistics() const noexcept {
    HaptronModbusStatistics result = statistics_;
    result.buffered_bytes = receive_buffer_.size();
    return result;
}

std::size_t HaptronModbusResponseParser::buffered_bytes() const noexcept {
    return receive_buffer_.size();
}

void HaptronModbusResponseParser::ParseAvailable(
    std::vector<HaptronModbusResult>* results) {
    if (results == nullptr) return;

    while (!receive_buffer_.empty()) {
        auto candidate = std::find(receive_buffer_.begin(),
                                   receive_buffer_.end(), slave_address_);
        if (candidate == receive_buffer_.end()) {
            statistics_.noise_bytes_discarded += receive_buffer_.size();
            receive_buffer_.clear();
            return;
        }

        const std::size_t prefix = static_cast<std::size_t>(
            std::distance(receive_buffer_.begin(), candidate));
        if (prefix != 0) {
            statistics_.noise_bytes_discarded += prefix;
            receive_buffer_.erase(receive_buffer_.begin(), candidate);
        }
        if (receive_buffer_.size() < 2) return;

        const std::uint8_t function = receive_buffer_[1];
        if (function == static_cast<std::uint8_t>(
                            kHaptronReadInputRegisters | 0x80U)) {
            constexpr std::size_t kExceptionFrameSize = 5;
            if (receive_buffer_.size() < kExceptionFrameSize) return;
            if (!HasValidCrc(receive_buffer_.data(), kExceptionFrameSize)) {
                ++statistics_.crc_errors;
                ++statistics_.noise_bytes_discarded;
                receive_buffer_.erase(receive_buffer_.begin());
                continue;
            }
            HaptronModbusResult result;
            result.exception = true;
            result.exception_code = receive_buffer_[2];
            result.error = "modbus_exception_"
                           + std::to_string(result.exception_code);
            results->push_back(result);
            ++statistics_.exception_responses;
            receive_buffer_.erase(receive_buffer_.begin(),
                                  receive_buffer_.begin()
                                      + kExceptionFrameSize);
            continue;
        }

        if (function != kHaptronReadInputRegisters) {
            ++statistics_.protocol_errors;
            ++statistics_.noise_bytes_discarded;
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }
        if (receive_buffer_.size() < 3) return;
        if (receive_buffer_[2] != kHaptronPayloadSize) {
            // This also rejects an echoed request, whose third byte is 0x00.
            ++statistics_.protocol_errors;
            ++statistics_.noise_bytes_discarded;
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }
        if (receive_buffer_.size() < kHaptronResponseSize) return;
        if (!HasValidCrc(receive_buffer_.data(), kHaptronResponseSize)) {
            ++statistics_.crc_errors;
            ++statistics_.noise_bytes_discarded;
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }

        HaptronModbusResult result;
        bool finite = true;
        for (std::size_t axis = 0; axis < result.wrench_si.size(); ++axis) {
            const float value = DecodeBigEndianFloat(
                receive_buffer_.data() + 3 + axis * sizeof(float));
            result.wrench_si[axis] = static_cast<double>(value);
            finite = finite && std::isfinite(value);
        }
        if (finite) {
            result.valid = true;
            results->push_back(result);
            ++statistics_.valid_responses;
        } else {
            ++statistics_.numeric_errors;
        }
        receive_buffer_.erase(receive_buffer_.begin(),
                              receive_buffer_.begin() + kHaptronResponseSize);
    }
}

void HaptronModbusResponseParser::UpdateBufferStatistics() noexcept {
    statistics_.buffered_bytes = receive_buffer_.size();
    statistics_.peak_buffered_bytes = std::max(
        statistics_.peak_buffered_bytes, receive_buffer_.size());
}

// Force calibration, compensation and tool-frame conversion implementation.
#include <force_calibration.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include <Eigen/SVD>
#include <openssl/evp.h>

#include <json.hpp>

namespace {

using json = nlohmann::json;

// Both excitation matrices checked below are dimensionless and have equally
// scaled column blocks. Requiring sigma_min / sigma_max >= 0.05 is equivalent
// to limiting their 2-norm condition number to 20. This is deliberately much
// stronger than a numerical-rank epsilon: small, nearly collinear pose changes
// no longer qualify as an identifiable calibration window.
constexpr double kMinimumExcitationSingularValueRatio = 0.05;

Eigen::Matrix3d Skew(const Eigen::Vector3d& value) {
    Eigen::Matrix3d result;
    result << 0.0, -value.z(), value.y(),
              value.z(), 0.0, -value.x(),
              -value.y(), value.x(), 0.0;
    return result;
}

bool Finite(const Eigen::VectorXd& value) {
    return value.array().isFinite().all();
}

bool ExcitationWellConditioned(const Eigen::MatrixXd& design,
                               double* singular_value_ratio) {
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(design);
    const Eigen::VectorXd singular_values = svd.singularValues();
    if (singular_values.size() == 0
        || !singular_values.array().isFinite().all()) {
        if (singular_value_ratio != nullptr) *singular_value_ratio = 0.0;
        return false;
    }
    const double maximum = singular_values.maxCoeff();
    const double minimum = singular_values.minCoeff();
    const double ratio = maximum > 0.0 ? minimum / maximum : 0.0;
    if (singular_value_ratio != nullptr) *singular_value_ratio = ratio;
    return std::isfinite(ratio)
        && ratio >= kMinimumExcitationSingularValueRatio;
}

double MaximumDirectionSeparationRad(
    const std::vector<Eigen::Vector3d>& unit_directions) {
    double maximum = 0.0;
    for (std::size_t lhs = 0; lhs < unit_directions.size(); ++lhs) {
        for (std::size_t rhs = lhs + 1; rhs < unit_directions.size(); ++rhs) {
            const double cosine = std::clamp(
                unit_directions[lhs].dot(unit_directions[rhs]), -1.0, 1.0);
            maximum = std::max(maximum, std::acos(cosine));
        }
    }
    return maximum;
}

json VectorToJson(const Eigen::Vector3d& value) {
    return json::array({value.x(), value.y(), value.z()});
}

json MatrixToJson(const Eigen::Matrix3d& value) {
    json output = json::array();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) output.push_back(value(row, col));
    }
    return output;
}

bool JsonToVector(const json& input, Eigen::Vector3d& output) {
    if (!input.is_array() || input.size() != 3) return false;
    for (int i = 0; i < 3; ++i) {
        if (!input[i].is_number()) return false;
        output[i] = input[i].get<double>();
    }
    return output.array().isFinite().all();
}

bool JsonToMatrix(const json& input, Eigen::Matrix3d& output) {
    if (!input.is_array() || input.size() != 9) return false;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const auto& item = input[row * 3 + col];
            if (!item.is_number()) return false;
            output(row, col) = item.get<double>();
        }
    }
    return output.array().isFinite().all();
}

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) *error = message;
}

}  // namespace

bool ForceCalibration::Validate(std::string* error) const {
    if (schema_version != kSchemaVersion) {
        SetError(error, "unsupported force calibration schema_version");
        return false;
    }
    if (sensor_id.empty() || sensor_id.find("REPLACE_") != std::string::npos) {
        SetError(error, "force calibration sensor_id is empty");
        return false;
    }
    if (probe_model.empty() || probe_model.find("REPLACE_") != std::string::npos) {
        SetError(error, "force calibration probe_model is empty");
        return false;
    }
    if (created_at.empty() || created_at.find("REPLACE_") != std::string::npos) {
        SetError(error, "force calibration created_at is missing or a placeholder");
        return false;
    }
    if (!probe_model_sha256.empty()) {
        const bool valid_digest = probe_model_sha256.size() == 64
            && std::all_of(probe_model_sha256.begin(),
                           probe_model_sha256.end(),
                           [](unsigned char value) { return std::isxdigit(value) != 0; });
        if (!valid_digest) {
            SetError(error, "probe_model_sha256 must be 64 hexadecimal characters");
            return false;
        }
    }
    if (!gravity_base_n.array().isFinite().all()
        || !force_bias_n.array().isFinite().all()
        || !torque_bias_nm.array().isFinite().all()
        || !center_of_mass_sensor_m.array().isFinite().all()
        || !rotation_tool_from_sensor.array().isFinite().all()
        || !translation_sensor_to_tool_m.array().isFinite().all()
        || !probe_tcp_sensor_m.array().isFinite().all()) {
        SetError(error, "force calibration contains a non-finite number");
        return false;
    }
    const std::array<double, 6> quality_values{{
        force_residual_rms_n,
        torque_residual_rms_nm,
        force_residual_max_n,
        torque_residual_max_nm,
        accepted_force_residual_max_n,
        accepted_torque_residual_max_nm}};
    if (!std::all_of(quality_values.begin(), quality_values.end(),
                     [](double value) {
                         return std::isfinite(value) && value >= 0.0;
                     })) {
        SetError(error, "force calibration quality contains an invalid value");
        return false;
    }
    if (calibration_residuals_verified) {
        if (accepted_force_residual_max_n <= 0.0
            || accepted_torque_residual_max_nm <= 0.0
            || accepted_force_residual_max_n > 0.5
            || accepted_torque_residual_max_nm > 0.05
            || force_residual_max_n > accepted_force_residual_max_n
            || torque_residual_max_nm > accepted_torque_residual_max_nm
            || force_residual_rms_n > accepted_force_residual_max_n
            || torque_residual_rms_nm > accepted_torque_residual_max_nm
            || force_residual_rms_n > force_residual_max_n + 1e-12
            || torque_residual_rms_nm > torque_residual_max_nm + 1e-12) {
            SetError(error,
                     "verified calibration RMS/maximum residual limits are not satisfied");
            return false;
        }
    }
    const Eigen::Matrix3d should_be_identity =
        rotation_tool_from_sensor.transpose() * rotation_tool_from_sensor;
    if ((should_be_identity - Eigen::Matrix3d::Identity()).norm() > 1e-6
        || std::abs(rotation_tool_from_sensor.determinant() - 1.0) > 1e-6) {
        SetError(error, "rotation_tool_from_sensor is not a proper rotation");
        return false;
    }
    if (gravity_base_n.norm() < 0.1 || gravity_base_n.norm() > 50.0) {
        SetError(error, "gravity vector magnitude is outside 0.1..50 N");
        return false;
    }
    if (force_bias_n.cwiseAbs().maxCoeff() > 50.0
        || torque_bias_nm.cwiseAbs().maxCoeff() > 5.0
        || center_of_mass_sensor_m.norm() > 0.5
        || translation_sensor_to_tool_m.norm() > 0.5
        || probe_tcp_sensor_m.norm() > 1.0) {
        SetError(error, "force calibration exceeds physical safety bounds");
        return false;
    }
    if (tool_chain_verified && probe_tcp_sensor_m.norm() < 0.01) {
        SetError(error, "verified probe TCP is implausibly close to the sensor origin");
        return false;
    }
    return true;
}

bool ForceCalibration::LoadJson(const std::string& path, std::string* error) {
    std::ifstream stream(path);
    if (!stream) {
        SetError(error, "cannot open force calibration file: " + path);
        return false;
    }

    json input = json::parse(stream, nullptr, false);
    if (input.is_discarded() || !input.is_object()) {
        SetError(error, "invalid JSON in force calibration file: " + path);
        return false;
    }

    try {
        schema_version = input.at("schema_version").get<int>();
        sensor_id = input.at("sensor_id").get<std::string>();
        probe_model = input.at("probe_model").get<std::string>();
        probe_model_sha256 = input.value("probe_model_sha256", std::string());
        created_at = input.value("created_at", std::string());
        tool_chain_verified = input.value("tool_chain_verified", false);
        if (input.contains("quality")) {
            const auto& quality = input.at("quality");
            calibration_residuals_verified =
                quality.value("residuals_verified", false);
            force_residual_rms_n = quality.value("force_rms_n", 0.0);
            torque_residual_rms_nm = quality.value("torque_rms_nm", 0.0);
            force_residual_max_n = quality.value("force_max_n", 0.0);
            torque_residual_max_nm = quality.value("torque_max_nm", 0.0);
            accepted_force_residual_max_n =
                quality.value("accepted_force_max_n", 0.5);
            accepted_torque_residual_max_nm =
                quality.value("accepted_torque_max_nm", 0.05);
        } else {
            calibration_residuals_verified = false;
        }

        const auto& units = input.at("units");
        if (!units.is_object()
            || units.value("force", std::string()) != "N"
            || units.value("torque", std::string()) != "N*m"
            || units.value("length", std::string()) != "m") {
            SetError(error, "force calibration units must be N, N*m and m");
            return false;
        }

        const auto& model = input.at("gravity_model");
        const auto& transform = input.at("sensor_to_tool");
        if (!JsonToVector(model.at("gravity_base_n"), gravity_base_n)
            || !JsonToVector(model.at("force_bias_n"), force_bias_n)
            || !JsonToVector(model.at("torque_bias_nm"), torque_bias_nm)
            || !JsonToVector(model.at("center_of_mass_sensor_m"),
                             center_of_mass_sensor_m)
            || !JsonToMatrix(transform.at("rotation_row_major"),
                             rotation_tool_from_sensor)
            || !JsonToVector(transform.at("translation_sensor_to_tool_m"),
                             translation_sensor_to_tool_m)
            || !JsonToVector(input.at("probe_tcp_sensor_m"), probe_tcp_sensor_m)) {
            SetError(error, "force calibration has an invalid vector or matrix");
            return false;
        }
    } catch (const std::exception& exception) {
        SetError(error, std::string("missing/invalid force calibration field: ")
                            + exception.what());
        return false;
    }
    return Validate(error);
}

bool ForceCalibration::SaveJson(const std::string& path, std::string* error) const {
    if (!Validate(error)) return false;

    json output;
    output["schema_version"] = schema_version;
    output["sensor_id"] = sensor_id;
    output["probe_model"] = probe_model;
    output["probe_model_sha256"] = probe_model_sha256;
    output["created_at"] = created_at;
    output["tool_chain_verified"] = tool_chain_verified;
    output["quality"] = {
        {"residuals_verified", calibration_residuals_verified},
        {"force_rms_n", force_residual_rms_n},
        {"torque_rms_nm", torque_residual_rms_nm},
        {"force_max_n", force_residual_max_n},
        {"torque_max_nm", torque_residual_max_nm},
        {"accepted_force_max_n", accepted_force_residual_max_n},
        {"accepted_torque_max_nm", accepted_torque_residual_max_nm}};
    output["units"] = {{"force", "N"}, {"torque", "N*m"}, {"length", "m"}};
    output["gravity_model"] = {
        {"gravity_base_n", VectorToJson(gravity_base_n)},
        {"force_bias_n", VectorToJson(force_bias_n)},
        {"torque_bias_nm", VectorToJson(torque_bias_nm)},
        {"center_of_mass_sensor_m", VectorToJson(center_of_mass_sensor_m)}};
    output["sensor_to_tool"] = {
        {"rotation_row_major", MatrixToJson(rotation_tool_from_sensor)},
        {"translation_sensor_to_tool_m", VectorToJson(translation_sensor_to_tool_m)}};
    output["probe_tcp_sensor_m"] = VectorToJson(probe_tcp_sensor_m);

    const std::filesystem::path output_path(path);
    std::error_code ec;
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path(), ec);
        if (ec) {
            SetError(error, "cannot create force calibration directory: "
                                + output_path.parent_path().string());
            return false;
        }
    }
    std::ofstream stream(path);
    if (!stream) {
        SetError(error, "cannot write force calibration file: " + path);
        return false;
    }
    stream << std::setw(2) << output << '\n';
    if (!stream.good()) {
        SetError(error, "failed while writing force calibration file: " + path);
        return false;
    }
    return true;
}

CompensatedWrench ForceCalibration::Compensate(
    const Eigen::Matrix<double, 6, 1>& raw_wrench_sensor,
    const Eigen::Matrix3d& rotation_base_from_sensor,
    std::uint64_t sequence,
    std::int64_t timestamp_ns) const {
    CompensatedWrench result;
    result.sequence = sequence;
    result.timestamp_ns = timestamp_ns;

    std::string validation_error;
    if (!Validate(&validation_error)) {
        result.error = validation_error;
        return result;
    }
    if (!raw_wrench_sensor.array().isFinite().all()
        || !rotation_base_from_sensor.array().isFinite().all()) {
        result.error = "non-finite wrench or sensor rotation";
        return result;
    }

    const Eigen::Vector3d gravity_sensor =
        rotation_base_from_sensor.transpose() * gravity_base_n;
    const Eigen::Vector3d expected_force = force_bias_n + gravity_sensor;
    const Eigen::Vector3d expected_torque =
        torque_bias_nm + center_of_mass_sensor_m.cross(gravity_sensor);

    const Eigen::Vector3d compensated_force =
        raw_wrench_sensor.head<3>() - expected_force;
    const Eigen::Vector3d compensated_torque =
        raw_wrench_sensor.tail<3>() - expected_torque;
    result.sensor << compensated_force, compensated_torque;

    // Shift the moment from the sensor origin to the tool origin, then rotate.
    const Eigen::Vector3d torque_at_tool_sensor =
        compensated_torque
        - translation_sensor_to_tool_m.cross(compensated_force);
    result.tool.head<3>() = rotation_tool_from_sensor * compensated_force;
    result.tool.tail<3>() = rotation_tool_from_sensor * torque_at_tool_sensor;
    result.valid = true;
    return result;
}

bool ForceCalibration::Fit(const std::vector<ForceCalibrationSample>& samples,
                           ForceCalibration& result,
                           double* force_rms_n,
                           double* torque_rms_nm,
                           std::string* error,
                           double* force_max_n,
                           double* torque_max_nm) {
    if (samples.size() < 6) {
        SetError(error, "at least six calibration samples are required");
        return false;
    }

    const int count = static_cast<int>(samples.size());
    Eigen::MatrixXd force_a(3 * count, 6);
    Eigen::VectorXd force_b(3 * count);
    for (int i = 0; i < count; ++i) {
        const auto& sample = samples[i];
        if (!sample.rotation_base_from_sensor.array().isFinite().all()
            || !sample.raw_wrench_sensor.array().isFinite().all()) {
            SetError(error, "calibration sample contains a non-finite number");
            return false;
        }
        const Eigen::Matrix3d orthogonality =
            sample.rotation_base_from_sensor.transpose()
            * sample.rotation_base_from_sensor;
        if ((orthogonality - Eigen::Matrix3d::Identity()).norm() > 1e-5) {
            SetError(error, "calibration sample rotation is not orthonormal");
            return false;
        }
        if (std::abs(sample.rotation_base_from_sensor.determinant() - 1.0)
            > 1e-5) {
            SetError(error, "calibration sample rotation is not a proper rotation");
            return false;
        }
        force_a.block<3, 3>(3 * i, 0) =
            sample.rotation_base_from_sensor.transpose();
        force_a.block<3, 3>(3 * i, 3) = Eigen::Matrix3d::Identity();
        force_b.segment<3>(3 * i) = sample.raw_wrench_sensor.head<3>();
    }

    double force_excitation_ratio = 0.0;
    if (!ExcitationWellConditioned(force_a, &force_excitation_ratio)) {
        std::ostringstream message;
        message << "force calibration poses are ill-conditioned: "
                << "normalized sigma_min/sigma_max="
                << force_excitation_ratio << " is below "
                << kMinimumExcitationSingularValueRatio
                << " (maximum condition number 20)";
        SetError(error, message.str());
        return false;
    }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> force_qr(force_a);
    force_qr.setThreshold(1e-10);
    if (force_qr.rank() < 6) {
        SetError(error, "calibration poses do not sufficiently excite gravity directions");
        return false;
    }
    const Eigen::VectorXd force_solution = force_qr.solve(force_b);
    if (!Finite(force_solution)) {
        SetError(error, "force calibration solve produced non-finite values");
        return false;
    }
    result.gravity_base_n = force_solution.head<3>();
    result.force_bias_n = force_solution.tail<3>();

    const double gravity_magnitude = result.gravity_base_n.norm();
    if (!std::isfinite(gravity_magnitude) || gravity_magnitude <= 1e-9) {
        SetError(error,
                 "force calibration did not identify a usable gravity vector");
        return false;
    }

    Eigen::MatrixXd torque_a(3 * count, 6);
    Eigen::MatrixXd normalized_torque_excitation(3 * count, 6);
    Eigen::VectorXd torque_b(3 * count);
    std::vector<Eigen::Vector3d> gravity_directions_sensor;
    gravity_directions_sensor.reserve(samples.size());
    for (int i = 0; i < count; ++i) {
        const Eigen::Vector3d gravity_sensor =
            samples[i].rotation_base_from_sensor.transpose()
            * result.gravity_base_n;
        const Eigen::Vector3d gravity_direction =
            gravity_sensor / gravity_magnitude;
        torque_a.block<3, 3>(3 * i, 0) = -Skew(gravity_sensor);
        torque_a.block<3, 3>(3 * i, 3) = Eigen::Matrix3d::Identity();
        normalized_torque_excitation.block<3, 3>(3 * i, 0) =
            -Skew(gravity_direction);
        normalized_torque_excitation.block<3, 3>(3 * i, 3) =
            Eigen::Matrix3d::Identity();
        torque_b.segment<3>(3 * i) = samples[i].raw_wrench_sensor.tail<3>();
        gravity_directions_sensor.push_back(gravity_direction);
    }
    double torque_excitation_ratio = 0.0;
    if (!ExcitationWellConditioned(normalized_torque_excitation,
                                   &torque_excitation_ratio)) {
        const double maximum_direction_separation_deg =
            MaximumDirectionSeparationRad(gravity_directions_sensor)
            * 180.0 / 3.14159265358979323846;
        std::ostringstream message;
        message << "torque calibration gravity directions are ill-conditioned: "
                << "normalized sigma_min/sigma_max="
                << torque_excitation_ratio << " is below "
                << kMinimumExcitationSingularValueRatio
                << " (maximum condition number 20; maximum direction "
                << "separation=" << maximum_direction_separation_deg
                << " deg)";
        SetError(error, message.str());
        return false;
    }
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> torque_qr(torque_a);
    torque_qr.setThreshold(1e-10);
    if (torque_qr.rank() < 6) {
        SetError(error, "calibration poses cannot identify center of mass and torque bias");
        return false;
    }
    const Eigen::VectorXd torque_solution = torque_qr.solve(torque_b);
    if (!Finite(torque_solution)) {
        SetError(error, "torque calibration solve produced non-finite values");
        return false;
    }
    result.center_of_mass_sensor_m = torque_solution.head<3>();
    result.torque_bias_nm = torque_solution.tail<3>();

    const Eigen::VectorXd force_error = force_a * force_solution - force_b;
    const Eigen::VectorXd torque_error = torque_a * torque_solution - torque_b;
    const double force_squared_error = force_error.squaredNorm();
    const double torque_squared_error = torque_error.squaredNorm();
    double maximum_force_error = 0.0;
    double maximum_torque_error = 0.0;
    for (int index = 0; index < count; ++index) {
        maximum_force_error = std::max(
            maximum_force_error,
            force_error.segment<3>(3 * index).norm());
        maximum_torque_error = std::max(
            maximum_torque_error,
            torque_error.segment<3>(3 * index).norm());
    }
    result.force_residual_rms_n =
        std::sqrt(force_squared_error / static_cast<double>(count));
    result.torque_residual_rms_nm =
        std::sqrt(torque_squared_error / static_cast<double>(count));
    result.force_residual_max_n = maximum_force_error;
    result.torque_residual_max_nm = maximum_torque_error;
    if (force_rms_n != nullptr) {
        *force_rms_n = result.force_residual_rms_n;
    }
    if (torque_rms_nm != nullptr) {
        *torque_rms_nm = result.torque_residual_rms_nm;
    }
    if (force_max_n != nullptr) *force_max_n = maximum_force_error;
    if (torque_max_nm != nullptr) *torque_max_nm = maximum_torque_error;
    return true;
}

Eigen::Matrix3d RotationBaseFromControllerEuler(
    const Eigen::Vector3d& rx_ry_rz_rad) {
    const Eigen::AngleAxisd roll(rx_ry_rz_rad.x(), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(rx_ry_rz_rad.y(), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(rx_ry_rz_rad.z(), Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}

bool ComputeFileSha256(const std::string& path,
                       std::string& digest_hex,
                       std::string* error) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        SetError(error, "cannot open file for SHA-256: " + path);
        return false;
    }

    EVP_MD_CTX* context = EVP_MD_CTX_new();
    if (context == nullptr) {
        SetError(error, "cannot allocate SHA-256 context");
        return false;
    }
    bool success = EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1;
    std::array<char, 8192> buffer{};
    while (success && stream.good()) {
        stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize count = stream.gcount();
        if (count > 0) {
            success = EVP_DigestUpdate(
                context, buffer.data(), static_cast<std::size_t>(count)) == 1;
        }
    }
    if (stream.bad()) success = false;

    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned int digest_size = 0;
    if (success) {
        success = EVP_DigestFinal_ex(context, digest.data(), &digest_size) == 1
            && digest_size == 32;
    }
    EVP_MD_CTX_free(context);
    if (!success) {
        SetError(error, "failed to calculate SHA-256 for: " + path);
        return false;
    }

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (unsigned int index = 0; index < digest_size; ++index) {
        output << std::setw(2) << static_cast<unsigned int>(digest[index]);
    }
    digest_hex = output.str();
    return true;
}
