#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "haptron_modbus.hpp"

// Order used by the sensor wire protocol and by WrenchSample::wrench_si.
enum WrenchAxis : std::size_t {
    kForceX = 0,
    kForceY = 1,
    kForceZ = 2,
    kTorqueX = 3,
    kTorqueY = 4,
    kTorqueZ = 5,
};

enum class ForceSensorIoStatus {
    kDisconnected,
    kOpening,
    kStreaming,
    kStopped,
    kIoError,
};

const char* ForceSensorIoStatusName(ForceSensorIoStatus status) noexcept;

enum class ForceSensorProtocol {
    kHaptronModbusRtu,
    kLegacyAa55Stream,
};

const char* ForceSensorProtocolName(ForceSensorProtocol protocol) noexcept;

/**
 * One force/torque observation in SI units.
 *
 * wrench_si is ordered as [Fx, Fy, Fz, Tx, Ty, Tz], with forces in newtons
 * and torques in newton-metres. timestamp is suitable for logs and wire
 * messages; monotonic_timestamp is the clock used for staleness decisions.
 */
struct WrenchSample {
    std::array<double, 6> wrench_si{};
    std::uint64_t sequence{0};
    std::chrono::system_clock::time_point timestamp{};
    std::chrono::steady_clock::time_point monotonic_timestamp{};
    bool checksum_valid{false};
    bool valid{false};
    bool stale{true};
    ForceSensorIoStatus io_status{ForceSensorIoStatus::kDisconnected};
    int io_error{0};

    bool IsStale(std::chrono::milliseconds maximum_age) const noexcept;
};

/** Parser statistics reset on Reset(). */
struct ForceSensorStatistics {
    std::uint64_t bytes_received{0};
    std::uint64_t valid_frames{0};
    std::uint64_t checksum_errors{0};
    std::uint64_t numeric_errors{0};
    std::uint64_t noise_bytes_discarded{0};
    std::uint64_t buffer_overflow_events{0};
    std::uint64_t buffer_bytes_discarded{0};
    std::size_t buffered_bytes{0};
    std::size_t peak_buffered_bytes{0};

};

/**
 * Streaming parser for the sensor's 31-byte AA 55 frames.
 *
 * It accepts arbitrary chunks, so split frames, multiple frames per read and
 * noise between frames are all supported. No serial device is required, which
 * makes this class the protocol test seam.
 */
class ForceSensorFrameParser {
public:
    static constexpr std::size_t kFrameSize = 31;
    static constexpr std::size_t kPayloadOffset = 6;
    static constexpr std::size_t kPayloadSize = 24;
    static constexpr std::size_t kChecksumOffset = 30;
    static constexpr std::size_t kDefaultMaximumBufferBytes = 4096;

    explicit ForceSensorFrameParser(
        std::size_t maximum_buffer_bytes = kDefaultMaximumBufferBytes);

    std::vector<WrenchSample> Feed(const std::uint8_t* data,
                                   std::size_t length);
    std::vector<WrenchSample> Feed(
        const std::uint8_t* data,
        std::size_t length,
        std::chrono::system_clock::time_point timestamp,
        std::chrono::steady_clock::time_point monotonic_timestamp);

    void Reset();
    ForceSensorStatistics statistics() const noexcept;

    static std::uint8_t ComputeChecksum(const std::uint8_t* data,
                                        std::size_t length) noexcept;

private:
    void ParseAvailable(
        std::chrono::system_clock::time_point timestamp,
        std::chrono::steady_clock::time_point monotonic_timestamp,
        std::vector<WrenchSample>* samples);
    void UpdateBufferStatistics() noexcept;

    std::size_t maximum_buffer_bytes_;
    std::vector<std::uint8_t> receive_buffer_;
    ForceSensorStatistics statistics_;
    std::uint64_t next_sequence_{1};
};

struct ForceSensorConfig {
    std::string device{"/dev/ttyUSB0"};
    unsigned int baud_rate{115200};
    ForceSensorProtocol protocol{ForceSensorProtocol::kHaptronModbusRtu};
    std::uint8_t modbus_slave_address{kHaptronDefaultSlaveAddress};
    std::chrono::milliseconds query_period{20};
    std::chrono::milliseconds response_timeout{50};
    std::chrono::milliseconds poll_timeout{50};
    std::chrono::milliseconds stale_after{50};
    std::size_t read_chunk_bytes{512};
    std::size_t maximum_buffer_bytes{
        ForceSensorFrameParser::kDefaultMaximumBufferBytes};
};

/**
 * Owns the serial descriptor and publishes a mutex-protected latest sample.
 * Start() opens the configured path synchronously. The worker performs only
 * serial reads and frame parsing, and Stop() is safe to call repeatedly.
 */
class ForceSensorReader {
public:
    explicit ForceSensorReader(ForceSensorConfig config = ForceSensorConfig{});
    ~ForceSensorReader();

    ForceSensorReader(const ForceSensorReader&) = delete;
    ForceSensorReader& operator=(const ForceSensorReader&) = delete;

    bool Start();
    void Stop() noexcept;

    WrenchSample LatestSample() const;
    std::string LastError() const;

private:
    void WorkerLoop();
    void LegacyStreamWorkerLoop();
    void HaptronModbusWorkerLoop();
    void PublishIoStatus(ForceSensorIoStatus status,
                         int error_number,
                         const std::string& message);
    void ClosePort() noexcept;

    ForceSensorConfig config_;
    ForceSensorFrameParser legacy_parser_;
    HaptronModbusResponseParser modbus_parser_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<int> serial_fd_{-1};
    std::thread worker_;

    mutable std::mutex lifecycle_mutex_;
    mutable std::mutex state_mutex_;
    WrenchSample latest_sample_;
    std::string last_error_;
};
