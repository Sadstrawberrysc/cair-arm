#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

constexpr std::uint8_t kHaptronDefaultSlaveAddress = 1;
constexpr std::uint8_t kHaptronReadInputRegisters = 0x04;
constexpr std::uint16_t kHaptronStartRegister = 0x0038;
constexpr std::uint16_t kHaptronRegisterCount = 0x000C;
constexpr std::size_t kHaptronRequestSize = 8;
constexpr std::size_t kHaptronPayloadSize = 24;
constexpr std::size_t kHaptronResponseSize = 29;

/**
 * Result of one Haptron six-axis Modbus RTU response.
 *
 * wrench_si is ordered [Fx, Fy, Fz, Tx, Ty, Tz]. The reference program
 * treats the six wire float32 values as SI newtons and newton-metres.
 */
struct HaptronModbusResult {
    std::array<double, 6> wrench_si{};
    bool valid{false};
    bool exception{false};
    std::uint8_t exception_code{0};
    std::string error;
};

struct HaptronModbusStatistics {
    std::uint64_t bytes_received{0};
    std::uint64_t valid_responses{0};
    std::uint64_t exception_responses{0};
    std::uint64_t crc_errors{0};
    std::uint64_t numeric_errors{0};
    std::uint64_t protocol_errors{0};
    std::uint64_t noise_bytes_discarded{0};
    std::uint64_t buffer_overflow_events{0};
    std::size_t buffered_bytes{0};
    std::size_t peak_buffered_bytes{0};
};

std::uint16_t HaptronModbusCrc16(const std::uint8_t* data,
                                 std::size_t length) noexcept;

std::array<std::uint8_t, kHaptronRequestSize> BuildHaptronReadRequest(
    std::uint8_t slave_address = kHaptronDefaultSlaveAddress) noexcept;

/**
 * Streaming parser for Haptron Modbus function-04 responses.
 *
 * It accepts partial reads, leading noise, request echo, and multiple replies.
 * A normal response is 29 bytes: slave, 04, 18, six big-endian float32 values,
 * and CRC16-Modbus low byte then high byte. Modbus exception replies are also
 * surfaced explicitly.
 */
class HaptronModbusResponseParser {
public:
    explicit HaptronModbusResponseParser(
        std::uint8_t slave_address = kHaptronDefaultSlaveAddress,
        std::size_t maximum_buffer_bytes = 512);

    std::vector<HaptronModbusResult> Feed(const std::uint8_t* data,
                                          std::size_t length);
    void Reset();
    void DiscardBufferedData() noexcept;

    HaptronModbusStatistics statistics() const noexcept;
    std::size_t buffered_bytes() const noexcept;

private:
    void ParseAvailable(std::vector<HaptronModbusResult>* results);
    void UpdateBufferStatistics() noexcept;

    std::uint8_t slave_address_;
    std::size_t maximum_buffer_bytes_;
    std::vector<std::uint8_t> receive_buffer_;
    HaptronModbusStatistics statistics_;
};
