#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

#include <force_sensor.hpp>

namespace {

bool Check(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

void StoreLittleEndian(float value, std::uint8_t* output) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    output[0] = static_cast<std::uint8_t>(bits & 0xffU);
    output[1] = static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
    output[2] = static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
    output[3] = static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
}

void StoreBigEndian(float value, std::uint8_t* output) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    output[0] = static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
    output[1] = static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
    output[2] = static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
    output[3] = static_cast<std::uint8_t>(bits & 0xffU);
}

}  // namespace

int main() {
    bool ok = true;
    const std::array<float, 6> expected{{1.25F, -2.5F, 3.75F,
                                         -0.1F, 0.2F, -0.3F}};

    std::array<std::uint8_t, ForceSensorFrameParser::kFrameSize> frame{};
    frame[0] = 0xaa;
    frame[1] = 0x55;
    for (std::size_t axis = 0; axis < expected.size(); ++axis) {
        StoreLittleEndian(expected[axis],
                          frame.data() + ForceSensorFrameParser::kPayloadOffset
                              + axis * sizeof(float));
    }
    frame[ForceSensorFrameParser::kChecksumOffset] =
        ForceSensorFrameParser::ComputeChecksum(
            frame.data() + ForceSensorFrameParser::kPayloadOffset,
            ForceSensorFrameParser::kPayloadSize);

    ForceSensorFrameParser legacy_parser;
    auto legacy_results = legacy_parser.Feed(frame.data(), 9);
    ok &= Check(legacy_results.empty(), "split AA55 prefix must remain buffered");
    legacy_results = legacy_parser.Feed(frame.data() + 9, frame.size() - 9);
    ok &= Check(legacy_results.size() == 1 && legacy_results[0].valid,
                "split AA55 frame must parse");
    for (std::size_t axis = 0; axis < expected.size() && !legacy_results.empty();
         ++axis) {
        ok &= Check(std::abs(legacy_results[0].wrench_si[axis] - expected[axis])
                        < 1e-6,
                    "AA55 decoded value");
    }
    frame.back() ^= 0x01U;
    ok &= Check(legacy_parser.Feed(frame.data(), frame.size()).empty(),
                "bad AA55 checksum must be rejected");
    ok &= Check(legacy_parser.statistics().checksum_errors >= 1,
                "AA55 checksum error statistic");

    std::array<std::uint8_t, kHaptronResponseSize> response{};
    response[0] = kHaptronDefaultSlaveAddress;
    response[1] = kHaptronReadInputRegisters;
    response[2] = kHaptronPayloadSize;
    for (std::size_t axis = 0; axis < expected.size(); ++axis) {
        StoreBigEndian(expected[axis], response.data() + 3 + axis * sizeof(float));
    }
    const std::uint16_t crc = HaptronModbusCrc16(response.data(), response.size() - 2);
    response[response.size() - 2] = static_cast<std::uint8_t>(crc & 0xffU);
    response[response.size() - 1] = static_cast<std::uint8_t>((crc >> 8U) & 0xffU);

    HaptronModbusResponseParser modbus_parser;
    auto modbus_results = modbus_parser.Feed(response.data(), 5);
    ok &= Check(modbus_results.empty(), "split Modbus prefix must remain buffered");
    modbus_results = modbus_parser.Feed(response.data() + 5, response.size() - 5);
    ok &= Check(modbus_results.size() == 1 && modbus_results[0].valid,
                "split Haptron Modbus response must parse");
    response.back() ^= 0x01U;
    ok &= Check(modbus_parser.Feed(response.data(), response.size()).empty(),
                "bad Modbus CRC must be rejected");
    ok &= Check(modbus_parser.statistics().crc_errors >= 1,
                "Modbus CRC error statistic");
    return ok ? 0 : 1;
}
