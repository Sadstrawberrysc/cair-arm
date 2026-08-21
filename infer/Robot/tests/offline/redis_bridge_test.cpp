#include <algorithm>
#include <cstdint>
#include <iostream>
#include <set>
#include <string>

#include <json.hpp>
#include <redis_bridge.hpp>

namespace {

bool Check(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

std::set<std::string> Keys(const nlohmann::json& value) {
    std::set<std::string> keys;
    for (auto item = value.begin(); item != value.end(); ++item) {
        keys.insert(item.key());
    }
    return keys;
}

}  // namespace

int main() {
    bool ok = true;
    const std::string payload = R"({
        "version": 1,
        "session_id": "offline-session",
        "timestamp_unix_ms": 123456,
        "sequence": 7,
        "phase_idx": 1,
        "action_state": true,
        "terminate": false,
        "parameters": {"y": 0.001, "rz": -2.5}
    })";

    RedisCommandSnapshot command;
    std::string error;
    ok &= Check(RedisBridge::ParseCommandJson(payload, command, &error),
                "valid Redis v1 command must parse");
    ok &= Check(command.valid && command.protocol_version == 1,
                "parsed command metadata");
    ok &= Check(command.session_id == "offline-session"
                    && command.producer_sequence == 7,
                "session and producer sequence");

    command.subscriber_connected = true;
    RedisCommandDecision decision = RedisBridge::EvaluateCommandForControl(
        command, command.received_timestamp_ns + 10'000'000, 500, -2.0);
    ok &= Check(decision.valid && decision.fresh,
                "fresh connected command must be admitted");
    ok &= Check(decision.intent.desired_force_n == -2.0,
                "configured force is used when command omits force");

    decision = RedisBridge::EvaluateCommandForControl(
        command, command.received_timestamp_ns + 501'000'000, 500, -2.0);
    ok &= Check(!decision.valid && decision.hold_reason == "stale_redis_command",
                "stale command must fail closed");
    command.subscriber_connected = false;
    decision = RedisBridge::EvaluateCommandForControl(
        command, command.received_timestamp_ns + 10'000'000, 500, -2.0);
    ok &= Check(!decision.valid
                    && decision.hold_reason == "redis_subscriber_disconnected",
                "disconnected subscriber must fail closed");

    RedisCommandSnapshot invalid;
    ok &= Check(!RedisBridge::ParseCommandJson(
                    R"({"version":1,"session_id":"s","timestamp_unix_ms":1,"sequence":0,"terminate":false,"parameters":{"y":0,"rz":0}})",
                    invalid, &error),
                "v1 sequence zero must be rejected");

    RedisSensorMessage sensor;
    sensor.sequence = 9;
    sensor.timestamp_ns = 10;
    sensor.source_timestamp_unix_ns = 11;
    sensor.wrench_valid = true;
    sensor.contact_valid = true;
    sensor.checksum_valid = true;
    sensor.sensor_io_status = "streaming";
    sensor.control_state = "armed";
    sensor.raw_wrench_sensor << 1, 2, 3, 4, 5, 6;
    sensor.compensated_wrench_tool << 6, 5, 4, 3, 2, 1;
    sensor.legacy_contact_point_sensor_m << 0.1, 0.2, 0.3;
    sensor.contact_point_probe_m << 0.01, 0.02, 0.03;

    const nlohmann::json legacy = nlohmann::json::parse(
        RedisBridge::BuildLegacySensorJson(sensor));
    ok &= Check(legacy.is_array() && legacy.size() == 4,
                "legacy sensor schema remains a four-element array");

    const nlohmann::json sensor_v1 = nlohmann::json::parse(
        RedisBridge::BuildSensorV1Json(sensor));
    const std::set<std::string> expected_sensor_keys{
        "compensated_wrench_tool", "contact", "control_state", "fault",
        "published_unix_ms", "raw_wrench_sensor", "sensor", "sequence",
        "source_timestamp_unix_ns", "timestamp_monotonic_ns", "units",
        "valid", "version"};
    ok &= Check(Keys(sensor_v1) == expected_sensor_keys,
                "robot:sensor:v1 top-level schema");

    RedisStatusContext context;
    context.session_id = "offline-session";
    context.producer_sequence = 7;
    const nlohmann::json status = nlohmann::json::parse(
        RedisBridge::BuildStatusJson(Rm75SupervisorState::kHold,
                                     "hold", "offline", "", 7, context));
    const std::set<std::string> expected_status_keys{
        "command_age_ms", "command_sequence", "error_code", "message",
        "parameters", "phase_idx", "producer_sequence", "session_id",
        "state", "status", "timestamp_unix_ms", "version"};
    ok &= Check(Keys(status) == expected_status_keys,
                "robot:status:channel top-level schema");
    return ok ? 0 : 1;
}
