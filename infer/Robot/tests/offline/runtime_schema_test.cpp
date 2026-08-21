#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>

#include <runtime_schema.hpp>
#include <rm75_runtime_logging.hpp>

namespace {

bool Check(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}

std::size_t CsvColumnCount(const std::string& line) {
    return static_cast<std::size_t>(
               std::count(line.begin(), line.end(), ','))
        + 1;
}

}  // namespace

int main() {
    bool ok = true;
    ok &= Check(kRuntimeSummarySchemaVersion == 2,
                "runtime summary schema remains v2");
    const std::set<std::string_view> summary_keys(
        kRuntimeSummaryV2TopLevelKeys.begin(),
        kRuntimeSummaryV2TopLevelKeys.end());
    ok &= Check(summary_keys.size() == kRuntimeSummaryV2TopLevelKeys.size(),
                "summary top-level keys are unique");
    ok &= Check(summary_keys.count("control") == 1
                    && summary_keys.count("runtime_tare") == 1
                    && summary_keys.count("redis") == 1
                    && summary_keys.count("servo") == 1,
                "summary v2 retains safety and protocol sections");

    std::istringstream input{std::string(kRuntimeCsvHeader)};
    std::string column;
    std::size_t column_count = 0;
    std::string first;
    std::string last;
    while (std::getline(input, column, ',')) {
        if (column_count == 0) first = column;
        last = column;
        ++column_count;
    }
    ok &= Check(first == "cycle" && last == "fault",
                "CSV boundary columns remain cycle/fault");
    ok &= Check(column_count == 126, "CSV v2 column count remains fixed");
    ok &= Check(kRuntimeCsvHeader.find("command_session_id")
                    != std::string_view::npos
                    && kRuntimeCsvHeader.find("servo_discarded_sequence")
                        != std::string_view::npos
                    && kRuntimeCsvHeader.find("recovery_search_distance_m")
                        != std::string_view::npos,
                "CSV retains command, Servo and recovery diagnostics");

    const std::filesystem::path csv_path =
        std::filesystem::temp_directory_path()
        / "uspilot_runtime_schema_test.csv";
    std::error_code remove_error;
    std::filesystem::remove(csv_path, remove_error);
    AsyncRuntimeLogger logger;
    std::string logger_error;
    ok &= Check(logger.Start(csv_path.string(), &logger_error),
                "runtime logger starts on a temporary file");
    const auto started = std::chrono::steady_clock::now();
    logger.PushAndMeasure(RuntimeLogRow{}, started,
                          started + std::chrono::seconds(1));
    logger.Stop();
    std::ifstream csv(csv_path);
    std::string written_header;
    std::string written_row;
    std::getline(csv, written_header);
    std::getline(csv, written_row);
    ok &= Check(written_header == kRuntimeCsvHeader,
                "runtime writer emits the exact CSV v2 header");
    ok &= Check(CsvColumnCount(written_row) == column_count,
                "runtime writer row count matches the 126-column header");
    csv.close();
    std::filesystem::remove(csv_path, remove_error);

    RuntimeSummaryData data;
    data.mode = "dry_run";
    data.completion_reason = "duration_elapsed";
    data.configuration_profile = "explicit_cli";
    data.runtime_csv = "/tmp/runtime.csv";
    data.control.config.desired_force_n = -2.0;
    data.control.config.approach_speed_m_s = 0.020;
    data.control.config.scan_speed_m_s = 0.010;
    data.control.safety.raw_force_limit_n = 50.0;
    data.control.safety.raw_torque_limit_nm = 5.0;
    const nlohmann::json summary = BuildRuntimeSummary(data);
    std::set<std::string_view> built_keys;
    for (const auto& item : summary.items()) built_keys.insert(item.key());
    ok &= Check(built_keys == summary_keys,
                "summary builder emits the exact v2 top-level key set");
    ok &= Check(summary.at("schema_version") == 2
                    && summary.at("control").at("desired_force_n") == -2.0
                    && summary.at("control").at("approach_speed_cm_s") == 2.0
                    && summary.at("control").at("scan_speed_cm_s") == 1.0
                    && summary.at("control").at("raw_force_limit_n") == 50.0
                    && summary.at("runtime_csv") == "/tmp/runtime.csv",
                "summary builder preserves v2 values and public units");
    return ok ? 0 : 1;
}
