#include <force_calibration.hpp>

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string input_csv;
    std::string output_json;
    std::string sensor_id;
    std::string probe_model;
    std::string probe_file;
    std::string probe_model_sha256;
    Eigen::Vector3d sensor_to_tool_rpy_deg = Eigen::Vector3d::Zero();
    Eigen::Vector3d sensor_to_tool_translation_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d probe_tcp_sensor_m = Eigen::Vector3d::Zero();
    bool sensor_to_tool_rpy_set = false;
    bool sensor_to_tool_translation_set = false;
    bool probe_tcp_set = false;
    bool tool_chain_verified = false;
    double max_force_rms_n = 0.5;
    double max_torque_rms_nm = 0.05;
    bool allow_high_residual = false;
};

struct CaptureCsvMetadata {
    bool generated_by_capture = false;
    bool capture_complete = false;
    bool sensor_to_tool_rpy_set = false;
    Eigen::Vector3d sensor_to_tool_rpy_deg = Eigen::Vector3d::Zero();
};

void Usage(const char* program) {
    std::cout
        << "Usage: " << program
        << " --input samples.csv --output calibration.json"
           " --sensor-id ID --probe-model NAME [options]\n\n"
        << "CSV rows use SI units and exactly 15 numeric columns:\n"
        << "r00,r01,r02,r10,r11,r12,r20,r21,r22,fx,fy,fz,tx,ty,tz\n"
        << "R is the measured base-from-sensor rotation for a no-contact pose.\n"
        << "Collect at least six substantially different tool orientations.\n\n"
        << "Options:\n"
        << "  --probe-file STL          calculate/verify the model SHA-256\n"
        << "  --probe-sha256 HASH\n"
        << "  --sensor-to-tool-rpy-deg RX,RY,RZ\n"
        << "  --sensor-to-tool-translation-m X,Y,Z\n"
        << "  --probe-tcp-sensor-m X,Y,Z\n"
        << "    R_tool_from_sensor uses Rz*Ry*Rx. Translation is the\n"
        << "    sensor-origin to tool-origin vector expressed in sensor axes;\n"
        << "    probe TCP coordinates are also expressed in sensor axes.\n"
        << "  --tool-chain-verified     assert independently measured R/t/TCP\n"
        << "  --max-force-residual-n N       vector maximum, default/maximum 0.5\n"
        << "  --max-torque-residual-nm NM    vector maximum, default/maximum 0.05\n"
        << "  --allow-high-residual     write result despite failed residual gate\n";
}

bool ParseDouble(const std::string& text, double& output) {
    try {
        std::size_t used = 0;
        output = std::stod(text, &used);
        return used == text.size() && std::isfinite(output);
    } catch (...) {
        return false;
    }
}

bool ParseVector3(const std::string& text, Eigen::Vector3d& output) {
    std::stringstream stream(text);
    std::string item;
    int index = 0;
    while (std::getline(stream, item, ',')) {
        if (index >= 3 || !ParseDouble(item, output[index])) return false;
        ++index;
    }
    return index == 3;
}

bool ParseOptions(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--help" || argument == "-h") {
            Usage(argv[0]);
            std::exit(0);
        }
        auto value = [&]() -> const char* {
            return i + 1 < argc ? argv[++i] : nullptr;
        };
        if (argument == "--input") {
            const char* item = value();
            if (item == nullptr) return false;
            options.input_csv = item;
        } else if (argument == "--output") {
            const char* item = value();
            if (item == nullptr) return false;
            options.output_json = item;
        } else if (argument == "--sensor-id") {
            const char* item = value();
            if (item == nullptr) return false;
            options.sensor_id = item;
        } else if (argument == "--probe-model") {
            const char* item = value();
            if (item == nullptr) return false;
            options.probe_model = item;
        } else if (argument == "--probe-sha256") {
            const char* item = value();
            if (item == nullptr) return false;
            options.probe_model_sha256 = item;
        } else if (argument == "--probe-file") {
            const char* item = value();
            if (item == nullptr) return false;
            options.probe_file = item;
        } else if (argument == "--sensor-to-tool-rpy-deg") {
            const char* item = value();
            if (item == nullptr
                || !ParseVector3(item, options.sensor_to_tool_rpy_deg)) return false;
            options.sensor_to_tool_rpy_set = true;
        } else if (argument == "--sensor-to-tool-translation-m") {
            const char* item = value();
            if (item == nullptr
                || !ParseVector3(item, options.sensor_to_tool_translation_m)) return false;
            options.sensor_to_tool_translation_set = true;
        } else if (argument == "--probe-tcp-sensor-m") {
            const char* item = value();
            if (item == nullptr
                || !ParseVector3(item, options.probe_tcp_sensor_m)) return false;
            options.probe_tcp_set = true;
        } else if (argument == "--tool-chain-verified") {
            options.tool_chain_verified = true;
        } else if (argument == "--max-force-rms-n"
                   || argument == "--max-force-residual-n") {
            const char* item = value();
            if (item == nullptr || !ParseDouble(item, options.max_force_rms_n)) return false;
        } else if (argument == "--max-torque-rms-nm"
                   || argument == "--max-torque-residual-nm") {
            const char* item = value();
            if (item == nullptr || !ParseDouble(item, options.max_torque_rms_nm)) return false;
        } else if (argument == "--allow-high-residual") {
            options.allow_high_residual = true;
        } else {
            std::cerr << "Unknown option: " << argument << '\n';
            return false;
        }
    }
    return !options.input_csv.empty() && !options.output_json.empty()
        && !options.sensor_id.empty() && !options.probe_model.empty()
        && options.max_force_rms_n > 0.0 && options.max_force_rms_n <= 0.5
        && options.max_torque_rms_nm > 0.0
        && options.max_torque_rms_nm <= 0.05;
}

std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t begin = 0;
    while (begin <= line.size()) {
        const std::size_t comma = line.find(',', begin);
        fields.push_back(line.substr(begin, comma == std::string::npos
                                               ? std::string::npos
                                               : comma - begin));
        if (comma == std::string::npos) break;
        begin = comma + 1;
    }
    return fields;
}

bool ReadSamples(const std::string& path,
                 std::vector<ForceCalibrationSample>& samples,
                 CaptureCsvMetadata& metadata,
                 std::string& error) {
    std::ifstream stream(path);
    if (!stream) {
        error = "cannot open calibration samples: " + path;
        return false;
    }
    std::string line;
    int line_number = 0;
    while (std::getline(stream, line)) {
        ++line_number;
        if (line.empty()) continue;
        if (line[0] == '#') {
            if (line == "# generated_by=force_sensor_calibration_capture") {
                metadata.generated_by_capture = true;
            } else if (line == "# capture_complete=true") {
                metadata.capture_complete = true;
            } else {
                constexpr const char* kRpyPrefix =
                    "# sensor_to_tool_rpy_deg=";
                if (line.rfind(kRpyPrefix, 0) == 0) {
                    Eigen::Vector3d parsed = Eigen::Vector3d::Zero();
                    if (!ParseVector3(line.substr(std::strlen(kRpyPrefix)),
                                      parsed)) {
                        error = "line " + std::to_string(line_number)
                            + " has invalid capture sensor_to_tool_rpy_deg";
                        return false;
                    }
                    if (metadata.sensor_to_tool_rpy_set
                        && (metadata.sensor_to_tool_rpy_deg - parsed).norm()
                               > 1e-12) {
                        error = "capture CSV contains conflicting sensor-to-tool RPY metadata";
                        return false;
                    }
                    metadata.sensor_to_tool_rpy_deg = parsed;
                    metadata.sensor_to_tool_rpy_set = true;
                }
            }
            continue;
        }
        const std::vector<std::string> fields = Split(line);
        if (fields.size() != 15) {
            // Permit one conventional header row.
            if (samples.empty() && line.find("r00") != std::string::npos) continue;
            error = "line " + std::to_string(line_number)
                + " must contain exactly 15 columns";
            return false;
        }
        std::vector<double> values(15);
        bool numeric = true;
        for (int i = 0; i < 15; ++i) numeric &= ParseDouble(fields[i], values[i]);
        if (!numeric) {
            if (samples.empty() && line.find("r00") != std::string::npos) continue;
            error = "line " + std::to_string(line_number) + " contains non-numeric data";
            return false;
        }
        ForceCalibrationSample sample;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                sample.rotation_base_from_sensor(row, col) = values[row * 3 + col];
            }
        }
        for (int i = 0; i < 6; ++i) sample.raw_wrench_sensor[i] = values[9 + i];
        samples.push_back(sample);
    }
    if (samples.empty()) {
        error = "calibration CSV contains no samples";
        return false;
    }
    if (metadata.generated_by_capture) {
        const std::string filename =
            std::filesystem::path(path).filename().string();
        if (filename.find(".partial") != std::string::npos) {
            error = "generated capture .partial files cannot be calibrated";
            return false;
        }
        if (!metadata.capture_complete) {
            error = "generated capture CSV is not marked capture_complete=true";
            return false;
        }
        if (!metadata.sensor_to_tool_rpy_set) {
            error = "generated capture CSV has no sensor-to-tool RPY metadata";
            return false;
        }
    }
    return true;
}

std::string CurrentUtcTimestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
    gmtime_r(&now, &utc);
    std::ostringstream output;
    output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return output.str();
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        Usage(argv[0]);
        return 2;
    }

    std::vector<ForceCalibrationSample> samples;
    CaptureCsvMetadata capture_metadata;
    std::string error;
    if (!ReadSamples(options.input_csv, samples, capture_metadata, error)) {
        std::cerr << error << '\n';
        return 3;
    }
    if (capture_metadata.generated_by_capture) {
        if (!options.sensor_to_tool_rpy_set) {
            std::cerr << "generated capture CSV requires explicit "
                         "--sensor-to-tool-rpy-deg matching capture metadata\n";
            return 3;
        }
        const Eigen::Matrix3d capture_rotation =
            RotationBaseFromControllerEuler(
                capture_metadata.sensor_to_tool_rpy_deg * (M_PI / 180.0));
        const Eigen::Matrix3d requested_rotation =
            RotationBaseFromControllerEuler(
                options.sensor_to_tool_rpy_deg * (M_PI / 180.0));
        if ((capture_rotation - requested_rotation).norm() > 1e-10) {
            std::cerr << "--sensor-to-tool-rpy-deg does not match capture CSV metadata\n";
            return 3;
        }
    }

    if (!options.probe_file.empty()) {
        const std::string probe_filename =
            std::filesystem::path(options.probe_file).filename().string();
        if (options.probe_model != options.probe_file
            && options.probe_model != probe_filename) {
            std::cerr << "--probe-model does not match --probe-file\n";
            return 3;
        }
        std::string calculated_digest;
        if (!ComputeFileSha256(options.probe_file, calculated_digest, &error)) {
            std::cerr << error << '\n';
            return 3;
        }
        std::transform(options.probe_model_sha256.begin(),
                       options.probe_model_sha256.end(),
                       options.probe_model_sha256.begin(),
                       [](unsigned char value) {
                           return static_cast<char>(std::tolower(value));
                       });
        if (!options.probe_model_sha256.empty()
            && options.probe_model_sha256 != calculated_digest) {
            std::cerr << "--probe-sha256 does not match --probe-file\n";
            return 3;
        }
        options.probe_model_sha256 = calculated_digest;
    }
    if (options.tool_chain_verified
        && (!options.sensor_to_tool_rpy_set
            || !options.sensor_to_tool_translation_set
            || !options.probe_tcp_set
            || options.probe_model_sha256.empty())) {
        std::cerr << "--tool-chain-verified requires explicit R/t/TCP and probe SHA-256\n";
        return 3;
    }

    ForceCalibration calibration;
    calibration.sensor_id = options.sensor_id;
    calibration.probe_model = options.probe_model;
    calibration.probe_model_sha256 = options.probe_model_sha256;
    calibration.created_at = CurrentUtcTimestamp();
    calibration.tool_chain_verified = options.tool_chain_verified;
    calibration.rotation_tool_from_sensor = RotationBaseFromControllerEuler(
        options.sensor_to_tool_rpy_deg * (M_PI / 180.0));
    calibration.translation_sensor_to_tool_m =
        options.sensor_to_tool_translation_m;
    calibration.probe_tcp_sensor_m = options.probe_tcp_sensor_m;
    double force_rms_n = 0.0;
    double torque_rms_nm = 0.0;
    double force_max_n = 0.0;
    double torque_max_nm = 0.0;
    if (!ForceCalibration::Fit(samples,
                               calibration,
                               &force_rms_n,
                               &torque_rms_nm,
                               &error,
                               &force_max_n,
                               &torque_max_nm)) {
        std::cerr << "Calibration failed: " << error << '\n';
        return 4;
    }

    std::cout << std::fixed << std::setprecision(6)
              << "samples: " << samples.size() << '\n'
              << "force_rms_n: " << force_rms_n << '\n'
              << "torque_rms_nm: " << torque_rms_nm << '\n'
              << "force_max_n: " << force_max_n << '\n'
              << "torque_max_nm: " << torque_max_nm << '\n'
              << "gravity_base_n: " << calibration.gravity_base_n.transpose() << '\n'
              << "force_bias_n: " << calibration.force_bias_n.transpose() << '\n'
              << "torque_bias_nm: " << calibration.torque_bias_nm.transpose() << '\n'
              << "center_of_mass_sensor_m: "
              << calibration.center_of_mass_sensor_m.transpose() << '\n';

    const bool residual_gate_passed =
        force_max_n <= options.max_force_rms_n
        && torque_max_nm <= options.max_torque_rms_nm;
    calibration.accepted_force_residual_max_n = options.max_force_rms_n;
    calibration.accepted_torque_residual_max_nm = options.max_torque_rms_nm;
    calibration.calibration_residuals_verified = residual_gate_passed;
    if (!options.allow_high_residual && !residual_gate_passed) {
        std::cerr << "Residual gate failed; calibration file was not written.\n";
        return 5;
    }
    if (!residual_gate_passed) {
        std::cerr << "WARNING: high-residual diagnostic output is marked "
                     "residuals_verified=false and cannot be used by --execute.\n";
    }
    if (!calibration.SaveJson(options.output_json, &error)) {
        std::cerr << "Cannot save calibration: " << error << '\n';
        return 6;
    }
    std::cout << "calibration_written: " << options.output_json << '\n';
    return 0;
}
