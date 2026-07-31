#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

struct ForceCalibrationSample {
    Eigen::Matrix3d rotation_base_from_sensor = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 6, 1> raw_wrench_sensor =
        Eigen::Matrix<double, 6, 1>::Zero();
};

struct CompensatedWrench {
    Eigen::Matrix<double, 6, 1> sensor =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> tool =
        Eigen::Matrix<double, 6, 1>::Zero();
    std::uint64_t sequence = 0;
    std::int64_t timestamp_ns = 0;
    bool valid = false;
    std::string error;
};

// Static gravity/bias model for a force/torque sensor mounted on the tool.
// All public values use SI units: N, N*m and m.
class ForceCalibration {
public:
    static constexpr int kSchemaVersion = 1;

    int schema_version = kSchemaVersion;
    std::string sensor_id;
    std::string probe_model;
    std::string probe_model_sha256;
    std::string created_at;
    bool tool_chain_verified = false;
    bool calibration_residuals_verified = false;
    double force_residual_rms_n = 0.0;
    double torque_residual_rms_nm = 0.0;
    double force_residual_max_n = 0.0;
    double torque_residual_max_nm = 0.0;
    double accepted_force_residual_max_n = 0.6;
    double accepted_torque_residual_max_nm = 0.1;

    // Model in the sensor coordinate system:
    // measured_force  = force_bias + R_sensor_from_base * gravity_base
    // measured_torque = torque_bias + center_of_mass x gravity_sensor
    Eigen::Vector3d gravity_base_n = Eigen::Vector3d::Zero();
    Eigen::Vector3d force_bias_n = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_bias_nm = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_of_mass_sensor_m = Eigen::Vector3d::Zero();

    // R_tool_from_sensor uses the controller ZYX convention (Rz*Ry*Rx).
    // translation_sensor_to_tool_m is the vector from the sensor origin to
    // the configured tool origin, expressed in sensor axes. The probe TCP is
    // likewise expressed from the sensor origin in sensor axes.
    Eigen::Matrix3d rotation_tool_from_sensor = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation_sensor_to_tool_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d probe_tcp_sensor_m = Eigen::Vector3d::Zero();

    bool Validate(std::string* error = nullptr) const;
    bool LoadJson(const std::string& path, std::string* error = nullptr);
    bool SaveJson(const std::string& path, std::string* error = nullptr) const;

    CompensatedWrench Compensate(
        const Eigen::Matrix<double, 6, 1>& raw_wrench_sensor,
        const Eigen::Matrix3d& rotation_base_from_sensor,
        std::uint64_t sequence = 0,
        std::int64_t timestamp_ns = 0) const;

    static bool Fit(const std::vector<ForceCalibrationSample>& samples,
                    ForceCalibration& result,
                    double* force_rms_n = nullptr,
                    double* torque_rms_nm = nullptr,
                    std::string* error = nullptr,
                    double* force_max_n = nullptr,
                    double* torque_max_nm = nullptr);
};

Eigen::Matrix3d RotationBaseFromControllerEuler(
    const Eigen::Vector3d& rx_ry_rz_rad);

bool ComputeFileSha256(const std::string& path,
                       std::string& digest_hex,
                       std::string* error = nullptr);
