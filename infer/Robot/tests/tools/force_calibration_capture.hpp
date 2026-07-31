#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <force_calibration.hpp>
#include <force_sensor.hpp>
#include <realman_command.hpp>

// One already-paired robot/sensor observation. Pairing is deliberately kept
// outside this module: the pure capture function only verifies that the two
// monotonic timestamps are sufficiently close.
struct ForceCalibrationCaptureObservation {
    RobotStateSnapshot robot;
    WrenchSample wrench;
};

// All motion limits are maximum spans over any two observations in the
// window. SI units are used throughout.
struct ForceCalibrationCaptureLimits {
    std::size_t minimum_samples = 2;
    std::chrono::milliseconds maximum_pair_time_delta{20};
    // A capture must span real time as well as contain distinct sequence
    // numbers. This rejects response bursts/backlogs that would otherwise
    // masquerade as a stationary averaging window.
    std::chrono::milliseconds minimum_window_duration{500};
    double maximum_joint_span_rad = 0.02 * 3.14159265358979323846 / 180.0;
    double maximum_position_span_m = 0.0001;
    double maximum_orientation_span_rad =
        0.08 * 3.14159265358979323846 / 180.0;
};

enum class ForceCalibrationCaptureError {
    kNone = 0,
    kInvalidLimits,
    kInsufficientSamples,
    kInvalidSensorRotation,
    kRobotInvalid,
    kRobotStale,
    kRobotError,
    kWrenchInvalid,
    kWrenchStale,
    kWrenchChecksumInvalid,
    kWrenchIoError,
    kNonFiniteValue,
    kTimestampMissing,
    kTimestampSkew,
    kRobotSequenceNotIncreasing,
    kSensorSequenceNotIncreasing,
    kRobotTimestampNotIncreasing,
    kSensorTimestampNotIncreasing,
    kWindowTooShort,
    kWindowMotion,
};

const char* ForceCalibrationCaptureErrorString(
    ForceCalibrationCaptureError error) noexcept;

struct ForceCalibrationCaptureDiagnostics {
    std::size_t observation_count = 0;
    std::size_t rejected_index = std::numeric_limits<std::size_t>::max();
    std::uint64_t first_sensor_sequence = 0;
    std::uint64_t last_sensor_sequence = 0;
    double maximum_pair_time_delta_ms = 0.0;
    double robot_window_duration_ms = 0.0;
    double sensor_window_duration_ms = 0.0;
    double maximum_joint_span_rad = 0.0;
    double maximum_position_span_m = 0.0;
    double maximum_orientation_span_rad = 0.0;
    Eigen::Matrix<double, 6, 1> mean_raw_wrench_sensor =
        Eigen::Matrix<double, 6, 1>::Zero();
    std::string message;
};

struct ForceCalibrationCaptureResult {
    bool valid = false;
    ForceCalibrationCaptureError error =
        ForceCalibrationCaptureError::kInsufficientSamples;
    ForceCalibrationSample sample;
    ForceCalibrationCaptureDiagnostics diagnostics;
};

// Validates a stationary, synchronized no-contact capture window and reduces
// it to one calibration sample. This function performs no I/O and never reads
// the current clock, so recorded windows have deterministic results.
ForceCalibrationCaptureResult CaptureForceCalibrationWindow(
    const std::vector<ForceCalibrationCaptureObservation>& observations,
    const Eigen::Matrix3d& rotation_tool_from_sensor,
    const ForceCalibrationCaptureLimits& limits);

// Emits the CSV metadata tuple with enough significant digits for an exact
// double round trip. The returned string does not include the metadata key.
std::string FormatForceCalibrationCaptureRpyDeg(
    const Eigen::Vector3d& sensor_to_tool_rpy_deg);

enum class ForceCalibrationCapturePublishStatus {
    kPublished = 0,
    // The final name is valid, but the hard-link fallback could not remove the
    // old partial name. This is a successful publication with a warning.
    kPublishedPartialRetained,
    kDestinationExists,
    kFailed,
};

struct ForceCalibrationCapturePublishResult {
    ForceCalibrationCapturePublishStatus status =
        ForceCalibrationCapturePublishStatus::kFailed;
    std::string message;

    bool published() const noexcept {
        return status == ForceCalibrationCapturePublishStatus::kPublished
            || status
                   == ForceCalibrationCapturePublishStatus::kPublishedPartialRetained;
    }
};

// Publishes a closed partial file under its final name. Without overwrite this
// operation is an atomic no-clobber: it never falls back to exists()+rename().
// Every failed publication keeps the partial file for recovery.
ForceCalibrationCapturePublishResult PublishForceCalibrationCaptureFile(
    const std::string& partial_path,
    const std::string& output_path,
    bool overwrite);
