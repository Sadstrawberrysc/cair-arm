#pragma once

#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

enum class ContactEstimateError {
    None = 0,
    ModelNotLoaded,
    NonFiniteWrench,
    ForceTooSmall,
    IllConditionedSystem = 5,
    ResidualTooLarge,
    NoValidSurface,
};

const char *ContactEstimateErrorString(ContactEstimateError error) noexcept;

struct ContactEstimate {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    double residual = std::numeric_limits<double>::infinity();
    double point_error_m = std::numeric_limits<double>::infinity();
    bool valid = false;
    ContactEstimateError error = ContactEstimateError::ModelNotLoaded;
};

class ContactLocation
{
public:
    // Loads an ASCII STL transactionally. On failure the model is empty and a
    // diagnostic is available through lastError(); this function never exits.
    bool LoadSTL(const std::string &filename);
    const std::string &lastError() const noexcept { return last_error_; }

    // The wrench order is [Fx, Fy, Fz, Mx, My, Mz] in SI units. The returned
    // point is expressed in the same frame as the wrench and mesh vertices.
    ContactEstimate estimateContactPoint(
        const Eigen::Matrix<double, 6, 1> &wrench,
        double min_force_norm = 1e-3,
        double max_point_error_m = 0.003) const;

private:
    std::vector<Eigen::Vector3d> normal;
    std::vector<Eigen::Matrix3d> vertex;
    std::string last_error_;
};
