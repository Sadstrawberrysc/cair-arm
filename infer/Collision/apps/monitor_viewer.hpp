#pragma once
#include "collision/monitor.hpp"
#include <memory>

class MonitorViewer {
  public:
    MonitorViewer(uspilot::collision::RobotModel model,
                  const uspilot::collision::MeshDirectories &directories, bool drop_zero);
    ~MonitorViewer();
    void SetPlannedPath(const std::vector<uspilot::collision::JointPositions>& path,
        const std::string& tip_link = "", const Eigen::Isometry3d& tip_from_tcp = Eigen::Isometry3d::Identity());
    void MarkPlannedPathStale();
    void SetPlanningStatus(const std::string& status);
    void SetGoalPreview(std::vector<uspilot::collision::CollisionBodyPreview> bodies,
        const uspilot::collision::StateCheck& check, const uspilot::collision::CollisionScene& scene,
        const uspilot::collision::JointPositions& captured_joints, const Eigen::Vector3d& tcp);
    void ClearGoalPreview();
    bool TakePlanningRequest();
    bool TakeExecutionRequest();
    bool TakeStopRequest();
    void SetExecutionStatus(const std::string& status);
    void SetMappedSurface(const std::optional<Eigen::Vector3d>& point, const std::string& detail);
    bool Draw(const uspilot::collision::MonitorFrame &frame, const uspilot::collision::MonitorResult &result);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
