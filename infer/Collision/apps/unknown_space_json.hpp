#pragma once
#include "collision/collision_checker.hpp"
#include <json.hpp>

namespace uspilot::collision::app {
inline nlohmann::json UnknownSpaceDiagnostics(const StateCheck& check) {
    using Json = nlohmann::json;
    const auto point = [](const std::optional<Eigen::Vector3d>& p) -> Json {
        return p && p->allFinite() ? Json::array({p->x(), p->y(), p->z()}) : Json(nullptr);
    };
    const auto flag = [](const std::optional<bool>& value) -> Json {
        return value ? Json(*value) : Json(nullptr);
    };
    Json items = Json::array();
    for (const auto& d : check.unknown_space_diagnostics)
        items.push_back({{"body", d.body}, {"reason", d.reason},
            {"cell_center_base_m", point(d.cell_center_base_m)},
            {"body_to_cell_distance_lower_bound_m", d.body_to_cell_distance_lower_bound_m ?
                Json(*d.body_to_cell_distance_lower_bound_m) : Json(nullptr)},
            {"query_margin_m", d.query_margin_m}, {"first_cell_scope", d.first_cell_scope},
            {"cell_evidence", d.cell_evidence},
            {"geometry_only_status", d.geometry_only_status}, {"geometry_only_reason", d.geometry_only_reason},
            {"geometry_only_cell_evidence", d.geometry_only_cell_evidence},
            {"geometry_only_failure_point_base_m", point(d.geometry_only_failure_point_base_m)},
            {"center_inside_captured_robot", flag(d.center_inside_captured_robot)},
            {"capture_robot_occludes_cell", flag(d.capture_robot_occludes_cell)},
            {"geometry_only_center_inside_captured_robot", flag(d.geometry_only_center_inside_captured_robot)},
            {"geometry_only_capture_robot_occludes_cell", flag(d.geometry_only_capture_robot_occludes_cell)}});
    return items;
}
} // namespace uspilot::collision::app
