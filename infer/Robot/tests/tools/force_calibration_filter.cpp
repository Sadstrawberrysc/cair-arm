#include <force_calibration_self_collision.hpp>
#include <force_calibration_path.hpp>
#include <realman_kinematics.hpp>
#include <iostream>

namespace cc=calibration_collision;
int main(int argc,char** argv) {
    try {
        if (argc!=6) throw std::runtime_error("Usage: force_calibration_filter INPUT_PATH PREFIX_COUNT CONFIG OUTPUT_PATH REPORT");
        auto plan=cc::Read(argv[1]); const auto config=cc::Read(argv[3]); cc::Guard guard(config);
        const auto prefix=std::stoul(argv[2]);
        if (plan.at("joint_unit")!="deg" || prefix==0 || prefix>=plan.at("points").size())
            throw std::runtime_error("Invalid preserved prefix or path unit");
        ForceCalibrationPathOptions height;
        const auto& constraints=plan.at("workspace_constraints");
        height.minimum_body_z_m=constraints.at("minimum_body_z_m").get<double>();
        for (const auto& b:constraints.at("body_spheres")) {
            const auto& p=b.at("center_m");
            height.body_spheres.push_back({b.at("joint_index").get<int>(),
                Eigen::Vector3d(p[0].get<double>(),p[1].get<double>(),p[2].get<double>()),
                b.at("radius_m").get<double>(),b.at("name").get<std::string>()});
        }
        RMKinematics kinematics;
        cc::Json report={{"robot_motion_enabled",false},{"scope","self_collision_and_component_height_only"},
            {"preserved_count",prefix},{"removed",cc::Json::array()},{"model_id",guard.ModelId()}};
        cc::Json kept=cc::Json::array();
        for (std::size_t i=0;i<prefix;++i) kept.push_back(plan["points"][i]);
        auto previous=cc::Radians(kept.back().at("joints_deg"));
        if (config.contains("start_joints_deg")) previous=cc::Radians(config.at("start_joints_deg"));
        const auto start=guard.State(previous);
        report["assumed_start_joints_deg"]=config.value("start_joints_deg",kept.back()["joints_deg"]);
        report["start_note"]="offline start assumption, not measured robot state; capture rechecks actual start";
        report["start_state"]=cc::Diagnostic(start);
        if (!start.Valid()) {
            report["status"]="invalid_assumed_start";
            std::ofstream(argv[5])<<report.dump(2)<<'\n';
            throw std::runtime_error("Assumed start collides under new geometry; see report; no executable path produced");
        }
        const auto anchor=cc::Radians(config.value("start_joints_deg",plan.at("center_joints_deg")));
        auto leg_failure=[&](const cc::Joints& from,const cc::Joints& to)->cc::Json {
            if (!ForceCalibrationAbovePlane(from,to,kinematics,height))
                return {{"status","height_segment_rejected"}};
            const auto segment=guard.Segment(from,to);
            if (!segment.Valid()) return {{"status",cc::c::CheckStatusName(segment.status)},
                {"detail",segment.detail},{"failure",cc::Diagnostic(segment.failure)}};
            return nullptr;
        };
        std::size_t detours=0;
        for (std::size_t i=prefix;i<plan["points"].size();++i) {
            auto point=plan["points"][i];const auto q=cc::Radians(point.at("joints_deg"));
            const auto state=guard.State(q);
            bool via_center=false;
            cc::Json rejection={{"original_index",point.value("original_index",point.at("index"))}};
            if (!state.Valid()) rejection["reason"]=cc::Diagnostic(state);
            else {
                const auto direct=leg_failure(previous,q);
                if (!direct.is_null()) {
                    const auto first=leg_failure(previous,anchor),second=leg_failure(anchor,q);
                    if (first.is_null() && second.is_null()) via_center=true;
                    else rejection["reason"]={{"status","no_certified_route"},{"direct",direct},
                        {"via_start_first",first},{"via_start_second",second}};
                }
            }
            if (rejection.contains("reason")) {report["removed"].push_back(rejection);continue;}
            point["original_index"]=point.value("original_index",point.at("index"));
            point["index"]=kept.size()+1;point["via_center"]=via_center;
            if (via_center) ++detours;
            kept.push_back(point);previous=q;
        }
        report["status"]="filtered";report["retained_remaining"]=kept.size()-prefix;
        report["removed_count"]=report["removed"].size();report["total_count"]=kept.size();
        report["via_center_count"]=detours;
        plan["points"]=kept;plan["pose_count"]=kept.size();plan["preserved_prefix_count"]=prefix;
        plan["self_collision_guard"]=config;plan["self_collision_guard"]["model_id"]=guard.ModelId();
        plan["self_collision_checked_remaining"]=true;plan["environment_collision_checked"]=false;
        plan["return_to_center"]=false;plan["self_collision_filter_report"]=argv[5];
        if (config.contains("start_joints_deg")) {
            plan["original_center_joints_deg"]=plan["center_joints_deg"];
            plan["center_joints_deg"]=config.at("start_joints_deg");
        }
        // Runtime checks the actual measured start; never claim offline readiness.
        plan["collision_checked"]=false;
        std::ofstream(argv[4])<<plan.dump(2)<<'\n';std::ofstream(argv[5])<<report.dump(2)<<'\n';
        std::cout<<report.dump(2)<<'\n';return kept.size()>prefix?0:2;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
