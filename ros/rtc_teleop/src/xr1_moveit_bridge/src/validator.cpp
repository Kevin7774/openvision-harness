#include "xr1_moveit_bridge/validator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <istream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/collision_detection/collision_env.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <nlohmann/json.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

namespace xr1_moveit_bridge {
namespace {

using Json = nlohmann::json;
using JointMap = std::map<std::string, double>;

constexpr std::uint32_t kSchemaVersion = 1;

struct WorldBox {
    std::string object_id;
    std::string frame_id;
    std::array<double, 3> center_m{};
    std::array<double, 3> size_m{};
};

struct CandidateRequest {
    std::size_t rank{0};
    JointMap approach_joints_rad;
    JointMap grasp_joints_rad;
};

struct ValidationRequest {
    std::string urdf_path;
    std::string srdf_path;
    std::string group_name;
    std::size_t path_samples{0};
    JointMap current_joints_rad;
    std::vector<WorldBox> world_boxes;
    std::vector<CandidateRequest> candidates;
};

struct StateMetrics {
    bool bounds_ok{false};
    bool self_collision_free{false};
    bool world_collision_free{false};
    double self_distance_m{std::numeric_limits<double>::max()};
    double world_distance_m{std::numeric_limits<double>::max()};
};

struct CandidateResult {
    std::size_t rank{0};
    bool bounds_ok{false};
    bool approach_self_collision_free{false};
    bool grasp_self_collision_free{false};
    bool approach_world_collision_free{false};
    bool grasp_world_collision_free{false};
    bool path_collision_free{false};
    double minimum_self_distance_m{std::numeric_limits<double>::max()};
    double minimum_world_distance_m{std::numeric_limits<double>::max()};
    std::string first_collision_phase;
    std::size_t first_collision_sample{0};
};

int writeError(std::ostream &error_output, const std::string &code, const std::string &message) {
    error_output << Json{{"ok", false}, {"error_code", code}, {"message", message}}.dump() << '\n';
    return 2;
}

bool readString(const Json &object, const char *key, std::string &value) {
    const auto iterator = object.find(key);
    if (iterator == object.end() || !iterator->is_string()) {
        return false;
    }
    value = iterator->get_ref<const std::string &>();
    return !value.empty();
}

bool readSize(const Json &object, const char *key, std::size_t &value) {
    const auto iterator = object.find(key);
    if (iterator == object.end()) {
        return false;
    }
    std::uint64_t parsed = 0;
    if (iterator->is_number_unsigned()) {
        parsed = iterator->get<std::uint64_t>();
    } else if (iterator->is_number_integer()) {
        const auto signed_value = iterator->get<std::int64_t>();
        if (signed_value < 0) {
            return false;
        }
        parsed = static_cast<std::uint64_t>(signed_value);
    } else {
        return false;
    }
    if (parsed > std::numeric_limits<std::size_t>::max()) {
        return false;
    }
    value = static_cast<std::size_t>(parsed);
    return true;
}

bool readVector3(const Json &object, const char *key, std::array<double, 3> &value) {
    const auto iterator = object.find(key);
    if (iterator == object.end() || !iterator->is_array() || iterator->size() != value.size()) {
        return false;
    }
    for (std::size_t index = 0; index < value.size(); ++index) {
        if (!(*iterator)[index].is_number()) {
            return false;
        }
        value[index] = (*iterator)[index].get<double>();
        if (!std::isfinite(value[index])) {
            return false;
        }
    }
    return true;
}

bool readJointMap(const Json &object, const char *key, JointMap &joints) {
    const auto iterator = object.find(key);
    if (iterator == object.end() || !iterator->is_object() || iterator->empty()) {
        return false;
    }
    for (const auto &[name, value] : iterator->items()) {
        if (name.empty() || !value.is_number()) {
            return false;
        }
        const double position = value.get<double>();
        if (!std::isfinite(position)) {
            return false;
        }
        joints.emplace(name, position);
    }
    return true;
}

bool parseWorldBoxes(const Json &document, std::vector<WorldBox> &boxes) {
    const auto iterator = document.find("world_boxes");
    if (iterator == document.end() || !iterator->is_array()) {
        return false;
    }
    std::set<std::string> object_ids;
    for (const auto &item : *iterator) {
        if (!item.is_object()) {
            return false;
        }
        WorldBox box;
        if (!readString(item, "object_id", box.object_id) || !readString(item, "frame_id", box.frame_id) ||
            !readVector3(item, "center_m", box.center_m) || !readVector3(item, "size_m", box.size_m) ||
            std::any_of(box.size_m.begin(), box.size_m.end(), [](double size) { return size <= 0.0; }) ||
            !object_ids.insert(box.object_id).second) {
            return false;
        }
        boxes.push_back(std::move(box));
    }
    return true;
}

bool parseCandidates(const Json &document, std::vector<CandidateRequest> &candidates) {
    const auto iterator = document.find("candidates");
    if (iterator == document.end() || !iterator->is_array() || iterator->empty()) {
        return false;
    }
    std::set<std::size_t> ranks;
    for (const auto &item : *iterator) {
        if (!item.is_object()) {
            return false;
        }
        CandidateRequest candidate;
        if (!readSize(item, "rank", candidate.rank) || candidate.rank == 0 ||
            !readJointMap(item, "approach_joints_rad", candidate.approach_joints_rad) ||
            !readJointMap(item, "grasp_joints_rad", candidate.grasp_joints_rad) ||
            !ranks.insert(candidate.rank).second) {
            return false;
        }
        candidates.push_back(std::move(candidate));
    }
    return true;
}

bool parseRequest(const Json &document, ValidationRequest &request, std::string &error_code) {
    if (!document.is_object()) {
        error_code = "invalid_request";
        return false;
    }
    std::size_t schema_version = 0;
    if (!readSize(document, "schema_version", schema_version) || schema_version != kSchemaVersion) {
        error_code = "unsupported_schema";
        return false;
    }
    if (!readString(document, "urdf_path", request.urdf_path) ||
        !readString(document, "srdf_path", request.srdf_path) ||
        !readString(document, "group_name", request.group_name) ||
        !readSize(document, "path_samples", request.path_samples) || request.path_samples == 0 ||
        request.path_samples > 1000 || !readJointMap(document, "current_joints_rad", request.current_joints_rad) ||
        !parseWorldBoxes(document, request.world_boxes) || !parseCandidates(document, request.candidates)) {
        error_code = "invalid_request";
        return false;
    }
    return true;
}

bool validateJointNames(const moveit::core::RobotModel &model, const JointMap &joints) {
    const auto &variable_names = model.getVariableNames();
    const std::set<std::string> known(variable_names.begin(), variable_names.end());
    return std::all_of(joints.begin(), joints.end(),
                       [&known](const auto &joint) { return known.find(joint.first) != known.end(); });
}

bool addWorldBoxes(planning_scene::PlanningScene &scene, const std::vector<WorldBox> &boxes) {
    for (const auto &box : boxes) {
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = box.frame_id;
        object.id = box.object_id;
        object.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {box.size_m[0], box.size_m[1], box.size_m[2]};

        geometry_msgs::msg::Pose pose;
        pose.position.x = box.center_m[0];
        pose.position.y = box.center_m[1];
        pose.position.z = box.center_m[2];
        pose.orientation.w = 1.0;
        object.primitives.push_back(std::move(primitive));
        object.primitive_poses.push_back(std::move(pose));
        if (!scene.processCollisionObjectMsg(object)) {
            return false;
        }
    }
    return true;
}

moveit::core::RobotState makeState(const moveit::core::RobotModelConstPtr &model, const JointMap &joints) {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setVariablePositions(joints);
    state.update(true);
    return state;
}

StateMetrics measureState(const planning_scene::PlanningScene &scene, moveit::core::RobotState &state,
                          const moveit::core::JointModelGroup *group) {
    StateMetrics metrics;
    metrics.bounds_ok = state.satisfiesBounds(group);
    state.updateCollisionBodyTransforms();

    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = group->getName();
    collision_request.distance = true;

    collision_detection::CollisionResult self_result;
    scene.checkSelfCollision(collision_request, self_result, state);
    metrics.self_collision_free = !self_result.collision;
    metrics.self_distance_m = self_result.distance;

    collision_detection::CollisionResult world_result;
    scene.getCollisionEnv()->checkRobotCollision(collision_request, world_result, state,
                                                 scene.getAllowedCollisionMatrix());
    metrics.world_collision_free = !world_result.collision;
    metrics.world_distance_m = world_result.distance;
    return metrics;
}

void includeMetrics(CandidateResult &result, const StateMetrics &metrics, const std::string &phase,
                    std::size_t sample_index) {
    result.bounds_ok = result.bounds_ok && metrics.bounds_ok;
    result.minimum_self_distance_m = std::min(result.minimum_self_distance_m, metrics.self_distance_m);
    result.minimum_world_distance_m = std::min(result.minimum_world_distance_m, metrics.world_distance_m);
    if (result.first_collision_phase.empty() &&
        (!metrics.bounds_ok || !metrics.self_collision_free || !metrics.world_collision_free)) {
        result.first_collision_phase = phase;
        result.first_collision_sample = sample_index;
    }
    result.path_collision_free =
        result.path_collision_free && metrics.bounds_ok && metrics.self_collision_free && metrics.world_collision_free;
}

void validateSegment(const planning_scene::PlanningScene &scene, const moveit::core::RobotState &start,
                     const moveit::core::RobotState &target, const moveit::core::JointModelGroup *group,
                     std::size_t path_samples, const std::string &phase, CandidateResult &result) {
    moveit::core::RobotState sample(start.getRobotModel());
    for (std::size_t index = 0; index <= path_samples; ++index) {
        const double ratio = static_cast<double>(index) / static_cast<double>(path_samples);
        start.interpolate(target, ratio, sample, group);
        includeMetrics(result, measureState(scene, sample, group), phase, index);
    }
}

CandidateResult validateCandidate(const planning_scene::PlanningScene &scene,
                                  const moveit::core::RobotModelConstPtr &model,
                                  const moveit::core::JointModelGroup *group, const moveit::core::RobotState &current,
                                  const CandidateRequest &candidate, std::size_t path_samples) {
    CandidateResult result;
    result.rank = candidate.rank;
    result.bounds_ok = true;
    result.path_collision_free = true;

    auto approach = makeState(model, candidate.approach_joints_rad);
    auto grasp = makeState(model, candidate.grasp_joints_rad);
    const auto approach_metrics = measureState(scene, approach, group);
    const auto grasp_metrics = measureState(scene, grasp, group);
    result.approach_self_collision_free = approach_metrics.self_collision_free;
    result.grasp_self_collision_free = grasp_metrics.self_collision_free;
    result.approach_world_collision_free = approach_metrics.world_collision_free;
    result.grasp_world_collision_free = grasp_metrics.world_collision_free;

    validateSegment(scene, current, approach, group, path_samples, "approach", result);
    validateSegment(scene, approach, grasp, group, path_samples, "grasp", result);
    return result;
}

Json resultJson(const CandidateResult &result) {
    return Json{
        {"rank", result.rank},
        {"bounds_ok", result.bounds_ok},
        {"approach_self_collision_free", result.approach_self_collision_free},
        {"grasp_self_collision_free", result.grasp_self_collision_free},
        {"approach_world_collision_free", result.approach_world_collision_free},
        {"grasp_world_collision_free", result.grasp_world_collision_free},
        {"path_collision_free", result.path_collision_free},
        {"minimum_self_distance_m", result.minimum_self_distance_m},
        {"minimum_world_distance_m", result.minimum_world_distance_m},
        {"first_collision_phase",
         result.first_collision_phase.empty() ? Json(nullptr) : Json(result.first_collision_phase)},
        {"first_collision_sample",
         result.first_collision_phase.empty() ? Json(nullptr) : Json(result.first_collision_sample)},
    };
}

}  // namespace

int runValidator(std::istream &input, std::ostream &output, std::ostream &error_output) {
    const std::string payload((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const Json document = Json::parse(payload, nullptr, false);
    if (document.is_discarded()) {
        return writeError(error_output, "invalid_json", "request is not valid JSON");
    }

    ValidationRequest request;
    std::string parse_error;
    if (!parseRequest(document, request, parse_error)) {
        return writeError(error_output, parse_error, "request does not satisfy schema version 1");
    }

    auto urdf_model = std::make_shared<urdf::Model>();
    if (!urdf_model->initFile(request.urdf_path)) {
        return writeError(error_output, "urdf_load_failed", request.urdf_path);
    }
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initFile(*urdf_model, request.srdf_path)) {
        return writeError(error_output, "srdf_load_failed", request.srdf_path);
    }
    auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    if (robot_model->isEmpty() || !robot_model->hasJointModelGroup(request.group_name)) {
        return writeError(error_output, "model_group_missing", request.group_name);
    }
    const auto *group = robot_model->getJointModelGroup(request.group_name);
    if (!validateJointNames(*robot_model, request.current_joints_rad)) {
        return writeError(error_output, "unknown_current_joint", "current joint map contains an unknown variable");
    }
    for (const auto &candidate : request.candidates) {
        if (!validateJointNames(*robot_model, candidate.approach_joints_rad) ||
            !validateJointNames(*robot_model, candidate.grasp_joints_rad)) {
            return writeError(error_output, "unknown_candidate_joint", "candidate contains an unknown variable");
        }
    }

    planning_scene::PlanningScene scene(robot_model);
    if (!addWorldBoxes(scene, request.world_boxes)) {
        return writeError(error_output, "world_object_failed", "failed to add one or more world boxes");
    }
    const auto current = makeState(robot_model, request.current_joints_rad);

    Json candidate_results = Json::array();
    for (const auto &candidate : request.candidates) {
        candidate_results.push_back(
            resultJson(validateCandidate(scene, robot_model, group, current, candidate, request.path_samples)));
    }
    output << Json{
                  {"ok", true},
                  {"schema_version", kSchemaVersion},
                  {"backend", "moveit2_planning_scene"},
                  {"model_name", robot_model->getName()},
                  {"group_name", request.group_name},
                  {"path_samples", request.path_samples},
                  {"world_object_count", request.world_boxes.size()},
                  {"candidates", std::move(candidate_results)},
              }
                  .dump()
           << '\n';
    return 0;
}

}  // namespace xr1_moveit_bridge
