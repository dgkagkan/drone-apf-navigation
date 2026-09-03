#include "drone_swarm/global_map_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace drone_swarm
{

std::size_t VoxelKeyHash::operator()(const VoxelKey & key) const noexcept
{
  const auto hash_combine = [](std::size_t seed, std::int64_t value) {
      const auto value_hash = std::hash<std::int64_t>{}(value);
      return seed ^ (value_hash + static_cast<std::size_t>(0x9e3779b9) +
        (seed << 6U) + (seed >> 2U));
    };
  std::size_t result = 0;
  result = hash_combine(result, key.x);
  result = hash_combine(result, key.y);
  return hash_combine(result, key.z);
}

GlobalMapFusion::GlobalMapFusion(
  const double hit_log_odds,
  const double miss_log_odds,
  const double min_log_odds,
  const double max_log_odds,
  const double occupied_threshold_log_odds)
: hit_log_odds_(hit_log_odds),
  miss_log_odds_(miss_log_odds),
  min_log_odds_(std::min(min_log_odds, max_log_odds)),
  max_log_odds_(std::max(min_log_odds, max_log_odds)),
  occupied_threshold_log_odds_(occupied_threshold_log_odds)
{
}

double GlobalMapFusion::clampLogOdds(const double value) const
{
  return std::clamp(value, min_log_odds_, max_log_odds_);
}

void GlobalMapFusion::appendChanged(FusionUpdate & update, const VoxelKey & key)
{
  update.changed_keys.push_back(key);
}

FusionUpdate GlobalMapFusion::integrate(
  const std::string & drone_id,
  const std::vector<VoxelObservation> & observations)
{
  return integrateCollapsed(drone_id, collapseObservations(observations));
}

std::vector<VoxelObservation> GlobalMapFusion::collapseObservations(
  const std::vector<VoxelObservation> & observations)
{
  // A point cloud can contain many rays crossing the same voxel. Keep one
  // observation per voxel per cloud, with an occupied endpoint taking
  // precedence over a free-space ray through the same voxel.
  std::unordered_map<VoxelKey, bool, VoxelKeyHash> collapsed;
  collapsed.reserve(observations.size());
  for (const auto & observation : observations) {
    auto found = collapsed.find(observation.key);
    if (found == collapsed.end()) {
      collapsed.emplace(observation.key, observation.occupied);
    } else {
      found->second = found->second || observation.occupied;
    }
  }

  std::vector<VoxelObservation> result;
  result.reserve(collapsed.size());
  for (const auto & [key, occupied] : collapsed) {
    result.push_back({key, occupied});
  }
  return result;
}

FusionUpdate GlobalMapFusion::integrateCollapsed(
  const std::string & drone_id,
  const std::vector<VoxelObservation> & observations)
{
  FusionUpdate update;
  if (drone_id.empty()) return update;

  auto & local = drone_evidence_[drone_id];
  for (const auto & observation : observations) {
    const auto & key = observation.key;
    const bool occupied = observation.occupied;
    const auto local_found = local.find(key);
    const double before_local = local_found == local.end() ? 0.0 : local_found->second;
    const double after_local = clampLogOdds(
      before_local + (occupied ? hit_log_odds_ : miss_log_odds_));
    const double local_delta = after_local - before_local;
    if (std::abs(local_delta) < 1.0e-12) continue;

    if (std::abs(after_local) < 1.0e-12) {
      if (local_found != local.end()) local.erase(local_found);
    } else {
      local[key] = after_local;
    }

    const auto global_found = global_evidence_.find(key);
    const double before_global = global_found == global_evidence_.end() ? 0.0 :
      global_found->second;
    const double after_global = before_global + local_delta;
    if (std::abs(after_global) < 1.0e-12) {
      if (global_found != global_evidence_.end()) global_evidence_.erase(global_found);
    } else {
      global_evidence_[key] = after_global;
    }
    appendChanged(update, key);
  }
  return update;
}

FusionUpdate GlobalMapFusion::removeDrone(const std::string & drone_id)
{
  FusionUpdate update;
  const auto drone_found = drone_evidence_.find(drone_id);
  if (drone_found == drone_evidence_.end()) return update;

  for (const auto & [key, local_value] : drone_found->second) {
    const auto global_found = global_evidence_.find(key);
    if (global_found == global_evidence_.end()) continue;
    const double after_global = global_found->second - local_value;
    if (std::abs(after_global) < 1.0e-12) {
      global_evidence_.erase(global_found);
    } else {
      global_found->second = after_global;
    }
    appendChanged(update, key);
  }
  drone_evidence_.erase(drone_found);
  return update;
}

double GlobalMapFusion::globalLogOdds(const VoxelKey & key) const
{
  const auto found = global_evidence_.find(key);
  return found == global_evidence_.end() ? 0.0 : found->second;
}

double GlobalMapFusion::droneLogOdds(
  const std::string & drone_id, const VoxelKey & key) const
{
  const auto drone_found = drone_evidence_.find(drone_id);
  if (drone_found == drone_evidence_.end()) return 0.0;
  const auto found = drone_found->second.find(key);
  return found == drone_found->second.end() ? 0.0 : found->second;
}

bool GlobalMapFusion::globalOccupied(const VoxelKey & key) const
{
  return globalLogOdds(key) > occupied_threshold_log_odds_;
}

bool GlobalMapFusion::droneOccupied(
  const std::string & drone_id, const VoxelKey & key) const
{
  return droneLogOdds(drone_id, key) > occupied_threshold_log_odds_;
}

std::vector<VoxelKey> GlobalMapFusion::globalOccupiedKeys() const
{
  std::vector<VoxelKey> keys;
  for (const auto & [key, value] : global_evidence_) {
    if (value > occupied_threshold_log_odds_) keys.push_back(key);
  }
  return keys;
}

std::vector<VoxelKey> GlobalMapFusion::droneOccupiedKeys(
  const std::string & drone_id) const
{
  std::vector<VoxelKey> keys;
  const auto found = drone_evidence_.find(drone_id);
  if (found == drone_evidence_.end()) return keys;
  for (const auto & [key, value] : found->second) {
    if (value > occupied_threshold_log_odds_) keys.push_back(key);
  }
  return keys;
}

std::size_t GlobalMapFusion::globalEvidenceCount() const
{
  return global_evidence_.size();
}

std::size_t GlobalMapFusion::droneEvidenceCount(const std::string & drone_id) const
{
  const auto found = drone_evidence_.find(drone_id);
  return found == drone_evidence_.end() ? 0U : found->second.size();
}

}  // namespace drone_swarm
