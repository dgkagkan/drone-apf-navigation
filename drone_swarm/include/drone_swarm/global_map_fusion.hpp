#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace drone_swarm
{

struct VoxelKey
{
  std::int64_t x {0};
  std::int64_t y {0};
  std::int64_t z {0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept;
};

struct VoxelObservation
{
  VoxelKey key;
  bool occupied {false};
};

struct FusionUpdate
{
  std::vector<VoxelKey> changed_keys;
};

// Keeps independent, bounded evidence for each drone and a global sum of that
// evidence. This makes a drone contribution removable without rebuilding the
// complete map and gives deterministic behavior for conflicting observations.
class GlobalMapFusion
{
public:
  GlobalMapFusion(
    double hit_log_odds,
    double miss_log_odds,
    double min_log_odds,
    double max_log_odds,
    double occupied_threshold_log_odds = 0.0);

  FusionUpdate integrate(
    const std::string & drone_id,
    const std::vector<VoxelObservation> & observations);

  // Collapse duplicate ray observations without touching shared fusion state.
  // Mapping workers can run this step in parallel and serialize only the
  // short state-update step below.
  static std::vector<VoxelObservation> collapseObservations(
    const std::vector<VoxelObservation> & observations);

  FusionUpdate integrateCollapsed(
    const std::string & drone_id,
    const std::vector<VoxelObservation> & observations);

  FusionUpdate removeDrone(const std::string & drone_id);

  double globalLogOdds(const VoxelKey & key) const;
  double droneLogOdds(const std::string & drone_id, const VoxelKey & key) const;
  bool globalOccupied(const VoxelKey & key) const;
  bool droneOccupied(const std::string & drone_id, const VoxelKey & key) const;

  std::vector<VoxelKey> globalOccupiedKeys() const;
  std::vector<VoxelKey> droneOccupiedKeys(const std::string & drone_id) const;
  std::size_t globalEvidenceCount() const;
  std::size_t droneEvidenceCount(const std::string & drone_id) const;

private:
  using Evidence = std::unordered_map<VoxelKey, double, VoxelKeyHash>;

  double clampLogOdds(double value) const;
  static void appendChanged(FusionUpdate & update, const VoxelKey & key);

  double hit_log_odds_;
  double miss_log_odds_;
  double min_log_odds_;
  double max_log_odds_;
  double occupied_threshold_log_odds_;
  Evidence global_evidence_;
  std::unordered_map<std::string, Evidence> drone_evidence_;
};

}  // namespace drone_swarm
