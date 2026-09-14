#pragma once

#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

#include <octomap/OcTreeKey.h>

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
  std::vector<VoxelObservation> changed_voxels;
};

struct OccupancyFusionConfig
{
  double hit_probability {0.70};
  double miss_probability {0.35};
  double min_probability {0.12};
  double max_probability {0.90};
  double occupied_probability {0.50};
  // Zero retains observations until miss evidence clears them (no unseen TTL).
  double dynamic_timeout_sec {0.0};
  double static_confirmation_sec {8.0};
  std::uint32_t static_confirmation_hits {12};
};

// By default, occupied evidence persists and verified free rays reduce its
// confidence. A positive timeout opts into a temporal layer: recent hits expire
// unless repeated observations promote them into the persistent layer.
class GlobalMapFusion
{
public:
  explicit GlobalMapFusion(const OccupancyFusionConfig & config);

  FusionUpdate integrateScan(
    const octomap::KeySet & free_cells,
    const octomap::KeySet & occupied_cells,
    std::int64_t observation_time_ns);
  FusionUpdate decay(std::int64_t current_time_ns);
  void clear();

  bool occupied(const VoxelKey & key) const;
  bool staticOccupied(const VoxelKey & key) const;
  bool dynamicOccupied(const VoxelKey & key) const;
  double persistentLogOdds(const VoxelKey & key) const;
  std::vector<VoxelKey> occupiedKeys() const;
  std::vector<VoxelKey> staticOccupiedKeys() const;
  std::vector<VoxelKey> dynamicOccupiedKeys() const;
  std::size_t stateCount() const {return cells_.size();}
  bool empty() const {return occupied_count_ == 0U;}

private:
  struct Cell
  {
    double persistent_log_odds {0.0};
    std::int64_t first_confirmation_hit_ns {-1};
    std::int64_t last_hit_ns {-1};
    std::int64_t last_free_ns {-1};
    std::int64_t last_observed_ns {-1};
    std::uint32_t confirmation_hits {0};
    bool static_confirmed {false};
  };

  struct DynamicExpiration
  {
    std::int64_t expires_at_ns {0};
    std::int64_t hit_time_ns {0};
    VoxelKey key;
  };

  struct EarlierExpiration
  {
    bool operator()(
      const DynamicExpiration & lhs,
      const DynamicExpiration & rhs) const
    {
      return lhs.expires_at_ns > rhs.expires_at_ns;
    }
  };

  static VoxelKey fromOctomapKey(const octomap::OcTreeKey & key);
  double clampLogOdds(double value) const;
  bool dynamicOccupied(const Cell & cell, std::int64_t at_time_ns) const;
  bool staticOccupied(const Cell & cell) const;
  bool occupied(const Cell & cell, std::int64_t at_time_ns) const;
  void integrateFree(
    const VoxelKey & key, std::int64_t observation_time_ns,
    FusionUpdate & update);
  void integrateHit(
    const VoxelKey & key, std::int64_t observation_time_ns,
    FusionUpdate & update);
  void appendTransition(
    FusionUpdate & update, const VoxelKey & key,
    bool before_occupied, bool after_occupied);

  double hit_log_odds_ {0.0};
  double miss_log_odds_ {0.0};
  double min_log_odds_ {0.0};
  double max_log_odds_ {0.0};
  double occupied_threshold_log_odds_ {0.0};
  std::int64_t dynamic_timeout_ns_ {0};
  std::int64_t static_confirmation_ns_ {0};
  std::uint32_t static_confirmation_hits_ {1};
  std::int64_t map_time_ns_ {0};
  std::size_t occupied_count_ {0};
  std::unordered_map<VoxelKey, Cell, VoxelKeyHash> cells_;
  std::priority_queue<
    DynamicExpiration, std::vector<DynamicExpiration>, EarlierExpiration>
  dynamic_expirations_;
};

}  // namespace drone_swarm
