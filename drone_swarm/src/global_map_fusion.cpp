#include "drone_swarm/global_map_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace drone_swarm
{

namespace
{

double probabilityToLogOdds(const double probability)
{
  const double bounded = std::clamp(probability, 0.001, 0.999);
  return std::log(bounded / (1.0 - bounded));
}

std::int64_t secondsToNanoseconds(const double seconds)
{
  constexpr double nanoseconds_per_second = 1.0e9;
  const double bounded = std::max(0.0, seconds);
  if (bounded >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) /
    nanoseconds_per_second)
  {
    return std::numeric_limits<std::int64_t>::max();
  }
  return static_cast<std::int64_t>(bounded * nanoseconds_per_second);
}

std::int64_t expirationAfter(
  const std::int64_t hit_time_ns, const std::int64_t timeout_ns)
{
  const auto maximum = std::numeric_limits<std::int64_t>::max();
  if (timeout_ns >= maximum - hit_time_ns) return maximum;
  return hit_time_ns + timeout_ns + 1;
}

}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey & key) const noexcept
{
  const auto hash_combine = [](std::size_t seed, const std::int64_t value) {
      const auto value_hash = std::hash<std::int64_t>{}(value);
      return seed ^ (value_hash + static_cast<std::size_t>(0x9e3779b9) +
        (seed << 6U) + (seed >> 2U));
    };
  std::size_t result = 0;
  result = hash_combine(result, key.x);
  result = hash_combine(result, key.y);
  return hash_combine(result, key.z);
}

GlobalMapFusion::GlobalMapFusion(const OccupancyFusionConfig & config)
: hit_log_odds_(probabilityToLogOdds(config.hit_probability)),
  miss_log_odds_(probabilityToLogOdds(config.miss_probability)),
  min_log_odds_(probabilityToLogOdds(config.min_probability)),
  max_log_odds_(probabilityToLogOdds(config.max_probability)),
  occupied_threshold_log_odds_(probabilityToLogOdds(config.occupied_probability)),
  dynamic_timeout_ns_(secondsToNanoseconds(config.dynamic_timeout_sec)),
  static_confirmation_ns_(secondsToNanoseconds(config.static_confirmation_sec)),
  static_confirmation_hits_(std::max<std::uint32_t>(1U, config.static_confirmation_hits))
{
  if (min_log_odds_ > max_log_odds_) std::swap(min_log_odds_, max_log_odds_);
}

VoxelKey GlobalMapFusion::fromOctomapKey(const octomap::OcTreeKey & key)
{
  return {key[0], key[1], key[2]};
}

double GlobalMapFusion::clampLogOdds(const double value) const
{
  return std::clamp(value, min_log_odds_, max_log_odds_);
}

bool GlobalMapFusion::dynamicOccupied(
  const Cell & cell, const std::int64_t at_time_ns) const
{
  if (cell.last_hit_ns < 0 || cell.last_hit_ns < cell.last_free_ns) return false;
  if (dynamic_timeout_ns_ <= 0) return false;
  if (at_time_ns < cell.last_hit_ns) return true;
  return at_time_ns - cell.last_hit_ns <= dynamic_timeout_ns_;
}

bool GlobalMapFusion::staticOccupied(const Cell & cell) const
{
  return cell.static_confirmed &&
         cell.persistent_log_odds > occupied_threshold_log_odds_;
}

bool GlobalMapFusion::occupied(const Cell & cell, const std::int64_t at_time_ns) const
{
  return staticOccupied(cell) || dynamicOccupied(cell, at_time_ns);
}

void GlobalMapFusion::appendTransition(
  FusionUpdate & update, const VoxelKey & key,
  const bool before_occupied, const bool after_occupied)
{
  if (before_occupied == after_occupied) return;
  update.changed_voxels.push_back({key, after_occupied});
  if (after_occupied) {
    ++occupied_count_;
  } else if (occupied_count_ > 0U) {
    --occupied_count_;
  }
}

void GlobalMapFusion::integrateFree(
  const VoxelKey & key, const std::int64_t observation_time_ns,
  FusionUpdate & update)
{
  const auto found = cells_.find(key);
  if (found == cells_.end()) return;
  auto & cell = found->second;
  if (observation_time_ns < cell.last_observed_ns) return;
  const bool before_occupied = occupied(cell, map_time_ns_);

  cell.persistent_log_odds = clampLogOdds(cell.persistent_log_odds + miss_log_odds_);
  cell.last_free_ns = std::max(cell.last_free_ns, observation_time_ns);
  cell.last_observed_ns = std::max(cell.last_observed_ns, observation_time_ns);
  if (cell.static_confirmed &&
    cell.persistent_log_odds <= occupied_threshold_log_odds_)
  {
    cell.static_confirmed = false;
  }

  const bool after_occupied = occupied(cell, map_time_ns_);
  appendTransition(update, key, before_occupied, after_occupied);
  // Keep positive but currently-free evidence until its dynamic deadline. This
  // lets repeated hits confirm real geometry despite an occasional noisy miss,
  // while the composed map still clears the transient obstacle immediately.
  if (!after_occupied && !cell.static_confirmed &&
    cell.persistent_log_odds <= occupied_threshold_log_odds_)
  {
    cells_.erase(found);
  }
}

void GlobalMapFusion::integrateHit(
  const VoxelKey & key, const std::int64_t observation_time_ns,
  FusionUpdate & update)
{
  const auto found = cells_.find(key);
  if (found != cells_.end() && observation_time_ns < found->second.last_observed_ns) {
    return;
  }
  auto & cell = cells_[key];
  const bool before_occupied = occupied(cell, map_time_ns_);

  cell.persistent_log_odds = clampLogOdds(cell.persistent_log_odds + hit_log_odds_);
  if (observation_time_ns >= cell.last_hit_ns) {
    if (cell.first_confirmation_hit_ns < 0) {
      cell.first_confirmation_hit_ns = observation_time_ns;
      cell.confirmation_hits = 1;
    } else if (cell.confirmation_hits < std::numeric_limits<std::uint32_t>::max()) {
      ++cell.confirmation_hits;
    }
    cell.last_hit_ns = observation_time_ns;
    if (dynamic_timeout_ns_ > 0) {
      dynamic_expirations_.push({
        expirationAfter(observation_time_ns, dynamic_timeout_ns_),
        observation_time_ns, key});
    }
  }
  cell.last_observed_ns = std::max(cell.last_observed_ns, observation_time_ns);

  // Without a temporal layer, a measured surface belongs to the persistent
  // map immediately. Sparse ground returns must not vanish simply because the
  // sensor no longer sees them; probabilistic misses still clear the surface.
  if (dynamic_timeout_ns_ == 0 || (!cell.static_confirmed &&
    cell.confirmation_hits >= static_confirmation_hits_ &&
    cell.first_confirmation_hit_ns >= 0 &&
    cell.last_hit_ns - cell.first_confirmation_hit_ns >= static_confirmation_ns_))
  {
    cell.static_confirmed = true;
  }

  const bool after_occupied = occupied(cell, map_time_ns_);
  appendTransition(update, key, before_occupied, after_occupied);
}

FusionUpdate GlobalMapFusion::integrateScan(
  const octomap::KeySet & free_cells,
  const octomap::KeySet & occupied_cells,
  const std::int64_t observation_time_ns)
{
  const std::int64_t safe_time_ns = std::max<std::int64_t>(0, observation_time_ns);
  // Expire old dynamic cells before applying the new evidence. Besides
  // producing the correct transitions, this keeps occupied_count_ consistent
  // when a scan arrives after a long gap.
  FusionUpdate update = decay(safe_time_ns);

  // computeUpdate makes these sets disjoint. Misses are handled first so a hit
  // at the same timestamp from another sensor wins the safety conflict.
  if (free_cells.size() < cells_.size()) {
    for (const auto & key : free_cells) integrateFree(fromOctomapKey(key), safe_time_ns, update);
  } else {
    std::vector<VoxelKey> observed_free;
    observed_free.reserve(std::min(free_cells.size(), cells_.size()));
    for (const auto & [key, cell] : cells_) {
      (void)cell;
      const octomap::OcTreeKey octomap_key(
        static_cast<octomap::key_type>(key.x),
        static_cast<octomap::key_type>(key.y),
        static_cast<octomap::key_type>(key.z));
      if (free_cells.find(octomap_key) != free_cells.end()) observed_free.push_back(key);
    }
    for (const auto & key : observed_free) integrateFree(key, safe_time_ns, update);
  }
  for (const auto & key : occupied_cells) integrateHit(fromOctomapKey(key), safe_time_ns, update);
  return update;
}

FusionUpdate GlobalMapFusion::decay(const std::int64_t current_time_ns)
{
  FusionUpdate update;
  const std::int64_t safe_time_ns = std::max<std::int64_t>(0, current_time_ns);
  const std::int64_t previous_time_ns = map_time_ns_;
  map_time_ns_ = std::max(map_time_ns_, safe_time_ns);
  while (!dynamic_expirations_.empty() &&
    dynamic_expirations_.top().expires_at_ns <= map_time_ns_)
  {
    const auto expiration = dynamic_expirations_.top();
    dynamic_expirations_.pop();
    const auto found = cells_.find(expiration.key);
    if (found == cells_.end() || found->second.last_hit_ns != expiration.hit_time_ns) {
      continue;
    }
    const bool before_occupied = occupied(found->second, previous_time_ns);
    const bool after_occupied = occupied(found->second, map_time_ns_);
    appendTransition(update, found->first, before_occupied, after_occupied);
    if (!after_occupied && !found->second.static_confirmed) cells_.erase(found);
  }
  return update;
}

void GlobalMapFusion::clear()
{
  cells_.clear();
  occupied_count_ = 0;
  map_time_ns_ = 0;
  dynamic_expirations_ = {};
}

bool GlobalMapFusion::occupied(const VoxelKey & key) const
{
  const auto found = cells_.find(key);
  return found != cells_.end() && occupied(found->second, map_time_ns_);
}

bool GlobalMapFusion::staticOccupied(const VoxelKey & key) const
{
  const auto found = cells_.find(key);
  return found != cells_.end() && staticOccupied(found->second);
}

bool GlobalMapFusion::dynamicOccupied(const VoxelKey & key) const
{
  const auto found = cells_.find(key);
  return found != cells_.end() && dynamicOccupied(found->second, map_time_ns_);
}

double GlobalMapFusion::persistentLogOdds(const VoxelKey & key) const
{
  const auto found = cells_.find(key);
  return found == cells_.end() ? 0.0 : found->second.persistent_log_odds;
}

std::vector<VoxelKey> GlobalMapFusion::occupiedKeys() const
{
  std::vector<VoxelKey> keys;
  keys.reserve(occupied_count_);
  for (const auto & [key, cell] : cells_) {
    if (occupied(cell, map_time_ns_)) keys.push_back(key);
  }
  return keys;
}

std::vector<VoxelKey> GlobalMapFusion::staticOccupiedKeys() const
{
  std::vector<VoxelKey> keys;
  for (const auto & [key, cell] : cells_) {
    if (staticOccupied(cell)) keys.push_back(key);
  }
  return keys;
}

std::vector<VoxelKey> GlobalMapFusion::dynamicOccupiedKeys() const
{
  std::vector<VoxelKey> keys;
  for (const auto & [key, cell] : cells_) {
    if (dynamicOccupied(cell, map_time_ns_) && !staticOccupied(cell)) keys.push_back(key);
  }
  return keys;
}

}  // namespace drone_swarm
