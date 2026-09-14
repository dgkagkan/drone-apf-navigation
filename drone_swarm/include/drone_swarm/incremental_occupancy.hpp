#pragma once

#include <array>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <octomap/OcTree.h>

#include "drone_swarm/global_map_fusion.hpp"

namespace drone_swarm
{

using OccupiedKeys = std::unordered_set<VoxelKey, VoxelKeyHash>;

// Only used for the unpruned, occupied-only global tree. OctoMap's recursive
// deleteNode can delete an empty inner node without freeing its children array
// (assertion in OcTreeDataNode's destructor). Use public structural operations
// to collapse emptied inner nodes into genuine leaves before deleting them.
inline void eraseGlobalOccupiedLeaf(octomap::OcTree & tree, const octomap::OcTreeKey & key)
{
  if (!tree.getRoot()) return;
  constexpr auto key_bits = std::numeric_limits<octomap::key_type>::digits;
  std::array<octomap::OcTreeNode *, key_bits> parents {};
  std::array<unsigned int, key_bits> child_indices {};
  auto * node = tree.getRoot();
  const auto depth = tree.getTreeDepth();
  for (unsigned int level = 0; level < depth; ++level) {
    const auto child = octomap::computeChildIdx(key, depth - level - 1);
    if (!tree.nodeChildExists(node, child)) return;
    parents[level] = node;
    child_indices[level] = child;
    node = tree.getNodeChild(node, child);
  }
  for (unsigned int level = depth; level > 0; --level) {
    auto * parent = parents[level - 1];
    tree.deleteNodeChild(parent, child_indices[level - 1]);
    if (tree.nodeHasChildren(parent)) return;
    // Expanding then pruning identical children releases the empty children
    // array through OctoMap itself; it does not change occupied-map semantics.
    tree.expandNode(parent);
    tree.pruneNode(parent);
  }
  tree.clear();
}

inline void updateGlobalOccupiedTree(
  octomap::OcTree & tree,
  const std::unordered_map<VoxelKey, bool, VoxelKeyHash> & changes,
  const bool empty)
{
  if (empty) {
    // deleteNode alone leaves a root leaf when the final occupied cell is removed.
    tree.clear();
    return;
  }
  const auto octomap_key = [](const VoxelKey & key) {
      return octomap::OcTreeKey(
        static_cast<octomap::key_type>(key.x),
        static_cast<octomap::key_type>(key.y),
        static_cast<octomap::key_type>(key.z));
    };
  // Insert first so a nonempty final map never temporarily becomes an empty
  // root (which OctoMap would otherwise expand as a coarse occupied leaf).
  for (const auto & [key, occupied] : changes) {
    if (occupied) tree.setNodeValue(octomap_key(key), tree.getProbHitLog(), true);
  }
  for (const auto & [key, occupied] : changes) {
    if (!occupied) eraseGlobalOccupiedLeaf(tree, octomap_key(key));
  }
  if (!changes.empty()) tree.updateInnerOccupancy();
}

struct LocalCloudUpdate
{
  octomap::KeySet free_cells;
  octomap::KeySet occupied_cells;
  std::vector<VoxelObservation> local_changes;
};

// Keep a complete probabilistic OcTree for every drone, while exposing a
// temporal occupied view for RViz. The same raw ray evidence is returned so the
// global map can fuse free and occupied observations directly across sensors.
inline LocalCloudUpdate insertLocalCloud(
  octomap::OcTree & tree, GlobalMapFusion & local_fusion,
  const octomap::Pointcloud & scan, const octomap::point3d & origin,
  const double max_range, const std::int64_t observation_time_ns)
{
  LocalCloudUpdate result;
  tree.computeUpdate(
    scan, origin, result.free_cells, result.occupied_cells, max_range);
  for (const auto & key : result.free_cells) tree.updateNode(key, false, true);
  for (const auto & key : result.occupied_cells) tree.updateNode(key, true, true);
  tree.updateInnerOccupancy();
  result.local_changes = local_fusion.integrateScan(
    result.free_cells, result.occupied_cells, observation_time_ns).changed_voxels;
  return result;
}

// Cached per-drone output only. Global occupancy is intentionally not derived
// as a union of these sets: every sensor's free rays must be able to clear old
// global evidence, regardless of which drone originally observed the voxel.
class LocalOccupiedMaps
{
public:
  bool apply(
    const std::string & drone_id, const std::vector<VoxelObservation> & changes)
  {
    auto & local = local_keys_[drone_id];
    bool changed = false;
    for (const auto & change : changes) {
      if (change.occupied) {
        changed = local.insert(change.key).second || changed;
      } else {
        changed = local.erase(change.key) != 0U || changed;
      }
    }
    return changed;
  }

  bool clear(const std::string & drone_id)
  {
    const auto found = local_keys_.find(drone_id);
    if (found == local_keys_.end()) return false;
    const bool changed = !found->second.empty();
    local_keys_.erase(found);
    return changed;
  }

  std::vector<VoxelKey> localKeys(const std::string & drone_id) const
  {
    const auto local = local_keys_.find(drone_id);
    if (local == local_keys_.end()) return {};
    return {local->second.begin(), local->second.end()};
  }

private:
  std::unordered_map<std::string, OccupiedKeys> local_keys_;
};

}  // namespace drone_swarm
