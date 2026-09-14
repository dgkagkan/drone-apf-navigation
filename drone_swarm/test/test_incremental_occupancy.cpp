#include <gtest/gtest.h>

#include <random>

#include "drone_swarm/incremental_occupancy.hpp"

using drone_swarm::GlobalMapFusion;
using drone_swarm::LocalOccupiedMaps;
using drone_swarm::OccupancyFusionConfig;
using drone_swarm::OccupiedKeys;
using drone_swarm::VoxelKey;
using drone_swarm::insertLocalCloud;

TEST(IncrementalOccupancy, LocalCachesAreIndependentAndClearable)
{
  LocalOccupiedMaps maps;
  const VoxelKey shared {1, 2, 3};
  const VoxelKey unique {4, 5, 6};

  EXPECT_TRUE(maps.apply("one", {{shared, true}, {unique, true}}));
  EXPECT_TRUE(maps.apply("two", {{shared, true}}));
  EXPECT_FALSE(maps.apply("two", {{shared, true}}));
  EXPECT_TRUE(maps.apply("one", {{shared, false}}));
  EXPECT_EQ(maps.localKeys("one"), std::vector<VoxelKey>({unique}));
  EXPECT_EQ(maps.localKeys("two"), std::vector<VoxelKey>({shared}));
  EXPECT_TRUE(maps.clear("one"));
  EXPECT_FALSE(maps.clear("one"));
  EXPECT_TRUE(maps.localKeys("one").empty());
  EXPECT_EQ(maps.localKeys("two"), std::vector<VoxelKey>({shared}));
}

TEST(IncrementalOccupancy, ArbitraryDroneCountMatchesIndependentReferenceSets)
{
  LocalOccupiedMaps maps;
  std::unordered_map<std::string, OccupiedKeys> reference;
  std::mt19937 random(42);
  for (int step = 0; step < 3000; ++step) {
    const auto drone = "drone_" + std::to_string(random() % 12);
    const VoxelKey key {
      static_cast<std::int64_t>(random() % 50),
      static_cast<std::int64_t>(random() % 4), 0};
    if (random() % 13 == 0) {
      maps.clear(drone);
      reference.erase(drone);
    } else {
      const bool occupied = random() % 2 != 0;
      maps.apply(drone, {{key, occupied}});
      if (occupied) reference[drone].insert(key);
      else reference[drone].erase(key);
    }
    for (const auto & [id, local] : reference) {
      const auto actual = maps.localKeys(id);
      EXPECT_EQ(OccupiedKeys(actual.begin(), actual.end()), local);
    }
  }
}

TEST(IncrementalOccupancy, LocalOctomapMatchesBatchInsertionAndReturnsRawRays)
{
  octomap::OcTree tree(0.5);
  octomap::OcTree reference_tree(0.5);
  OccupancyFusionConfig config;
  config.dynamic_timeout_sec = 100.0;
  config.static_confirmation_sec = 1000.0;
  GlobalMapFusion local_fusion(config);
  bool observed_free_ray = false;
  bool observed_temporal_removal = false;
  for (int step = 0; step < 40; ++step) {
    octomap::Pointcloud scan;
    // Initially occupy a nearby wall; later rays pass through it to a farther wall.
    for (int y = -10; y <= 10; ++y) {
      scan.push_back(step < 4 ? 4.0F : 9.0F, y * 0.5F, 1.0F);
    }
    reference_tree.insertPointCloud(scan, {0.0F, 0.0F, 1.0F}, 12.0, true, false);
    reference_tree.updateInnerOccupancy();
    const auto update = insertLocalCloud(
      tree, local_fusion, scan, {0, 0, 1}, 12.0,
      static_cast<std::int64_t>(step + 1) * 1000000000LL);
    observed_free_ray = observed_free_ray || !update.free_cells.empty();
    for (const auto & change : update.local_changes) {
      if (!change.occupied) observed_temporal_removal = true;
    }
    for (auto it = reference_tree.begin_leafs(); it != reference_tree.end_leafs(); ++it) {
      const auto * actual_node = tree.search(it.getKey());
      ASSERT_NE(actual_node, nullptr);
      EXPECT_FLOAT_EQ(actual_node->getLogOdds(), it->getLogOdds());
    }
    EXPECT_EQ(tree.size(), reference_tree.size());
  }
  EXPECT_TRUE(observed_free_ray);
  EXPECT_TRUE(observed_temporal_removal);
}

TEST(IncrementalOccupancy, SerializedTreeMatchesCoalescedOccupiedTransitions)
{
  octomap::OcTree tree(0.5);
  std::mt19937 random(73);
  OccupiedKeys expected;
  std::unordered_map<VoxelKey, bool, drone_swarm::VoxelKeyHash> pending;
  for (int step = 0; step < 2000; ++step) {
    const VoxelKey key {
      32768 + static_cast<std::int64_t>(random() % 16), 32768, 32768};
    const bool occupied = random() % 2 != 0;
    pending[key] = occupied;
    if (occupied) expected.insert(key);
    else expected.erase(key);
    if (step % 7 != 0) continue;
    drone_swarm::updateGlobalOccupiedTree(tree, pending, expected.empty());
    pending.clear();
    OccupiedKeys actual;
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
      ASSERT_TRUE(tree.isNodeOccupied(*it));
      ASSERT_EQ(it.getDepth(), tree.getTreeDepth());
      const auto & key_value = it.getKey();
      actual.insert({key_value[0], key_value[1], key_value[2]});
    }
    ASSERT_EQ(actual, expected);
  }
  drone_swarm::updateGlobalOccupiedTree(tree, {}, true);
  EXPECT_EQ(tree.size(), 0U);
  drone_swarm::updateGlobalOccupiedTree(tree, {{{32770, 32768, 32768}, true}}, false);
  EXPECT_EQ(tree.getNumLeafNodes(), 1U);
  // Replace the only cell in a single coalesced batch.
  drone_swarm::updateGlobalOccupiedTree(
    tree, {{{32770, 32768, 32768}, false}, {{32772, 32768, 32768}, true}}, false);
  EXPECT_EQ(tree.getNumLeafNodes(), 1U);
}
