#include <gtest/gtest.h>

#include <cstdint>

#include "drone_swarm/global_map_fusion.hpp"

namespace
{

constexpr std::int64_t second = 1000000000LL;

octomap::OcTreeKey octomapKey(
  const octomap::key_type x, const octomap::key_type y, const octomap::key_type z)
{
  return {x, y, z};
}

drone_swarm::VoxelKey voxelKey(
  const std::int64_t x, const std::int64_t y, const std::int64_t z)
{
  return {x, y, z};
}

octomap::KeySet cells(const octomap::OcTreeKey & key)
{
  octomap::KeySet result;
  result.insert(key);
  return result;
}

octomap::KeySet noCells()
{
  return octomap::KeySet();
}

drone_swarm::OccupancyFusionConfig transientConfig()
{
  drone_swarm::OccupancyFusionConfig config;
  config.dynamic_timeout_sec = 2.0;
  config.static_confirmation_sec = 10.0;
  config.static_confirmation_hits = 10;
  return config;
}

}  // namespace

TEST(GlobalMapFusion, DefaultRetainsSparseSurfaceWithoutNewObservations)
{
  drone_swarm::GlobalMapFusion fusion({});
  const auto key = octomapKey(32780, 32768, 32768);
  const auto voxel = voxelKey(32780, 32768, 32768);
  fusion.integrateScan(noCells(), cells(key), second);
  EXPECT_TRUE(fusion.decay(60 * second).changed_voxels.empty());
  EXPECT_TRUE(fusion.occupied(voxel));
  EXPECT_FALSE(fusion.empty());
  EXPECT_TRUE(fusion.integrateScan(noCells(), cells(key), 61 * second).changed_voxels.empty());
}

TEST(GlobalMapFusion, DefaultDoesNotToggleSurfaceOnOneNoisyFreeRay)
{
  drone_swarm::GlobalMapFusion fusion({});
  const auto key = octomapKey(32780, 32768, 32768);
  const auto voxel = voxelKey(32780, 32768, 32768);
  fusion.integrateScan(noCells(), cells(key), second);
  for (int step = 2; step < 20; step += 2) {
    EXPECT_TRUE(fusion.integrateScan(cells(key), noCells(), step * second).changed_voxels.empty());
    EXPECT_TRUE(fusion.occupied(voxel));
    EXPECT_TRUE(fusion.integrateScan(noCells(), cells(key), (step + 1) * second).
      changed_voxels.empty());
  }
}

TEST(GlobalMapFusion, DefaultStillClearsConfirmedGeometryWithRepeatedFreeRays)
{
  drone_swarm::GlobalMapFusion fusion({});
  const auto key = octomapKey(32780, 32768, 32768);
  const auto voxel = voxelKey(32780, 32768, 32768);
  for (int step = 1; step <= 10; ++step) {
    fusion.integrateScan(noCells(), cells(key), step * second);
  }
  for (int step = 11; step <= 13; ++step) {
    fusion.integrateScan(cells(key), noCells(), step * second);
    EXPECT_TRUE(fusion.occupied(voxel));
  }
  const auto update = fusion.integrateScan(cells(key), noCells(), 14 * second);
  ASSERT_EQ(update.changed_voxels.size(), 1U);
  EXPECT_FALSE(update.changed_voxels.front().occupied);
  EXPECT_TRUE(fusion.empty());
}

TEST(GlobalMapFusion, HitIsImmediatelyOccupiedAndExpiresWithoutAnotherRay)
{
  drone_swarm::GlobalMapFusion fusion(transientConfig());
  const auto octomap_key = octomapKey(32769, 32768, 32768);
  const auto voxel_key = voxelKey(32769, 32768, 32768);

  const auto hit = fusion.integrateScan(noCells(), cells(octomap_key), second);
  ASSERT_EQ(hit.changed_voxels.size(), 1U);
  EXPECT_TRUE(hit.changed_voxels.front().occupied);
  EXPECT_TRUE(fusion.occupied(voxel_key));
  EXPECT_TRUE(fusion.dynamicOccupied(voxel_key));
  EXPECT_FALSE(fusion.staticOccupied(voxel_key));

  EXPECT_TRUE(fusion.decay(3 * second).changed_voxels.empty());
  const auto expired = fusion.decay(3 * second + 1);
  ASSERT_EQ(expired.changed_voxels.size(), 1U);
  EXPECT_FALSE(expired.changed_voxels.front().occupied);
  EXPECT_TRUE(fusion.empty());
  EXPECT_EQ(fusion.stateCount(), 0U);
}

TEST(GlobalMapFusion, VerifiedFreeRayImmediatelyClearsTransientObstacle)
{
  drone_swarm::GlobalMapFusion fusion(transientConfig());
  const auto octomap_key = octomapKey(32769, 32768, 32768);
  const auto voxel_key = voxelKey(32769, 32768, 32768);

  fusion.integrateScan(noCells(), cells(octomap_key), second);
  const auto cleared = fusion.integrateScan(cells(octomap_key), noCells(), 2 * second);

  ASSERT_EQ(cleared.changed_voxels.size(), 1U);
  EXPECT_FALSE(cleared.changed_voxels.front().occupied);
  EXPECT_FALSE(fusion.occupied(voxel_key));
  EXPECT_EQ(fusion.stateCount(), 1U);
  fusion.decay(4 * second);
  EXPECT_EQ(fusion.stateCount(), 0U);
}

TEST(GlobalMapFusion, OccasionalMissDoesNotPreventStableGeometryConfirmation)
{
  auto config = transientConfig();
  config.static_confirmation_sec = 2.0;
  config.static_confirmation_hits = 3;
  drone_swarm::GlobalMapFusion fusion(config);
  const auto octomap_key = octomapKey(32775, 32768, 32768);
  const auto voxel_key = voxelKey(32775, 32768, 32768);

  fusion.integrateScan(noCells(), cells(octomap_key), second);
  fusion.integrateScan(cells(octomap_key), noCells(), second + second / 2);
  EXPECT_FALSE(fusion.occupied(voxel_key));
  fusion.integrateScan(noCells(), cells(octomap_key), 2 * second);
  fusion.integrateScan(noCells(), cells(octomap_key), 3 * second);

  EXPECT_TRUE(fusion.staticOccupied(voxel_key));
}

TEST(GlobalMapFusion, StableGeometryPromotesThenProbabilisticMissesClearIt)
{
  auto config = transientConfig();
  config.static_confirmation_sec = 2.0;
  config.static_confirmation_hits = 3;
  config.miss_probability = 0.30;
  drone_swarm::GlobalMapFusion fusion(config);
  const auto octomap_key = octomapKey(32770, 32768, 32768);
  const auto voxel_key = voxelKey(32770, 32768, 32768);

  fusion.integrateScan(noCells(), cells(octomap_key), second);
  fusion.integrateScan(noCells(), cells(octomap_key), 2 * second);
  fusion.integrateScan(noCells(), cells(octomap_key), 3 * second);
  ASSERT_TRUE(fusion.staticOccupied(voxel_key));

  fusion.decay(6 * second);
  EXPECT_TRUE(fusion.occupied(voxel_key));
  EXPECT_FALSE(fusion.dynamicOccupied(voxel_key));
  EXPECT_TRUE(fusion.staticOccupied(voxel_key));

  fusion.integrateScan(cells(octomap_key), noCells(), 7 * second);
  fusion.integrateScan(cells(octomap_key), noCells(), 8 * second);
  EXPECT_TRUE(fusion.staticOccupied(voxel_key));
  fusion.integrateScan(cells(octomap_key), noCells(), 9 * second);
  EXPECT_FALSE(fusion.occupied(voxel_key));
  EXPECT_TRUE(fusion.empty());
}

TEST(GlobalMapFusion, HitWinsAConflictingObservationAtTheSameTimestamp)
{
  drone_swarm::GlobalMapFusion fusion(transientConfig());
  const auto octomap_key = octomapKey(32771, 32768, 32768);
  const auto voxel_key = voxelKey(32771, 32768, 32768);

  fusion.integrateScan(noCells(), cells(octomap_key), second);
  fusion.integrateScan(cells(octomap_key), noCells(), second);
  EXPECT_TRUE(fusion.dynamicOccupied(voxel_key));
}

TEST(GlobalMapFusion, OlderCrossSensorEvidenceCannotOverwriteNewerFreeSpace)
{
  auto config = transientConfig();
  config.static_confirmation_sec = 1.0;
  config.static_confirmation_hits = 2;
  drone_swarm::GlobalMapFusion fusion(config);
  const auto octomap_key = octomapKey(32776, 32768, 32768);
  const auto voxel_key = voxelKey(32776, 32768, 32768);

  fusion.integrateScan(noCells(), cells(octomap_key), second);
  fusion.integrateScan(cells(octomap_key), noCells(), 3 * second);
  fusion.integrateScan(noCells(), cells(octomap_key), 2 * second);

  EXPECT_FALSE(fusion.occupied(voxel_key));
  EXPECT_FALSE(fusion.staticOccupied(voxel_key));
}

TEST(GlobalMapFusion, FreeSpaceDoesNotAllocateUnknownVoxels)
{
  drone_swarm::GlobalMapFusion fusion(transientConfig());
  fusion.integrateScan(
    cells(octomapKey(32772, 32768, 32768)), noCells(), second);

  EXPECT_TRUE(fusion.empty());
  EXPECT_EQ(fusion.stateCount(), 0U);
}

TEST(GlobalMapFusion, IntegratingAfterGapExpiresOlderTransientCellsFirst)
{
  drone_swarm::GlobalMapFusion fusion(transientConfig());
  const auto first = octomapKey(32773, 32768, 32768);
  const auto second_key = octomapKey(32774, 32768, 32768);

  fusion.integrateScan(noCells(), cells(first), second);
  const auto update = fusion.integrateScan(noCells(), cells(second_key), 4 * second);

  ASSERT_EQ(update.changed_voxels.size(), 2U);
  EXPECT_FALSE(fusion.occupied(voxelKey(32773, 32768, 32768)));
  EXPECT_TRUE(fusion.occupied(voxelKey(32774, 32768, 32768)));
  EXPECT_EQ(fusion.occupiedKeys().size(), 1U);
}
