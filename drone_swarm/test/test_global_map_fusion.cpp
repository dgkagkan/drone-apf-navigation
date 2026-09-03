#include <gtest/gtest.h>

#include <vector>

#include "drone_swarm/global_map_fusion.hpp"

namespace
{

drone_swarm::VoxelObservation observation(
  const std::int64_t x, const std::int64_t y, const std::int64_t z, const bool occupied)
{
  return {{x, y, z}, occupied};
}

}  // namespace

TEST(GlobalMapFusion, KeepsContributionsFromEveryDrone)
{
  drone_swarm::GlobalMapFusion fusion(0.9, -0.4, -2.0, 2.0);
  fusion.integrate("drone_1", {observation(1, 2, 3, true)});
  fusion.integrate("drone_2", {observation(4, 5, 6, true)});

  EXPECT_TRUE(fusion.globalOccupied({1, 2, 3}));
  EXPECT_TRUE(fusion.globalOccupied({4, 5, 6}));
  EXPECT_EQ(fusion.droneEvidenceCount("drone_1"), 1U);
  EXPECT_EQ(fusion.droneEvidenceCount("drone_2"), 1U);
}

TEST(GlobalMapFusion, FreeRayCanClearAnEarlierOccupiedVoxel)
{
  drone_swarm::GlobalMapFusion fusion(0.9, -0.9, -2.0, 2.0);
  fusion.integrate("drone_1", {observation(1, 0, 0, true)});
  fusion.integrate("drone_1", {observation(1, 0, 0, false)});

  EXPECT_FALSE(fusion.globalOccupied({1, 0, 0}));
  EXPECT_DOUBLE_EQ(fusion.globalLogOdds({1, 0, 0}), 0.0);
}

TEST(GlobalMapFusion, ConflictingDronesAreFusedAsEvidence)
{
  drone_swarm::GlobalMapFusion fusion(0.9, -0.4, -2.0, 2.0);
  fusion.integrate("drone_1", {observation(1, 0, 0, true)});
  fusion.integrate("drone_2", {observation(1, 0, 0, false)});

  EXPECT_GT(fusion.globalLogOdds({1, 0, 0}), 0.0);
  EXPECT_TRUE(fusion.globalOccupied({1, 0, 0}));
}

TEST(GlobalMapFusion, RemovingOneDroneLeavesOtherEvidenceIntact)
{
  drone_swarm::GlobalMapFusion fusion(0.9, -0.4, -2.0, 2.0);
  fusion.integrate("drone_1", {observation(1, 0, 0, true)});
  fusion.integrate("drone_2", {observation(1, 0, 0, true)});
  fusion.removeDrone("drone_1");

  EXPECT_TRUE(fusion.globalOccupied({1, 0, 0}));
  EXPECT_DOUBLE_EQ(fusion.globalLogOdds({1, 0, 0}), 0.9);
  EXPECT_EQ(fusion.droneEvidenceCount("drone_1"), 0U);
}

TEST(GlobalMapFusion, DuplicatePointsProduceOneBoundedUpdate)
{
  drone_swarm::GlobalMapFusion fusion(0.9, -0.4, -2.0, 2.0);
  fusion.integrate("drone_1", {
    observation(1, 0, 0, true), observation(1, 0, 0, true),
    observation(1, 0, 0, false)});

  EXPECT_DOUBLE_EQ(fusion.droneLogOdds("drone_1", {1, 0, 0}), 0.9);
}
