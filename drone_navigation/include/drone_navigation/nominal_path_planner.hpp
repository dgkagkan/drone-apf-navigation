#pragma once

#include <cstddef>
#include <vector>

namespace drone_navigation
{

struct NominalPathPoint
{
  double x {0.0};
  double y {0.0};
  double z {0.0};
  double heading_enu_rad {0.0};
};

struct NominalPathParameters
{
  double speed_m_s {20.0};
  double max_bank_rad {0.8726646260};
  double max_pitch_rad {0.2443460953};
  double sample_distance_m {2.0};
  double arrival_radius_m {25.0};
  double altitude_tolerance_m {2.0};
  std::size_t max_points {3000};
};

std::vector<NominalPathPoint> generateNominalPath(
  const NominalPathPoint & start,
  const NominalPathPoint & goal,
  const NominalPathParameters & parameters);

}  // namespace drone_navigation
