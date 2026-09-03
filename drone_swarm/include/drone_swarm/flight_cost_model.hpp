#pragma once

#include <cstdint>

namespace drone_swarm
{

struct FlightPoint
{
  double x {0.0};
  double y {0.0};
  double z {0.0};
};

struct FlightCostParameters
{
  double vertical_speed_m_s {3.0};
  double fixed_wing_turn_rate_rad_s {0.4363323129985824};
  double fixed_wing_detour_factor {1.08};
  double multicopter_detour_factor {1.03};
  double transition_time_s {8.0};
  double multicopter_power_w {450.0};
  double fixed_wing_power_w {220.0};
  double transition_power_w {650.0};
  double climb_power_per_m_s_w {100.0};
  double multicopter_speed_power_coefficient {2.0};
  double fixed_wing_reference_speed_m_s {15.0};
};

struct FlightState
{
  FlightPoint position;
  double heading_ned_rad {0.0};
  bool fixed_wing {false};
};

struct FlightTarget
{
  FlightPoint position;
  double horizontal_speed_m_s {15.0};
  bool fixed_wing {true};
};

struct FlightEstimate
{
  double time_s {0.0};
  double energy_wh {0.0};
};

class FlightCostModel
{
public:
  explicit FlightCostModel(FlightCostParameters parameters);

  FlightEstimate estimateFromState(
    const FlightState & state,
    const FlightTarget & target) const;

  FlightEstimate estimateBetweenTargets(
    const FlightPoint & incoming_reference,
    const FlightTarget & from,
    const FlightTarget & to) const;

private:
  FlightEstimate estimateSegment(
    const FlightPoint & from,
    double initial_heading_ned_rad,
    bool initial_fixed_wing,
    const FlightTarget & target) const;

  FlightCostParameters parameters_;
};

}  // namespace drone_swarm
