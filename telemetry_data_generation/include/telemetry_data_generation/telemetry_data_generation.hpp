#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace telemetry_data_generation {

/// Evaluates the following trajectory using the system time.
/// @f$ P(t) = [(R sin(2 π f t); R cos(2 π f t); K t)] @f$ where:
///
/// - @f$ P(t) @f$ is the position function with respect to the time.
/// - @f$ R @f$ is the radius, which is 10 meter.
/// - @f$ f @f$ is the frequency, which is 1/10 Hz.
/// - @f$ K @f$ is the elevation constant, which is 0.1m/s.
///
/// The orientation is computed from the speed vector:
///
/// @f$ P'(t) = [(R 2 π f cos(2 π f t); -R 2 π f sin(2 π f t); K)] @f$ where:
///
/// @f$ Orientation(t) = [α, β, γ] @f$
/// α = wrap(-π/2 + 2π / 10 * t, -π, π)
/// β = -atan2(dP_z/dt, sqrt((dP_x/dt)^2 + (dP_y/dt)^2)) = -atan2(K, R 2 π f)
/// γ = atan2(dP_y/dt, dP_x/dt) = wrap(2 π f t, -π, π)
geometry_msgs::msg::PoseStamped ComputeNextPose(double t);

}  // telemetry_data_generation