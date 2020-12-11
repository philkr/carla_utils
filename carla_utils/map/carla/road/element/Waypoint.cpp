// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/element/Waypoint.h"

#include <functional>

namespace std {

  using WaypointHash = hash<carla::road::element::Waypoint>;

  WaypointHash::result_type WaypointHash::operator()(const argument_type &waypoint) const {
    WaypointHash::result_type seed = 0u;
    seed ^= std::hash<unsigned int>()(waypoint.road_id) << 1;
    seed ^= std::hash<unsigned int>()(waypoint.section_id) << 2;
    seed ^= std::hash<unsigned int>()(waypoint.lane_id) << 3;
    seed ^= std::hash<float>()(static_cast<float>((int)(waypoint.s * 200.0))) << 4;
    return seed;
  }

} // namespace std
