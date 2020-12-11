// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Memory.h"
#include "carla/NonCopyable.h"
#include "carla/road/Junction.h"
#include "carla/road/RoadTypes.h"
#include "carla/geom/BoundingBox.h"
#include "carla/client/Waypoint.h"

#include <memory>
#include <vector>

namespace carla {
namespace client {

  class Map;

  class Junction
    : public std::enable_shared_from_this<Junction>,
    private NonCopyable
  {
  public:

    carla::road::JuncId GetId() const {
      return _id;
    }

    std::vector<std::pair<std::shared_ptr<Waypoint>,std::shared_ptr<Waypoint>>> GetWaypoints(
        road::Lane::LaneType type = road::Lane::LaneType::Driving) const;

    geom::BoundingBox GetBoundingBox() const;

  private:

    friend class Map;

    Junction(std::shared_ptr<const Map> parent, const road::Junction *junction);

    std::shared_ptr<const Map> _parent;

    geom::BoundingBox _bounding_box;

    road::JuncId _id;
  };

} // namespace client
} // namespace carla
