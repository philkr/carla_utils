#include "RoadInfoGeometry.h"
#include "Geometry.h"

namespace carla {
namespace road {
namespace element {

RoadInfoGeometry::RoadInfoGeometry(double s, std::unique_ptr<Geometry> &&geom)
  : RoadInfo(s),
    _geom(std::move(geom)) {
  DEBUG_ASSERT(_geom != nullptr);
}

const Geometry &RoadInfoGeometry::GetGeometry() const {
  return *_geom;
}

} // namespace element
} // namespace road
} // namespace carla
