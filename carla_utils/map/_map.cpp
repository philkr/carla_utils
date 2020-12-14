/*
<%
setup_pybind11(cfg)
from pathlib import Path
cfg['sources'] = [str(s) for s in Path(filepath).parent.rglob("*.cpp") if s.name != '_map.cpp' and s.name[0] != '.']
cfg['compiler_args'] = ['-std=c++17', '-O2']
cfg['parallel'] = True
%>
*/

// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
//#include <carla/FileSystem.h>
#include <carla/opendrive/OpenDriveParser.h>
#include <carla/road/Junction.h>
#include <carla/road/Lane.h>
#include <carla/road/element/RoadInfoSignal.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/client/Landmark.h>
#include <carla/client/Junction.h>
#include <carla/road/element/LaneMarking.h>
#include <carla/road/element/Waypoint.h>
#include <carla/road/Signal.h>
#include <carla/road/SignalType.h>

#include <ostream>
#include <fstream>
#include <optional>

namespace py = pybind11;

//static auto GetJunctionWaypoints(const carla::road::Junction &self, const carla::road::Lane::LaneType lane_type) {
//  auto topology = self.GetWaypoints(lane_type);
//  py::list result;
//  for (auto &pair : topology) {
//    result.append(py::make_tuple(pair.first, pair.second));
//  }
//  return result;
//}
//
static auto GetLaneValidities(const carla::road::element::RoadInfoSignal &self){
  auto &validities = self.GetValidities();
  py::list result;
  for(auto &validity : validities) {
    result.append(py::make_tuple(validity._from_lane, validity._to_lane));
  }
  return result;
}
//
//static carla::geom::GeoLocation ToGeolocation(
//    const carla::road::Map &self,
//    const carla::geom::Location &location) {
//  return self.GetGeoReference().Transform(location);
//}

template<int N, int M, typename T>
std::array<std::array<T, N>, M> to_matrix(const std::array<T, N*M> & v) {
    std::array<std::array<T, N>, M> r;
    std::copy(&v[0], &v[N*M], &r[0][0]);
    return r;
}
#define _STR(...) [](py::object self) {\
  std::vector<std::string> names = {__VA_ARGS__};\
  py::list args;\
  for (const auto & n: names)\
      args.append(py::str(n+"=") + py::str(self.attr(n.c_str())));\
  return py::str(self.attr("__class__").attr("__name__"))+py::str("(") + py::str(", ").attr("join")(args) + py::str(")");\
}

PYBIND11_MODULE(_map, m) {
  namespace cc = carla::client;
  namespace cr = carla::road;
  namespace cg = carla::geom;
  namespace cre = carla::road::element;

  // ===========================================================================
  // -- Enums ------------------------------------------------------------------
  // ===========================================================================

  py::enum_<cr::Lane::LaneType>(m, "LaneType")
    .value("NONE", cr::Lane::LaneType::None) // None is reserved in Python3
    .value("Driving", cr::Lane::LaneType::Driving)
    .value("Stop", cr::Lane::LaneType::Stop)
    .value("Shoulder", cr::Lane::LaneType::Shoulder)
    .value("Biking", cr::Lane::LaneType::Biking)
    .value("Sidewalk", cr::Lane::LaneType::Sidewalk)
    .value("Border", cr::Lane::LaneType::Border)
    .value("Restricted", cr::Lane::LaneType::Restricted)
    .value("Parking", cr::Lane::LaneType::Parking)
    .value("Bidirectional", cr::Lane::LaneType::Bidirectional)
    .value("Median", cr::Lane::LaneType::Median)
    .value("Special1", cr::Lane::LaneType::Special1)
    .value("Special2", cr::Lane::LaneType::Special2)
    .value("Special3", cr::Lane::LaneType::Special3)
    .value("RoadWorks", cr::Lane::LaneType::RoadWorks)
    .value("Tram", cr::Lane::LaneType::Tram)
    .value("Rail", cr::Lane::LaneType::Rail)
    .value("Entry", cr::Lane::LaneType::Entry)
    .value("Exit", cr::Lane::LaneType::Exit)
    .value("OffRamp", cr::Lane::LaneType::OffRamp)
    .value("OnRamp", cr::Lane::LaneType::OnRamp)
    .value("Any", cr::Lane::LaneType::Any)
  ;

  py::enum_<cre::LaneMarking::LaneChange>(m, "LaneChange")
    .value("NONE", cre::LaneMarking::LaneChange::None)
    .value("Right", cre::LaneMarking::LaneChange::Right)
    .value("Left", cre::LaneMarking::LaneChange::Left)
    .value("Both", cre::LaneMarking::LaneChange::Both)
  ;

  py::enum_<cre::LaneMarking::Color>(m, "LaneMarkingColor")
    .value("Standard", cre::LaneMarking::Color::Standard)
    .value("Blue", cre::LaneMarking::Color::Blue)
    .value("Green", cre::LaneMarking::Color::Green)
    .value("Red", cre::LaneMarking::Color::Red)
    .value("White", cre::LaneMarking::Color::White)
    .value("Yellow", cre::LaneMarking::Color::Yellow)
    .value("Other", cre::LaneMarking::Color::Other)
  ;

  py::enum_<cre::LaneMarking::Type>(m, "LaneMarkingType")
    .value("NONE", cre::LaneMarking::Type::None)
    .value("Other", cre::LaneMarking::Type::Other)
    .value("Broken", cre::LaneMarking::Type::Broken)
    .value("Solid", cre::LaneMarking::Type::Solid)
    .value("SolidSolid", cre::LaneMarking::Type::SolidSolid)
    .value("SolidBroken", cre::LaneMarking::Type::SolidBroken)
    .value("BrokenSolid", cre::LaneMarking::Type::BrokenSolid)
    .value("BrokenBroken", cre::LaneMarking::Type::BrokenBroken)
    .value("BottsDots", cre::LaneMarking::Type::BottsDots)
    .value("Grass", cre::LaneMarking::Type::Grass)
    .value("Curb", cre::LaneMarking::Type::Curb)
  ;

  py::enum_<cr::SignalOrientation>(m, "LandmarkOrientation")
    .value("Positive", cr::SignalOrientation::Positive)
    .value("Negative", cr::SignalOrientation::Negative)
    .value("Both", cr::SignalOrientation::Both)
  ;
  // ===========================================================================
  // -- Map --------------------------------------------------------------------
  // ===========================================================================

  py::class_<cc::Map, std::shared_ptr<cc::Map>>(m, "Map")
    .def(py::init<std::string, std::string>(), py::arg("name"), py::arg("xodr_content"))
    .def_property_readonly("name", &cc::Map::GetName)
    .def("get_waypoint", &cc::Map::GetWaypoint, py::arg("location"), py::arg("project_to_road")=true, py::arg("lane_type")=cr::Lane::LaneType::Driving)
    .def("get_waypoint_xodr", &cc::Map::GetWaypointXODR)
    .def("get_topology", &cc::Map::GetTopology)
    .def("generate_waypoints", &cc::Map::GenerateWaypoints)
    .def("transform_to_geolocation", [](const carla::client::Map &self, const carla::geom::Location &location) { return self.GetGeoReference().Transform(location); })
    .def("to_opendrive", &cc::Map::GetOpenDrive)
//    .def("save_to_disk", &SaveOpenDriveToDisk, arg("path")="")
    .def("get_crosswalks", &cc::Map::GetAllCrosswalkZones)
    .def("get_all_landmarks", &cc::Map::GetAllLandmarks)
    .def("get_all_landmarks_from_id", &cc::Map::GetLandmarksFromId)
    .def("get_all_landmarks_of_type", &cc::Map::GetAllLandmarksOfType)
    .def("get_landmark_group", &cc::Map::GetLandmarkGroup)
    .def("__str__", _STR("name"))
  ;

  // ===========================================================================
  // -- Helper objects ---------------------------------------------------------
  // ===========================================================================

  py::class_<cre::LaneMarking>(m, "LaneMarking")
    .def_readonly("type", &cre::LaneMarking::type)
    .def_readonly("color", &cre::LaneMarking::color)
    .def_readonly("lane_change", &cre::LaneMarking::lane_change)
    .def_readonly("width", &cre::LaneMarking::width)
    .def("__str__", _STR("type", "color", "lane_change", "width"))
  ;

  py::class_<cc::Waypoint,std::shared_ptr<cc::Waypoint>>(m, "Waypoint")
    .def_property_readonly("id", &cc::Waypoint::GetId)
    .def_property_readonly("transform", &cc::Waypoint::GetTransform)
    .def_property_readonly("is_intersection", &cc::Waypoint::IsJunction) // deprecated
    .def_property_readonly("is_junction", &cc::Waypoint::IsJunction)
    .def_property_readonly("lane_width", &cc::Waypoint::GetLaneWidth)
    .def_property_readonly("road_id", &cc::Waypoint::GetRoadId)
    .def_property_readonly("section_id", &cc::Waypoint::GetSectionId)
    .def_property_readonly("lane_id", &cc::Waypoint::GetLaneId)
    .def_property_readonly("s", &cc::Waypoint::GetDistance)
    .def_property_readonly("junction_id", &cc::Waypoint::GetJunctionId)
    .def_property_readonly("lane_change", &cc::Waypoint::GetLaneChange)
    .def_property_readonly("lane_type", &cc::Waypoint::GetType)
    .def_property_readonly("right_lane_marking", &cc::Waypoint::GetRightLaneMarking)
    .def_property_readonly("left_lane_marking", &cc::Waypoint::GetLeftLaneMarking)
    .def("next", &cc::Waypoint::GetNext)
    .def("previous", &cc::Waypoint::GetPrevious)
    .def("next_until_lane_end", &cc::Waypoint::GetNextUntilLaneEnd)
    .def("previous_until_lane_start", &cc::Waypoint::GetPreviousUntilLaneStart)
    .def("get_right_lane", &cc::Waypoint::GetRight)
    .def("get_left_lane", &cc::Waypoint::GetLeft)
    .def("get_junction", &cc::Waypoint::GetJunction)
    .def("get_landmarks", &cc::Waypoint::GetAllLandmarksInDistance, py::arg("distance"), py::arg("stop_at_junction")=false)
    .def("get_landmarks_of_type", &cc::Waypoint::GetLandmarksOfTypeInDistance, py::arg("distance"), py::arg("type"), py::arg("stop_at_junction")=false)
    .def("__str__", _STR("id", "road_id", "section_id", "lane_id", "s", "lane_change", "lane_type"))
  ;

  py::class_<cc::Junction, std::shared_ptr<cc::Junction>>(m, "Junction")
    .def_property_readonly("id", &cc::Junction::GetId)
    .def_property_readonly("bounding_box", &cc::Junction::GetBoundingBox)
    .def("get_waypoints", &cc::Junction::GetWaypoints)
  ;

  py::class_<cr::SignalType>(m, "LandmarkType")
    .def_property_readonly_static("Danger", &cr::SignalType::Danger)
    .def_property_readonly_static("LanesMerging", &cr::SignalType::LanesMerging)
    .def_property_readonly_static("CautionPedestrian", &cr::SignalType::CautionPedestrian)
    .def_property_readonly_static("CautionBicycle", &cr::SignalType::CautionBicycle)
    .def_property_readonly_static("LevelCrossing", &cr::SignalType::LevelCrossing)
    .def_property_readonly_static("StopSign", &cr::SignalType::StopSign)
    .def_property_readonly_static("YieldSign", &cr::SignalType::YieldSign)
    .def_property_readonly_static("MandatoryTurnDirection", &cr::SignalType::MandatoryTurnDirection)
    .def_property_readonly_static("MandatoryLeftRightDirection", &cr::SignalType::MandatoryLeftRightDirection)
    .def_property_readonly_static("TwoChoiceTurnDirection", &cr::SignalType::TwoChoiceTurnDirection)
    .def_property_readonly_static("Roundabout", &cr::SignalType::Roundabout)
    .def_property_readonly_static("PassRightLeft", &cr::SignalType::PassRightLeft)
    .def_property_readonly_static("AccessForbidden", &cr::SignalType::AccessForbidden)
    .def_property_readonly_static("AccessForbiddenMotorvehicles", &cr::SignalType::AccessForbiddenMotorvehicles)
    .def_property_readonly_static("AccessForbiddenTrucks", &cr::SignalType::AccessForbiddenTrucks)
    .def_property_readonly_static("AccessForbiddenBicycle", &cr::SignalType::AccessForbiddenBicycle)
    .def_property_readonly_static("AccessForbiddenWeight", &cr::SignalType::AccessForbiddenWeight)
    .def_property_readonly_static("AccessForbiddenWidth", &cr::SignalType::AccessForbiddenWidth)
    .def_property_readonly_static("AccessForbiddenHeight", &cr::SignalType::AccessForbiddenHeight)
    .def_property_readonly_static("AccessForbiddenWrongDirection", &cr::SignalType::AccessForbiddenWrongDirection)
    .def_property_readonly_static("ForbiddenUTurn", &cr::SignalType::ForbiddenUTurn)
    .def_property_readonly_static("MaximumSpeed", &cr::SignalType::MaximumSpeed)
    .def_property_readonly_static("ForbiddenOvertakingMotorvehicles", &cr::SignalType::ForbiddenOvertakingMotorvehicles)
    .def_property_readonly_static("ForbiddenOvertakingTrucks", &cr::SignalType::ForbiddenOvertakingTrucks)
    .def_property_readonly_static("AbsoluteNoStop", &cr::SignalType::AbsoluteNoStop)
    .def_property_readonly_static("RestrictedStop", &cr::SignalType::RestrictedStop)
    .def_property_readonly_static("HasWayNextIntersection", &cr::SignalType::HasWayNextIntersection)
    .def_property_readonly_static("PriorityWay", &cr::SignalType::PriorityWay)
    .def_property_readonly_static("PriorityWayEnd", &cr::SignalType::PriorityWayEnd)
    .def_property_readonly_static("CityBegin", &cr::SignalType::CityBegin)
    .def_property_readonly_static("CityEnd", &cr::SignalType::CityEnd)
    .def_property_readonly_static("Highway", &cr::SignalType::Highway)
    .def_property_readonly_static("DeadEnd", &cr::SignalType::DeadEnd)
    .def_property_readonly_static("RecomendedSpeed", &cr::SignalType::RecomendedSpeed)
    .def_property_readonly_static("RecomendedSpeedEnd", &cr::SignalType::RecomendedSpeedEnd)
  ;

  py::class_<cc::Landmark, std::shared_ptr<cc::Landmark>>(m, "Landmark")
    .def_property_readonly("road_id", &cc::Landmark::GetRoadId)
    .def_property_readonly("distance", &cc::Landmark::GetDistance)
    .def_property_readonly("s", &cc::Landmark::GetS)
    .def_property_readonly("t", &cc::Landmark::GetT)
    .def_property_readonly("id", &cc::Landmark::GetId)
    .def_property_readonly("name", &cc::Landmark::GetName)
    .def_property_readonly("is_dynamic", &cc::Landmark::IsDynamic)
    .def_property_readonly("orientation", &cc::Landmark::GetOrientation)
    .def_property_readonly("z_offset", &cc::Landmark::GetZOffset)
    .def_property_readonly("country", &cc::Landmark::GetCountry)
    .def_property_readonly("type", &cc::Landmark::GetType)
    .def_property_readonly("sub_type", &cc::Landmark::GetSubType)
    .def_property_readonly("value", &cc::Landmark::GetValue)
    .def_property_readonly("unit", &cc::Landmark::GetUnit)
    .def_property_readonly("height", &cc::Landmark::GetHeight)
    .def_property_readonly("width", &cc::Landmark::GetWidth)
    .def_property_readonly("text", &cc::Landmark::GetText)
    .def_property_readonly("h_offset", &cc::Landmark::GethOffset)
    .def_property_readonly("pitch", &cc::Landmark::GetPitch)
    .def_property_readonly("roll", &cc::Landmark::GetRoll)
    .def_property_readonly("waypoint", &cc::Landmark::GetWaypoint)
    .def_property_readonly("transform", &cc::Landmark::GetTransform)
    .def("get_lane_validities", &GetLaneValidities)
    .def("__str__", _STR("id", "name", "type", "value", "transform", "width", "height"))
  ;

  /******* Geometry *******/
  namespace cg = carla::geom;
  py::class_<cg::Vector2D>(m, "Vector2D")
    .def(py::init<float, float>(), py::arg("x")=0.0f, py::arg("y")=0.0f)
    .def_readwrite("x", &cg::Vector2D::x)
    .def_readwrite("y", &cg::Vector2D::y)
    .def("__eq__", &cg::Vector2D::operator==)
    .def("__ne__", &cg::Vector2D::operator!=)
    .def(py::self += py::self)
    .def(py::self + py::self)
    .def(py::self -= py::self)
    .def(py::self - py::self)
    .def(py::self *= double())
    .def(py::self * double())
    .def(double() * py::self)
    .def(py::self /= double())
    .def(py::self / double())
    .def(double() / py::self)
  ;

  py::class_<cg::Vector3D>(m, "Vector3D")
    .def(py::init<float, float, float>(), py::arg("x")=0.0f, py::arg("y")=0.0f, py::arg("z")=0.0f)
    .def(py::init<const cg::Location &>())
    .def_readwrite("x", &cg::Vector3D::x)
    .def_readwrite("y", &cg::Vector3D::y)
    .def_readwrite("z", &cg::Vector3D::z)
    .def("__eq__", &cg::Vector3D::operator==)
    .def("__ne__", &cg::Vector3D::operator!=)
    .def(py::self += py::self)
    .def(py::self + py::self)
    .def(py::self -= py::self)
    .def(py::self - py::self)
    .def(py::self *= double())
    .def(py::self * double())
    .def(double() * py::self)
    .def(py::self /= double())
    .def(py::self / double())
    .def(double() / py::self)
    .def("__str__", _STR("x", "y", "z"))
  ;

  py::class_<cg::Location, cg::Vector3D>(m, "Location")
    .def(py::init<float, float, float>(), py::arg("x")=0.0f, py::arg("y")=0.0f, py::arg("z")=0.0f)
    .def(py::init<const cg::Vector3D &>())
    .def("distance", &cg::Location::Distance)
    .def("__eq__", &cg::Location::operator==)
    .def("__ne__", &cg::Location::operator!=)
    .def("__str__", _STR("x", "y", "z"))
  ;
  py::implicitly_convertible<cg::Vector3D, cg::Location>();
  py::implicitly_convertible<cg::Location, cg::Vector3D>();


  py::class_<cg::Rotation>(m, "Rotation")
    .def(py::init<float, float, float>(), py::arg("pitch")=0.0f, py::arg("yaw")=0.0f, py::arg("roll")=0.0f)
    .def_readwrite("pitch", &cg::Rotation::pitch)
    .def_readwrite("yaw", &cg::Rotation::yaw)
    .def_readwrite("roll", &cg::Rotation::roll)
    .def("get_forward_vector", &cg::Rotation::GetForwardVector)
    .def("get_right_vector", &cg::Rotation::GetRightVector)
    .def("get_up_vector", &cg::Rotation::GetUpVector)
    .def("__eq__", &cg::Rotation::operator==)
    .def("__ne__", &cg::Rotation::operator!=)
    .def("__str__", _STR("pitch", "yaw", "roll"))
  ;

  py::class_<cg::Transform>(m, "Transform")
    .def(py::init<cg::Location, cg::Rotation>(),
         py::arg("location")=cg::Location(), py::arg("rotation")=cg::Rotation())
    .def_readwrite("location", &cg::Transform::location)
    .def_readwrite("rotation", &cg::Transform::rotation)
    .def("transform", +[](const cg::Transform &self, cg::Vector3D location) {
      self.TransformPoint(location);
      return location;
    })
    .def("get_forward_vector", &cg::Transform::GetForwardVector)
    .def("get_right_vector", &cg::Transform::GetRightVector)
    .def("get_up_vector", &cg::Transform::GetUpVector)
    .def("get_matrix", [](const cg::Transform &self){return to_matrix<4,4>(self.GetMatrix());})
    .def("get_inverse_matrix", [](const cg::Transform &self){return to_matrix<4,4>(self.GetInverseMatrix());})
    .def("__eq__", &cg::Transform::operator==)
    .def("__ne__", &cg::Transform::operator!=)
    .def("__str__", _STR("location", "rotation"))
  ;

  py::class_<cg::BoundingBox>(m, "BoundingBox")
    .def(py::init<cg::Location, cg::Vector3D, cg::Rotation>(),
      py::arg("location")=cg::Location(), py::arg("extent")=cg::Vector3D(), py::arg("rotation")=cg::Rotation())
    .def_readwrite("location", &cg::BoundingBox::location)
    .def_readwrite("extent", &cg::BoundingBox::extent)
    .def_readwrite("rotation", &cg::BoundingBox::rotation)
    .def("contains", &cg::BoundingBox::Contains)
    .def("get_local_vertices", &cg::BoundingBox::GetLocalVertices)
    .def("get_world_vertices", &cg::BoundingBox::GetWorldVertices)
    .def("__eq__", &cg::BoundingBox::operator==)
    .def("__ne__", &cg::BoundingBox::operator!=)
    .def("__str__", _STR("location", "extent", "rotation"))
  ;

  py::class_<cg::GeoLocation>(m, "GeoLocation")
    .def(py::init<double, double, double>(), py::arg("latitude")=0.0, py::arg("longitude")=0.0, py::arg("altitude")=0.0)
    .def_readwrite("latitude", &cg::GeoLocation::latitude)
    .def_readwrite("longitude", &cg::GeoLocation::longitude)
    .def_readwrite("altitude", &cg::GeoLocation::altitude)
    .def("__eq__", &cg::GeoLocation::operator==)
    .def("__ne__", &cg::GeoLocation::operator!=)
    .def("__str__", _STR("latitude", "longitude", "altitude"))
  ;
}
