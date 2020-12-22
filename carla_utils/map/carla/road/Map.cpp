// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/Map.h"
#include "carla/Exception.h"
#include "carla/geom/Math.h"
//#include "carla/road/MeshFactory.h"
#include "carla/road/element/LaneCrossingCalculator.h"
#include "carla/road/element/RoadInfoCrosswalk.h"
#include "carla/road/element/RoadInfoElevation.h"
#include "carla/road/element/RoadInfoGeometry.h"
#include "carla/road/element/RoadInfoLaneOffset.h"
#include "carla/road/element/RoadInfoLaneWidth.h"
#include "carla/road/element/RoadInfoMarkRecord.h"
#include "carla/road/element/RoadInfoSignal.h"

#include <vector>
#include <unordered_map>
#include <stdexcept>

namespace carla {
namespace road {
  using namespace carla::road::element;

  /// We use this epsilon to shift the waypoints away from the edges of the lane
  /// sections to avoid floating point precision errors.
  static constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

  // ===========================================================================
  // -- Static local methods ---------------------------------------------------
  // ===========================================================================

  template <typename T>
  static std::vector<T> ConcatVectors(std::vector<T> dst, std::vector<T> src) {
    if (src.size() > dst.size()) {
      return ConcatVectors(src, dst);
    }
    dst.insert(
        dst.end(),
        std::make_move_iterator(src.begin()),
        std::make_move_iterator(src.end()));
    return dst;
  }

  static double GetDistanceAtStartOfLane(const Lane &lane) {
    if (lane.GetId() <= 0) {
      return lane.GetDistance() + 10.0 * EPSILON;
    } else {
      return lane.GetDistance() + lane.GetLength() - 10.0 * EPSILON;
    }
  }

  static double GetDistanceAtEndOfLane(const Lane &lane) {
    if (lane.GetId() > 0) {
      return lane.GetDistance() + 10.0 * EPSILON;
    } else {
      return lane.GetDistance() + lane.GetLength() - 10.0 * EPSILON;
    }
  }

  /// Return a waypoint for each drivable lane on @a lane_section.
  template <typename FuncT>
  static void ForEachDrivableLaneImpl(
      RoadId road_id,
      const LaneSection &lane_section,
      double distance,
      FuncT &&func) {
    for (const auto &pair : lane_section.GetLanes()) {
      const auto &lane = pair.second;
      if (lane.GetId() == 0) {
        continue;
      }
      if ((static_cast<uint32_t>(lane.GetType()) & static_cast<uint32_t>(Lane::LaneType::Driving)) > 0) {
        std::forward<FuncT>(func)(Waypoint{
            road_id,
            lane_section.GetId(),
            lane.GetId(),
            distance < 0.0 ? GetDistanceAtStartOfLane(lane) : distance});
      }
    }
  }

  template <typename FuncT>
  static void ForEachLaneImpl(
      RoadId road_id,
      const LaneSection &lane_section,
      double distance,
      Lane::LaneType lane_type,
      FuncT &&func) {
    for (const auto &pair : lane_section.GetLanes()) {
      const auto &lane = pair.second;
      if (lane.GetId() == 0) {
        continue;
      }
      if ((static_cast<uint32_t>(lane.GetType()) & static_cast<uint32_t>(lane_type)) > 0) {
        std::forward<FuncT>(func)(Waypoint{
            road_id,
            lane_section.GetId(),
            lane.GetId(),
            distance < 0.0 ? GetDistanceAtStartOfLane(lane) : distance});
      }
    }
  }

  /// Return a waypoint for each drivable lane on each lane section of @a road.
  template <typename FuncT>
  static void ForEachDrivableLane(const Road &road, FuncT &&func) {
    for (const auto &lane_section : road.GetLaneSections()) {
      ForEachDrivableLaneImpl(
          road.GetId(),
          lane_section,
          -1.0, // At start of the lane
          std::forward<FuncT>(func));
    }
  }

  /// Return a waypoint for each lane of the specified type on each lane section of @a road.
  template <typename FuncT>
  static void ForEachLane(const Road &road, Lane::LaneType lane_type, FuncT &&func) {
    for (const auto &lane_section : road.GetLaneSections()) {
      ForEachLaneImpl(
          road.GetId(),
          lane_section,
          -1.0, // At start of the lane
          lane_type,
          std::forward<FuncT>(func));
    }
  }

  /// Return a waypoint for each drivable lane at @a distance on @a road.
  template <typename FuncT>
  static void ForEachDrivableLaneAt(const Road &road, double distance, FuncT &&func) {
    for (const auto &lane_section : road.GetLaneSectionsAt(distance)) {
      ForEachDrivableLaneImpl(
          road.GetId(),
          lane_section,
          distance,
          std::forward<FuncT>(func));
    }
  }

  /// Assumes road_id and section_id are valid.
  static bool IsLanePresent(const MapData &data, Waypoint waypoint) {
    const auto &section = data.GetRoad(waypoint.road_id).GetLaneSectionById(waypoint.section_id);
    return section.ContainsLane(waypoint.lane_id);
  }

  Map::Map(MapData m) : _data(std::move(m)) {
    CreateKDtree();
  }
  // ===========================================================================
  // -- Map: Geometry ----------------------------------------------------------
  // ===========================================================================

  std::optional<Waypoint> Map::GetClosestWaypointOnRoad(
      const geom::Location &pos,
      uint32_t lane_type) const {
    // Find the closest point on the lane
    std::vector<Waypoint> query_result =
        _kdtree->GetNearestNeighboursWithFilter(pos,
        [&](const Waypoint & w, float dist) {
          const Lane &lane = GetLane(w);
          return (lane_type & static_cast<uint32_t>(lane.GetType())) > 0;
        });

    if (query_result.size() == 0) {
      return std::optional<Waypoint>{};
    }
    // Find the closest points two neighbors
    Waypoint q = query_result[0];
    const auto & lane = GetLane(q);
    Waypoint a = q, b = q;
    a.s = std::max(q.s-1, lane.GetDistance());
    b.s = std::min(q.s+1, lane.GetDistance() + lane.GetLength() - EPSILON);

    // All that geometry makes my head hurt, let's just brute force it
    auto lq = ComputeTransform(q).location;
    auto la = ComputeTransform(a).location;
    auto lb = ComputeTransform(b).location;
    for(int i=0; i<5; i++) {
      if ((pos-la).SquaredLength() > (pos-lb).SquaredLength()) {
        a = q;
        la = lq;
      } else {
        b = q;
        lb = lq;
      }
      q.s = 0.5 * (a.s + b.s);
      lq = ComputeTransform(q).location;
    }
    return q;
  }
  std::optional<Waypoint> Map::GetWaypoint(
      const geom::Location &pos,
      uint32_t lane_type) const {
    std::optional<Waypoint> w = GetClosestWaypointOnRoad(pos, lane_type);

    if (!w.has_value()) {
      return w;
    }

    const auto dist = geom::Math::Distance2D(ComputeTransform(*w).location, pos);
    const auto lane_width_info = GetLane(*w).GetInfo<RoadInfoLaneWidth>(w->s);
    const auto half_lane_width =
        lane_width_info->GetPolynomial().Evaluate(w->s) * 0.5;

    if (dist < half_lane_width) {
      return w;
    }

    return std::optional<Waypoint>{};
  }

  std::optional<Waypoint> Map::GetWaypoint(
      RoadId road_id,
      LaneId lane_id,
      float s) const {

    // define the waypoint with the known parameters
    Waypoint waypoint;
    waypoint.road_id = road_id;
    waypoint.lane_id = lane_id;
    waypoint.s = s;

    // check the road
    if (!_data.ContainsRoad(waypoint.road_id)) {
      return std::optional<Waypoint>{};
    }
    const Road &road = _data.GetRoad(waypoint.road_id);

    // check the 's' distance
    if (s < 0.0f || s >= road.GetLength()) {
      return std::optional<Waypoint>{};
    }

    // check the section
    bool lane_found = false;
    for (auto &section : road.GetLaneSectionsAt(s)) {
      if (section.ContainsLane(lane_id)) {
        waypoint.section_id = section.GetId();
        lane_found = true;
        break;
      }
    }

    // check the lane id
    if (!lane_found) {
      return std::optional<Waypoint>{};
    }

    return waypoint;
  }

  geom::Transform Map::ComputeTransform(Waypoint waypoint) const {
    return GetLane(waypoint).ComputeTransform(waypoint.s);
  }

  // ===========================================================================
  // -- Map: Road information --------------------------------------------------
  // ===========================================================================

  Lane::LaneType Map::GetLaneType(const Waypoint waypoint) const {
    return GetLane(waypoint).GetType();
  }

  double Map::GetLaneWidth(const Waypoint waypoint) const {
    const auto s = waypoint.s;

    const auto &lane = GetLane(waypoint);
    RELEASE_ASSERT(lane.GetRoad() != nullptr);
    RELEASE_ASSERT(s <= lane.GetRoad()->GetLength());

    const auto lane_width_info = lane.GetInfo<RoadInfoLaneWidth>(s);
    RELEASE_ASSERT(lane_width_info != nullptr);

    return lane_width_info->GetPolynomial().Evaluate(s);
  }

  JuncId Map::GetJunctionId(RoadId road_id) const {
    return _data.GetRoad(road_id).GetJunctionId();
  }

  bool Map::IsJunction(RoadId road_id) const {
    return _data.GetRoad(road_id).IsJunction();
  }

  std::pair<const RoadInfoMarkRecord *, const RoadInfoMarkRecord *>
      Map::GetMarkRecord(const Waypoint waypoint) const {
    // if lane Id is 0, just return a pair of nulls
    if (waypoint.lane_id == 0)
      return std::make_pair(nullptr, nullptr);

    const auto s = waypoint.s;

    const auto &current_lane = GetLane(waypoint);
    RELEASE_ASSERT(current_lane.GetRoad() != nullptr);
    RELEASE_ASSERT(s <= current_lane.GetRoad()->GetLength());

    const auto inner_lane_id = waypoint.lane_id < 0 ?
        waypoint.lane_id + 1 :
        waypoint.lane_id - 1;

    const auto &inner_lane = current_lane.GetRoad()->GetLaneById(waypoint.section_id, inner_lane_id);

    auto current_lane_info = current_lane.GetInfo<RoadInfoMarkRecord>(s);
    auto inner_lane_info = inner_lane.GetInfo<RoadInfoMarkRecord>(s);

    return std::make_pair(current_lane_info, inner_lane_info);
  }

  std::vector<Map::SignalSearchData> Map::GetSignalsInDistance(
      Waypoint waypoint, double distance, bool stop_at_junction) const {

    const auto &lane = GetLane(waypoint);
    const bool forward = (waypoint.lane_id <= 0);
    const double signed_distance = forward ? distance : -distance;
    const double relative_s = waypoint.s - lane.GetDistance();
    const double remaining_lane_length = forward ? lane.GetLength() - relative_s : relative_s;
    DEBUG_ASSERT(remaining_lane_length >= 0.0);

    auto &road =_data.GetRoad(waypoint.road_id);
    std::vector<SignalSearchData> result;

    // If after subtracting the distance we are still in the same lane, return
    // same waypoint with the extra distance.
    if (distance <= remaining_lane_length) {
      auto signals = road.GetInfosInRange<RoadInfoSignal>(
          waypoint.s, waypoint.s + signed_distance);
      for(auto* signal : signals){
        double distance_to_signal = 0;
        if (waypoint.lane_id < 0){
          distance_to_signal = signal->GetDistance() - waypoint.s;
        } else {
          distance_to_signal = waypoint.s - signal->GetDistance();
        }
        Waypoint signal_waypoint = GetNext(waypoint, distance_to_signal).front();
        SignalSearchData signal_data{signal, signal_waypoint, distance_to_signal};
        result.emplace_back(signal_data);
      }
      return result;
    }
    const double signed_remaining_length = forward ? remaining_lane_length : -remaining_lane_length;

    //result = road.GetInfosInRange<RoadInfoSignal>(waypoint.s, waypoint.s + signed_remaining_length);
    auto signals = road.GetInfosInRange<RoadInfoSignal>(
        waypoint.s, waypoint.s + signed_remaining_length);
    for(auto* signal : signals){
      double distance_to_signal = 0;
      if (waypoint.lane_id < 0){
        distance_to_signal = signal->GetDistance() - waypoint.s;
      } else {
        distance_to_signal = waypoint.s - signal->GetDistance();
      }
      Waypoint signal_waypoint = GetNext(waypoint, distance_to_signal).front();
      SignalSearchData signal_data{signal, signal_waypoint, distance_to_signal};
      result.emplace_back(signal_data);
    }
    // If we run out of remaining_lane_length we have to go to the successors.
    for (const auto &successor : GetSuccessors(waypoint)) {
      if(_data.GetRoad(successor.road_id).IsJunction() && stop_at_junction){
        continue;
      }
      auto sucessor_signals = GetSignalsInDistance(
          successor, distance - remaining_lane_length, stop_at_junction);
      for(auto& signal : sucessor_signals){
        signal.accumulated_s += remaining_lane_length;
      }
      result = ConcatVectors(result, sucessor_signals);
    }
    return result;
  }

  std::vector<const element::RoadInfoSignal*>
      Map::GetAllSignalReferences() const {
    std::vector<const element::RoadInfoSignal*> result;
    for (const auto& road_pair : _data.GetRoads()) {
      const auto &road = road_pair.second;
      auto road_infos = road.GetInfos<element::RoadInfoSignal>();
      for(const auto* road_info : road_infos) {
        result.push_back(road_info);
      }
    }
    return result;
  }

  std::vector<LaneMarking> Map::CalculateCrossedLanes(
      const geom::Location &origin,
      const geom::Location &destination) const {
    return LaneCrossingCalculator::Calculate(*this, origin, destination);
  }

  std::vector<geom::Location> Map::GetAllCrosswalkZones() const {
    std::vector<geom::Location> result;

    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      std::vector<const RoadInfoCrosswalk *> crosswalks = road.GetInfos<RoadInfoCrosswalk>();
      if (crosswalks.size() > 0) {
        for (auto crosswalk : crosswalks) {
          // waypoint only at start position
          std::vector<geom::Location> points;
          Waypoint waypoint;
          geom::Transform base;
          for (const auto &section : road.GetLaneSectionsAt(crosswalk->GetS())) {
            // get the section with the center lane
            for (const auto &lane : section.GetLanes()) {
              // is the center line
              if (lane.first == 0) {
                // get the center point
                waypoint.road_id = pair.first;
                waypoint.section_id = section.GetId();
                waypoint.lane_id = 0;
                waypoint.s = crosswalk->GetS();
                base = ComputeTransform(waypoint);
              }
            }
          }

          // move perpendicular ('t')
          geom::Transform pivot = base;
          pivot.rotation.yaw -= geom::Math::ToDegrees<float>(static_cast<float>(crosswalk->GetHeading()));
          pivot.rotation.yaw -= 90;   // move perpendicular to 's' for the lateral offset
          geom::Vector3D v(static_cast<float>(crosswalk->GetT()), 0.0f, 0.0f);
          pivot.TransformPoint(v);
          // restore pivot position and orientation
          pivot = base;
          pivot.location = v;
          pivot.rotation.yaw -= geom::Math::ToDegrees<float>(static_cast<float>(crosswalk->GetHeading()));

          // calculate all the corners
          for (auto corner : crosswalk->GetPoints()) {
            geom::Vector3D v2(
                static_cast<float>(corner.u),
                static_cast<float>(corner.v),
                static_cast<float>(corner.z));
            // set the width larger to contact with the sidewalk (in case they have gutter area)
            if (corner.u < 0) {
              v2.x -= 1.0f;
            } else {
              v2.x += 1.0f;
            }
            pivot.TransformPoint(v2);
            result.push_back(v2);
          }
        }
      }
    }
    return result;
  }

  // ===========================================================================
  // -- Map: Waypoint generation -----------------------------------------------
  // ===========================================================================

  std::vector<Waypoint> Map::GetSuccessors(const Waypoint waypoint) const {
    const auto &next_lanes = GetLane(waypoint).GetNextLanes();
    std::vector<Waypoint> result;
    result.reserve(next_lanes.size());
    for (auto *next_lane : next_lanes) {
      RELEASE_ASSERT(next_lane != nullptr);
      const auto lane_id = next_lane->GetId();
      RELEASE_ASSERT(lane_id != 0);
      const auto *section = next_lane->GetLaneSection();
      RELEASE_ASSERT(section != nullptr);
      const auto *road = next_lane->GetRoad();
      RELEASE_ASSERT(road != nullptr);
      const auto distance = GetDistanceAtStartOfLane(*next_lane);
      result.emplace_back(Waypoint{road->GetId(), section->GetId(), lane_id, distance});
    }
    return result;
  }

  std::vector<Waypoint> Map::GetPredecessors(const Waypoint waypoint) const {
    const auto &prev_lanes = GetLane(waypoint).GetPreviousLanes();
    std::vector<Waypoint> result;
    result.reserve(prev_lanes.size());
    for (auto *next_lane : prev_lanes) {
      RELEASE_ASSERT(next_lane != nullptr);
      const auto lane_id = next_lane->GetId();
      RELEASE_ASSERT(lane_id != 0);
      const auto *section = next_lane->GetLaneSection();
      RELEASE_ASSERT(section != nullptr);
      const auto *road = next_lane->GetRoad();
      RELEASE_ASSERT(road != nullptr);
      const auto distance = GetDistanceAtEndOfLane(*next_lane);
      result.emplace_back(Waypoint{road->GetId(), section->GetId(), lane_id, distance});
    }
    return result;
  }

  std::vector<Waypoint> Map::GetNext(
      const Waypoint waypoint,
      const double distance) const {
    RELEASE_ASSERT(distance > 0.0);
    const auto &lane = GetLane(waypoint);
    const bool forward = (waypoint.lane_id <= 0);
    const double signed_distance = forward ? distance : -distance;
    const double relative_s = waypoint.s - lane.GetDistance();
    const double remaining_lane_length = forward ? lane.GetLength() - relative_s : relative_s;
    DEBUG_ASSERT(remaining_lane_length >= 0.0);

    // If after subtracting the distance we are still in the same lane, return
    // same waypoint with the extra distance.
    if (distance <= remaining_lane_length) {
      Waypoint result = waypoint;
      result.s += signed_distance;
      result.s += forward ? -EPSILON : EPSILON;
      RELEASE_ASSERT(result.s > 0.0);
      return { result };
    }

    // If we run out of remaining_lane_length we have to go to the successors.
    std::vector<Waypoint> result;
    for (const auto &successor : GetSuccessors(waypoint)) {
      DEBUG_ASSERT(
          successor.road_id != waypoint.road_id ||
          successor.section_id != waypoint.section_id ||
          successor.lane_id != waypoint.lane_id);
      result = ConcatVectors(result, GetNext(successor, distance - remaining_lane_length));
    }
    return result;
  }

  std::vector<Waypoint> Map::GetPrevious(
      const Waypoint waypoint,
      const double distance) const {
    RELEASE_ASSERT(distance > 0.0);
    const auto &lane = GetLane(waypoint);
    const bool forward = !(waypoint.lane_id <= 0);
    const double signed_distance = forward ? distance : -distance;
    const double relative_s = waypoint.s - lane.GetDistance();
    const double remaining_lane_length = forward ? lane.GetLength() - relative_s : relative_s;
    DEBUG_ASSERT(remaining_lane_length >= 0.0);

    // If after subtracting the distance we are still in the same lane, return
    // same waypoint with the extra distance.
    if (distance <= remaining_lane_length) {
      Waypoint result = waypoint;
      result.s += signed_distance;
      result.s += forward ? -EPSILON : EPSILON;
      RELEASE_ASSERT(result.s > 0.0);
      return { result };
    }

    // If we run out of remaining_lane_length we have to go to the successors.
    std::vector<Waypoint> result;
    for (const auto &successor : GetPredecessors(waypoint)) {
      DEBUG_ASSERT(
          successor.road_id != waypoint.road_id ||
          successor.section_id != waypoint.section_id ||
          successor.lane_id != waypoint.lane_id);
      result = ConcatVectors(result, GetPrevious(successor, distance - remaining_lane_length));
    }
    return result;
  }

  std::optional<Waypoint> Map::GetRight(Waypoint waypoint) const {
    RELEASE_ASSERT(waypoint.lane_id != 0);
    if (waypoint.lane_id > 0) {
      ++waypoint.lane_id;
    } else {
      --waypoint.lane_id;
    }
    return IsLanePresent(_data, waypoint) ? waypoint : std::optional<Waypoint>{};
  }

  std::optional<Waypoint> Map::GetLeft(Waypoint waypoint) const {
    RELEASE_ASSERT(waypoint.lane_id != 0);
    if (std::abs(waypoint.lane_id) == 1) {
      waypoint.lane_id *= -1;
    } else if (waypoint.lane_id > 0) {
      --waypoint.lane_id;
    } else {
      ++waypoint.lane_id;
    }
    return IsLanePresent(_data, waypoint) ? waypoint : std::optional<Waypoint>{};
  }

  std::vector<Waypoint> Map::GenerateWaypoints(const double distance) const {
    RELEASE_ASSERT(distance > 0.0);
    std::vector<Waypoint> result;
    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      for (double s = EPSILON; s < (road.GetLength() - EPSILON); s += distance) {
        ForEachDrivableLaneAt(road, s, [&](auto &&waypoint) {
          result.emplace_back(waypoint);
        });
      }
    }
    return result;
  }

  std::vector<Waypoint> Map::GenerateWaypointsOnRoadEntries(Lane::LaneType lane_type) const {
    std::vector<Waypoint> result;
    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      // right lanes start at s 0
      for (const auto &lane_section : road.GetLaneSectionsAt(0.0)) {
        for (const auto &lane : lane_section.GetLanes()) {
          // add only the right (negative) lanes
          if (lane.first < 0 &&
              static_cast<uint32_t>(lane.second.GetType()) & static_cast<uint32_t>(lane_type)) {
            result.emplace_back(Waypoint{ road.GetId(), lane_section.GetId(), lane.second.GetId(), 0.0 });
          }
        }
      }
      // left lanes start at s max
      const auto road_len = road.GetLength();
      for (const auto &lane_section : road.GetLaneSectionsAt(road_len)) {
        for (const auto &lane : lane_section.GetLanes()) {
          // add only the left (positive) lanes
          if (lane.first > 0 &&
              static_cast<uint32_t>(lane.second.GetType()) & static_cast<uint32_t>(lane_type)) {
            result.emplace_back(
              Waypoint{ road.GetId(), lane_section.GetId(), lane.second.GetId(), road_len });
          }
        }
      }
    }
    return result;
  }

  std::vector<Waypoint> Map::GenerateWaypointsInRoad(
      RoadId road_id,
      Lane::LaneType lane_type) const {
    std::vector<Waypoint> result;
    if(_data.GetRoads().count(road_id)) {
      const auto &road = _data.GetRoads().at(road_id);
      // right lanes start at s 0
      for (const auto &lane_section : road.GetLaneSectionsAt(0.0)) {
        for (const auto &lane : lane_section.GetLanes()) {
          // add only the right (negative) lanes
          if (lane.first < 0 &&
              static_cast<uint32_t>(lane.second.GetType()) & static_cast<uint32_t>(lane_type)) {
            result.emplace_back(Waypoint{ road.GetId(), lane_section.GetId(), lane.second.GetId(), 0.0 });
          }
        }
      }
      // left lanes start at s max
      const auto road_len = road.GetLength();
      for (const auto &lane_section : road.GetLaneSectionsAt(road_len)) {
        for (const auto &lane : lane_section.GetLanes()) {
          // add only the left (positive) lanes
          if (lane.first > 0 &&
              static_cast<uint32_t>(lane.second.GetType()) & static_cast<uint32_t>(lane_type)) {
            result.emplace_back(
              Waypoint{ road.GetId(), lane_section.GetId(), lane.second.GetId(), road_len });
          }
        }
      }
    }
    return result;
  }

  std::vector<std::pair<Waypoint, Waypoint>> Map::GenerateTopology() const {
    std::vector<std::pair<Waypoint, Waypoint>> result;
    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      ForEachDrivableLane(road, [&](auto &&waypoint) {
        for (auto &&successor : GetSuccessors(waypoint)) {
          result.push_back({waypoint, successor});
        }
      });
    }
    return result;
  }

  std::vector<std::pair<Waypoint, Waypoint>> Map::GetJunctionWaypoints(JuncId id, Lane::LaneType lane_type) const {
    std::vector<std::pair<Waypoint, Waypoint>> result;
    const Junction * junction = GetJunction(id);
    for(auto &connections : junction->GetConnections()) {
      const Road &road = _data.GetRoad(connections.second.connecting_road);
      ForEachLane(road, lane_type, [&](auto &&waypoint) {
        const auto& lane = GetLane(waypoint);
        const double final_s = GetDistanceAtEndOfLane(lane);
        Waypoint lane_end(waypoint);
        lane_end.s = final_s;
        result.push_back({waypoint, lane_end});
      });
    }
    return result;
  }

  const Lane &Map::GetLane(Waypoint waypoint) const {
    return _data.GetRoad(waypoint.road_id).GetLaneById(waypoint.section_id, waypoint.lane_id);
  }

  // ===========================================================================
  // -- Map: Private functions -------------------------------------------------
  // ===========================================================================

  Junction* Map::GetJunction(JuncId id) {
    return _data.GetJunction(id);
  }

  const Junction* Map::GetJunction(JuncId id) const {
    return _data.GetJunction(id);
  }
//
//  geom::Mesh Map::GenerateMesh(
//      const double distance,
//      const float extra_width,
//      const  bool smooth_junctions) const {
//    RELEASE_ASSERT(distance > 0.0);
//    geom::MeshFactory mesh_factory;
//    geom::Mesh out_mesh;
//
//    mesh_factory.road_param.resolution = static_cast<float>(distance);
//    mesh_factory.road_param.extra_lane_width = extra_width;
//
//    // Generate roads outside junctions
//    for (auto &&pair : _data.GetRoads()) {
//      const auto &road = pair.second;
//      if (road.IsJunction()) {
//        continue;
//      }
//      out_mesh += *mesh_factory.Generate(road);
//    }
//
//    // Generate roads within junctions and smooth them
//    for (const auto &junc_pair : _data.GetJunctions()) {
//      const auto &junction = junc_pair.second;
//      std::vector<std::unique_ptr<geom::Mesh>> lane_meshes;
//      for(const auto &connection_pair : junction.GetConnections()) {
//        const auto &connection = connection_pair.second;
//        const auto &road = _data.GetRoads().at(connection.connecting_road);
//        for (auto &&lane_section : road.GetLaneSections()) {
//          for (auto &&lane_pair : lane_section.GetLanes()) {
//            lane_meshes.push_back(mesh_factory.Generate(lane_pair.second));
//          }
//        }
//      }
//      if(smooth_junctions) {
//        out_mesh += *mesh_factory.MergeAndSmooth(lane_meshes);
//      } else {
//        geom::Mesh junction_mesh;
//        for(auto& lane : lane_meshes) {
//          junction_mesh += *lane;
//        }
//        out_mesh += junction_mesh;
//      }
//    }
//
//    return out_mesh;
//  }

//  std::vector<std::unique_ptr<geom::Mesh>> Map::GenerateChunkedMesh(
//      const rpc::OpendriveGenerationParameters& params) const {
//    geom::MeshFactory mesh_factory(params);
//    std::vector<std::unique_ptr<geom::Mesh>> out_mesh_list;
//
//    std::unordered_map<JuncId, geom::Mesh> junction_map;
//    for (auto &&pair : _data.GetRoads()) {
//      const auto &road = pair.second;
//      if (!road.IsJunction()) {
//        std::vector<std::unique_ptr<geom::Mesh>> road_mesh_list =
//            mesh_factory.GenerateAllWithMaxLen(road);
//
//        out_mesh_list.insert(
//            out_mesh_list.end(),
//            std::make_move_iterator(road_mesh_list.begin()),
//            std::make_move_iterator(road_mesh_list.end()));
//      }
//    }
//
//    // Generate roads within junctions and smooth them
//    for (const auto &junc_pair : _data.GetJunctions()) {
//      const auto &junction = junc_pair.second;
//      std::vector<std::unique_ptr<geom::Mesh>> lane_meshes;
//      std::vector<std::unique_ptr<geom::Mesh>> sidewalk_lane_meshes;
//      for(const auto &connection_pair : junction.GetConnections()) {
//        const auto &connection = connection_pair.second;
//        const auto &road = _data.GetRoads().at(connection.connecting_road);
//        for (auto &&lane_section : road.GetLaneSections()) {
//          for (auto &&lane_pair : lane_section.GetLanes()) {
//            const auto &lane = lane_pair.second;
//            if (lane.GetType() != road::Lane::LaneType::Sidewalk) {
//              lane_meshes.push_back(mesh_factory.Generate(lane));
//            } else {
//              sidewalk_lane_meshes.push_back(mesh_factory.Generate(lane));
//            }
//          }
//        }
//      }
//      if(params.smooth_junctions) {
//        auto merged_mesh = mesh_factory.MergeAndSmooth(lane_meshes);
//        for(auto& lane : sidewalk_lane_meshes) {
//          *merged_mesh += *lane;
//        }
//        out_mesh_list.push_back(std::move(merged_mesh));
//      } else {
//        std::unique_ptr<geom::Mesh> junction_mesh = std::make_unique<geom::Mesh>();
//        for(auto& lane : lane_meshes) {
//          *junction_mesh += *lane;
//        }
//        for(auto& lane : sidewalk_lane_meshes) {
//          *junction_mesh += *lane;
//        }
//        out_mesh_list.push_back(std::move(junction_mesh));
//      }
//    }
//
//    auto min_pos = geom::Vector2D(
//        out_mesh_list.front()->GetVertices().front().x,
//        out_mesh_list.front()->GetVertices().front().y);
//    auto max_pos = min_pos;
//    for (auto & mesh : out_mesh_list) {
//      auto vertex = mesh->GetVertices().front();
//      min_pos.x = std::min(min_pos.x, vertex.x);
//      min_pos.y = std::min(min_pos.y, vertex.y);
//      max_pos.x = std::max(max_pos.x, vertex.x);
//      max_pos.y = std::max(max_pos.y, vertex.y);
//    }
//    size_t mesh_amount_x = static_cast<size_t>((max_pos.x - min_pos.x)/params.max_road_length) + 1;
//    size_t mesh_amount_y = static_cast<size_t>((max_pos.y - min_pos.y)/params.max_road_length) + 1;
//    std::vector<std::unique_ptr<geom::Mesh>> result;
//    result.reserve(mesh_amount_x*mesh_amount_y);
//    for (size_t i = 0; i < mesh_amount_x*mesh_amount_y; ++i) {
//      result.emplace_back(std::make_unique<geom::Mesh>());
//    }
//    for (auto & mesh : out_mesh_list) {
//      auto vertex = mesh->GetVertices().front();
//      size_t x_pos = static_cast<size_t>((vertex.x - min_pos.x) / params.max_road_length);
//      size_t y_pos = static_cast<size_t>((vertex.y - min_pos.y) / params.max_road_length);
//      *(result[x_pos + mesh_amount_x*y_pos]) += *mesh;
//    }
//
//    return result;
//  }

  geom::Mesh Map::GetAllCrosswalkMesh() const {
    geom::Mesh out_mesh;

    // Get the crosswalk vertices for the current map
    const std::vector<geom::Location> crosswalk_vertex = GetAllCrosswalkZones();
    if (crosswalk_vertex.empty()) {
      return out_mesh;
    }

    // Create a a list of triangle fans with material "crosswalk"
    out_mesh.AddMaterial("crosswalk");
    size_t start_vertex_index = 0;
    size_t i = 0;
    std::vector<geom::Vector3D> vertices;
    // Iterate the vertices until a repeated one is found, this indicates
    // the triangle fan is done and another one must start
    do {
      // Except for the first iteration && triangle fan done
      if (i != 0 && crosswalk_vertex[start_vertex_index] == crosswalk_vertex[i]) {
        // Create the actual fan
        out_mesh.AddTriangleFan(vertices);
        vertices.clear();
        // End the loop if i reached the end of the vertex list
        if (i >= crosswalk_vertex.size() - 1) {
          break;
        }
        start_vertex_index = ++i;
      }
      // Append a new Vector3D that will be added to the triangle fan
      vertices.push_back(crosswalk_vertex[i++]);
    } while (i < crosswalk_vertex.size());

    out_mesh.EndMaterial();
    return out_mesh;
  }

  // ===========================================================================
  // -- Map: Private functions -------------------------------------------------
  // ===========================================================================

  void Map::CreateKDtree() {
    const double min_delta_s = 0.5;    // segments of minimum 1m through the road

    // Generate waypoints at start of every lane
    std::vector<Waypoint> topology;
    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      ForEachLane(road, Lane::LaneType::Any, [&](auto &&waypoint) {
        if(waypoint.lane_id != 0) {
          topology.push_back(waypoint);
        }
      });
    }

    // Loop through all lanes
    _kdtree = std::make_unique<KDtree>();
    for (auto &waypoint : topology) {
      const Lane &lane = GetLane(waypoint);
      auto current_waypoint = waypoint;
      for(double s=0; s<lane.GetLength()+min_delta_s-EPSILON; s+=min_delta_s) {
        current_waypoint.s = lane.GetDistance()+std::min<double>(s, lane.GetLength()-EPSILON);
        geom::Transform t = lane.ComputeTransform(current_waypoint.s);
        _kdtree->insert(t.location, current_waypoint);
      }
    }
    // Add segments to Rtree
    _kdtree->build();
  }


} // namespace road
} // namespace carla
