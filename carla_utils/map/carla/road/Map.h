// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/geom/Mesh.h"
#include "carla/geom/Rtree.h"
#include "carla/geom/Transform.h"
#include "carla/NonCopyable.h"
#include "carla/road/element/LaneMarking.h"
#include "carla/road/element/RoadInfoMarkRecord.h"
#include "carla/road/element/Waypoint.h"
#include "carla/road/MapData.h"
#include "carla/road/RoadTypes.h"
//#include "carla/rpc/OpendriveGenerationParameters.h"
#include "nanoflann.hpp"

#include <optional>
#include <tuple>
#include <vector>

namespace carla {
namespace road {
namespace detail {
  inline const float * ptr(const geom::Vector2D & v) {
    return &v.x;
  }
  inline const float * ptr(const geom::Vector3D & v) {
    return &v.x;
  }
  template<typename T> struct D{};
  template<> struct D<geom::Vector2D>{ static constexpr size_t dim = 2; };
  template<> struct D<geom::Vector3D>{ static constexpr size_t dim = 3; };
  template<> struct D<geom::Location>{ static constexpr size_t dim = 3; };

  template <typename _FilterType, typename _DistanceType=float, typename _IndexType = size_t, typename _CountType = size_t>
  class FilteredKNNResultSet: public nanoflann::KNNResultSet<_DistanceType, _IndexType, _CountType> {
  protected:
    typedef nanoflann::KNNResultSet<_DistanceType, _IndexType, _CountType> Base;
    typedef _FilterType FilterType;
    FilterType filter;
  public:
    inline FilteredKNNResultSet(_CountType capacity_, _FilterType filter_): Base(capacity_), filter(filter_){
    }
    inline bool addPoint(_DistanceType dist, _IndexType index) {
      if (!filter(index, dist)) return true;
      return Base::addPoint(dist, index);
    }
  };

  template<typename K, typename V>
  class KDtree {
  protected:
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, KDtree<K, V>>, KDtree<K,V>, D<K>::dim> kd_tree_t;
    typedef std::tuple<K, V> Element;
    std::vector<K> k_;
    std::vector<V> v_;
    std::unique_ptr<kd_tree_t> kd_tree_;
  public:
    KDtree(const KDtree&) = delete;
    KDtree() = default;

    int insert(const K & k, const V & v) {
      k_.push_back(k);
      v_.push_back(v);
      return k_.size()-1;
    }
    void build(){
      kd_tree_ = std::make_unique<kd_tree_t>(D<K>::dim, *this);
      kd_tree_->buildIndex();
    }

    /// Return nearest neighbors with a user defined filter.
    /// The filter reveices as an argument a TreeElement value and needs to
    /// return a bool to accept or reject the value
    /// [&](Rtree::TreeElement const &element){if (IsOk(element)) return true;
    /// else return false;}
    template <typename Filter>
    std::vector<V> GetNearestNeighboursWithFilter(
        const K &k,
        Filter filter,
        size_t number_neighbours = 1) const {
      std::vector<size_t> indices(number_neighbours);
      std::vector<float> distances(number_neighbours);

      auto value_filter = [&filter, this](size_t i, float d)->bool {return filter(v_[i], d);};
      FilteredKNNResultSet<decltype(value_filter)> resultSet(number_neighbours, value_filter);
      resultSet.init(&indices[0], &distances[0]);
      kd_tree_->findNeighbors(resultSet, ptr(k), nanoflann::SearchParams());

      // Get the results
      std::vector<V> r;
      for(size_t i=0; i<resultSet.size(); i++)
        r.push_back(v_[indices[i]]);
      return r;
    }

    std::vector<V> GetNearestNeighbours(const K &k, size_t number_neighbours = 1) const {
      std::vector<size_t> indices(number_neighbours);
      std::vector<float> distances(number_neighbours);
      size_t n = kd_tree_->knnSearch(ptr(k), number_neighbours, &indices[0], &distances[0]);
      std::vector<V> r;
      for(size_t i=0; i<n; i++)
        r.push_back(v_[indices[i]]);
      return r;
    }


	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return k_.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
	    return ptr(k_[idx])[dim];
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
  };
}  // namespace detail


  class Map : private MovableNonCopyable {
  public:

    using Waypoint = element::Waypoint;

    /// ========================================================================
    /// -- Constructor ---------------------------------------------------------
    /// ========================================================================

    Map(MapData m);

    /// ========================================================================
    /// -- Georeference --------------------------------------------------------
    /// ========================================================================

    const geom::GeoLocation &GetGeoReference() const {
      return _data.GetGeoReference();
    }

    /// ========================================================================
    /// -- Geometry ------------------------------------------------------------
    /// ========================================================================

    std::optional<element::Waypoint> GetClosestWaypointOnRoad(
        const geom::Location &location,
        uint32_t lane_type = static_cast<uint32_t>(Lane::LaneType::Driving)) const;

    std::optional<element::Waypoint> GetClosestWaypointOnRoad_new(
        const geom::Location &location,
        uint32_t lane_type = static_cast<uint32_t>(Lane::LaneType::Driving)) const;

    std::optional<element::Waypoint> GetClosestWaypointOnRoad_old(
        const geom::Location &location,
        uint32_t lane_type = static_cast<uint32_t>(Lane::LaneType::Driving)) const;

    std::optional<element::Waypoint> GetWaypoint(
        const geom::Location &location,
        uint32_t lane_type = static_cast<uint32_t>(Lane::LaneType::Driving)) const;

    std::optional<element::Waypoint> GetWaypoint(
        RoadId road_id,
        LaneId lane_id,
        float s) const;

    geom::Transform ComputeTransform(Waypoint waypoint) const;

    /// ========================================================================
    /// -- Road information ----------------------------------------------------
    /// ========================================================================

    const Lane &GetLane(Waypoint waypoint) const;

    Lane::LaneType GetLaneType(Waypoint waypoint) const;

    double GetLaneWidth(Waypoint waypoint) const;

    JuncId GetJunctionId(RoadId road_id) const;

    bool IsJunction(RoadId road_id) const;

    std::pair<const element::RoadInfoMarkRecord *, const element::RoadInfoMarkRecord *>
        GetMarkRecord(Waypoint waypoint) const;

    std::vector<element::LaneMarking> CalculateCrossedLanes(
        const geom::Location &origin,
        const geom::Location &destination) const;

    /// Returns a list of locations defining 2d areas,
    /// when a location is repeated an area is finished
    std::vector<geom::Location> GetAllCrosswalkZones() const;

    /// Data structure for the signal search
    struct SignalSearchData {
      const element::RoadInfoSignal *signal;
      Waypoint waypoint;
      double accumulated_s = 0;
    };

    /// Searches signals from an initial waypoint until the defined distance.
    std::vector<SignalSearchData> GetSignalsInDistance(
        Waypoint waypoint, double distance, bool stop_at_junction = false) const;

    /// Return all RoadInfoSignal in the map
    std::vector<const element::RoadInfoSignal*>
        GetAllSignalReferences() const;

    /// ========================================================================
    /// -- Waypoint generation -------------------------------------------------
    /// ========================================================================

    /// Return the list of waypoints placed at the entrance of each drivable
    /// successor lane; i.e., the list of each waypoint in the next road segment
    /// that a vehicle could drive from @a waypoint.
    std::vector<Waypoint> GetSuccessors(Waypoint waypoint) const;
    std::vector<Waypoint> GetPredecessors(Waypoint waypoint) const;

    /// Return the list of waypoints at @a distance such that a vehicle at @a
    /// waypoint could drive to.
    std::vector<Waypoint> GetNext(Waypoint waypoint, double distance) const;
    /// Return the list of waypoints at @a distance in the reversed direction
    /// that a vehicle at @a waypoint could drive to.
    std::vector<Waypoint> GetPrevious(Waypoint waypoint, double distance) const;

    /// Return a waypoint at the lane of @a waypoint's right lane.
    std::optional<Waypoint> GetRight(Waypoint waypoint) const;

    /// Return a waypoint at the lane of @a waypoint's left lane.
    std::optional<Waypoint> GetLeft(Waypoint waypoint) const;

    /// Generate all the waypoints in @a map separated by @a approx_distance.
    std::vector<Waypoint> GenerateWaypoints(double approx_distance) const;

    /// Generate waypoints on each @a lane at the start of each @a road
    std::vector<Waypoint> GenerateWaypointsOnRoadEntries(Lane::LaneType lane_type = Lane::LaneType::Driving) const;

    /// Generate waypoints at the entry of each lane of the specified road
    std::vector<Waypoint> GenerateWaypointsInRoad(RoadId road_id, Lane::LaneType lane_type = Lane::LaneType::Driving) const;

    /// Generate the minimum set of waypoints that define the topology of @a
    /// map. The waypoints are placed at the entrance of each lane.
    std::vector<std::pair<Waypoint, Waypoint>> GenerateTopology() const;

    /// Generate waypoints of the junction
    std::vector<std::pair<Waypoint, Waypoint>> GetJunctionWaypoints(JuncId id, Lane::LaneType lane_type) const;

    Junction* GetJunction(JuncId id);

    const Junction* GetJunction(JuncId id) const;

//    /// Buids a mesh based on the OpenDRIVE
//    geom::Mesh GenerateMesh(
//        const double distance,
//        const float extra_width = 0.6f,
//        const  bool smooth_junctions = true) const;

//    std::vector<std::unique_ptr<geom::Mesh>> GenerateChunkedMesh(
//        const rpc::OpendriveGenerationParameters& params) const;

    /// Buids a mesh of all crosswalks based on the OpenDRIVE
    geom::Mesh GetAllCrosswalkMesh() const;

    geom::Mesh GenerateWalls(const double distance, const float wall_height) const;

    const std::unordered_map<SignId, std::unique_ptr<Signal>>& GetSignals() const {
      return _data.GetSignals();
    }

    const std::unordered_map<ContId, std::unique_ptr<Controller>>& GetControllers() const {
      return _data.GetControllers();
    }

#ifdef LIBCARLA_WITH_GTEST
    MapData &GetMap() {
      return _data;
    }
#endif // LIBCARLA_WITH_GTEST

private:

    friend MapBuilder;
    MapData _data;

    using Rtree = geom::SegmentCloudRtree<Waypoint>;
    Rtree _rtree;

    void CreateRtree();

    /// Helper Functions for constructing the rtree element list
    void AddElementToRtree(
        std::vector<Rtree::TreeElement> &rtree_elements,
        geom::Transform &current_transform,
        geom::Transform &next_transform,
        Waypoint &current_waypoint,
        Waypoint &next_waypoint);

    void AddElementToRtreeAndUpdateTransforms(
        std::vector<Rtree::TreeElement> &rtree_elements,
        geom::Transform &current_transform,
        Waypoint &current_waypoint,
        Waypoint &next_waypoint);

    // ---
    void CreateKDtree();
    using KDtree = detail::KDtree<geom::Location, Waypoint>;
    std::unique_ptr<KDtree> _kdtree;
  };

} // namespace road
} // namespace carla
