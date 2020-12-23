// Copyright (c) 2020 Philipp Krähenbühl
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Location.h"
#include "Vector2D.h"
#include "Vector3D.h"
#include "nanoflann.hpp"

#include <memory>
#include <vector>


namespace carla {
namespace geom {

  inline const float * ptr(const Vector2D & v) {
    return &v.x;
  }
  inline const float * ptr(const Vector3D & v) {
    return &v.x;
  }
  template<typename T> struct D{};
  template<> struct D<Vector2D>{ static constexpr size_t dim = 2; };
  template<> struct D<Vector3D>{ static constexpr size_t dim = 3; };
  template<> struct D<Location>{ static constexpr size_t dim = 3; };

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
} // namespace geom
} // namespace carla
