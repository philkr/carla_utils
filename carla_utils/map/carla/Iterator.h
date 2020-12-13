// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

namespace carla {
namespace iterator {

  template<typename R, typename I, typename F>
  class TransformIterator {
  protected:
    I iterator_;
    F f_;
  public:
    // iterator traits
    using difference_type = typename I::difference_type;
    using value_type = R;
    using pointer = std::remove_reference_t<R>*;
    using reference = std::remove_reference_t<R>&;
    using iterator_category = std::forward_iterator_tag;

    // Iterator definition
    TransformIterator(I iterator, F f):iterator_(iterator),f_(f) {}
    value_type operator*() { return f_(*iterator_); }
    TransformIterator& operator++() {iterator_++; return *this;}
    TransformIterator operator++(int) {TransformIterator retval = *this; ++(*this); return retval;}
    bool operator==(TransformIterator other) const {return iterator_ == other.iterator_;}
    bool operator!=(TransformIterator other) const {return !(*this == other);}
  };
  template<typename R, typename I, typename F>
  TransformIterator<R, I, F> make_transform_iterator(I iterator, F f) {
    return TransformIterator<R, I, F>(iterator, f);
  }

  /// Creates an iterator over non-const references to the keys of a map.
  template <typename It>
  inline static auto make_map_keys_iterator(It it) {
    using first_value_type = typename It::value_type::first_type;
    using decay_first_value_type = typename std::remove_cv_t<typename std::remove_reference_t<first_value_type>>;
    using ref_to_first = decay_first_value_type &;
    return make_transform_iterator<ref_to_first>(it, [](auto &pair) -> ref_to_first { return pair.first; });
  }

  /// Creates an iterator over const references to the keys of a map.
  template <typename It>
  inline static auto make_map_keys_const_iterator(It it) {
    using first_value_type = typename It::value_type::first_type;
    using decay_first_value_type = typename std::remove_cv_t<typename std::remove_reference_t<first_value_type>>;
    using const_ref_to_first = const decay_first_value_type &;
    return make_transform_iterator<const_ref_to_first>(it, [](const auto &pair) -> const_ref_to_first { return pair.first; });
  }

  /// Creates an iterator over non-const references to the values of a map.
  template <typename It>
  inline static auto make_map_values_iterator(It it) {
    using second_value_type = typename It::value_type::second_type;
    using decay_second_value_type = typename std::remove_cv_t<typename std::remove_reference_t<second_value_type>>;
    using ref_to_second = decay_second_value_type &;
    return make_transform_iterator<ref_to_second>(it, [](auto &pair) -> ref_to_second { return pair.second; });
  }

  /// Creates an iterator over const references to the values of a map.
  template <typename It>
  inline static auto make_map_values_const_iterator(It it) {
    using second_value_type = typename It::value_type::second_type;
    using decay_second_value_type = typename std::remove_cv_t<typename std::remove_reference_t<second_value_type>>;
    using const_ref_to_second = const decay_second_value_type &;
    return make_transform_iterator<const_ref_to_second>(it, [](const auto &pair) -> const_ref_to_second { return pair.second; });
  }

} // namespace iterator
} // namespace carla
