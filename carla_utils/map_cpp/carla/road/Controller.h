// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy; see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/NonCopyable.h"
#include "carla/road/RoadTypes.h"

#include <unordered_set>

namespace carla {
namespace road {

  class MapBuilder;

  class Controller : private MovableNonCopyable {

  public:

    Controller(
        ContId id,
        std::string name,
        uint32_t sequence)
      : _id(id),
        _name(name),
        _sequence(sequence){}

    const ContId& GetControllerId() const{
      return _id;
    }

    const std::string& GetName() const {
      return _name;
    }

    const uint32_t &GetSequence() const {
      return _sequence;
    }

    const std::unordered_set<SignId>&  GetSignals() const {
      return _signals;
    }

    const std::unordered_set<JuncId>&  GetJunctions() const {
      return _junctions;
    }

  private:

    friend MapBuilder;

    ContId _id;
    std::string _name;
    uint32_t _sequence;

    std::unordered_set<JuncId> _junctions;
    std::unordered_set<SignId> _signals;
  };

} // namespace road
} // namespace carla
