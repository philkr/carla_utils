// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cmath>
#include <limits>
#include <algorithm>

namespace carla {
namespace geom {

  class Vector3D {
  public:

    // =========================================================================
    // -- Public data members --------------------------------------------------
    // =========================================================================

    float x = 0.0f;

    float y = 0.0f;

    float z = 0.0f;

    // =========================================================================
    // -- Constructors ---------------------------------------------------------
    // =========================================================================

    Vector3D() = default;

    Vector3D(float ix, float iy, float iz)
      : x(ix),
        y(iy),
        z(iz) {}

    // =========================================================================
    // -- Other methods --------------------------------------------------------
    // =========================================================================

    float SquaredLength() const {
      return x * x + y * y + z * z;
    }

    float Length() const {
       return std::sqrt(SquaredLength());
    }

    Vector3D MakeUnitVector() const {
      const float length = Length();
      const float k = 1.0f / length;
      return Vector3D(x * k, y * k, z * k);
    }

    Vector3D MakeSafeUnitVector(const float epsilon) const  {
      const float length = Length();
      const float k = (length > std::max(epsilon, 0.0f)) ? (1.0f / length) : 1.0f;
      return Vector3D(x * k, y * k, z * k);
    }

    // =========================================================================
    // -- Arithmetic operators -------------------------------------------------
    // =========================================================================

    Vector3D &operator+=(const Vector3D &rhs) {
      x += rhs.x;
      y += rhs.y;
      z += rhs.z;
      return *this;
    }

    friend Vector3D operator+(Vector3D lhs, const Vector3D &rhs) {
      lhs += rhs;
      return lhs;
    }

    Vector3D &operator-=(const Vector3D &rhs) {
      x -= rhs.x;
      y -= rhs.y;
      z -= rhs.z;
      return *this;
    }

    friend Vector3D operator-(Vector3D lhs, const Vector3D &rhs) {
      lhs -= rhs;
      return lhs;
    }

    Vector3D &operator*=(float rhs) {
      x *= rhs;
      y *= rhs;
      z *= rhs;
      return *this;
    }

    friend Vector3D operator*(Vector3D lhs, float rhs) {
      lhs *= rhs;
      return lhs;
    }

    friend Vector3D operator*(float lhs, Vector3D rhs) {
      rhs *= lhs;
      return rhs;
    }

    Vector3D &operator/=(float rhs) {
      x /= rhs;
      y /= rhs;
      z /= rhs;
      return *this;
    }

    friend Vector3D operator/(Vector3D lhs, float rhs) {
      lhs /= rhs;
      return lhs;
    }

    friend Vector3D operator/(float lhs, Vector3D rhs) {
      rhs /= lhs;
      return rhs;
    }

    // =========================================================================
    // -- Comparison operators -------------------------------------------------
    // =========================================================================

    bool operator==(const Vector3D &rhs) const {
      return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
    }

    bool operator!=(const Vector3D &rhs) const {
      return !(*this == rhs);
    }

    // =========================================================================
    // -- Conversions to UE4 types ---------------------------------------------
    // =========================================================================

#ifdef LIBCARLA_INCLUDED_FROM_UE4

    /// These 2 methods are explicitly deleted to avoid creating them by other users,
    /// unlike locations, some vectors have units and some don't, by removing
    /// these methods we found several places were the conversion from cm to m was missing
    Vector3D(const FVector &v) = delete;
    Vector3D& operator=(const FVector &rhs) = delete;

    /// Return a Vector3D converted from centimeters to meters.
    Vector3D ToMeters() const {
      return *this * 1e-2f;
    }

    /// Return a Vector3D converted from meters to centimeters.
    Vector3D ToCentimeters() const {
      return *this * 1e2f;
    }

    FVector ToFVector() const {
      return FVector{x, y, z};
    }

#endif // LIBCARLA_INCLUDED_FROM_UE4
  };

} // namespace geom
} // namespace carla
