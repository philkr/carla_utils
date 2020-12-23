// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/element/Geometry.h"

#include "carla/Debug.h"
#include "carla/Exception.h"
#include "carla/geom/Location.h"
#include "carla/geom/Math.h"
#include "carla/geom/Vector2D.h"

#include <array>

/********** odrSpiral **************/

/* ===================================================
 *  file:       odrSpiral.c
 * ---------------------------------------------------
 *  purpose:	free method for computing spirals
 *              in OpenDRIVE applications
 * ---------------------------------------------------
 *  using methods of CEPHES library
 * ---------------------------------------------------
 *  first edit:	09.03.2010 by M. Dupuis @ VIRES GmbH
 *  last mod.:  02.05.2017 by Michael Scholz @ German Aerospace Center (DLR)
 * ===================================================
    Copyright 2010 VIRES Simulationstechnologie GmbH
	Copyright 2017 German Aerospace Center (DLR)
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
        http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.


    NOTE:
    The methods have been realized using the CEPHES library
        http://www.netlib.org/cephes/
    and do neither constitute the only nor the exclusive way of implementing
    spirals for OpenDRIVE applications. Their sole purpose is to facilitate
    the interpretation of OpenDRIVE spiral data.
 */

/* ====== INCLUSIONS ====== */
#include <stdio.h>
#if defined(_WIN32) || defined(_WIN64)
   /* Probably Windows */
   #include <io.h>
#else
   /* Supposedly Unix */
   #include <unistd.h>
#endif
#define _USE_MATH_DEFINES
#include <math.h>

/* ====== LOCAL VARIABLES ====== */

/* S(x) for small x */
static double sn[6] = {
-2.99181919401019853726E3,
 7.08840045257738576863E5,
-6.29741486205862506537E7,
 2.54890880573376359104E9,
-4.42979518059697779103E10,
 3.18016297876567817986E11,
};
static double sd[6] = {
/* 1.00000000000000000000E0,*/
 2.81376268889994315696E2,
 4.55847810806532581675E4,
 5.17343888770096400730E6,
 4.19320245898111231129E8,
 2.24411795645340920940E10,
 6.07366389490084639049E11,
};

/* C(x) for small x */
static double cn[6] = {
-4.98843114573573548651E-8,
 9.50428062829859605134E-6,
-6.45191435683965050962E-4,
 1.88843319396703850064E-2,
-2.05525900955013891793E-1,
 9.99999999999999998822E-1,
};
static double cd[7] = {
 3.99982968972495980367E-12,
 9.15439215774657478799E-10,
 1.25001862479598821474E-7,
 1.22262789024179030997E-5,
 8.68029542941784300606E-4,
 4.12142090722199792936E-2,
 1.00000000000000000118E0,
};

/* Auxiliary function f(x) */
static double fn[10] = {
  4.21543555043677546506E-1,
  1.43407919780758885261E-1,
  1.15220955073585758835E-2,
  3.45017939782574027900E-4,
  4.63613749287867322088E-6,
  3.05568983790257605827E-8,
  1.02304514164907233465E-10,
  1.72010743268161828879E-13,
  1.34283276233062758925E-16,
  3.76329711269987889006E-20,
};
static double fd[10] = {
/*  1.00000000000000000000E0,*/
  7.51586398353378947175E-1,
  1.16888925859191382142E-1,
  6.44051526508858611005E-3,
  1.55934409164153020873E-4,
  1.84627567348930545870E-6,
  1.12699224763999035261E-8,
  3.60140029589371370404E-11,
  5.88754533621578410010E-14,
  4.52001434074129701496E-17,
  1.25443237090011264384E-20,
};

/* Auxiliary function g(x) */
static double gn[11] = {
  5.04442073643383265887E-1,
  1.97102833525523411709E-1,
  1.87648584092575249293E-2,
  6.84079380915393090172E-4,
  1.15138826111884280931E-5,
  9.82852443688422223854E-8,
  4.45344415861750144738E-10,
  1.08268041139020870318E-12,
  1.37555460633261799868E-15,
  8.36354435630677421531E-19,
  1.86958710162783235106E-22,
};
static double gd[11] = {
/*  1.00000000000000000000E0,*/
  1.47495759925128324529E0,
  3.37748989120019970451E-1,
  2.53603741420338795122E-2,
  8.14679107184306179049E-4,
  1.27545075667729118702E-5,
  1.04314589657571990585E-7,
  4.60680728146520428211E-10,
  1.10273215066240270757E-12,
  1.38796531259578871258E-15,
  8.39158816283118707363E-19,
  1.86958710162783236342E-22,
};


static double polevl( double x, double* coef, int n )
{
    double ans;
    double *p = coef;
    int i;

    ans = *p++;
    i   = n;

    do
    {
        ans = ans * x + *p++;
    }
    while (--i);

    return ans;
}

static double p1evl( double x, double* coef, int n )
{
    double ans;
    double *p = coef;
    int i;

    ans = x + *p++;
    i   = n - 1;

    do
    {
        ans = ans * x + *p++;
    }
    while (--i);

    return ans;
}


static void fresnel( double xxa, double *ssa, double *cca )
{
    double f, g, cc, ss, c, s, t, u;
    double x, x2;

    x  = fabs( xxa );
    x2 = x * x;

    if ( x2 < 2.5625 )
    {
        t = x2 * x2;
        ss = x * x2 * polevl (t, sn, 5) / p1evl (t, sd, 6);
        cc = x * polevl (t, cn, 5) / polevl (t, cd, 6);
    }
    else if ( x > 36974.0 )
    {
        cc = 0.5;
        ss = 0.5;
    }
    else
    {
        x2 = x * x;
        t = M_PI * x2;
        u = 1.0 / (t * t);
        t = 1.0 / t;
        f = 1.0 - u * polevl (u, fn, 9) / p1evl(u, fd, 10);
        g = t * polevl (u, gn, 10) / p1evl (u, gd, 11);

        t = M_PI * 0.5 * x2;
        c = cos (t);
        s = sin (t);
        t = M_PI * x;
        cc = 0.5 + (f * s - g * c) / t;
        ss = 0.5 - (f * c + g * s) / t;
    }

    if ( xxa < 0.0 )
    {
        cc = -cc;
        ss = -ss;
    }

    *cca = cc;
    *ssa = ss;
}


/**
* compute the actual "standard" spiral, starting with curvature 0
* @param s      run-length along spiral
* @param cDot   first derivative of curvature [1/m2]
* @param x      resulting x-coordinate in spirals local co-ordinate system [m]
* @param y      resulting y-coordinate in spirals local co-ordinate system [m]
* @param t      tangent direction at s [rad]
*/

void odrSpiral( double s, double cDot, double *x, double *y, double *t )
{
    double a;

    a = 1.0 / sqrt( fabs( cDot ) );
    a *= sqrt( M_PI );

    fresnel( s / a, y, x );

    *x *= a;
    *y *= a;

    if ( cDot < 0.0 )
        *y *= -1.0;

    *t = s * s * cDot * 0.5;
}
/********** END odrSpiral **************/

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace carla {
namespace road {
namespace element {

  void DirectedPoint::ApplyLateralOffset(float lateral_offset) {
    /// @todo Z axis??
    auto normal_x =  std::sin(static_cast<float>(tangent));
    auto normal_y = -std::cos(static_cast<float>(tangent));
    location.x += lateral_offset * normal_x;
    location.y += lateral_offset * normal_y;
  }

  DirectedPoint GeometryLine::PosFromDist(double dist) const {
    DEBUG_ASSERT(_length > 0.0);
    dist = geom::Math::Clamp(dist, 0.0, _length);
    DirectedPoint p(_start_position, _heading);
    p.location.x += static_cast<float>(dist * std::cos(p.tangent));
    p.location.y += static_cast<float>(dist * std::sin(p.tangent));
    return p;
  }

  DirectedPoint GeometryArc::PosFromDist(double dist) const {
    dist = geom::Math::Clamp(dist, 0.0, _length);
    DEBUG_ASSERT(_length > 0.0);
    DEBUG_ASSERT(std::fabs(_curvature) > 1e-15);
    const double radius = 1.0 / _curvature;
    constexpr double pi_half = geom::Math::Pi<double>() / 2.0;
    DirectedPoint p(_start_position, _heading);
    p.location.x += static_cast<float>(radius * std::cos(p.tangent + pi_half));
    p.location.y += static_cast<float>(radius * std::sin(p.tangent + pi_half));
    p.tangent += dist * _curvature;
    p.location.x -= static_cast<float>(radius * std::cos(p.tangent + pi_half));
    p.location.y -= static_cast<float>(radius * std::sin(p.tangent + pi_half));
    return p;
  }

  // helper function for rotating points
  geom::Vector2D RotatebyAngle(double angle, double x, double y) {
    const double cos_a = std::cos(angle);
    const double sin_a = std::sin(angle);
    return geom::Vector2D(
    static_cast<float>(x * cos_a - y * sin_a),
    static_cast<float>(y * cos_a + x * sin_a));
  }

  DirectedPoint GeometrySpiral::PosFromDist(double dist) const {
    dist = geom::Math::Clamp(dist, 0.0, _length);
    DEBUG_ASSERT(_length > 0.0);
    DirectedPoint p(_start_position, _heading);

    const double curve_end = (_curve_end);
    const double curve_start = (_curve_start);
    const double curve_dot = (curve_end - curve_start) / (_length);
    const double s_o = curve_start / curve_dot;
    double s = s_o + dist;

    double x;
    double y;
    double t;
    odrSpiral(s, curve_dot, &x, &y, &t);

    double x_o;
    double y_o;
    double t_o;
    odrSpiral(s_o, curve_dot, &x_o, &y_o, &t_o);

    x = x - x_o;
    y = y - y_o;
    t = t - t_o;

    geom::Vector2D pos = RotatebyAngle(_heading - t_o, x, y);
    p.location.x += pos.x;
    p.location.y += pos.y;
    p.tangent = _heading + t;

    return p;
  }

  /// @todo
  std::pair<float, float> GeometrySpiral::DistanceTo(const geom::Location &location) const {
    // Not analytic, discretize and find nearest point
    // throw_exception(std::runtime_error("not implemented"));
    return {location.x - _start_position.x, location.y - _start_position.y};
  }

  DirectedPoint GeometryPoly3::PosFromDist(double dist) const {
    throw std::logic_error("GeometryPoly3::PosFromDist not implemented!");
//    auto result = _rtree.GetNearestNeighbours(
//        Rtree::BPoint(static_cast<float>(dist))).front();
//
//    auto &val1 = result.second.first;
//    auto &val2 = result.second.second;
//
//    double rate = (val2.s - dist) / (val2.s - val1.s);
//    double u = rate * val1.u + (1.0 - rate) * val2.u;
//    double v = rate * val1.v + (1.0 - rate) * val2.v;
//    double tangent = atan((rate * val1.t + (1.0 - rate) * val2.t)); // ?
//
//    geom::Vector2D pos = RotatebyAngle(_heading, u, v);
//    DirectedPoint p(_start_position, _heading + tangent);
//    p.location.x += pos.x;
//    p.location.y += pos.y;
//    return p;
  }

  std::pair<float, float> GeometryPoly3::DistanceTo(const geom::Location & /*p*/) const {
    // No analytical expression (Newton-Raphson?/point search)
    // throw_exception(std::runtime_error("not implemented"));
    return {_start_position.x, _start_position.y};
  }

  void GeometryPoly3::PreComputeSpline() {
    throw std::logic_error("GeometryPoly3::PreComputeSpline not implemented!");
//    // Roughly the interval size in m
//    constexpr double interval_size = 0.3;
//    const double delta_u = interval_size; // interval between values of u
//    double current_s = 0;
//    double current_u = 0;
//    double last_u = 0;
//    double last_v = _poly.Evaluate(current_u);
//    double last_s = 0;
//    RtreeValue last_val{last_u, last_v, last_s, _poly.Tangent(current_u)};
//    while (current_s < _length + delta_u) {
//      current_u += delta_u;
//      double current_v = _poly.Evaluate(current_u);
//      double du = current_u - last_u;
//      double dv = current_v - last_v;
//      double ds = sqrt(du * du + dv * dv);
//      current_s += ds;
//      double current_t = _poly.Tangent(current_u);
//      RtreeValue current_val{current_u, current_v, current_s, current_t};
//
//      Rtree::BPoint p1(static_cast<float>(last_s));
//      Rtree::BPoint p2(static_cast<float>(current_s));
//      _rtree.InsertElement(Rtree::BSegment(p1, p2), last_val, current_val);
//
//      last_u = current_u;
//      last_v = current_v;
//      last_s = current_s;
//      last_val = current_val;
//
//    }
  }

  DirectedPoint GeometryParamPoly3::PosFromDist(double dist) const {
    throw std::logic_error("GeometryParamPoly3::PosFromDist not implemented!");
//    auto result = _rtree.GetNearestNeighbours(
//        Rtree::BPoint(static_cast<float>(dist))).front();
//
//    auto &val1 = result.second.first;
//    auto &val2 = result.second.second;
//    double rate = (val2.s - dist) / (val2.s - val1.s);
//    double u = rate * val1.u + (1.0 - rate) * val2.u;
//    double v = rate * val1.v + (1.0 - rate) * val2.v;
//    double t_u = (rate * val1.t_u + (1.0 - rate) * val2.t_u);
//    double t_v = (rate * val1.t_v + (1.0 - rate) * val2.t_v);
//    double tangent = atan2(t_v, t_u); // ?
//
//    geom::Vector2D pos = RotatebyAngle(_heading, u, v);
//    DirectedPoint p(_start_position, _heading + tangent);
//    p.location.x += pos.x;
//    p.location.y += pos.y;
//    return p;
  }
  std::pair<float, float> GeometryParamPoly3::DistanceTo(const geom::Location &) const {
    // No analytical expression (Newton-Raphson?/point search)
    // throw_exception(std::runtime_error("not implemented"));
    return {_start_position.x, _start_position.y};
  }

  void GeometryParamPoly3::PreComputeSpline() {
    throw std::logic_error("GeometryParamPoly3::PreComputeSpline not implemented!");
//    // Roughly the interval size in m
//    constexpr double interval_size = 0.5;
//    size_t number_intervals =
//        std::max(static_cast<size_t>(_length / interval_size), size_t(5));
//    double delta_p = 1.0 / number_intervals;
//    if (_arcLength) {
//        delta_p *= _length;
//    }
//    double param_p = 0;
//    double current_s = 0;
//    double last_u = _polyU.Evaluate(param_p);
//    double last_v = _polyV.Evaluate(param_p);
//    double last_s = 0;
//    RtreeValue last_val{
//        last_u,
//        last_v,
//        last_s,
//        _polyU.Tangent(param_p),
//        _polyV.Tangent(param_p) };
//    for(size_t i = 0; i < number_intervals; ++i) {
//      param_p += delta_p;
//      double current_u = _polyU.Evaluate(param_p);
//      double current_v = _polyV.Evaluate(param_p);
//      double du = current_u - last_u;
//      double dv = current_v - last_v;
//      double ds = sqrt(du * du + dv * dv);
//      current_s += ds;
//      double current_t_u = _polyU.Tangent(param_p);
//      double current_t_v = _polyV.Tangent(param_p);
//      RtreeValue current_val{
//          current_u,
//          current_v,
//          current_s,
//          current_t_u,
//          current_t_v };
//
//      Rtree::BPoint p1(static_cast<float>(last_s));
//      Rtree::BPoint p2(static_cast<float>(current_s));
//      _rtree.InsertElement(Rtree::BSegment(p1, p2), last_val, current_val);
//
//      last_u = current_u;
//      last_v = current_v;
//      last_s = current_s;
//      last_val = current_val;
//
//      if(current_s > _length){
//        break;
//      }
//    }
  }
} // namespace element
} // namespace road
} // namespace carla
