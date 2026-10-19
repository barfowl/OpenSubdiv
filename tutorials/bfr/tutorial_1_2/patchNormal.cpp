//
//   Copyright 2026 Pixar
//
//   Licensed under the terms set forth in the LICENSE.txt file available at
//   https://opensubdiv.org/license.
//

#include "./patchNormal.h"

#include <cmath>

//
//  Definitions of lower level functions for computing patch normals.
//
namespace internal {

namespace {
    //
    //  Supporting classes and utilities...
    //
    //  Since OpenSubdiv does not have a class for supporting arithmetic
    //  operations between points and vectors in R3 (e.g. Imath's Vec3<T>
    //  classes) we need to define something for local use here. In the
    //  spirit of Imath::Vec3, this class follows some of its conventions
    //  (e.g. naming, use of operator% for cross product) but is stripped
    //  down to the bare minimum functionality required here.
    //
    //  Beware if adapting this code to make use of Imath as one of the
    //  most heavily used constructors here is one that initializes a new
    //  Vec3f from an array (i.e. Vec3f(float const p[])) which has no
    //  counterpart in Imath (for good reason).
    //
    class Vec3f {
    public:
        //  Constructors, assignment and destructor:
        Vec3f() { }
        Vec3f(float const p[]) : x(p[0]), y(p[1]), z(p[2]) { }
        Vec3f(float a, float b, float c) : x(a), y(b), z(c) { }

        Vec3f(Vec3f const & src) = default;
        Vec3f & operator=(Vec3f const & src) = default;
        ~Vec3f() = default;

        //  Value retrieval:
        void GetValue(float p[]) const { p[0] = x, p[1] = y, p[2] = z; }

        //  Arithmetic operators (only those required below are defined):
        Vec3f operator*(float s) const {
            return Vec3f(x * s, y * s, z * s);
        }
        Vec3f operator+(Vec3f const & v) const {
            return Vec3f(x + v.x, y + v.y, z + v.z);
        }
        Vec3f operator%(Vec3f const & v) const {
            return Vec3f(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
        }
        void Zero() { x = y = z = 0.0f; }

        //  Length and normalization operations:
        float Length2() const { return x*x + y*y + z*z; }
        float Length()  const { return std::sqrt(Length2()); }

        bool Normalize(float eps = 0.0f) {
            float lenSqrd = Length2();
            //  Remember 'eps' is relative to length, not length-squared:
            if (lenSqrd > (eps * eps)) {
                float len = std::sqrt(lenSqrd);
                x /= len, y /= len, z /= len;
                return true;
            } else {
                x = y = z = 0.0f;
                return false;
            }
        }

    public:
        float x, y, z;
    };

    //  Scalar pre-multiplication (s * Vec3f) defined separately:
    inline Vec3f operator*(float s, Vec3f & v) { return v * s; }

} // end namespace


//
//  All patch normal computation functions take an optional tolerance to
//  detect suspiciously small vector results whose accumulated floating
//  point error may produce sporadic results. If not specified, a default
//  non-zero tolerance will be inferred locally from the scale of given
//  vectors and a default proportional tolerance.
//
//  The proportional tolerance constant here has been assigned the same
//  value as the similar tolerance from the GPU shaders:
//
inline float constexpr DfltToleranceMultiplier = 0.00001f;

//
//  This first function, computing simply (N = ds X dt) is potentially
//  so simple it may not warrant including. It's value over a similar
//  function defined externally is that a default tolerance will be
//  inferred and applied consistently. If already working with explicit
//  tolerances, this is probably unnecessary.
//
bool
computePatchNormal(float const aDs[3], float const aDt[3],
                   float aN[3], float tolerance) {

    Vec3f ds(aDs), dt(aDt);

    if (tolerance < 0.0f) {
        //  Yields a length between 1 and 2 times the largest of the two:
        tolerance = (ds.Length() + dt.Length()) * DfltToleranceMultiplier;
    }
    //  Could use the tolerance to zero out ds or dt here, but that would
    //  not be consistent with current GPU shader implementations...

    Vec3f N = ds % dt;

    bool normalized = N.Normalize(tolerance);
    N.GetValue(aN);
    return normalized;
}

//
//  This second function resolves cases where (N = ds X dt = 0), which can
//  occur for some features (e.g. smooth corners) and common degenerate
//  cases (e.g. collapsed boundaries at poles).
//
//  Resolving the degeneracy here requires all second order derivatives
//  and additional knowledge about the parameterization, i.e. the (s,t)
//  location at which N(s,t) = 0 and the shape of the domain (quad or tri).
//
//  The method here uses the Taylor series expansion of the function N(s,t)
//  to obtain a non-zero N in the neighborhood of (s,t) where N(s,t) = 0.
//  The leading non-zero term of the series -- typically the first -- yields
//  the desired result.
//
bool
computePatchNormal(float s, float t, bool isQuadPatch,
           float const aDs[3],  float const aDt[3],
           float const aDss[3], float const aDst[3], float const aDtt[3],
           float aN[3], float tolerance) {

    //
    //  Assign locals from the input args and compute lengths (squared)
    //  of derivatives for use in determining and applying tolerances:
    //
    Vec3f ds(aDs), dt(aDt);
    Vec3f dss(aDss), dst(aDst), dtt(aDtt);

    float dsLen2  = ds.Length2();
    float dtLen2  = dt.Length2();
    float dssLen2 = dss.Length2();
    float dstLen2 = dst.Length2();
    float dttLen2 = dtt.Length2();

    if (tolerance < 0.0f) {
        //  Yields a length between 1 and 2.2 (sqrt(5)) times the largest:
        tolerance  = std::sqrt(dsLen2 + dtLen2 + dssLen2 + dstLen2 + dttLen2);
        tolerance *= DfltToleranceMultiplier;
    }
    if (tolerance > 0.0f) {
        float tolLen2 = tolerance * tolerance;

        if (dsLen2 < tolLen2) ds.Zero(), dsLen2 = 0.0f;
        if (dtLen2 < tolLen2) dt.Zero(), dtLen2 = 0.0f;
        if (dssLen2 < tolLen2) dss.Zero(), dssLen2 = 0.0f;
        if (dstLen2 < tolLen2) dst.Zero(), dstLen2 = 0.0f;
        if (dttLen2 < tolLen2) dtt.Zero(), dttLen2 = 0.0f;
    }

    //
    //  First determine the scalar multipliers of normal derivatives
    //  in the terms of the series. These multipliers reflect the small
    //  epsilon around the (s,t) where N(s,t) = 0, but since we can
    //  factor them out, their absolute magnitudes are not significant,
    //  only their signs -- but these signs are critical.
    //
    //  It is essential the neighborhood containing the epsilon remain
    //  within the domain of the patch, and negative signs determined
    //  here will ensure this. If a derivative on a boundary is zero,
    //  its direction often flips as it crosses the boundary, so a step
    //  back toward the interior from the outer boundaries is needed.
    //  The shape of the domain is necessary here to determine where
    //  the desired neighborhood lies relative to (s,t) on a boundary.
    //
    float sMultiplier = 1.0f;
    float tMultiplier = 1.0f;

    if (isQuadPatch) {
        //  Negate ds or dt if moving past either outer boundary:
        sMultiplier = (s < 1.0f) ? 1.0f : -1.0f;
        tMultiplier = (t < 1.0f) ? 1.0f : -1.0f;
    } else {
        //  Negate ds and dt if moving past the diagonal boundary,
        //  doubling the negative direction at the corners to stay
        //  within the triangular domain:
        if ((s + t) >= 1.0f) {
            if (s > 0.0f) sMultiplier = (t > 0.0f) ? -1.0f : -2.0f;
            if (t > 0.0f) tMultiplier = (s > 0.0f) ? -1.0f : -2.0f;
        }
    }

    //
    //  Use the 1st order Taylor series term when 1st derivatives are
    //  non-zero, otherwise that 1st order term also evaluates to zero,
    //  so use the 2nd:
    //
    Vec3f N;

    bool hasNonZero1stDerivs = (dsLen2 + dtLen2) > 0.0f;
    if (hasNonZero1stDerivs) {
        //
        //  1st order derivs of N(s,t) require 2nd order derivs of P(s,t):
        //
        Vec3f dNds = (dss % dt) + (ds % dst);
        Vec3f dNdt = (dst % dt) + (ds % dtt);

        N = sMultiplier * dNds + tMultiplier * dNdt;
    } else {
        //
        //  2nd order derivs of N(s,t) require 3rd order derivs of P(s,t),
        //  but 3rd order derivs are unavailable here. Fortunately the
        //  ds = dt = 0 condition leads to all 3rd order terms cancelling,
        //  which greatly simplifies the expression to the following:
        //
        Vec3f dNdss = dss % dst;
        Vec3f dNdst = dss % dtt;
        Vec3f dNdtt = dst % dtt;

        N = sMultiplier * sMultiplier * dNdss +
            sMultiplier * tMultiplier * dNdst +
            tMultiplier * tMultiplier * dNdtt;
    }
    bool normalized = N.Normalize(tolerance);
    N.GetValue(aN);
    return normalized;
}

} // end namespace internal
