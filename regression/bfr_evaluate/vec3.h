//
//   Copyright 2021 Pixar
//
//   Licensed under the Apache License, Version 2.0 (the "Apache License")
//   with the following modification; you may not use this file except in
//   compliance with the Apache License and the following modification to it:
//   Section 6. Trademarks. is deleted and replaced with:
//
//   6. Trademarks. This License does not grant permission to use the trade
//      names, trademarks, service marks, or product names of the Licensor
//      and its affiliates, except as required to comply with Section 4(c) of
//      the License and to reproduce the content of the NOTICE file.
//
//   You may obtain a copy of the Apache License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the Apache License with the above modification is
//   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
//   KIND, either express or implied. See the Apache License for the specific
//   language governing permissions and limitations under the Apache License.
//
#ifndef OPENSUBDIV3_REGRESSION_BFR_EVALUATE_VEC3_H
#define OPENSUBDIV3_REGRESSION_BFR_EVALUATE_VEC3_H

#include <cmath>

//
//  Simple interpolatable struct for (x,y,z) positions and normals:
//
template <typename REAL>
struct Vec3 {
    Vec3<REAL>() { }
    Vec3<REAL>(REAL x, REAL y, REAL z) { p[0] = x, p[1] = y, p[2] = z; }

    //  Clear() and AddWithWeight() required for interpolation:
    void Clear( void * =0 ) { p[0] = p[1] = p[2] = 0.0f; }

    void AddWithWeight(Vec3<REAL> const & src, REAL weight) {
        p[0] += weight * src.p[0];
        p[1] += weight * src.p[1];
        p[2] += weight * src.p[2];
    }

    //  Element access via []:
    REAL const & operator[](int i) const { return p[i]; }
    REAL       & operator[](int i)       { return p[i]; }

    //  Element access via []:
    REAL const * Coords() const { return p; }
    REAL       * Coords()       { return p; }

    //  Additional useful mathematical operations:
    Vec3<REAL> operator-(Vec3<REAL> const & x) const {
        return Vec3<REAL>(p[0] - x.p[0], p[1] - x.p[1], p[2] - x.p[2]);
    }
    Vec3<REAL> operator+(Vec3<REAL> const & x) const {
        return Vec3<REAL>(p[0] + x.p[0], p[1] + x.p[1], p[2] + x.p[2]);
    }
    Vec3<REAL> operator*(REAL s) const {
        return Vec3<REAL>(p[0] * s, p[1] * s, p[2] * s);
    }
    Vec3<REAL> Cross(Vec3<REAL> const & x) const {
        return Vec3<REAL>(p[1]*x.p[2] - p[2]*x.p[1],
                          p[2]*x.p[0] - p[0]*x.p[2],
                          p[0]*x.p[1] - p[1]*x.p[0]);
    }
    REAL Dot(Vec3<REAL> const & x) const {
        return p[0]*x.p[0] + p[1]*x.p[1] + p[2]*x.p[2];
    }
    REAL Length() const {
        return std::sqrt(this->Dot(*this));
    }

    //  Static method to compute normal vector:
    static
    Vec3<REAL> ComputeNormal(Vec3<REAL> const & Du, Vec3<REAL> const & Dv,
                             REAL eps = 0.0f) {
        Vec3<REAL> N = Du.Cross(Dv);
        REAL lenSqrd = N.Dot(N);
        if (lenSqrd <= eps) return Vec3<REAL>(0.0f, 0.0f, 0.0f);
        return N * (1.0f / std::sqrt(lenSqrd));
    }

    //  Member variables (XYZ coordinates):
    REAL p[3];
};

typedef Vec3<float>  Vec3f;
typedef Vec3<double> Vec3d;

#endif /* OPENSUBDIV3_REGRESSION_BFR_EVALUATE_VEC3_H */
