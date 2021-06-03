//
//   Copyright 2021
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

#ifndef OPENSUBDIV3_BFR_PARAMETERIZATION_H
#define OPENSUBDIV3_BFR_PARAMETERIZATION_H

#include "../version.h"

#include "../bfr/types.h"
#include "../sdc/types.h"


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Parameterization is a simple class that provides information about the
//  parameterization of faces of a particular size for a given subdivision
//  scheme.
//
//  The subdivision scheme is essential in determining how a face is
//  parameterized, e.g. a triangle is regular for the Loop scheme and so
//  has a very simple parameterization -- unlike a triangle for the
//  Catmull-Clark scheme, which must be quadrangulated.
//
class Parameterization {
public:
    //
    //  The three kinds of parameterizations:  quadrilateral, triangle and
    //  quadrangulated N-sided polygon.  This is not intended for general
    //  use, but is made public for clients that do need to distinguish:
    //
    enum Type { QUAD, TRI, QPOLY };

public:
    Parameterization() : _faceSize(0) { }
    Parameterization(Sdc::SchemeType scheme);
    Parameterization(Sdc::SchemeType scheme, int faceSize);
    ~Parameterization() { }

    bool IsValid() const { return (_faceSize > 0); }

    int  GetFaceSize() const { return _faceSize; }
    Type GetType() const { return (Type) _type; }

    //  Redefine for a different face of the same subdivision scheme:
    void Resize(int faceSize);

public:
    //
    //  Methods to query common features of a parameterization -- beware of
    //  going overboard here, and methods returning aggregates need to be
    //  careful about types (e.g. use of Coord[])
    //
    //  Methods for corners and boundaries require a corner or boundary
    //  index.  The parameter "t" for boundaries locally parameterizes a
    //  boundary edge over [0,1] in a counter-clockwise orientation.
    //
    //  Methods returning singular (u,v) coordinates:
    void GetCornerCoord(int index, float & u, float & v) const;
    void GetCornerCoord(int index, Coord & uv) const;

    void GetBoundaryCoord(int index, float t, float & u, float & v) const;
    void GetBoundaryCoord(int index, float t, Coord & uv) const;

    void GetCenterCoord(float & u, float & v) const;
    void GetCenterCoord(Coord & uv) const;

    //  Potential methods returning multiple (u,v) coordinates:
    int GetCornerCoords(Coord uvs[]) const;
    int GetBoundaryCoords(int index, float t0, float dt, int n, Coord uvs[]) const;

public:
    //
    //  Conversion utilities for dealing with quadrangulated polygons -- note
    //  why instance methods are preferred here to static (possible future
    //  UDim option to reduce floating point precision loss)
    //
    //  May want to avoid "continuous" to avoid confusion with parametric or
    //  geometric continuity, consider "piece-wise" as an alternative...
    bool IsContinuous() const { return _type != QPOLY; }

    int ConvertQPolyUVToNormalizedSubQuad(float u, float v,
                                          float & uOut, float & vOut) const;
    void ConvertQPolyUVFromNormalizedSubQuad(float u, float v, int subQuad,
                                             float & uOut, float & vOut) const;

private:
    void initialize();

    unsigned int _faceSize  : 16;
    unsigned int _type      :  4;
    unsigned int _scheme    :  4;
    unsigned int _qPolyUDim :  8;
};

inline void
Parameterization::initialize() {

    Sdc::SchemeType schemeType = (Sdc::SchemeType) _scheme;
    if (Sdc::SchemeTypeTraits::GetRegularFaceSize(schemeType) == 3) {
        _type = TRI;
        //  Reset size as 0 for now for non-tris, possibly assert()
        if (_faceSize != 3) _faceSize = 0;
    } else if (_faceSize == 4) {
        _type = QUAD;
    } else {
        _type = QPOLY;
        _qPolyUDim = 0;  // eventually make this integer sqrt(faceSize)
    }
}

inline
Parameterization::Parameterization(Sdc::SchemeType scheme) :
        _scheme(scheme), _qPolyUDim(0) {

    _faceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(scheme);
    initialize();
}

inline
Parameterization::Parameterization(Sdc::SchemeType scheme, int faceSize) :
        _faceSize(faceSize), _scheme(scheme), _qPolyUDim(0) {

    initialize();
}

inline void
Parameterization::Resize(int faceSize) {

    _faceSize = faceSize;
    initialize();
}

//
//  Topological queries:
//
inline void
Parameterization::GetCornerCoord(int corner, Coord & coord) const {

    GetCornerCoord(corner, coord[0], coord[1]);
}

inline void
Parameterization::GetBoundaryCoord(int edge, float t, Coord & coord) const {

    GetBoundaryCoord(edge, t, coord[0], coord[1]);
}

inline void
Parameterization::GetCenterCoord(Coord & coord) const {

    GetCenterCoord(coord[0], coord[1]);
}

//
//  Eventually want QPOLY parameterization to make use of _qPolyUDim, which
//  will be set to the integer sqrt(N) to reduce roundoff for large N.  At
//  this point, all UV tiles for subquads are sequential in U...
//
inline int
Parameterization::ConvertQPolyUVToNormalizedSubQuad(
        float u, float v, float & uOut, float & vOut) const {

    int subQuad = (int)u;
    uOut = 2.0 * (u - subQuad);
    vOut = 2.0 *  v;
    return subQuad;
}

inline void
Parameterization::ConvertQPolyUVFromNormalizedSubQuad(
        float u, float v, int subQuad, float & uOut, float & vOut) const {

    uOut = 0.5 * u + subQuad;
    vOut = 0.5 * v;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_PARAMETERIZATION */
