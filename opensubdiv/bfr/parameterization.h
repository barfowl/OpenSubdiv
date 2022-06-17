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

#include "../bfr/limits.h"
#include "../sdc/types.h"

#include <cmath>
#include <cassert>

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
    enum Type { QUAD, TRI, QUAD_SUBFACES };

public:
    Parameterization() : _faceSize(0) { }
    Parameterization(Sdc::SchemeType scheme, int faceSize);
    ~Parameterization() { }

    bool IsValid() const { return (_faceSize > 0); }

    int  GetFaceSize() const { return _faceSize; }
    Type GetType() const { return (Type) _type; }

public:
    //
    //  Methods to query common features of a parameterization.
    //
    //  Methods for vertices and edges require an index of the vertex
    //  or edge.  The edge parameter "t" locally parameterizes the edge
    //  over [0,1] in a counter-clockwise orientation.
    //
    template <typename REAL>
    void GetVertexCoord(int vertexIndex, REAL uvCoord[2]) const;

    template <typename REAL>
    void GetEdgeCoord(int edgeIndex, REAL t, REAL uvCoord[2]) const;

    template <typename REAL>
    void GetCenterCoord(REAL uvCoord[2]) const;

public:
    //
    //  Methods to deal with discontinuous parameterizations, i.e. those
    //  partitioned into sub-faces:
    //
    bool HasSubFaces() const { return (_type == QUAD_SUBFACES); }

    template <typename REAL>
    int GetSubFace(REAL const uvCoord[2]) const;

    //
    //  Conversion methods to/from sub-face coordinates -- only for use
    //  with instances partitioned into sub-faces:
    //
    //  Note that sub-face coordinates that are normalized correspond
    //  to coordinates for Ptex faces.
    //
    template <typename REAL>
    int ConvertCoordToSubFace(
                REAL const uvCoord[2], REAL subFaceCoord[2]) const;
    template <typename REAL>
    int ConvertCoordToNormalizedSubFace(
                REAL const uvCoord[2], REAL subFaceCoord[2]) const;

    template <typename REAL>
    void ConvertSubFaceToCoord(int subFace,
                REAL const subFaceCoord[2], REAL uvCoord[2]) const;
    template <typename REAL>
    void ConvertNormalizedSubFaceToCoord(int subFace,
                REAL const subFaceCoord[2], REAL uvCoord[2]) const;

private:
    template <typename REAL, bool NORMALIZED>
    int convertCoordToSubFace(
                REAL const uvCoord[2], REAL subFaceCoord[2]) const;
    template <typename REAL, bool NORMALIZED>
    void convertSubFaceToCoord(int subFace,
                REAL const subFaceCoord[2], REAL uvCoord[2]) const;

private:
    unsigned char  _type;
    unsigned char  _uDim;
    unsigned short _faceSize;
};

//
//  Inline construction and resizing methods:
//
inline
Parameterization::Parameterization(Sdc::SchemeType scheme, int faceSize) {

    int regFaceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(scheme);

    _type     = (unsigned char) ((regFaceSize == 4) ? QUAD : TRI);
    _faceSize = (unsigned short) std::min(faceSize, Limits::MaxFaceSize());
    _uDim     = 0;

    if (_faceSize != regFaceSize) {
        if (_faceSize < 3) {
            //  Reset size to 0 (invalid) for degenerate faces of all schemes:
            _faceSize = 0;
        } else if (regFaceSize == 3) {
            //  Reset size to 0 (invalid) for non-triangles of tri schemes:
            _faceSize = 0;
        } else {
            //  Quad sub-faces -- use int sqrt for udim to preserve accuracy:
            _type = QUAD_SUBFACES;
            _uDim = (_faceSize < 10) ?
                    (unsigned char)(2 + (_faceSize > 4)) :
                    (unsigned char)(1 + (int) std::sqrt((float)(_faceSize-1)));
        }
    }
}

//
//  Simple coordinate queries:
//
template <typename REAL>
void
Parameterization::GetVertexCoord(int vertex, REAL uv[2]) const {

    switch (GetType()) {
    case QUAD:
        uv[0] = (REAL) (vertex && (vertex < 3));
        uv[1] = (REAL) (vertex > 1);
        break;
    case TRI:
        uv[0] = (REAL) (vertex == 1);
        uv[1] = (REAL) (vertex == 2);
        break;
    case QUAD_SUBFACES:
        uv[0] = (REAL) (vertex % _uDim);
        uv[1] = (REAL) (vertex / _uDim);
        break;
    default:
        uv[0] = -1.0f;
        uv[1] = -1.0f;
        break;
    }
}

template <typename REAL>
void
Parameterization::GetEdgeCoord(int edge, REAL t, REAL uv[2]) const {

    switch (GetType()) {
    case QUAD:
        switch (edge) {
        case 0: uv[0] = t;        uv[1] = 0.0f;     break;
        case 1: uv[0] = 1.0f;     uv[1] = t;        break;
        case 2: uv[0] = 1.0f - t; uv[1] = 1.0f;     break;
        case 3: uv[0] = 0.0f;     uv[1] = 1.0f - t; break;
        }
        break;

    case TRI:
        switch (edge) {
        case 0: uv[0] = t;        uv[1] = 0.0f;     break;
        case 1: uv[0] = 1.0f - t; uv[1] = t;        break;
        case 2: uv[0] = 0.0f;     uv[1] = 1.0f - t; break;
        }
        break;

    case QUAD_SUBFACES:
        if (t < 0.5f) {
            GetVertexCoord(edge, uv);
            uv[0] += t;
        } else {
            GetVertexCoord((edge + 1) % _faceSize, uv);
            uv[1] += 1.0f - t;
        }
        break;
    default:
        uv[0] = -1.0f;
        uv[1] = -1.0f;
        break;
    }
}

template <typename REAL>
void
Parameterization::GetCenterCoord(REAL uv[2]) const {

    if (GetType() == TRI) {
        uv[0] = 1.0f / 3.0f;
        uv[1] = 1.0f / 3.0f;
    } else {
        uv[0] = 0.5f;
        uv[1] = 0.5f;
    }
}

//
//  Sub-face coordinate conversion methods:
//
template <typename REAL>
int
Parameterization::GetSubFace(REAL const uvCoord[2]) const {

    return HasSubFaces() ? (_uDim * (int)uvCoord[1] + (int)uvCoord[0]) : 0;
}

//  Private conversions used by the public conversions:
template <typename REAL, bool NORMALIZED>
inline int
Parameterization::convertCoordToSubFace(
        REAL const uvCoord[2], REAL subCoord[2]) const {

    assert(HasSubFaces());

    //  Be sure this assignment always supports conversion in-place:
    int uTile = (int) uvCoord[0];
    int vTile = (int) uvCoord[1];

    if (NORMALIZED) {
        subCoord[0] = (uvCoord[0] - (REAL) uTile) * 2.0f;
        subCoord[1] = (uvCoord[1] - (REAL) vTile) * 2.0f;
    } else {
        subCoord[0] = (uvCoord[0] - (REAL) uTile);
        subCoord[1] = (uvCoord[1] - (REAL) vTile);
    }
    return _uDim * vTile + uTile;
}
template <typename REAL, bool NORMALIZED>
inline void
Parameterization::convertSubFaceToCoord(
        int subFace, REAL const subCoord[2], REAL uvCoord[2]) const {

    assert(HasSubFaces());

    //  Be sure this assignment always supports conversion in-place:
    int uTile = subFace % _uDim;
    int vTile = subFace / _uDim;

    if (NORMALIZED) {
        uvCoord[0] = (REAL) uTile + subCoord[0] * 0.5f;
        uvCoord[1] = (REAL) vTile + subCoord[1] * 0.5f;
    } else {
        uvCoord[0] = (REAL) uTile + subCoord[0];
        uvCoord[1] = (REAL) vTile + subCoord[1];
    }
}

//  Conversions to unnormalized sub-face coordinates:
template <typename REAL>
inline int
Parameterization::ConvertCoordToSubFace(
        REAL const uvCoord[2], REAL subCoord[2]) const {

    return convertCoordToSubFace<REAL,false>(uvCoord, subCoord);
}
template <typename REAL>
inline void
Parameterization::ConvertSubFaceToCoord(
        int subFace, REAL const subCoord[2], REAL uvCoord[2]) const {

    convertSubFaceToCoord<REAL,false>(subFace, subCoord, uvCoord);
}

//  Conversions to normalized sub-face coordinates:
template <typename REAL>
inline int
Parameterization::ConvertCoordToNormalizedSubFace(
        REAL const uvCoord[2], REAL subCoord[2]) const {

    return convertCoordToSubFace<REAL,true>(uvCoord, subCoord);
}
template <typename REAL>
inline void
Parameterization::ConvertNormalizedSubFaceToCoord(
        int subFace, REAL const subCoord[2], REAL uvCoord[2]) const {

    convertSubFaceToCoord<REAL,true>(subFace, subCoord, uvCoord);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_PARAMETERIZATION */
