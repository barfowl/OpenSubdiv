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

#include <cmath>

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
    //  Methods to query common features of a parameterization.
    //
    //  Methods for vertices and edges require an index of the vertex
    //  or edge.  The edge parameter "t" locally parameterizes the edge
    //  over [0,1] in a counter-clockwise orientation.
    //
    template <typename REAL>
    void GetVertexCoord(int vertexIndex, REAL * u, REAL * v) const;

    template <typename REAL>
    void GetEdgeCoord(int edgeIndex, REAL t, REAL * u, REAL * v) const;

    template <typename REAL>
    void GetCenterCoord(REAL * u, REAL * v) const;

public:
    //
    //  Utilities to convert (u,v) coordinates to Ptex coordinates -- for
    //  either direct use with Ptex per-face textures, or as an alternative
    //  parameterization for non-quads with quad-based subdivision schemes
    //  (as used in other places in OpenSubdiv).
    //
    //  The (u,v) coordinates of quads and triangles will pass through the
    //  conversions unchanged when used with the appropriate schemes as Bfr
    //  and Ptex parameterizations are consistent in these cases.
    //
    //  Note that instance methods are preferred here to static methods as
    //  the conversion depends on both the face size and the subdivision
    //  scheme (from which a temporary instance can be trivially created).
    //
    template <typename REAL>
    void ConvertUvToPtex(REAL   inU,   REAL   inV,
                         REAL * ptexU, REAL * ptexV, int * ptexFace) const;
    template <typename REAL>
    void ConvertPtexToUv(REAL   ptexU, REAL   ptexV, int   ptexFace,
                         REAL * outU,  REAL * outV) const;

    //  Method to query if parameterizations is continuous, i.e. two or
    //  more (u,v) locations can be interpolated to provide a meaningful
    //  result.
    //
    //  WIP - may want to avoid "continuous" to avoid confusion with
    //        parametric vs geometric continuity, so consider alternatives.
    //      - also, how useful is this without a method to provide some
    //        kind of reasonable interpolation in discontinuous cases?
    bool IsContinuous() const { return _type != QPOLY; }

private:
    void initialize();

    unsigned int _faceSize : 16;
    unsigned int _type     :  4;
    unsigned int _scheme   :  4;
    unsigned int _uDim     :  8;
};

//
//  Inline construction and resizing methods:
//
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

        //  Use int sqrt to reduce accuracy loss tiling with large sizes
        if (_faceSize < 10) {
            _uDim = 2 + (_faceSize > 4);
        } else {
            _uDim = 1 + (int) std::sqrt((float)(_faceSize - 1));
        }
    }
}

inline
Parameterization::Parameterization(Sdc::SchemeType scheme) :
        _scheme(scheme), _uDim(0) {

    _faceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(scheme);
    initialize();
}

inline
Parameterization::Parameterization(Sdc::SchemeType scheme, int faceSize) :
        _faceSize(faceSize), _scheme(scheme), _uDim(0) {

    initialize();
}

inline void
Parameterization::Resize(int faceSize) {

    _faceSize = faceSize;
    initialize();
}

//
//  Simple coordinate queries:
//
template <typename REAL>
void
Parameterization::GetVertexCoord(int vertex, REAL * u, REAL * v) const {

    switch (GetType()) {
    case QUAD:
        *u = (REAL) (vertex && (vertex < 3));
        *v = (REAL) (vertex > 1);
        break;
    case TRI:
        *u = (REAL) (vertex == 1);
        *v = (REAL) (vertex == 2);
        break;
    case QPOLY:
        *u = (REAL) (vertex % _uDim);
        *v = (REAL) (vertex / _uDim);
        break;
    }
}

template <typename REAL>
void
Parameterization::GetEdgeCoord(int edge, REAL t, REAL * u, REAL * v) const {

    switch (GetType()) {
    case QUAD:
        switch (edge) {
        case 0: *u = t;        *v = 0.0f;     break;
        case 1: *u = 1.0f;     *v = t;        break;
        case 2: *u = 1.0f - t; *v = 1.0f;     break;
        case 3: *u = 0.0f;     *v = 1.0f - t; break;
        }
        break;

    case TRI:
        switch (edge) {
        case 0: *u = t;        *v = 0.0f;     break;
        case 1: *u = 1.0f - t; *v = t;        break;
        case 2: *u = 0.0f;     *v = 1.0f - t; break;
        }
        break;

    case QPOLY:
        if (t < 0.5f) {
            GetVertexCoord(edge, u, v);
            *u += t;
        } else {
            GetVertexCoord((edge + 1) % _faceSize, u, v);
            *v += 1.0f - t;
        }
        break;
    }
}

template <typename REAL>
void
Parameterization::GetCenterCoord(REAL * u, REAL * v) const {

    if (GetType() == TRI) {
        *u = 1.0f / 3.0f;
        *v = 1.0f / 3.0f;
    } else {
        *u = 0.5f;
        *v = 0.5f;
    }
}

//
//  Ptex conversion methods:
//
template <typename REAL>
void
Parameterization::ConvertUvToPtex(REAL inU, REAL inV,
        REAL * ptexU, REAL * ptexV, int * ptexFace) const {

    if (_type == QPOLY) {
        int tileU = (int) inU;
        int tileV = (int) inV;

        *ptexFace = _uDim * tileV + tileU;
        *ptexU    = (inU - tileU) * 2.0f;
        *ptexV    = (inV - tileV) * 2.0f;
    } else {
        *ptexFace = 0;
        *ptexU    = inU;
        *ptexV    = inV;
    }
}

template <typename REAL>
void
Parameterization::ConvertPtexToUv(REAL ptexU, REAL ptexV, int ptexFace,
        REAL * outU, REAL * outV) const {

    if (_type == QPOLY) {
        int tileU = ptexFace % _uDim;
        int tileV = ptexFace / _uDim;

        *outU = (REAL) tileU + ptexU * 0.5f;
        *outV = (REAL) tileV + ptexV * 0.5f;
    } else {
        *outU = ptexU;
        *outV = ptexV;
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_PARAMETERIZATION */
