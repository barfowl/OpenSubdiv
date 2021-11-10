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
    //  Methods to query common features of a parameterization.
    //
    //  Methods for corners and boundaries require a corner or boundary
    //  index.  The parameter "t" for boundaries locally parameterizes a
    //  boundary edge over [0,1] in a counter-clockwise orientation.
    //
    //  WIP - beware of going overboard here in terms of functionality
    //        and overloading
    //      - methods returning aggregates also need to be careful about
    //        types, e.g. use of Coord[] is likely to be replaced
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
    //  Utilities to convert (u,v) coordinates to Ptex coordinates -- for
    //  either direct use with Ptex per-face textures, or as an alternative
    //  parameterization for non-quads with quad-based subdivision schemes
    //  (as used in other places in OpenSubdiv).
    //
    //  The (u,v) coordinates of quads and triangles will pass through the
    //  conversions unchanged when used with the appropriate schemes.
    //
    //  Note that instance methods are preferred here to static methods as
    //  the conversion depends on both the face size and the subdivision
    //  scheme (from which a temporary instance can be trivially created).
    //
    void ConvertUvToPtex(float   inU,   float   inV,
                         float & ptexU, float & ptexV, int & ptexFace) const;
    void ConvertPtexToUv(float   ptexU, float   ptexV, int   ptexFace,
                         float & outU,  float & outV) const;

    //  WIP - to be deprecated, now obsolete given the above methods...
    int ConvertQPolyUVToNormalizedSubQuad(float u, float v,
                                          float & uOut, float & vOut) const;
    void ConvertQPolyUVFromNormalizedSubQuad(float u, float v, int subQuad,
                                             float & uOut, float & vOut) const;

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

    unsigned int _faceSize  : 16;
    unsigned int _type      :  4;
    unsigned int _scheme    :  4;
    unsigned int _qPolyUDim :  8;
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
//  Inline topological queries:
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
//  Inline Ptex conversion methods:
//
//  WIP - Eventually the QPOLY parameterization will make use of a "udim"
//  member -- set to the integer sqrt(faceSize) to reduce roundoff for
//  large face sizes.  Until then, all UV tiles are sequential in U.
//
inline void
Parameterization::ConvertUvToPtex(float inU, float inV,
        float & ptexU, float & ptexV, int & ptexFace) const {

    if (_type == QPOLY) {
        ptexFace = (int) inU;
        ptexU    = 2.0 * (inU - ptexFace);
        ptexV    = 2.0 *  inV;
    } else {
        ptexFace = 0;
        ptexU    = inU;
        ptexV    = inV;
    }
}

inline void
Parameterization::ConvertPtexToUv(float ptexU, float ptexV, int ptexFace,
        float & outU,  float & outV) const {

    if (_type == QPOLY) {
        outU = 0.5 * ptexU + ptexFace;
        outV = 0.5 * ptexV;
    } else {
        outU = ptexU;
        outV = ptexV;
    }
}

inline int
Parameterization::ConvertQPolyUVToNormalizedSubQuad(
        float inU, float inV, float & outU, float & outV) const {

    int outSubQuad = 0;
    ConvertUvToPtex(inU, inV, outU, outV, outSubQuad);
    return outSubQuad;
}

inline void
Parameterization::ConvertQPolyUVFromNormalizedSubQuad(
        float inU, float inV, int inSubQuad, float & outU, float & outV) const {

    ConvertPtexToUv(inU, inV, inSubQuad, outU, outV);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_PARAMETERIZATION */
