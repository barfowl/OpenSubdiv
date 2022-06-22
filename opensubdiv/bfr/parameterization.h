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

#include "../sdc/types.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Parameterization is a simple class that provides information about the
//  parameterization of a face in a local (u,v) coordinate system. That
//  information is determined given the size of a face (i.e. its number of
//  vertices) and the subdivision scheme used to subdivide it.
//
//  The subdivision scheme is essential in determining how a face is
//  parameterized, e.g. a triangle is regular for the Loop scheme and so
//  has a very simple parameterization -- unlike a triangle for the
//  Catmull-Clark scheme, which must be quadrangulated.
//
class Parameterization {
public:
    //
    //  The three kinds of parameterizations defined are:  quadrilateral,
    //  triangle and quadrangulated sub-faces.  This is not intended for
    //  common use, but is publicly available for situations when it is
    //  necessary to distinguish:
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
    bool HasSubFaces() const;

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
    template <typename REAL>
    int convertCoordToSubFace(bool normalized,
                REAL const uvCoord[2], REAL subFaceCoord[2]) const;
    template <typename REAL>
    void convertSubFaceToCoord(bool normalized, int subFace,
                REAL const subFaceCoord[2], REAL uvCoord[2]) const;

private:
    unsigned char  _type;
    unsigned char  _uDim;
    unsigned short _faceSize;
};

//
//  Inline sub-face coordinate conversion methods:
//
inline bool
Parameterization::HasSubFaces() const {
    return (_type == QUAD_SUBFACES);
}

template <typename REAL>
inline int
Parameterization::GetSubFace(REAL const uvCoord[2]) const {
    return HasSubFaces() ? (_uDim * (int)uvCoord[1] + (int)uvCoord[0]) : 0;
}

//  Conversions to unnormalized sub-face coordinates:
template <typename REAL>
inline int
Parameterization::ConvertCoordToSubFace(
        REAL const uvCoord[2], REAL subCoord[2]) const {
    return convertCoordToSubFace<REAL>(false, uvCoord, subCoord);
}
template <typename REAL>
inline void
Parameterization::ConvertSubFaceToCoord(
        int subFace, REAL const subCoord[2], REAL uvCoord[2]) const {
    convertSubFaceToCoord<REAL>(false, subFace, subCoord, uvCoord);
}

//  Conversions to normalized sub-face coordinates:
template <typename REAL>
inline int
Parameterization::ConvertCoordToNormalizedSubFace(
        REAL const uvCoord[2], REAL subCoord[2]) const {
    return convertCoordToSubFace<REAL>(true, uvCoord, subCoord);
}
template <typename REAL>
inline void
Parameterization::ConvertNormalizedSubFaceToCoord(
        int subFace, REAL const subCoord[2], REAL uvCoord[2]) const {
    convertSubFaceToCoord<REAL>(true, subFace, subCoord, uvCoord);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_PARAMETERIZATION */
