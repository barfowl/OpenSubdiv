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

#ifndef OPENSUBDIV3_BFR_VERTEX_DESCRIPTOR_H
#define OPENSUBDIV3_BFR_VERTEX_DESCRIPTOR_H

#include "../version.h"

#include "../bfr/limits.h"
#include "../vtr/stackBuffer.h"

#include <cstring>
#include <cassert>
#include <algorithm>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  VertexDescriptor is a simple class that describes the full topological
//  neighborhood around a vertex of a mesh, i.e. its valence, the sizes
//  of its incident faces, sharpness values, etc.
//
//  It is used by subclasses of SurfaceFactory to provide a complete
//  topological description for each vertex of a face, i.e. invoked via
//  the virtual method:
//
//      int populateFaceVertexDescriptor(Index baseFace,
//                                       int cornerVertex,
//                                       VertexDescriptor & v) const;
//
//  Assignment of the full topology can be involved in the presence of
//  irregular faces, non-manifold topology or creasing around a vertex, but
//  many cases will be simple.  For example, to specify a regular boundary
//  vertex of a Catmark mesh without any optional sharpness:
//
//      int  numIncidentFaces = 2;
//      bool vertexOnBoundary = true;
//
//      vd.Initialize(numIncidentFaces);
//          vd.SetManifold(true);
//          vd.SetBoundary(vertexOnBoundary);
//          vd.SetCommonFaceSize(true);
//      vd.Finalize();
//
//  For a more general example, to assign a vertex of some valence whose
//  incident faces are of different sizes (e.g. required when triangles
//  appear around a vertex in an otherwise quad-dominant Catmark mesh):
//
//      int  numIncidentFaces = meshVertex.GetNumIncidentFaces();
//      bool vertexOnBoundary = meshVertex.IsBoundar();
//
//      vd.Initialize(numIncidentFaces);
//          vd.SetManifold(true);
//          vd.SetBoundary(vertexOnBoundary);
//
//          vd.SetCommonFaceSize(false);
//          for (int i = 0; i < numIncidentFaces; ++i) {
//              vd.SetIncidentFaceSize(i, meshVertex.GetIncidentFaceSize(i));
//          }
//      vd.Finalize();
//
//  These examples specify the incident faces as forming a manifold ring
//  (or half-ring) around the vertex, i.e. they can be specified as a
//  continuous, connected sequence in counter-clockwise order (and also
//  without degeneracies).  In the case of a boundary vertex, the first
//  face must be on the leading edge of the boundary while the last is on
//  the trailing edge.  For an interior vertex, which face is specified
//  first does not matter (since the set is periodic).
//
//  In both cases, the location of the base face in this sequence -- the
//  face whose corner vertex is being described here -- must be specified
//  in the return value to populateFaceVertexDescriptor() (e.g. when a
//  boundary vertex has 3 incident faces, a return value of 0, 1 or 2
//  will indicate which is the base face).
//
//  The corresponding methods to specify mesh control vertex indices (or
//  face-varying indices) complete the specification of the neighborhood:
//
//      int getFaceCornerVertexIndices(Index baseFace, int cornerVertex,
//                                     Index vertexIndices[]) const;
//
//      int getFaceCornerFVarValueIndices(Index baseFace, int cornerVertex,
//                                        Index fvarValueIndices[],
//                                        int   fvarChannel) const;
//
//  and are invoked by the Factory when needed.
//
//  For each incident face, the indices for all vertices of that face are
//  to be specified (not the one-ring or some other subset).  These indices
//  must also be specified in an orientation relative to the vertex, i.e.
//  for a vertex A and an indicident face with face-vertices that may be
//  stored internally as {D, C, A, B}, they must be specified with A first
//  as {A, B, C, D}.  This may seem a bit cumbersome, but it has clear
//  advantages when dealing with face-varying indices and unordered faces.
//
//  More compact ways of specifying vertex indices for ordered, manifold
//  cases may be worth exploring in future, but face-varying indices and
//  non-manifold (unordered) vertices will always require such a full set,
//  so both methods will need to co-exist.
//  
class VertexDescriptor {
public:
    VertexDescriptor() { }
    ~VertexDescriptor() { }

    //  The full declarartion must be enclosed by calls to these methods:
    //
    //  Note that the valence of a vertex will be internally clamped to a
    //  value determined by Bfr::Limits (typically 16-bits) that reflects
    //  topological limits elsewhere in OpenSubdiv, so beware of exceeding
    //  that limit.
    //
    void Initialize(int numIncidentFaces);
    void Finalize();

    //
    //  Three groups of methods describe the topology around a vertex:
    //      - simple properties (vertex is a boundary, manifold, etc.)
    //      - sizes of incident faces (constant or size for each face)
    //      - sharpness of the vertex and its incident edges (optional)
    //

    //  Manifold and boundary conditions:
    //
    //  The manifold property is a strict condition but preferred for
    //  efficiency and is usually available from common connected mesh
    //  representations.  When declaring the topology as "manifold",
    //  the Factory assumes the following:
    //
    //      - all incident faces are "ordered" (counter-clockwise)
    //      - all incident faces are consistently oriented
    //      - all incident edges are non-degenerate
    //
    //  If not certain that all of these conditions are met, it is best
    //  to not declare manifold -- leaving the Factory to make sense of
    //  the set of incident faces from the face-vertex indices that are
    //  provided elsewhere.
    //  
    void SetManifold(bool isManifold);
    bool IsManifold() const;
    bool IsOrdered() const;

    //  Boundary status is ignored if not manifold:
    void SetBoundary(bool isOnBoundary);
    bool IsBoundary() const;

    //  Sizes of incident faces -- must specify each if not common:
    void SetCommonFaceSize(bool incidentFacesHaveCommonSize);
    bool HasCommonFaceSize() const;

    void SetIncidentFaceSize(int faceIndex, int faceSize);
    int  GetIncidentFaceSize(int faceIndex) const;

    //  Optional vertex sharpness:
    void SetVertexSharpness(float sharpness);

    bool  HasVertexSharpness() const;
    float GetVertexSharpness() const;

    //  Optional edge sharpness -- the more general method assigns the
    //  sharpness to the leading and trailing edges of each face, but a
    //  simpler method allows direct assignment to edges when manifold.
    void SetIncidentFaceEdgeSharpness(int faceIndex, float leadingEdgeSharp,
                                                     float trailingEdgeSharp);

    void SetManifoldEdgeSharpness(int edgeIndex, float edgeSharpness);

    bool HasEdgeSharpness() const;

protected:
    friend class FaceVertex;

    typedef Vtr::internal::StackBuffer<int,8,true>    IntBuffer;
    typedef Vtr::internal::StackBuffer<float,16,true> FloatBuffer;

protected:
    //  Member variables assigned through the above interface:
    unsigned short _isInitialized : 1;
    unsigned short _isFinalized   : 1;

    unsigned short _isOrdered  : 1;
    unsigned short _isBoundary : 1;

    unsigned short _hasFaceSizes     : 1;
    unsigned short _hasEdgeSharpness : 1;
    unsigned short _wasFaceSizesSet  : 1;

    short _numFaces;
    float _vertSharpness;

    FloatBuffer _faceEdgeSharpness;
    IntBuffer   _faceSizeOffsets;
};

//
//  Public inline methods for simple assignment:
//  
inline void
VertexDescriptor::SetManifold(bool isManifold) {
    _isOrdered  = isManifold;
}
inline bool
VertexDescriptor::IsManifold() const {
    return _isOrdered;
}
inline bool
VertexDescriptor::IsOrdered() const {
    return _isOrdered;
}

inline void
VertexDescriptor::SetBoundary(bool isBoundary) {
    _isBoundary = isBoundary;
}
inline bool
VertexDescriptor::IsBoundary() const {
    return _isBoundary;
}

inline void
VertexDescriptor::SetCommonFaceSize(bool common) {
    _hasFaceSizes = !common;
    _wasFaceSizesSet = true;
}
inline bool
VertexDescriptor::HasCommonFaceSize() const {
    return !_hasFaceSizes;
}

inline void
VertexDescriptor::SetVertexSharpness(float vertSharpness) {
    _vertSharpness = vertSharpness;
}
inline float
VertexDescriptor::GetVertexSharpness() const {
    return _vertSharpness;
}

inline bool
VertexDescriptor::HasVertexSharpness() const {
    return _vertSharpness > 0.0f;
}
inline bool
VertexDescriptor::HasEdgeSharpness() const {
    return _hasEdgeSharpness;
}

inline void
VertexDescriptor::SetIncidentFaceSize(int incFaceIndex, int faceSize) {

    if ((int)_faceSizeOffsets.GetSize() != (_numFaces + 1)) {
        _faceSizeOffsets.SetSize(_numFaces + 1);
        std::memset(_faceSizeOffsets, 0, (_numFaces + 1) * sizeof(int));
        _hasFaceSizes = true;
    }
    _faceSizeOffsets[incFaceIndex] = std::min(faceSize, Limits::MaxFaceSize());
}
inline int
VertexDescriptor::GetIncidentFaceSize(int incFaceIndex) const {
    return _faceSizeOffsets[incFaceIndex];
}

inline void
VertexDescriptor::SetManifoldEdgeSharpness(int edgeIndex, float sharpness) {

    assert(IsManifold());
    if (!_hasEdgeSharpness) {
        _faceEdgeSharpness.SetSize(_numFaces * 2);
        std::memset(_faceEdgeSharpness, 0, (_numFaces * 2) * sizeof(float));
        _hasEdgeSharpness = true;
    }

    //  Assign the leading edge of the face after the edge (even index):
    if (edgeIndex < _numFaces) {
        _faceEdgeSharpness[2*edgeIndex] = sharpness;
    }

    //  Assign the trailing edge of the face before the edge (odd index):
    if (edgeIndex > 0) {
        _faceEdgeSharpness[2*edgeIndex-1] = sharpness;
    } else if (!IsBoundary()) {
        _faceEdgeSharpness[2*_numFaces-1] = sharpness;
    }
}
inline void
VertexDescriptor::SetIncidentFaceEdgeSharpness(int   faceIndex,
                                               float leadingEdgeSharpness,
                                               float trailingEdgeSharpness) {

    if (!_hasEdgeSharpness) {
        _faceEdgeSharpness.SetSize(_numFaces * 2);
        std::fill(&_faceEdgeSharpness[0],
                  &_faceEdgeSharpness[_numFaces*2], 0.0f);
        _hasEdgeSharpness = true;
    }

    _faceEdgeSharpness[2*faceIndex  ] = leadingEdgeSharpness;
    _faceEdgeSharpness[2*faceIndex+1] = trailingEdgeSharpness;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_VERTEX_DESCRIPTOR_H */
