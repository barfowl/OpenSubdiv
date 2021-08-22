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

#ifndef OPENSUBDIV3_BFR_VERTEX_TOPOLOGY_H
#define OPENSUBDIV3_BFR_VERTEX_TOPOLOGY_H

#include "../version.h"

#include "../bfr/types.h"
#include "../vtr/types.h"
#include "../vtr/stackBuffer.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  The VertexTopology class describes the topological neighborhood around
//  a vertex (the valence, size of incident faces, sharpness values, etc.).
//
//  It is used by subclasses of LimitSurfaceFactory to provide a complete
//  topological description for each vertex of a face, i.e. via the method:
//
//      int populateFaceCornerTopology(Index baseFace, int cornerVertex,
//                                     VertexTopology & vt) const;
//
//  Assignment of the full topology can be involved in the presence of
//  irregular faces, non-manifold topology or creasing around a vertex, but
//  many cases will be simple.  For example, to specify a regular boundary
//  vertex of a Catmark mesh:
//
//      int numIncidentFaces = 2;
//      bool vertexOnBoundary = true;
//
//      vt.Initialize(numIncidentFaces);
//          vt.SetOrdered(true);
//          vt.SetBoundary(vertexOnBoundary);
//          vt.SetCommonFaceSize();
//      vt.Finalize();
//
//  For a more general example, to assign a valence-5 vertex with incident
//  faces of different sizes (e.g. required when triangles appear around a
//  vertex in an otherwise quad-dominant Catmark mesh):
//
//      int numIncidentFaces = 5;
//      bool vertexOnBoundary = false;
//
//      vt.Initialize(numIncidentFaces);
//          vt.SetOrdered(true);
//          vt.SetBoundary(vertexOnBoundary);
//          int * incFaceSizes = vt.AccessFaceSizeBuffer();
//
//          for (int i = 0; i < numIncidentFaces; ++i) {
//              incFaceSizes[i] = myVertex.GetIncidentFaceSize(i);
//          }
//      vt.Finalize();
//
//  These examples specify incident faces as being "ordered", i.e. they
//  will be considered as occuring in a counter-clockwise order.  In the
//  case of a boundary vertex, the first face must be on the leading edge
//  while the last is on the trailing edge of the boundary.  For an
//  interior vertex, which face is first does not matter (since the set
//  is periodic).
//
//  In both cases, the location in this sequence of the base face -- the
//  face whose corner vertex is being described here -- must be specified
//  in the return value to populateFaceCornerTopology(). For example, when
//  a boundary vertex has 3 incident faces, a return value of 0, 1 or 2
//  will indicate which is the base face.
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
//  advantages when dealing with face-varying indices and un-ordered faces.
//
//  More compact ways of specifying vertex indices for ordered, manifold
//  cases may be worth exploring in future, but face-varying indices and
//  non-manifold (unordered) vertices will always require such a full set,
//  so both methods will need to co-exist.
//  
class VertexTopology {
public:
    VertexTopology() { }
    ~VertexTopology() { }

    //  The full declarartion must be enclosed by calls to these methods:
    void Initialize(int numIncidentFaces);
    void Finalize();

    //
    //  Topology is specified in three groups of methods:
    //
    //      - ordering and boundary/interior status
    //      - sizes incident faces (constant or size per face)
    //      - sharpness of vertex and/or incident edges
    //
    //  Face ordering and manifold conditions:
    void SetOrdered(bool incidentFacesAreOrdered);
    bool IsOrdered() const;

    void SetBoundary(bool isOnBoundary);
    bool IsBoundary() const;

    //  Sizes of incident faces -- must specify each if not common:
    void SetCommonFaceSize(bool incidentFacesHaveCommonSize);
    bool HasCommonFaceSize() const;

    int * AccessFaceSizeBuffer();

    //  Assigning vertex sharpness:
    void SetVertexSharpness(float sharpness);
    bool HasVertexSharpness() const;

    float GetVertexSharpness() const;

    //  Assigning edge sharpness:
    bool HasEdgeSharpness() const;

    float * AccessFaceEdgeSharpnessBuffer(bool clear);

protected:
    friend class CornerTopology;

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
VertexTopology::SetOrdered(bool isOrdered) {
    _isOrdered  = isOrdered;
}
inline bool
VertexTopology::IsOrdered() const {
    return _isOrdered;
}

inline void
VertexTopology::SetBoundary(bool isBoundary) {
    _isBoundary = isBoundary;
}
inline bool
VertexTopology::IsBoundary() const {
    return _isBoundary;
}

inline void
VertexTopology::SetCommonFaceSize(bool common) {
    _hasFaceSizes = !common;
    _wasFaceSizesSet = true;
}
inline bool
VertexTopology::HasCommonFaceSize() const {
    return !_hasFaceSizes;
}

inline void
VertexTopology::SetVertexSharpness(float vertSharpness) {
    _vertSharpness = vertSharpness;
}
inline float
VertexTopology::GetVertexSharpness() const {
    return _vertSharpness;
}

inline bool
VertexTopology::HasVertexSharpness() const {
    return _vertSharpness > 0.0f;
}
inline bool
VertexTopology::HasEdgeSharpness() const {
    return _hasEdgeSharpness;
}

inline int *
VertexTopology::AccessFaceSizeBuffer() {

    if (_faceSizeOffsets.GetSize() == 0) {
        _faceSizeOffsets.SetSize(_numFaces + 1);
    }
    _hasFaceSizes = true;
    return _faceSizeOffsets;
}

inline float *
VertexTopology::AccessFaceEdgeSharpnessBuffer(bool clear) {

    if (_faceEdgeSharpness.GetSize() == 0) {
        _faceEdgeSharpness.SetSize(_numFaces * 2);
    }
    if (clear) {
        std::fill(&_faceEdgeSharpness[0], &_faceEdgeSharpness[_numFaces*2], 0);
    }
    _hasEdgeSharpness = true;
    return _faceEdgeSharpness;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_VERTEX_TOPOLOGY_H */
