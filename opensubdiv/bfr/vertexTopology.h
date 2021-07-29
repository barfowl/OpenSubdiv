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
//          vt.SetOrdered(vertexOnBoundary);
//          vt.SetCommonFaceSize(4);
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
//          vt.SetOrdered(vertexOnBoundary);
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
    VertexTopology() : _isInitialized(false) { }
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
    void SetOrdered(bool isBoundaryVertex);

    bool IsOrdered() const { return _isOrdered; }
    bool IsBoundary() const { return _isBoundary; }

    //  Sizes of incident faces -- must specify each if not common:
    void SetCommonFaceSize(int size);
    int  GetCommonFaceSize() const { return _commonFaceSize; }

    int * AccessFaceSizeBuffer();

    //  Assigning vertex and edge sharpness:
    void  SetVertexSharpness(float sharpness);
    float GetVertexSharpness() const { return _vertSharpness; }

    float * AccessFaceEdgeSharpnessBuffer(bool clear);

protected:
    //
    //  WIP - Protected access is currently given to LimitSurfaceFactory
    //  to augment the description for its purposes, and to provide
    //  frequently used methods to access it.  Current plans are to strip
    //  this class down to a purely public class and moving all protected
    //  members and methods to some other internal class that either "is-a"
    //  or "has-a" instance of VertexTopology.  The protected methods that
    //  are inline will then be moved to a non-public header.
    //
    friend class LimitSurfaceFactory;
    friend class FaceTopology;

    //  Methods for inspection (many inlined below)
    int getFaceSize(int faceIndex) const;

    int getFaceNext(    int faceIndex) const;
    int getFacePrevious(int faceIndex) const;

    int getFaceAfter(int faceIndex, int step) const;
    int getFaceBefore(int faceIndex, int step) const;

    int getFaceVertexOffset(int faceIndex) const;

    int getFaceVertexAtCorner(int faceIndex, Index const indices[]) const;
    int getFaceVertexTrailing(int faceIndex, Index const indices[]) const;
    int getFaceVertexLeading( int faceIndex, Index const indices[]) const;

    //  Methods specific to inspection of face-varying indices:
    int getNumMatchingCornerIndices(Index match, Index const indices[]) const;
    bool moreThanTwoUniqueCornerIndices(Index const indices[]) const;

protected:
    //  WIP - full "int" size unnecessary for most members here, and since
    //  we have local arrays of these, that may matter -- so use "short"
    //  where possible internally but beware size conversion warnings
    unsigned int _isInitialized : 1;
    unsigned int _isOrdered     : 1;
    unsigned int _isBoundary    : 1;
    unsigned int _isInterior    : 1;
    unsigned int _hasSharpVert  : 1;
    unsigned int _hasSharpEdge  : 1;
    unsigned int _isFinalized   : 1;

    int _numFaces;
    int _commonFaceSize;

    float _vertSharpness;

    //  Use of cumulative offsets here inhibits use of short
    Vtr::internal::StackBuffer<int,8,true>    _faceSizeOffsets;
    Vtr::internal::StackBuffer<float,16,true> _faceEdgeSharpness;

    //  WIP - not specified directly, so may be moved elsewhere
    int _numFaceVerts;
};

//
//  Public inline methods for assignment:
//  
inline void
VertexTopology::SetOrdered(bool isBoundary) {

    _isOrdered  = true;
    _isBoundary = isBoundary;
    _isInterior = !isBoundary;
}
inline void
VertexTopology::SetCommonFaceSize(int size) {

    _commonFaceSize = size;
}

inline int *
VertexTopology::AccessFaceSizeBuffer() {

    if (_faceSizeOffsets.GetSize() == 0) {
        _faceSizeOffsets.SetSize(_numFaces + 1);
    }
    return _faceSizeOffsets;
}

inline void
VertexTopology::SetVertexSharpness(float sharpness) {

    _vertSharpness = sharpness;
    _hasSharpVert  = (sharpness > 0.0);
}

inline float *
VertexTopology::AccessFaceEdgeSharpnessBuffer(bool clear) {

    if (_faceEdgeSharpness.GetSize() == 0) {
        _faceEdgeSharpness.SetSize(_numFaces * 2);
    }
    if (clear) {
        std::fill(&_faceEdgeSharpness[0], &_faceEdgeSharpness[_numFaces*2], 0);
    }
    _hasSharpEdge = true;
    return _faceEdgeSharpness;
}

//
//  Non-public inline methods for traversing and gathering properties
//  of the incident faces of the vertex:
//  
inline int
VertexTopology::getFaceSize(int face) const {

    return _commonFaceSize ? _commonFaceSize :
            (_faceSizeOffsets[face+1] - _faceSizeOffsets[face]);
}

inline int
VertexTopology::getFaceNext(int face) const {
    assert(_isOrdered);
    return ((face + 1) == _numFaces ) ? 0 : (face + 1);
}
inline int
VertexTopology::getFacePrevious(int face) const {
    assert(_isOrdered);
    return face ? (face - 1) : (_numFaces - 1);
}

inline int
VertexTopology::getFaceAfter(int face, int step) const {
    assert(_isOrdered);
    return (face + step) % _numFaces;
}
inline int
VertexTopology::getFaceBefore(int face, int step) const {
    assert(_isOrdered);
    return (face - step + _numFaces) % _numFaces;
}

inline int
VertexTopology::getFaceVertexOffset(int face) const {

    return _commonFaceSize ? (face * _commonFaceSize) : _faceSizeOffsets[face];
}

inline int
VertexTopology::getFaceVertexAtCorner(int face, Index const indices[]) const {
    return indices[getFaceVertexOffset(face)];
}
inline int
VertexTopology::getFaceVertexLeading(int face, Index const indices[]) const {
    return indices[getFaceVertexOffset(face) + 1];
}
inline int
VertexTopology::getFaceVertexTrailing(int face, Index const indices[]) const {
    // There will be no index access issues using "face+1"
    return indices[getFaceVertexOffset(face+1) - 1];
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_VERTEX_TOPOLOGY_H */
