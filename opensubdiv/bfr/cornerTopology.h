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

#ifndef OPENSUBDIV3_BFR_CORNER_TOPOLOGY_H
#define OPENSUBDIV3_BFR_CORNER_TOPOLOGY_H

#include "../version.h"

#include "../bfr/cornerTag.h"
#include "../bfr/vertexTopology.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  CornerSubset is a simple struct and a companion of CornerTopology that
//  identifies a subset of the topology around a corner.  Such subsets are
//  what ultimately define the limit surface around a face and so are used
//  by higher level classes in conjunction with CornerTopology.
//
struct CornerSubset {
    CornerSubset() { }

    void Initialize(CornerTag tag) {
        _tag = tag;
        _numFacesBefore = 0;
        _numFacesAfter  = 0;
        _numFacesTotal  = 1;
        _localSharpness = 0.0f;
    }

    //  Simple get/set methods to avoid the tedious syntax of the tag:
    int GetNumFaces() const { return _numFacesTotal; }

    bool IsBoundary() const { return _tag._boundaryVerts; }
    bool IsSharp()    const { return _tag._infSharpVerts; }

    void SetBoundary(bool on) { _tag._boundaryVerts = on; }
    void SetSharp(bool on)    { _tag._infSharpVerts = on; }

    //  Methods comparing to a superset (not any arbitrary subset):
    bool MatchesExtentOfSuperset(CornerSubset const & sup) const {
        return (GetNumFaces() == sup.GetNumFaces()) &&
               (IsBoundary()  == sup.IsBoundary());
    }
    bool MatchesShapeOfSuperset(CornerSubset const & sup) const {
        return MatchesExtentOfSuperset(sup) &&
               (IsSharp() == sup.IsSharp());
    }

    //  Member tags containing boundary and sharp bits:
    CornerTag _tag;

    //  Members defining the extent of the subset:
    short _numFacesBefore;
    short _numFacesAfter;
    short _numFacesTotal;

    //  Member to override vertex sharpness (rarely used):
    float _localSharpness;
};

//
//  The CornerTopology class is the primary internal class for gathering
//  all topological information around the corner of a face.  As such, it
//  wraps an instance of the public VertexTopology class populated by the
//  Factory subclasses.  It extends an instance of VertexTopology with
//  additional topological information and methods to make it more widely
//  available and useful to internal classes.
//
//  One fundamental extension of CornerTopology is that it includes the
//  location of the face in the ring of incident faces around the vertex.
//  VertexTopology alone simple specifies the neighborhood of the vertex,
//  but the CornerTopology provides context relative to the face for
//  which all of this information is being gathered.
//
//  Several instances of CornerTopology (one for each corner of a face)
//  are necessary to fully define the limit surface for a face, but in
//  many cases, only a subset of the CornerTopology's incident faces will
//  actually contribute to the surface. A companion class for defining
//  that subset is defined elsewhere.
//  
class CornerTopology {
public:
    CornerTopology() { }
    ~CornerTopology() { }

    //  Methods to invoke before/after assigning VertexTopology:
    void Initialize(int faceSize, int regFaceSize);
    void Finalize(int faceInVertex);

    VertexTopology & GetVertexTopology() { return _vTop; }

    //  Methods to deal with connecting unordered incident faces:
    void ConnectUnOrderedFaces(Index const faceVertexIndices[]);

    bool AreUnOrderedFacesConnected() const;

    //  Methods to initialize/find subsets:
    int InitializeCompleteSubset(CornerSubset * subset) const;

    int FindConnectedSubset(CornerSubset * subset) const;

    int FindFaceVaryingSubset(CornerSubset       * fvarSubset,
                              Index const          fvarIndices[],
                              CornerSubset const & vtxSubset) const;

    //  Methods to control sharpness of the corner in a subset:
    void SharpenSubset(CornerSubset * subset) const;
    void SharpenSubset(CornerSubset * subset, float sharpness) const;
    void UnSharpenSubset(CornerSubset * subset) const;

    bool  HasImplicitSharpness() const;
    float GetImplicitSharpness() const;

public:
    //  Methods to query properties after finalization:
    CornerTag GetTag() const { return _tag; }

    int GetNumFaces() const { return _vTop._numFaces; }

    int GetFaceInVertex() const { return _faceInRing; }

    int GetNumFaceVertices() const { return _numFaceVerts; }

    bool HasCommonFaceSize() const { return (_commonFaceSize > 0); }
    int  GetCommonFaceSize() const { return _commonFaceSize; }

public:
    //  Methods to inspect and iterate through the incident faces:
    int GetFaceSize(int face) const;

    int GetFaceNext(    int face) const;
    int GetFacePrevious(int face) const;

    int GetFaceAfter(int step) const;
    int GetFaceBefore(int step) const;

    //  Methods to access indices associated with face-vertices:
    int GetFaceVertexOffset(int face) const;

    int GetFaceVertexAtCorner(          Index const indices[]) const;
    int GetFaceVertexAtCorner(int face, Index const indices[]) const;
    int GetFaceVertexTrailing(int face, Index const indices[]) const;
    int GetFaceVertexLeading( int face, Index const indices[]) const;

    //  Methods to access sharpness of the vertex or its incident edges:
    float GetVertexSharpness() const;
    float GetFaceEdgeSharpness(int faceEdge) const;
    float GetFaceEdgeSharpness(int face, bool trailingEdge) const;

private:
    //  Private methods for managing CornerSubsets:
    int findConnectedSubsetExtent(CornerSubset * subset) const;

    int findFVarSubsetExtent(CornerSubset const & vtxSubset,
                             CornerSubset       * fvarSubset,
                             Index const          fvarIndices[]) const;

    void adjustSubsetTags(CornerSubset       * subset,
                          CornerSubset const * superset = 0) const;

    bool subsetHasInfSharpEdges( CornerSubset const & subset) const;
    bool subsetHasSemiSharpEdges(CornerSubset const & subset) const;
    bool subsetHasIrregularFaces(CornerSubset const & subset) const;

private:
    //  Internal types and methods to connect and assess the topology for
    //  unordered faces given their associated face-vertex indices:
    struct Edge;

    int  gatherUnOrderedEdges(Edge        edges[],
                              short       faceEdgeIndices[],
                              Index const faceVertIndices[]) const;

    void markDuplicateEdges(Edge        edges[],
                            short const faceEdgeIndices[],
                            Index const faceVertIndices[]) const;

    void assignUnOrderedFaceNeighbors(Edge const  edges[],
                                      short const faceEdgeIndices[]);

    void assignUnOrderedTags(Edge const edges[], int numEdges);

private:
    typedef Vtr::internal::StackBuffer<short,16,true> ShortBuffer;

    VertexTopology _vTop;
    CornerTag      _tag;

    short _faceInRing;
    short _commonFaceSize;

    unsigned short _regFaceSize    :  4;
    unsigned short _isExpInfSharp  :  1;
    unsigned short _isExpSemiSharp :  1;
    unsigned short _isImpInfSharp  :  1;
    unsigned short _isImpSemiSharp :  1;

    short _numInfSharpEdges;
    short _numSemiSharpEdges;
    int   _numFaceVerts;

    ShortBuffer _faceEdgeNeighbors;
};

//
//  Inline methods for traversing incident faces of the vertex:
//
inline int
CornerTopology::GetFaceSize(int face) const {
    return _commonFaceSize ? _commonFaceSize :
            (_vTop._faceSizeOffsets[face+1] - _vTop._faceSizeOffsets[face]);
}

inline int
CornerTopology::GetFaceNext(int face) const {
    if (_tag._unOrderedFaces) {
        assert(_faceEdgeNeighbors.GetSize());
        return _faceEdgeNeighbors[2*face + 1];
    } else {
        return ((face + 1) == _vTop._numFaces ) ? 0 : (face + 1);
    }
}
inline int
CornerTopology::GetFacePrevious(int face) const {
    if (_tag._unOrderedFaces) {
        assert(_faceEdgeNeighbors.GetSize());
        return _faceEdgeNeighbors[2*face];
    } else {
        return face ? (face - 1) : (_vTop._numFaces - 1);
    }
}

inline int
CornerTopology::GetFaceAfter(int step) const {
    assert(step >= 0);
    if (_tag._unOrderedFaces) {
        assert(_faceEdgeNeighbors.GetSize());

        int face = _faceInRing;
        for ( ; step > 0; --step) {
            face =_faceEdgeNeighbors[2*face + 1];
            assert(face >= 0);
        }
        return face;
    } else {
        return (_faceInRing + step) % _vTop._numFaces;
    }
}
inline int
CornerTopology::GetFaceBefore(int step) const {
    assert(step >= 0);
    if (_tag._unOrderedFaces) {
        assert(_faceEdgeNeighbors.GetSize());

        int face = _faceInRing;
        for ( ; step > 0; --step) {
            face =_faceEdgeNeighbors[2*face];
            assert(face >= 0);
        }
        return face;
    } else {
        return (_faceInRing - step + _vTop._numFaces) % _vTop._numFaces;
    }
}
inline bool
CornerTopology::AreUnOrderedFacesConnected() const {
    return _faceEdgeNeighbors.GetSize() > 0;
}

//
//  Inline methods for accessing indices associated with indicent faces:
//
inline int
CornerTopology::GetFaceVertexOffset(int face) const {
    return _commonFaceSize ? (face * _commonFaceSize) :
                             _vTop._faceSizeOffsets[face];
}

inline int
CornerTopology::GetFaceVertexAtCorner(Index const indices[]) const {
    return indices[GetFaceVertexOffset(_faceInRing)];
}
inline int
CornerTopology::GetFaceVertexAtCorner(int face, Index const indices[]) const {
    return indices[GetFaceVertexOffset(face)];
}
inline int
CornerTopology::GetFaceVertexLeading(int face, Index const indices[]) const {
    return indices[GetFaceVertexOffset(face) + 1];
}
inline int
CornerTopology::GetFaceVertexTrailing(int face, Index const indices[]) const {
    // It is safe to use "face+1" here for the last face:
    return indices[GetFaceVertexOffset(face+1) - 1];
}

//
//  Inline methods for accessing face-edge sharpness values:
//
inline float
CornerTopology::GetVertexSharpness() const {
    return _vTop._vertSharpness;
}
inline float
CornerTopology::GetFaceEdgeSharpness(int faceEdge) const {
    return _vTop._faceEdgeSharpness[faceEdge];
}
inline float
CornerTopology::GetFaceEdgeSharpness(int face, bool trailing) const {
    return _vTop._faceEdgeSharpness[face*2 + trailing];
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_CORNER_TOPOLOGY_H */
