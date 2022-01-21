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
#include "../sdc/crease.h"

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

    //  Queries consistent with other classes:
    CornerTag GetTag() const { return _tag; }

    int GetNumFaces() const { return _numFacesTotal; }

    //  Simple get/set methods to avoid the tedious syntax of the tag:
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

    //  Methods supporting construction/initialization (subclass required
    //  to populate VertexTopology between Initialize and Finalize):
    void Initialize(int faceSize, int regFaceSize);
    void Finalize(int faceInVertex);

    VertexTopology & GetVertexTopology() { return _vTop; }

    void ConnectUnOrderedFaces(Index const faceVertexIndices[]);

public:
    //  Methods to initialize and find subsets:
    int GetVertexSubset(CornerSubset * subset) const;

    int FindFaceVaryingSubset(CornerSubset       * fvarSubset,
                              Index const          fvarIndices[],
                              CornerSubset const & vtxSubset) const;

    //  Methods to control sharpness of the corner in a subset:
    void SharpenSubset(CornerSubset * subset) const;
    void SharpenSubset(CornerSubset * subset, float sharpness) const;
    void UnSharpenSubset(CornerSubset * subset) const;

public:
    //
    //  Public methods to query simple properties:
    //
    CornerTag GetTag() const { return _tag; }

    int GetFace() const { return _faceInRing; }

    int GetNumFaces()        const { return _vTop._numFaces; }
    int GetNumFaceVertices() const { return _numFaceVerts; }

    bool HasCommonFaceSize() const { return (_commonFaceSize > 0); }
    int  GetCommonFaceSize() const { return _commonFaceSize; }

public:
    //
    //  Public methods to inspect incident faces and connected neighbors:
    //
    int GetFaceSize(int face) const;

    //  Get neighbors of a specific face (return -1 if unconnected):
    int GetFaceNext(    int face) const;
    int GetFacePrevious(int face) const;

    //  Find faces relative to this face (require a known safe step size):
    int GetFaceAfter (int stepForwardFromCornerFace) const;
    int GetFaceBefore(int stepBackwardFromCornerFace) const;

    //  Get first and last faces of a subset:
    int GetFaceFirst(CornerSubset const & subset) const;
    int GetFaceLast( CornerSubset const & subset) const;

public:
    //
    //  Public methods to access indices assigned to incident faces:
    //
    int GetFaceIndexOffset(int face) const;

    Index GetFaceIndexAtCorner(Index const indices[]) const;

    Index GetFaceIndexAtCorner(int face, Index const indices[]) const;
    Index GetFaceIndexTrailing(int face, Index const indices[]) const;
    Index GetFaceIndexLeading( int face, Index const indices[]) const;

    bool FaceIndicesMatchAtCorner(  int f1, int f2, Index const indices[])const;
    bool FaceIndicesMatchAtEdgeEnd( int f1, int f2, Index const indices[])const;
    bool FaceIndicesMatchAcrossEdge(int f1, int f2, Index const indices[])const;

public:
    //
    //  Public methods for sharpness of the vertex or its incident edges:
    //
    float GetVertexSharpness() const;

    float GetFaceEdgeSharpness(int faceEdge) const;
    float GetFaceEdgeSharpness(int face, bool trailingEdge) const;

    bool IsFaceEdgeSharp(    int face, bool trailingEdge) const;
    bool IsFaceEdgeInfSharp( int face, bool trailingEdge) const;
    bool IsFaceEdgeSemiSharp(int face, bool trailingEdge) const;

    bool  HasImplicitVertexSharpness() const;
    float GetImplicitVertexSharpness() const;

private:
    //  Internal convenience methods:
    bool isOrdered()   const { return _tag.IsOrdered(); }
    bool isUnOrdered() const { return _tag.IsUnOrdered(); }
    bool isBoundary()  const { return _tag.IsBoundary(); }
    bool isInterior()  const { return _tag.IsInterior(); }
    bool isManifold()  const { return _tag.IsManifold(); }

    int getConnectedFaceNext(int face) const;
    int getConnectedFacePrev(int face) const;

private:
    //  Internal methods for assembling and managing subsets:
    int initCompleteSubset(CornerSubset * subset) const;

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
    //  Internal methods to connect a set of unordered faces (given their
    //  associated face-vertex indices) and assess the resulting topology:
    struct Edge;

    int  createUnOrderedEdges(Edge        edges[],
                              short       faceEdgeIndices[],
                              Index const faceVertIndices[]) const;

    void markDuplicateEdges(Edge        edges[],
                            short const faceEdgeIndices[],
                            Index const faceVertIndices[]) const;

    void assignUnOrderedFaceNeighbors(Edge const  edges[],
                                      short const faceEdgeIndices[]);

    void finalizeUnOrderedTags(Edge const edges[], int numEdges);

    //  Ordered counterpart to the above method for finalizing tags
    void finalizeOrderedTags();

private:
    typedef Vtr::internal::StackBuffer<short,16,true> ShortBuffer;

    //  Private members:
    VertexTopology _vTop;
    CornerTag      _tag;

    short _faceInRing;
    short _commonFaceSize;

    unsigned short _regFaceSize    :  4;
    unsigned short _isExpInfSharp  :  1;
    unsigned short _isExpSemiSharp :  1;
    unsigned short _isImpInfSharp  :  1;
    unsigned short _isImpSemiSharp :  1;

    int _numFaceVerts;

    ShortBuffer _faceEdgeNeighbors;
};


//
//  Inline methods for inspecting/traversing incident faces of the vertex:
//
inline int
CornerTopology::GetFaceSize(int face) const {
    return _commonFaceSize ? _commonFaceSize :
            (_vTop._faceSizeOffsets[face+1] - _vTop._faceSizeOffsets[face]);
}

inline int
CornerTopology::getConnectedFaceNext(int face) const {
    return _faceEdgeNeighbors[2*face + 1];
}
inline int
CornerTopology::getConnectedFacePrev(int face) const {
    return _faceEdgeNeighbors[2*face];
}

inline int
CornerTopology::GetFaceNext(int face) const {
    if (isUnOrdered()) {
        return getConnectedFaceNext(face);
    } else if (face < (_vTop._numFaces - 1)) {
        return face + 1;
    } else {
        return isBoundary() ? -1 : 0;
    }
}
inline int
CornerTopology::GetFacePrevious(int face) const {
    if (isUnOrdered()) {
        return getConnectedFacePrev(face);
    } else if (face) {
        return face - 1;
    } else {
        return isBoundary() ? -1 : (_vTop._numFaces - 1);
    }
}

inline int
CornerTopology::GetFaceAfter(int step) const {
    assert(step >= 0);
    if (isOrdered()) {
        return (_faceInRing + step) % _vTop._numFaces;
    } else if (step == 1) {
        return getConnectedFaceNext(_faceInRing);
    } else if (step == 2) {
        return getConnectedFaceNext(getConnectedFaceNext(_faceInRing));
    } else {
        int face = _faceInRing;
        for ( ; step > 0; --step) {
            face = getConnectedFaceNext(face);
        }
        return face;
    }
}
inline int
CornerTopology::GetFaceBefore(int step) const {
    assert(step >= 0);
    if (isOrdered()) {
        return (_faceInRing - step + _vTop._numFaces) % _vTop._numFaces;
    } else if (step == 1) {
        return getConnectedFacePrev(_faceInRing);
    } else if (step == 2) {
        return getConnectedFacePrev(getConnectedFacePrev(_faceInRing));
    } else {
        int face = _faceInRing;
        for ( ; step > 0; --step) {
            face = getConnectedFacePrev(face);
        }
        return face;
    }
}

inline int
CornerTopology::GetFaceFirst(CornerSubset const & subset) const {
    return GetFaceBefore(subset._numFacesBefore);
}
inline int
CornerTopology::GetFaceLast( CornerSubset const & subset) const {
    return GetFaceAfter(subset._numFacesAfter);
}

//
//  Inline methods for accessing indices associated with indicent faces:
//
inline int
CornerTopology::GetFaceIndexOffset(int face) const {
    return _commonFaceSize ? (face * _commonFaceSize) :
                             _vTop._faceSizeOffsets[face];
}

inline Index
CornerTopology::GetFaceIndexAtCorner(Index const indices[]) const {
    return indices[GetFaceIndexOffset(_faceInRing)];
}
inline Index
CornerTopology::GetFaceIndexAtCorner(int face, Index const indices[]) const {
    return indices[GetFaceIndexOffset(face)];
}
inline Index
CornerTopology::GetFaceIndexLeading(int face, Index const indices[]) const {
    return indices[GetFaceIndexOffset(face) + 1];
}
inline Index
CornerTopology::GetFaceIndexTrailing(int face, Index const indices[]) const {
    // It is safe to use "face+1" here for the last face:
    return indices[GetFaceIndexOffset(face+1) - 1];
}

inline bool
CornerTopology::FaceIndicesMatchAtCorner(int facePrev, int faceNext,
                                         Index const indices[]) const {
    return GetFaceIndexAtCorner(facePrev, indices) ==
           GetFaceIndexAtCorner(faceNext, indices);
}
inline bool
CornerTopology::FaceIndicesMatchAtEdgeEnd(int facePrev, int faceNext,
                                         Index const indices[]) const {
    return GetFaceIndexTrailing(facePrev, indices) ==
           GetFaceIndexLeading(faceNext, indices);
}
inline bool
CornerTopology::FaceIndicesMatchAcrossEdge(int facePrev, int faceNext,
                                         Index const indices[]) const {
    return FaceIndicesMatchAtCorner (facePrev, faceNext, indices) &&
           FaceIndicesMatchAtEdgeEnd(facePrev, faceNext, indices);
}

//
//  Inline methods for accessing vertex and edge sharpness:
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

inline bool
CornerTopology::IsFaceEdgeSharp(int face, bool trailing) const {
    return Sdc::Crease::IsSharp(_vTop._faceEdgeSharpness[face*2+trailing]);
}
inline bool
CornerTopology::IsFaceEdgeInfSharp(int face, bool trailing) const {
    return Sdc::Crease::IsInfinite(_vTop._faceEdgeSharpness[face*2+trailing]);
}
inline bool
CornerTopology::IsFaceEdgeSemiSharp(int face, bool trailing) const {
    return Sdc::Crease::IsSemiSharp(_vTop._faceEdgeSharpness[face*2+trailing]);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_CORNER_TOPOLOGY_H */
