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

#include "../bfr/vertexTopology.h"
#include "../vtr/types.h"
#include "../vtr/stackBuffer.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {


//
//  CornerTags is a simple set of bits that identify exceptional properties
//  at the corner vertices of a face that warrant closer inspection (and
//  potential additional processing).  As with some bitfields in Far, this
//  supports bitwise-OR so that tags for the corners of a face can quickly
//  be combined to determine properties of the associated limit surface:
//
struct CornerTags {
    CornerTags() { }

    typedef unsigned short IntType;

    IntType _boundaryVerts      : 1;
    IntType _boundaryCorners    : 1;
    IntType _boundaryNonSharp   : 1;
    IntType _infSharpVerts      : 1;
    IntType _semiSharpVerts     : 1;
    IntType _anySharpEdges      : 1;
    IntType _unCommonFaceSizes  : 1;
    IntType _irregularFaceSizes : 1;
    IntType _unOrderedFaces     : 1;
    IntType _nonManifoldVerts   : 1;
    IntType _interiorVal2Verts  : 1;

    //  Integer/bit conversions and operations:
    void Clear() {
        std::memset(this, 0, sizeof(*this));
    }
    IntType GetBits() const {
        IntType bits;
        std::memcpy(&bits, this, sizeof(*this));
        return bits;
    }
    void SetBits(IntType bits) {
        std::memcpy(this, &bits, sizeof(*this));
    }
    void BitwiseOr(CornerTags const & tags) {
        SetBits(GetBits() | tags.GetBits());
    }
};


//
//  CornerSubset is a simple struct that identifies a topological subset
//  around a vertex (i.e. CornerTopology).  Such subsets are what define
//  the limit surface around a face:
//
struct CornerSubset {
    //  WIP - replace boundary and sharp bits with CornerTags
    CornerTags _tags;
    //bool IsBoundary() const { return _tags._boundaryVerts; }
    //bool IsSharp()    const { return _tags._infSharpVerts; }

    //  WIP - these two bits are now redundant -- consider removing
    unsigned short _isBoundary;
    unsigned short _isSharp;
    bool IsBoundary() const { return _isBoundary; }
    bool IsSharp() const { return _isSharp; }

    short _numFacesTotal;
    short _numFacesBefore;
    short _numFacesAfter;

    //  These members place the corner relative to a particular collection
    //  of subsets and so may be better off somewhere else...
    short _numOuterVerts;
    short _numOuterFaces;
};

//
//  The CornerTopology class wraps the public VertexTopology class.  It
//  extends VertexTopology with additional topological information and
//  makes it more widely available to internal classes.
//
//  One noteworthy extension of CornerTopology is that it includes the
//  location of the face in the ring of incident faces around the vertex.
//  VertexTopology alone simple specifies the neighborhood of the vertex,
//  but the CornerTopology provides context relative to the face for which
//  all of this information is being gathered.
//  
class CornerTopology {
public:
    CornerTopology() { }
    ~CornerTopology() { }

    //  Methods to invoke before/after assigning VertexTopology:
    void Initialize(int faceSize);
    void Finalize(int regFaceSize, int faceInVertex);

    VertexTopology & GetVertexTopology() { return _vTop; }

public:
    //  Methods to query properties after finalization:
    CornerTags GetTags() const { return _tags; }

    //  WIP - consider removing some of these given public access to tags
    bool IsOrdered() const { return !_tags._unOrderedFaces; }
    bool IsBoundary() const { return _tags._boundaryVerts; }

    bool IsVertexInfSharp() const { return _tags._infSharpVerts; }
    bool IsVertexSemiSharp() const { return _tags._semiSharpVerts; }

    bool HasEdgeSharpness() const { return _tags._anySharpEdges; }

    bool HasUnSharpenedBoundary() const { return _tags._boundaryNonSharp; }

    //  Queries for other members:
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

    int GetFaceVertexAtCorner(int face, Index const indices[]) const;
    int GetFaceVertexTrailing(int face, Index const indices[]) const;
    int GetFaceVertexLeading( int face, Index const indices[]) const;

    //  Methods to access sharpness of the vertex or its incident edges:
    float GetVertexSharpness() const;
    float GetFaceEdgeSharpness(int face, bool trailingEdge) const;

private:
    VertexTopology _vTop;
    CornerTags     _tags;

    short _faceInRing;
    short _commonFaceSize;
    int   _numFaceVerts;
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
    assert(!_tags._unOrderedFaces);
    return ((face + 1) == _vTop._numFaces ) ? 0 : (face + 1);
}
inline int
CornerTopology::GetFacePrevious(int face) const {
    assert(!_tags._unOrderedFaces);
    return face ? (face - 1) : (_vTop._numFaces - 1);
}

inline int
CornerTopology::GetFaceAfter(int step) const {
    assert(!_tags._unOrderedFaces);
    return (_faceInRing + step) % _vTop._numFaces;
}
inline int
CornerTopology::GetFaceBefore(int step) const {
    assert(!_tags._unOrderedFaces);
    return (_faceInRing - step + _vTop._numFaces) % _vTop._numFaces;
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
CornerTopology::GetFaceEdgeSharpness(int face, bool trailing) const {
    return _vTop._faceEdgeSharpness[face*2 + trailing];
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_CORNER_TOPOLOGY_H */
