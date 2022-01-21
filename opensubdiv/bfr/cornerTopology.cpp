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

#include "../bfr/cornerTopology.h"
#include "../sdc/crease.h"

#include <cstring>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Main initialize and finalize methods used to bracket the assignment
//  by clients to the VertexTopology member:
//
void
CornerTopology::Initialize(int faceSize, int regFaceSize) {

    _commonFaceSize = faceSize;
    _regFaceSize    = regFaceSize;
    _numFaceVerts   = 0;

    _isExpInfSharp  = false;
    _isExpSemiSharp = false;
    _isImpInfSharp  = false;
    _isImpSemiSharp = false;
    _numInfSharpEdges  = 0;
    _numSemiSharpEdges = 0;

    _vTop._isInitialized = false;
}

void
CornerTopology::Finalize(int faceInVertex) {

    assert(_vTop._isFinalized);

    //
    //  Initialize tags and other members based on the finalized content
    //  of the VertexTopology.  The tags can be grouped as follows:
    //
    //      - overall topology (ordering, boundary, manifold, etc.)
    //      - sizes of incident faces (common, ordered, etc.)
    //      - vertex sharpness
    //      - edge sharpness
    //
    //  Explicit information provided in VertexTopology may be ignored
    //  via tags set here if values do not deviate from the default.
    //
    _tag.Clear();

    //
    //  General topological properties:
    //
    bool isOrdered = _vTop.IsOrdered();
    if (isOrdered) {
        _tag._unOrderedFaces   = false;
        _tag._nonManifoldVerts = false;

        _tag._boundaryVerts    = _vTop.IsBoundary();
        _tag._boundaryNonSharp = _vTop.IsBoundary();
    } else {
        _tag._unOrderedFaces   = true;
        _tag._nonManifoldVerts = true;

        _tag._boundaryVerts    = false;
        _tag._boundaryNonSharp = false;
    }

    _faceInRing = faceInVertex;

    //
    //  Tags and members related to incident face sizes -- recall that
    //  face sizes are made available as the differences between offsets:
    //
    _tag._unCommonFaceSizes = !_vTop.HasCommonFaceSize();
    if (_tag._unCommonFaceSizes) {
        //  WIP - consider testing for degenerate faces (< 3) here and tag
        //      - may want to convert sizes to offsets here to combine
        _numFaceVerts = _vTop._faceSizeOffsets[_vTop._numFaces];
        _commonFaceSize = 0;
    } else {
        _numFaceVerts = _vTop._numFaces * _commonFaceSize;
    }

    _tag._irregularFaceSizes = (_commonFaceSize != _regFaceSize);

    //
    //  Tags related to vertex sharpness -- simply assign for now (recall
    //  a vertex may later be made sharp due to topology, creasing, etc.):
    //
    _isExpInfSharp  = Sdc::Crease::IsInfinite(_vTop._vertSharpness);
    _isExpSemiSharp = Sdc::Crease::IsSemiSharp(_vTop._vertSharpness);

    _tag._infSharpVerts  = _isExpInfSharp;
    _tag._semiSharpVerts = _isExpSemiSharp;

    //
    //  Tags related to edge sharpness -- test for any assigned values and
    //  set related tags based on inspection.
    // 
    //  We can do more here for a set or ordered faces, which are required
    //  to be manifold.  For unordered, all we can do is test for the
    //  presence of explicit sharpness values -- their association must be
    //  made later once the faces are connected (and so ordered).
    //
    _tag._infSharpEdges  = false;
    _tag._semiSharpEdges = false;
    _tag._infSharpDarts  = false;

    if (_vTop.HasEdgeSharpness()) {
        float const * sharpness = &_vTop._faceEdgeSharpness[0];

        if (isOrdered) {
            //  Detect any unsharpened boundary edges first:
            bool isBoundary = _tag._boundaryVerts;
            if (isBoundary) {
                int last = 2 * _vTop._numFaces - 1;
                _tag._boundaryNonSharp =
                        !Sdc::Crease::IsInfinite(sharpness[0]) ||
                        !Sdc::Crease::IsInfinite(sharpness[last]);
            }

            //  Detect the presence of interior inf-sharp and semi-sharp
            //  edges next (using leading edge of each face):
            _numInfSharpEdges  = 0;
            _numSemiSharpEdges = 0;
            for (int i = isBoundary; i < _vTop._numFaces; ++i ) {
                if (Sdc::Crease::IsInfinite(sharpness[2*i])) {
                    ++ _numInfSharpEdges;
                } else if (Sdc::Crease::IsSharp(sharpness[2*i])) {
                    ++ _numSemiSharpEdges;
                }
            }

            //  Mark the presence of non-boundary/interior edges:
            _tag._infSharpEdges  = (_numInfSharpEdges > 0);
            _tag._semiSharpEdges = (_numSemiSharpEdges > 0);
            _tag._infSharpDarts  = (_numInfSharpEdges == 1) && !isBoundary;

            //  Detect edges effectively making the vertex sharp -- note
            //  it can be both explicitly and implicitly sharp (e.g. low
            //  semi-sharp vertex value with a higher semi-sharp edge):
            int numInfSharpTotal = _numInfSharpEdges + isBoundary * 2;
            if (numInfSharpTotal > 2) {
                _isImpInfSharp = true;
            } else if ((numInfSharpTotal + _numSemiSharpEdges) > 2) {
                _isImpSemiSharp = true;
            }

            //  Mark the vertex inf-sharp if implicitly inf-sharp:
            if (!_isExpInfSharp && _isImpInfSharp) {
                _tag._infSharpVerts  = true;
                _tag._semiSharpVerts = false;
            }
        } else {
            //  Detect explicit sharpness values and set the associated tag:
            int numSharpness = 2 * _vTop._numFaces;
            for (int i = 0; i < numSharpness; ++i) {
                if (Sdc::Crease::IsInfinite(sharpness[i])) {
                    _tag._infSharpEdges = true;
                } else if (Sdc::Crease::IsSharp(sharpness[i])) {
                    _tag._semiSharpEdges = true;
                }
                if (_tag._infSharpEdges && _tag._semiSharpEdges) break;
            }
        }
    }
}

bool
CornerTopology::HasImplicitSharpness() const {

    return _isImpInfSharp || _isImpSemiSharp;
}

float
CornerTopology::GetImplicitSharpness() const {

    if (_isImpInfSharp) {
        return Sdc::Crease::SHARPNESS_INFINITE;
    }
    assert(_isImpSemiSharp);

    //
    //  Since this will be applied at an inf-sharp crease, there will be
    //  two inf-sharp edges in addition to the semi-sharp, so we only
    //  need find the max of the semi-sharp edges and whatever explicit
    //  vertex sharpness may have been assigned:
    //
    float sharpness = GetVertexSharpness();

    for (int i = 0; i < GetNumFaces(); ++i) {
        //  Use the trailing edge of every connected face:
        if (!_tag._unOrderedFaces || (_faceEdgeNeighbors[2*i+1] >= 0)) {
            sharpness = std::max(sharpness, GetFaceEdgeSharpness(2*i+1));
        }
    }
    return sharpness;
}

//
//  Methods to initialize and/or find subsets of the corner's topology:
//
int
CornerTopology::InitializeCompleteSubset(CornerSubset * subset) const {

    assert(GetTag().IsManifold());

    subset->Initialize(GetTag());

    subset->_numFacesBefore = subset->IsBoundary() ? GetFaceInVertex() : 0;
    subset->_numFacesAfter  = GetNumFaces() - subset->_numFacesBefore - 1;
    subset->_numFacesTotal  = GetNumFaces();

    return subset->_numFacesTotal;
}

int
CornerTopology::FindConnectedSubset(CornerSubset * subset) const {

    findConnectedSubsetExtent(subset);

    //  If unconnected faces form a manifold set, tags are accurate:
    if (!GetTag().IsManifold()) {
        adjustSubsetTags(subset);

        //  If on a non-manifold crease, make use of implicit sharpness:
        if (!subset->IsSharp() && HasImplicitSharpness()) {
            SharpenSubset(subset, GetImplicitSharpness());
        }
    }
    return subset->_numFacesTotal;
}

int
CornerTopology::findConnectedSubsetExtent(CornerSubset * subset) const {

    assert(AreUnOrderedFacesConnected());

    subset->Initialize(GetTag());
    subset->_tag._nonManifoldVerts = false;

    subset->_numFacesBefore = 0;
    subset->_numFacesAfter  = 0;
    subset->_numFacesTotal  = 1;

    int fStart = GetFaceInVertex();

    for (int f = GetFaceNext(fStart); f >= 0; f = GetFaceNext(f)) {
        if (f == fStart) {
            subset->SetBoundary(false);
            return subset->_numFacesTotal;
        }
        subset->_numFacesAfter ++;
        subset->_numFacesTotal ++;
    }
    for (int f = GetFacePrevious(fStart); f >= 0; f = GetFacePrevious(f)) {
        subset->_numFacesBefore ++;
        subset->_numFacesTotal ++;
    }
    subset->SetBoundary(true);
    return subset->_numFacesTotal;
}

int
CornerTopology::findFVarSubsetExtent(CornerSubset const & vtxSub,
                                     CornerSubset       * fvarSubsetPtr,
                                     Index const          fvarIndices[]) const {

    CornerSubset & fvarSub = *fvarSubsetPtr;

    //
    //  Initialize the face-varying subset by seeking forward and backward
    //  from the corner face to find edges that are not continuous wrt
    //  face-varying indices, and so delimit the relevant neighborhood
    //  contributing to the face-varying limit surface.  This is done in
    //  three steps:
    //
    //      - seeking counter-clockwise from the corner face
    //      - testing if a periodic subset is continuous at its end
    //      - seeking clockwise from the corner face
    //

    //
    //  Initialize as single face boundary -- return if only one face:
    //
    fvarSub.Initialize(vtxSub._tag);

    fvarSub.SetBoundary(true);

    if (vtxSub._numFacesTotal == 1) {
        return 1;
    }

    //
    //  Inspect/gather faces "after" (counter-clockwise order from)
    //  the corner face.  If all are included and the vtx subset is
    //  periodic, check the seam for the fvar subset.
    //
    int cornerFace = GetFaceInVertex();

    int numFacesAfterToVisit = vtxSub._numFacesAfter;
    if (numFacesAfterToVisit) {
        int thisFace = cornerFace;
        for (int i = 0; i < numFacesAfterToVisit; ++i) {
            int nextFace = GetFaceNext(thisFace);

            if (GetFaceVertexAtCorner(thisFace, fvarIndices) != 
                GetFaceVertexAtCorner(nextFace, fvarIndices)) {
                break;
            }
            if (GetFaceVertexTrailing(thisFace, fvarIndices) != 
                GetFaceVertexLeading(nextFace, fvarIndices)) {
                break;
            }
            ++ fvarSub._numFacesAfter;

            thisFace = nextFace;
        }
    }
    int numFacesAfterUnvisited = vtxSub._numFacesAfter -
                                 fvarSub._numFacesAfter;
    if (!vtxSub.IsBoundary() && (numFacesAfterUnvisited == 0)) {
        assert(vtxSub._numFacesBefore == 0);
        int prevFace = GetFacePrevious(cornerFace);

        if (GetFaceVertexLeading(cornerFace, fvarIndices) == 
            GetFaceVertexTrailing(prevFace, fvarIndices)) {
            fvarSub.SetBoundary(false);
        }
    }

    //
    //  Inspect/gather faces "before" (clockwise order from) the corner
    //  face.  Include any faces "after" in the periodic case that were
    //  interrupted by a discontinuity:
    //
    int numFacesBeforeToVisit = vtxSub._numFacesBefore;
    if (!vtxSub.IsBoundary()) {
        numFacesBeforeToVisit += numFacesAfterUnvisited;
    }
    if (numFacesBeforeToVisit) {
        int thisFace = cornerFace;
        for (int i = 0; i < numFacesBeforeToVisit; ++i) {
            int prevFace = GetFacePrevious(thisFace);

            if (GetFaceVertexAtCorner(thisFace, fvarIndices) != 
                GetFaceVertexAtCorner(prevFace, fvarIndices)) {
                break;
            }
            if (GetFaceVertexLeading(thisFace, fvarIndices) != 
                GetFaceVertexTrailing(prevFace, fvarIndices)) {
                break;
            }
            ++ fvarSub._numFacesBefore;

            thisFace = prevFace;
        }
    }

    fvarSub._numFacesTotal = 1 + fvarSub._numFacesBefore +
                                 fvarSub._numFacesAfter;

    return fvarSub._numFacesTotal;
}

int
CornerTopology::FindFaceVaryingSubset(CornerSubset       * fvarSubsetPtr,
                                      Index const          fvarIndices[],
                                      CornerSubset const & vtxSub) const {

    CornerSubset & fvarSub = *fvarSubsetPtr;

    findFVarSubsetExtent(vtxSub, fvarSubsetPtr, fvarIndices);

    //  Reset the sharpness if face-varying topology differs, as the rules
    //  for the FVar interpolation options (applied later) take precedence
    //  over all but those applied below:
    bool fvarTopologyMatchesVertex = fvarSub.MatchesExtentOfSuperset(vtxSub);

    if (fvarSub.IsSharp() && !fvarTopologyMatchesVertex) {
        UnSharpenSubset(fvarSubsetPtr);
    }

    //  Sharpen if the vertex is non-manifold:
    if (!fvarSub.IsSharp() && !_tag.IsManifold()) {
        SharpenSubset(fvarSubsetPtr);
    }

    //  Sharpen if the face-varying value is non-manifold, i.e. if there
    //  are any occurrences of the corner FVar index outside the subset:
    if (!fvarSub.IsSharp() && (fvarSub.GetNumFaces() < vtxSub.GetNumFaces())) {
        Index fvarMatch = GetFaceVertexAtCorner(fvarIndices);

        int numMatches = 0;
        for (int i = 0; i < GetNumFaces(); ++i) {
            numMatches += (GetFaceVertexAtCorner(i, fvarIndices) == fvarMatch);
            if (numMatches > fvarSub.GetNumFaces()) {
                SharpenSubset(fvarSubsetPtr);
                break;
            }
        }
    }

    //  Finally, adjust other topology tags if fvar topology differs:
    if (!fvarTopologyMatchesVertex) {
        adjustSubsetTags(&fvarSub, &vtxSub);
    }
    return fvarSubsetPtr->GetNumFaces();
}

//
//  Method to revise the tags for a subset of the corner, which may no
//  longer include properties that trigger exceptional behavior:
//
void
CornerTopology::SharpenSubset(CornerSubset * subset) const {

    //  Mark the subset sharp and ensure any related tags are also
    //  updated accordingly:

    subset->_tag._infSharpVerts  = true;
    subset->_tag._semiSharpVerts = false;
}
void
CornerTopology::UnSharpenSubset(CornerSubset * subset) const {

    //  Restore subset sharpness based on actual sharpness assignment:

    subset->_tag._infSharpVerts  = _isExpInfSharp;
    subset->_tag._semiSharpVerts = _isExpSemiSharp;
}
void
CornerTopology::SharpenSubset(CornerSubset * subset, float sharpness) const {

    //  Mark the subset according to sharpness value
    if (sharpness > subset->_localSharpness) {
        subset->_localSharpness = sharpness;

        subset->_tag._infSharpVerts  = Sdc::Crease::IsInfinite(sharpness);
        subset->_tag._semiSharpVerts = Sdc::Crease::IsSemiSharp(sharpness);
    }
}

bool
CornerTopology::subsetHasIrregularFaces(CornerSubset const & subset) const {

    assert(_tag.HasIrregularFaceSizes());

    if (!_tag._unCommonFaceSizes) return true;

    int f = GetFaceBefore(subset._numFacesBefore);
    for (int i = 0; i < subset.GetNumFaces(); ++i, f = GetFaceNext(f)) {
        if (GetFaceSize(f) != _regFaceSize) return true;
    }
    return false;
}

bool
CornerTopology::subsetHasInfSharpEdges(CornerSubset const & subset) const {

    assert(_tag.HasInfSharpEdges());

    int n = subset.GetNumFaces();
    if (n > 1) {
        int f = GetFaceBefore(subset._numFacesBefore);
        //  Skip first face of a boundary when inspecting leading edges:
        for (int i = subset.IsBoundary(); i < n; ++i, f = GetFaceNext(f)) {
            if (IsFaceEdgeInfSharp(f, 1)) return true;
        }
    }
    return false;
}

bool
CornerTopology::subsetHasSemiSharpEdges(CornerSubset const & subset) const {

    assert(_tag.HasSemiSharpEdges());

    int n = subset.GetNumFaces();
    if (n > 1) {
        int f = GetFaceBefore(subset._numFacesBefore);
        //  Skip first face of a boundary when inspecting leading edges:
        for (int i = subset.IsBoundary(); i < n; ++i, f = GetFaceNext(f)) {
            if (IsFaceEdgeSemiSharp(f, 1)) return true;
        }
    }
    return false;
}

void
CornerTopology::adjustSubsetTags(CornerSubset       * subset,
                                 CornerSubset const * superset) const {

    CornerTag & subsetTag = subset->_tag;

    //  Adjust any tags related to boundary or sharpness status (no other
    //  action for boundary at present):
    if (subsetTag.IsBoundary()) {
    }

    if (subsetTag.IsInfSharp()) {
        subsetTag._semiSharpVerts = false;
    }

    //  Adjust for the presence of irregular faces or sharp edges if the
    //  subset is actually a proper subset of the corner or the optionally
    //  provided superset:
    int  numSuperFaces = superset ? superset->GetNumFaces() : GetNumFaces();
    bool superBoundary = superset ? superset->IsBoundary()  : _tag.IsBoundary();

    if ((subset->GetNumFaces() < numSuperFaces) ||
        (subset->IsBoundary() != superBoundary)) {

        if (subsetTag._irregularFaceSizes) {
            subsetTag._irregularFaceSizes = subsetHasIrregularFaces(*subset);
        }
        if (subsetTag._infSharpEdges) {
            subsetTag._infSharpEdges = subsetHasInfSharpEdges(*subset);
            if (!subsetTag._infSharpEdges) {
                subsetTag._infSharpDarts = false;
            } else if (subset->IsBoundary()) {
                subsetTag._infSharpDarts = false;
                SharpenSubset(subset);
            }
        }
        if (subsetTag._semiSharpEdges) {
            subsetTag._semiSharpEdges = subsetHasSemiSharpEdges(*subset);
        }
    }
}

//
//  Main and supporting internal methods to connect unordered faces and
//  so allow for topological traversals of the incident faces:
//
//  While still rough, the process can be broken down as follows -- first
//  the following data is needed:
//
//    - local buffer of face-edge vertices, size = 2*N
//    - local buffer of face-edge edges, size = 2*N
//    - local array of Edges, dynamic (max 2*N)
//    - member buffer neighbors, size = 2*N
//
//  and is processed as follows:
//
//    - initialize/connect the set of dynamic Edges:
//        - assignUnOrderedEdges()
//        - requires:
//            - input:   CornerTopology, face-vertex indices
//            - output:  array of Edges, array of face-edge edges
//        - gather/assign the local face-edge vertex buffer
//        - initialize the output face-edge edge buffer (-1)
//        - for each face-edge vertex, intialize new Edge
//            - look for remaining face-edges with matching vertices
//            - change state of Edge based on occurrence
//        - post-process the set of dynamic Edges:
//            - test for duplicate corner vertex in face and adjust
//
//    - assign neighboring/connected faces from Edges
//        - assignUnOrderedFaceNeighbors()
//        - requires:
//            - input:   CornerTopology, array of Edges,
//                       array of face-edge edges
//            - output:  array of neighboring/connected faces
//        - inspects Edge for each face-edge edge
//        - assigns neighboring face for connected/interior edges
//
//    - determine overall vertex properties (non-manifold, sharp):
//        - assignUnOrderedProperties()
//        - requires:
//            - input:   CornerTopology, array of Edges
//            - output:  CornerTopology or give CornerTag?
//        - take inventory of the given Edges:
//            - count non-manifold, boundary, inf-sharp edges
//        - perform any remaining analysis
//        - assign vertex manifold and sharpness status
//
struct CornerTopology::Edge {
    //  Empty constructor intentional since we over-allocate what we need:
    Edge() { }

    unsigned short boundary    : 1;
    unsigned short interior    : 1;
    unsigned short nonManifold : 1;
    unsigned short degenerate  : 1;
    unsigned short duplicate   : 1;
    unsigned short infSharp    : 1;
    unsigned short semiSharp   : 1;

    short prevFace, nextFace;
    short vertex; // temporary, for debugging only

    void Clear() { std::memset(this, 0, sizeof(*this)); }

    //  Transition of state as incident faces are added:
    void SetBoundary()    { boundary = 1; }
    void SetInterior()    { boundary = 0, interior = 1; }
    void SetNonManifold() { boundary = 0, interior = 0, nonManifold = 1; }
    void SetDegenerate()  { SetNonManifold(), degenerate = 1; }
    void SetDuplicate()   { SetNonManifold(), duplicate = 1; }

    void SetSharpness(float sharpness) {
        if (sharpness > 0.0f) {
            if (Sdc::Crease::IsInfinite(sharpness)) {
                infSharp = true;
            } else {
                semiSharp = true;
            }
        }
    }

    void SetFaces(int prev, int next) { prevFace = prev, nextFace = next; }
};

void
CornerTopology::ConnectUnOrderedFaces(Index const fvIndices[]) {

bool debug = false;
if (debug) {
    Index vCorner = fvIndices[0];
    printf("        corner vertex:   %d\n", vCorner);

    printf("        face vertices:\n");
    Index const * fv = fvIndices;
    for (int i = 0; i < this->GetNumFaces(); ++i) {
        int fSize = this->GetFaceSize(i);
        printf("            face %2d (%d):  ", i, fSize);
        for (int j = 0; j < fSize; ++j) {
            printf(" %3d", fv[j]);
        }
        printf("\n");
        fv += this->GetFaceSize(i);
    }
    printf("        face-edge vertices:\n");
    for (int i = 0; i < GetNumFaces(); ++i) {
        printf("            face %2d:       %3d %3d\n", i,
            GetFaceVertexTrailing(i, fvIndices),
            GetFaceVertexLeading( i, fvIndices));
    }
}

    //
    //  There are two transient sets of data we need here:  a set of Edges
    //  that connect adjoining faces, and a set of indices (one for each
    //  of the 2*N face-edges) to identify the Edge for each face-edge.
    //
    //  IMPORTANT -- since these later edge indices are of the same type
    //  and size as the internal face-edge neighbors, we'll use that array
    //  to avoid a separate declaration (and possible allocation) and will
    //  update it in place later.
    //
    int numFaceEdges = GetNumFaces() * 2;

    Vtr::internal::StackBuffer<Edge,32,true> edges(numFaceEdges);

    _faceEdgeNeighbors.SetSize(numFaceEdges);

    short * feEdges = &_faceEdgeNeighbors[0];
    std::fill(feEdges, feEdges + numFaceEdges, -1);

    //  Edge initialization fails to detect some "duplicate" edges in a
    //  face, so post-process to catch these before continuing:
    int numEdges = gatherUnOrderedEdges(edges, feEdges, fvIndices);

    markDuplicateEdges(edges, feEdges, fvIndices);

    //  Use the connecting edges to assign neighboring faces (overwriting
    //  our edge indices) and update the properties of the corner:
    assignUnOrderedFaceNeighbors(edges, feEdges);

    assignUnOrderedTags(edges, numEdges);

if (debug) {
    Index vCorner = fvIndices[0];
    printf("        connecting edges:\n");
    for (int i = 0; i < numEdges; ++i) {
        Edge const & E = edges[i];
        printf("            edge %d (%3d to %3d):  ", i, vCorner, E.vertex);
        printf("non-manifold = %d, boundary = %d", E.nonManifold, E.boundary);
        bool edgeIsSingular = E.nonManifold || E.boundary;
        if (!edgeIsSingular) {
            printf(", prevFace = %d, nextFace = %d", E.prevFace, E.nextFace);
        }
        printf("\n");
    }

    printf("        neighboring faces:\n");
    short * feFaces = &_faceEdgeNeighbors[0];
    for (int i = 0; i < (numFaceEdges / 2); ++i) {
        printf("            face %2d:       prev =%3d, next =%3d\n", i,
            feFaces[i*2], feFaces[i*2+1]);
    }

    bool isNonManifold = _tag._nonManifoldVerts;
    printf("            manifold = %s\n", isNonManifold ? "false" : "TRUE");
    printf("            boundary = %d\n", _tag._boundaryVerts);
    printf("            sharp    = %d\n", _tag._infSharpVerts);
    //  WIP - test/assert here only when expecting RefinerSurfaceFactory's
    //        non-manifold verts to match:
    if (!isNonManifold) {
        printf("WARNING:  unordered vertex is MANIFOLD!\n");
    }
    assert(isNonManifold);
}
}

//
//  Identify a set of shared edges between unordered faces so that we can
//  establish connections between them.
//
//  The "face-edge edges" are really just half-edges that refer (by index)
//  to potentially shared Edges.  As Edges are created, these half-edges
//  are made to refer to them, after which the state of the edge may change
//  due to the presence or orientation of additional matching half-edges.
//
int
CornerTopology::gatherUnOrderedEdges(Edge        edges[],
                                     short       feEdges[],
                                     Index const fvIndices[]) const {

    //
    //  Gather the face-edge vertex indices into local buffer:
    //
    int feCount = 2 * this->GetNumFaces();

    std::vector<Index> feVerts(feCount, -1);

    for (int i = 0; i < feCount; ++i) {
        feVerts[i] = (i & 1) ?
            this->GetFaceVertexTrailing((i >> 1), fvIndices) :
            this->GetFaceVertexLeading( (i >> 1), fvIndices);
    }

    //
    //  Iterate through the face-edge vertices to find connecting edges:
    //
    Index vCorner = this->GetFaceVertexAtCorner(0, fvIndices);

    int nEdges = 0;

    for (int eOuter = 0; eOuter < feCount; ++eOuter) {
        if (feEdges[eOuter] >= 0) continue;

        //
        //  Identify the next end-vertex and its newly created edge:
        //
        Index vIndex = feVerts[eOuter];
        int  eIndex  = nEdges++;

        feEdges[eOuter] = eIndex;

        //
        //  Create/initialize new edge before searching for matching edges:
        //  if degenerate, skip any futher inspection, otherwise initialize
        //  as a boundary edge and classify its sharpness:
        //
        Edge & E = edges[eIndex];
        E.Clear();
        E.vertex = vIndex;

        if (vIndex == vCorner) {
            E.SetDegenerate();
            continue;
        }

        E.SetBoundary();
        if (_tag.HasSharpEdges()) {
            E.SetSharpness(GetFaceEdgeSharpness(eOuter));
        }

        //
        //  Search remaining face-edges for the same end-vertex:
        //
        int  eOuterFace       = (eOuter >> 1);
        bool eOuterIsTrailing = (eOuter & 1);

        for (int eInner = eOuter + 1; eInner < feCount; ++eInner) {
            if (feEdges[eInner] >= 0) continue;
            if (feVerts[eInner] != vIndex) continue;

            feEdges[eInner] = eIndex;

            //  If already non-manifold, nothing more to do, otherwise
            //  update the state of the edge based on this end-vertex:
            if (!E.nonManifold) {
                int  eInnerFace       = (eInner >> 1);
                bool eInnerIsTrailing = (eInner & 1);

                bool isReversed = (eInnerIsTrailing == eOuterIsTrailing);
                bool isRepeated = (eInnerFace == eOuterFace);
                if (isReversed || isRepeated || !E.boundary) {
                    //  Edge is reversed, repeated in the face or has more
                    //  than two incident faces -- make non-manifold:
                    E.SetNonManifold();
                } else if (E.boundary) {
                    //  Edge is a manifold boundary -- propote to interior
                    //  and assign the two connected faces:
                    E.SetInterior();
                    E.SetFaces(eInnerIsTrailing ? eInnerFace : eOuterFace,
                               eInnerIsTrailing ? eOuterFace : eInnerFace);
                }
            }
        }
    }
    return nEdges;
}

void
CornerTopology::markDuplicateEdges(Edge        edges[],
                                   short const feEdges[],
                                   Index const fvIndices[]) const {

    //
    //  The edge assignment thus far does not correctly detect the presence
    //  of all edges repeated or duplicated in the same face, e.g. for quad
    //  with vertices {A, B, A, C} the edge AB occurs both as AB and BA.
    //  When the face is oriented relative to corner B, we have {B, A, C, A}
    //  and edge BA will be detected as non-manifold -- but not from corner
    //  A or C.
    //
    //  So look for repeated instances of the corner vertex in the face and
    //  inspect its neighbors to see if they match the leading or trailing
    //  edges.
    //
    //  This is a trivial test for a quad:  if the opposite vertex matches
    //  the corner vertex, both the leading and trailing edges will be
    //  duplicated and so can immediately be marked non-manifold.  So deal
    //  with the common case of all neighboring quads separately.
    //
    if (_commonFaceSize == 3) return;

    Index vCorner = fvIndices[0];
    int numFaces = GetNumFaces();

    if (_commonFaceSize == 4) {
        Index const * fvOpposite = fvIndices + 2;
        for (int face = 0; face < numFaces; ++face, fvOpposite += 4) {
            if (*fvOpposite == vCorner) {
                edges[feEdges[2*face  ]].SetDuplicate();
                edges[feEdges[2*face+1]].SetDuplicate();
            }
        }
    } else {
        Index const * fv = fvIndices;

        for (int face = 0; face < numFaces; ++face) {
            int faceSize = GetFaceSize(face);

            for (int j = 2; j < (faceSize - 2); ++j) {
                if (fv[j] == vCorner) {
                    if (fv[j-1] == fv[1])
                        edges[feEdges[2*face]].SetDuplicate();
                    if (fv[j+1] == fv[faceSize-1])
                        edges[feEdges[2*face+1]].SetDuplicate();
                }
            }
            fv += faceSize;
        }
    }
}

void
CornerTopology::assignUnOrderedFaceNeighbors(Edge const  edges[],
                                             short const feEdges[]) {

    int numFaceEdges = 2 * GetNumFaces();
    assert((int)_faceEdgeNeighbors.GetSize() == numFaceEdges);

    short * feFaces = &_faceEdgeNeighbors[0];

    for (int i = 0; i < numFaceEdges; ++i) {
        assert(feEdges[i] >= 0);

        Edge const & E = edges[feEdges[i]];
        bool edgeIsSingular = E.nonManifold || E.boundary;
        if (edgeIsSingular) {
            feFaces[i] = -1;
        } else {
            bool faceEdgeIsTrailing = (i & 1);
            feFaces[i] = faceEdgeIsTrailing ? E.nextFace : E.prevFace;
        }
    }
}

void
CornerTopology::assignUnOrderedTags(Edge const edges[], int numEdges) {

    //
    //  Summarize properties of the corner given the number and nature of
    //  the edges around its vertex and initialize remaining members or
    //  tags that depend on them.
    //
    //  First, take inventory of relevant properties from the edges:
    //
    _numInfSharpEdges  = 0;
    _numSemiSharpEdges = 0;

    int numNonManifoldEdges = 0;
    int numSingularEdges    = 0;

    bool hasBoundaryEdges         = false;
    bool hasBoundaryEdgesNotSharp = false;
    bool hasDegenerateEdges       = false;
    bool hasDuplicateEdges        = false;

    for (int i = 0; i < numEdges; ++i) {
        Edge const & E = edges[i];

        if (E.interior) {
            _numInfSharpEdges  += E.infSharp;
            _numSemiSharpEdges += E.semiSharp;
        } else if (E.boundary) {
            hasBoundaryEdges = true;
            hasBoundaryEdgesNotSharp |= !E.infSharp;
        } else {
            ++ numNonManifoldEdges;
            hasDegenerateEdges |= E.degenerate;
            hasDuplicateEdges  |= E.duplicate;
        }

        //  Singular edges include all that are effectively inf-sharp:
        numSingularEdges += E.nonManifold || E.boundary || E.infSharp;
    }

    //
    //  Next determine whether manifold or not.  Some obvious tests quickly
    //  indicate if the corner is non-manifold, but ultimately it will be
    //  necessary to traverse the faces to confirm that they form a single
    //  connected set (e.g. two cones sharing their apex vertex may appear
    //  manifold to this point but as two connected sets are non-manifold).
    //
    bool isNonManifold       = false;
    bool isNonManifoldCrease = false;

    if (numNonManifoldEdges) {
        isNonManifold = true;

        if (!hasDegenerateEdges && !hasDuplicateEdges && !hasBoundaryEdges) {
            //  Special crease case that avoids sharpening: two interior
            //  non-manifold edges radiating more than two sets of faces:
            isNonManifoldCrease = (numNonManifoldEdges == 2) &&
                                  (GetNumFaces() > numEdges);
        }
    } else {
        //  Mismatch between number of incident faces and edges:
        isNonManifold = ((numEdges - GetNumFaces()) != hasBoundaryEdges);

        if (!isNonManifold) {
            //  If all faces are not connected, the set is non-manifold:
            CornerSubset subset;
            int numFacesInSubset = findConnectedSubsetExtent(&subset);
            if (numFacesInSubset < GetNumFaces()) {
                isNonManifold = true;
            }
        }
    }

    //
    //  Assign tags and other members related to the inventory of edges
    //  (boundary status is relevant if non-manifold as it can affect
    //  the presence of the limit surface):
    //
    _tag._nonManifoldVerts = isNonManifold;

    _tag._boundaryVerts    = hasBoundaryEdges;
    _tag._boundaryNonSharp = hasBoundaryEdgesNotSharp;

    _tag._infSharpEdges  = (_numInfSharpEdges > 0);
    _tag._semiSharpEdges = (_numSemiSharpEdges > 0);
    _tag._infSharpDarts  = (_numInfSharpEdges == 1) && !hasBoundaryEdges;

    //  Conditions effectively making the vertex sharp, include the usual
    //  excess of inf-sharp edges plus some non-manifold cases:
    if ((numSingularEdges > 2) || (isNonManifold && !isNonManifoldCrease)) {
        _isImpInfSharp = true;
    } else if ((numSingularEdges + _numSemiSharpEdges) > 2) {
        _isImpSemiSharp = true;
    }

    //  Mark the vertex inf-sharp if implicitly inf-sharp:
    if (!_isExpInfSharp && _isImpInfSharp) {
        _tag._infSharpVerts = true;
        _tag._semiSharpVerts = false;
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
