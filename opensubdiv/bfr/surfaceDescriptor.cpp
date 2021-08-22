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

#include <cstring>
#include <cstdio>

#include "../bfr/surfaceDescriptor.h"


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

namespace {
    //
    //  Utility function to adjust a set of CornerTags for a CornerSubset
    //  based on the presence/absence of features within that subset.
    //
    //  WIP - consider moving this to CornerTopology or FaceTopology
    //
    void
    reviseSubsetTagsFromTopology(CornerSubset & cSub,
                                 CornerTopology const & cTop,
                                 int regFaceSize) {

        //
        //  WIP - beware this assignment when transitioning to pure tags
        //        (i.e. replacing the _isBoundary and _isSharp members)
        //      - eventually the boundary & sharp status needs to be in tags
        //      - so will not want to copy tags here (bitwise OR?)
        //      - may be expecting a previous copy/init/OR of subset tags
        //
        bool isBoundary = cSub._isBoundary;
        bool isSharp    = cSub._isSharp;

        cSub._tags = cTop.GetTags();

        if (isBoundary) {
            cSub._tags._boundaryVerts     = true;
            cSub._tags._boundaryCorners   = (cSub._numFacesTotal == 1);
            cSub._tags._interiorVal2Verts = false;
        } else {
            cSub._tags._boundaryVerts     = false;
            cSub._tags._boundaryCorners   = false;
            cSub._tags._interiorVal2Verts = (cSub._numFacesTotal == 2);
        }
        cSub._tags._nonManifoldVerts = false;

        if (isSharp) {
            cSub._tags._infSharpVerts  = true;
            cSub._tags._semiSharpVerts = false;
        }

        if (cSub._tags._irregularFaceSizes) {
            if (cSub._tags._unCommonFaceSizes) {
                cSub._tags._irregularFaceSizes = false;

                //  Search for faces with irregular size:
                int face = cTop.GetFaceBefore(cSub._numFacesBefore);
                for (int i = 0; i < cSub._numFacesTotal; ++i) {
                    if (cTop.GetFaceSize(face) != regFaceSize) {
                        cSub._tags._irregularFaceSizes = true;
                        break;
                    }
                    face = cTop.GetFaceNext(face);
                }
            } else {
                cSub._tags._irregularFaceSizes = true;
            }
        }

        if (cSub._tags._anySharpEdges) {
            cSub._tags._anySharpEdges = false;

            if (cSub._numFacesTotal > 1) {
                //  Search for faces whose leading edges were sharpened,
                //  skipping the first face of a boundary subset (whose
                //  leading edge is a boundary):
                int face = cTop.GetFaceBefore(cSub._numFacesBefore);
                if (isBoundary) face = cTop.GetFaceNext(face);
                for (int i = isBoundary; i < cSub._numFacesTotal; ++i) {
                    if (cTop.GetFaceEdgeSharpness(face,0) > 0.0f) {
                        cSub._tags._anySharpEdges = true;
                        break;
                    }
                    face = cTop.GetFaceNext(face);
                }
            }
        }
    }
}

//
//  Main initialization methods -- one for vertex topology and the other
//  for face-varying topology (a subset of the vertex topology):
//
void
SurfaceDescriptor::Initialize(Index const vtxIndices[]) {

    assert(_topology._isFinalized);

    //  WIP - we eventually need the vertex indices to identify the
    //        adjacency/connectivity for unordered or non-manifold verts
    //      - but that may be dealt with as part of FaceTopology and its
    //        associated indices, so action here may not be necessary
    assert(!_topology.GetTags()._unOrderedFaces);

    //  Consider making this a member flag with other scheme/option
    //  related members:
    bool sharpenCorners =
            (_topology._schemeOptions.GetVtxBoundaryInterpolation() ==
                Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);

    _corners.SetSize(_topology._faceSize);

    _combinedTags.Clear();

    for (int i = 0; i < _topology._faceSize; ++i) {
        CornerTopology const & V = _topology.GetTopology(i);
        CornerSubset         & C = _corners[i];

        //
        //  If the vertex topology was unordered (potentially non-manifold)
        //  this will have to be determined later from the face-vertex
        //  indices -- in which case, other FaceTopology members will need
        //  updating.
        //
        bool vtxMatchesTopology = true;

        if (V.IsOrdered()) {
            C._isBoundary = V.IsBoundary();

            C._numFacesTotal = V.GetNumFaces();
            if (C._isBoundary) {
                C._numFacesBefore = V.GetFaceInVertex();
                C._numFacesAfter  = V.GetNumFaces() - 1 - V.GetFaceInVertex();
            } else {
                C._numFacesBefore = 0;
                C._numFacesAfter  = V.GetNumFaces() - 1;
            }

            C._isSharp = V.IsVertexInfSharp();
            if ((C._numFacesTotal == 1) && !C._isSharp && sharpenCorners) {
                C._isSharp = true;
                //  Consider sharpening the CornerTopology to avoid this
                vtxMatchesTopology = false;
            }
        } else {
            //  Use a sharp corner for linear face for non-manifold for now:
            C._isBoundary     = true;
            C._numFacesTotal  = 1;
            C._numFacesBefore = 0;
            C._numFacesAfter  = 0;
            C._isSharp        = false;

            vtxMatchesTopology = false;
        }

        C._numOuterFaces = -1;
        C._numOuterVerts = -1;

        if (vtxMatchesTopology) {
            C._tags = V.GetTags();
        } else {
            reviseSubsetTagsFromTopology(C, V, _topology._regFaceSize);
        }
        _combinedTags.BitwiseOr(C._tags);
    }
    initializeSubsetInventory();

    //
    //  WIP - valence-2 interior vertices created the awkward situation
    //        where neighboring vertices in the ring of each corner fold
    //        over and overlap the base face -- not just at the valence-2
    //        corner itself but its neighbors
    //      - it may help to tag the corners here so that we can detect
    //        and deal with this situation appropriately
    //      - but this could also be deferred to the Builder class to 
    //        deal with
    //
    assert(!_topology.GetTags()._interiorVal2Verts);

    //  Assign all member variables before returning:
    _indices = vtxIndices;

    _isFaceVarying = false;
    _matchesVertex = true;
    _isInitialized = true;
}

void
SurfaceDescriptor::InitializeFaceVarying(
        SurfaceDescriptor const & vtxSurface,
        Index             const   fvarIndices[]) {

    assert(&_topology == &vtxSurface._topology);
    assert(_topology._isFinalized);

    CornerSubset const * vtxCorners = &vtxSurface._corners[0];

    bool fvarSubsetsAllMatch = true;

    int faceSize =  _topology._faceSize;

    _corners.SetSize(faceSize);

    _combinedTags.Clear();

    Index const * fvarCornerIndices = fvarIndices;

    for (int corner = 0; corner < faceSize; ++corner) {
        CornerTopology const & vtxTop    = _topology.GetTopology(corner);
        CornerSubset const   & vtxCorner = vtxCorners[corner];

        //
        //  Initialize the extent of the face-varying subset then determine
        //  its sharpness -- initially inherited from the vertex subset
        //  (which takes precedence) but which may be applied to boundary
        //  subsets in other circumstances (typically the fvar interpolation
        //  option)
        //
        CornerSubset & fvarCorner = _corners[corner];

        initializeFVarSubset(vtxSurface, corner, fvarCornerIndices);

        if (fvarCorner._isBoundary && !fvarCorner._isSharp) {
            sharpenFVarSubset(vtxSurface, corner, fvarCornerIndices);
        }

        bool fvarSubsetMatches =
                (fvarCorner._isBoundary     == vtxCorner._isBoundary) &&
                (fvarCorner._isSharp        == vtxCorner._isSharp) &&
                (fvarCorner._numFacesBefore == vtxCorner._numFacesBefore) &&
                (fvarCorner._numFacesAfter  == vtxCorner._numFacesAfter);

        fvarSubsetsAllMatch &= fvarSubsetMatches;

        if (fvarSubsetMatches) {
            fvarCorner._tags = vtxCorner._tags;
        } else {
            reviseSubsetTagsFromTopology(fvarCorner, vtxTop,
                                         _topology._regFaceSize);
        }
        _combinedTags.BitwiseOr(fvarCorner._tags);

        fvarCornerIndices += vtxTop.GetNumFaceVertices();
    }

    if (fvarSubsetsAllMatch) {
        //  Copy the vertex subsets to initialize any other members:
        std::memcpy(_corners, vtxCorners, faceSize * sizeof(CornerSubset));
    } else {
        initializeSubsetInventory();
    }

    //  Assign all member variables before returning:
    _indices = fvarIndices;

    _isFaceVarying = true;
    _matchesVertex = fvarSubsetsAllMatch;
    _isInitialized = true;
}

//
//  Topological queries:
//
bool
SurfaceDescriptor::IsRegular() const {

    //
    //  WIP - beware, these tags are copied from FaceTopology rather than
    //  being composed from the CornerSubsets -- so regular subets may be
    //  excluded here (optimize later)
    //
    if (_combinedTags._irregularFaceSizes ||
        _combinedTags._anySharpEdges      || _combinedTags._semiSharpVerts ||
        _combinedTags._unOrderedFaces     || _combinedTags._interiorVal2Verts) {
        return false;
    }

    int regInteriorValence = (_topology._regFaceSize == 4) ? 4 : 6;
    int regBoundaryValence = (regInteriorValence / 2);

    for (int i = 0; i < _topology._faceSize; ++i) {
        CornerSubset const & corner = _corners[i];

        if (corner._isSharp) {
            if (corner._numFacesTotal != 1) return false;
        } else if (corner._isBoundary) {
            if (corner._numFacesTotal != regBoundaryValence) return false;
        } else {
            if (corner._numFacesTotal != regInteriorValence) return false;
        }
    }
    return true;
}


//
//  Internal methods supporting initialization:
//
void
SurfaceDescriptor::initializeSubsetInventory() {

    //
    //  This is not done locally (in isolation) for each corner as there
    //  are some pathological cases where the inventory of one corner
    //  depends on one or more others:
    //
    int nVal3IntAdjTris = 0;

    for (int corner = 0; corner < _topology._faceSize; ++corner) {
        CornerTopology const & V = _topology.GetTopology(corner);
        CornerSubset         & C = _corners[corner];

        //
        //  RECONSIDER -- the counting and gathering of the "after" faces
        //  may be better handled as follows:
        //      - include "trailing" edge/vert of first "after" face
        //      - include interior and "trailing" edge/vert (i.e. S - 2)
        //        of all other "after" faces
        //  * the gathering (of both control verts and control face-verts)
        //  may benefit from this more than the simple counting here...
        //

        int nVerts = 0;
        if (V.GetCommonFaceSize()) {
            int S = V.GetCommonFaceSize();

            if (!C._isBoundary) {
                if ((C._numFacesTotal == 3) && (S == 3)) {
                    nVerts += (++nVal3IntAdjTris == _topology._faceSize);
                } else {
                    nVerts += (C._numFacesTotal - 2) * (S - 2) - 1;
                }
            } else {
                if (C._numFacesAfter) {
                    nVerts += (C._numFacesAfter - 1) * (S - 2) + 1;
                }
                if (C._numFacesBefore) {
                    nVerts += C._numFacesBefore * (S - 2) - 1;
                }
            }
        } else {
            int cornerFace = V.GetFaceInVertex();

            if (!C._isBoundary) {
                assert(C._numFacesTotal == V.GetNumFaces());

                int nextFace = V.GetFaceAfter(2);
                if ((C._numFacesTotal == 3) && (V.GetFaceSize(nextFace) == 3)) {
                    nVerts += (++nVal3IntAdjTris == _topology._faceSize);
                } else {
                    for (int i = 2; i < C._numFacesTotal; ++i) {
                        int S = V.GetFaceSize(nextFace);
                        nVerts += S - 2;
                        nextFace = V.GetFaceNext(nextFace);
                    }
                    nVerts --;
                }
            } else {
                if (C._numFacesAfter) {
                    int nextFace = V.GetFaceNext(cornerFace);
                    for (int i = 1; i < C._numFacesAfter; ++i) {
                        nextFace = V.GetFaceNext(nextFace);
                        int S = V.GetFaceSize(nextFace);
                        nVerts += S - 2;
                    }
                    nVerts ++;
                }
                if (C._numFacesBefore) {
                    int nextFace = V.GetFaceBefore(C._numFacesBefore);
                    for (int i = 0; i < C._numFacesBefore; ++i) {
                        int S = V.GetFaceSize(nextFace);
                        nVerts += S - 2;
                        nextFace = V.GetFaceNext(nextFace);
                    }
                    nVerts --;
                }
            }
        }

        int nFaces = 0;
        if (!C._isBoundary) {
            nFaces += C._numFacesTotal - 2;
        } else {
            nFaces += C._numFacesAfter ? (C._numFacesAfter - 1) : 0;
            nFaces += C._numFacesBefore;
        }

        C._numOuterVerts = (short) nVerts;
        C._numOuterFaces = (short) nFaces;
    }
}

void
SurfaceDescriptor::initializeFVarSubset(SurfaceDescriptor const & vtxSurface,
        int corner, Index const fvarIndices[]) {

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
    CornerSubset const & vtxCorner  = vtxSurface._corners[corner];
    CornerSubset       & fvarCorner = _corners[corner];

    CornerTopology const & vtxTop = _topology.GetTopology(corner);

    bool vtxCornerIsPeriodic = !vtxCorner._isBoundary;

    int faceInVertex = vtxTop.GetFaceInVertex();

    //
    //  Initialize as boundary until determined otherwise (periodic)
    //
    fvarCorner._isBoundary = true;

    fvarCorner._numFacesAfter = 0;
    fvarCorner._numFacesTotal = 0;
    fvarCorner._numFacesBefore = 0;

    fvarCorner._isSharp = vtxCorner._isSharp;

    //  Skip the following search if only one face:
    if (vtxCorner._numFacesTotal == 1) {
        fvarCorner._numFacesTotal = 1;
        return;
    }

    //
    //  Inspect/gather faces "after" (counter-clockwise order from)
    //  the corner face.  If all are included and the vtx subset is
    //  periodic, check the seam for the fvar subset.
    //
    int numFacesAfterToVisit = vtxCorner._numFacesAfter;
    if (numFacesAfterToVisit) {
        int thisFace = faceInVertex;
        for (int i = 0; i < numFacesAfterToVisit; ++i) {
            int nextFace = vtxTop.GetFaceNext(thisFace);

            if (vtxTop.GetFaceVertexAtCorner(thisFace, fvarIndices) != 
                vtxTop.GetFaceVertexAtCorner(nextFace, fvarIndices)) {
                break;
            }
            if (vtxTop.GetFaceVertexTrailing(thisFace, fvarIndices) != 
                vtxTop.GetFaceVertexLeading(nextFace, fvarIndices)) {
                break;
            }
            ++ fvarCorner._numFacesAfter;

            thisFace = nextFace;
        }
    }
    int numFacesAfterUnvisited = vtxCorner._numFacesAfter -
                                 fvarCorner._numFacesAfter;
    if (vtxCornerIsPeriodic && (numFacesAfterUnvisited == 0)) {
        assert(vtxCorner._numFacesBefore == 0);
        int prevFace = vtxTop.GetFacePrevious(faceInVertex);

        if (vtxTop.GetFaceVertexLeading(faceInVertex, fvarIndices) == 
            vtxTop.GetFaceVertexTrailing(prevFace, fvarIndices)) {
            fvarCorner._isBoundary = false;
        }
    }

    //
    //  Inspect/gather faces "before" (clockwise order from) the corner
    //  face.  Include any faces "after" in the periodic case that were
    //  interrupted by a discontinuity:
    //
    int numFacesBeforeToVisit = vtxCorner._numFacesBefore;
    if (vtxCornerIsPeriodic) {
        numFacesBeforeToVisit += numFacesAfterUnvisited;
    }
    if (numFacesBeforeToVisit) {
        int thisFace = faceInVertex;
        for (int i = 0; i < numFacesBeforeToVisit; ++i) {
            int prevFace = vtxTop.GetFacePrevious(thisFace);

            if (vtxTop.GetFaceVertexAtCorner(thisFace, fvarIndices) != 
                vtxTop.GetFaceVertexAtCorner(prevFace, fvarIndices)) {
                break;
            }
            if (vtxTop.GetFaceVertexLeading(thisFace, fvarIndices) != 
                vtxTop.GetFaceVertexTrailing(prevFace, fvarIndices)) {
                break;
            }
            ++ fvarCorner._numFacesBefore;

            thisFace = prevFace;
        }
    }

    fvarCorner._numFacesTotal = 1 + fvarCorner._numFacesBefore +
                                    fvarCorner._numFacesAfter;
}


//
//  Local utilities to deal with face-varying assignments at the corner:
//
namespace {
    int
    getNumMatchingCornerIndices(CornerTopology const & corner,
                                Index                  indexToMatch,
                                Index          const   indices[]) {

        //  WIP - streamline this to increment indices[] by face sizes

        int numMatches = 0;
        for (int i = 0; i < corner.GetNumFaces(); ++i) {
            if (corner.GetFaceVertexAtCorner(i, indices) == indexToMatch) {
                numMatches ++;
            }
        }
        return numMatches;
    }

    bool
    moreThanTwoUniqueCornerIndices(CornerTopology const & corner,
                                    Index          const   indices[]) {

        //
        //  This is primarily used for face-varying indices -- where any
        //  more than three unique values is irrelant:

        //  WIP - potentially streamline this to increment indices[] by
        //  face sizes, especially when face-size is constant

        Index index1 = corner.GetFaceVertexAtCorner(0, indices);
        Index index2 = -1;

        for (int i = 1; i < corner.GetNumFaces(); ++i) {
            Index index = corner.GetFaceVertexAtCorner(i, indices);
            if (index != index1) {
                if (index2 < 0) {
                    index2 = index;
                } else if (index != index2) {
                    return true;
                }
            }
        }
        return false;
    }
}

void
SurfaceDescriptor::sharpenFVarSubset(SurfaceDescriptor const & vtxSurface,
        int corner, Index const fvarIndices[]) {

    CornerSubset const & vtxCorner  = vtxSurface._corners[corner];
    CornerSubset       & fvarCorner = _corners[corner];

    assert(!fvarCorner._isSharp);
    assert(fvarCorner._isBoundary);

    //
    //  Determine sharpness for a given (unsharpened) fvar subset in
    //  the following cases:
    //
    //      - the face-varying topology is non-manifold, i.e. the fvar
    //        corner value occurs outside the subset
    //
    //      - according to the face-varying interpolation option
    //
    //  where the non-manifold case takes precedence.
    //
    CornerTopology const & vtxTop = _topology.GetTopology(corner);

    int faceInVertex = vtxTop.GetFaceInVertex();

    Index fvarIndex = vtxTop.GetFaceVertexAtCorner(faceInVertex, fvarIndices);

    int fvarCount = getNumMatchingCornerIndices(vtxTop, fvarIndex, fvarIndices);

    bool fvarIsManifold = (fvarCount == fvarCorner._numFacesTotal);
    if (!fvarIsManifold) {
        fvarCorner._isSharp = true;
        return;
    }

    //
    //  Sharpen according to the face-varying interpolation option:
    //
    bool singleFaceSubset = (fvarCorner._numFacesTotal == 1);

    switch (_topology._schemeOptions.GetFVarLinearInterpolation()) {
    case Sdc::Options::FVAR_LINEAR_NONE:
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_ONLY:
        fvarCorner._isSharp = singleFaceSubset;
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS1:
        fvarCorner._isSharp = singleFaceSubset;
        if (!fvarCorner._isSharp) {
            bool fvarDiffersAtVertex = (fvarCount != vtxTop.GetNumFaces());
            bool moreThanTwoCornerFVars = fvarDiffersAtVertex &&
                    moreThanTwoUniqueCornerIndices(vtxTop, fvarIndices);

            fvarCorner._isSharp = moreThanTwoCornerFVars;
        }
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2:
        fvarCorner._isSharp = singleFaceSubset;
        if (!fvarCorner._isSharp) {
            bool fvarDiffersAtVertex = (fvarCount != vtxTop.GetNumFaces());
            bool moreThanTwoCornerFVars = fvarDiffersAtVertex &&
                    moreThanTwoUniqueCornerIndices(vtxTop, fvarIndices);
            bool concaveCornerSubset = fvarDiffersAtVertex &&
                (fvarCorner._numFacesTotal == (vtxTop.GetNumFaces()-1));
            bool singleEdgeDartSubset =
                (fvarCorner._numFacesTotal == vtxTop.GetNumFaces()) &&
                (fvarCorner._isBoundary != vtxCorner._isBoundary);

            fvarCorner._isSharp = moreThanTwoCornerFVars ||
                                  concaveCornerSubset ||
                                  singleEdgeDartSubset;
        }
        break;

    case Sdc::Options::FVAR_LINEAR_BOUNDARIES:
        fvarCorner._isSharp = true;
        break;

    case Sdc::Options::FVAR_LINEAR_ALL:
        assert("Unexpected FVAR_LINEAR_ALL interpolation" == 0);
        break;

    default:
        assert("Unknown FVarLinearInterpolation value" == 0);
        break;
    }
}

//
//  Miscellaneous methods for debugging:
//
void
SurfaceDescriptor::print(bool printVerts) const {

    FaceTopology const & top = _topology;

    bool isRegular = IsRegular();

    printf("    FaceTopology:\n");
    printf("       face size       = %d\n", top._faceSize);
    printf("       num-face-verts  = %d\n", top.GetNumFaceVertices());
    printf("    Properties:\n");
    printf("       is regular      = %d\n", isRegular);
    printf("    Combined tags:\n");
    printf("       inf-sharp verts  = %d\n", _combinedTags._infSharpVerts);
    printf("       semi-sharp verts = %d\n", _combinedTags._semiSharpVerts);
    printf("       any sharp edges  = %d\n", _combinedTags._anySharpEdges);
    printf("       unsharp boundary = %d\n", _combinedTags._boundaryNonSharp);
    printf("       irregular faces  = %d\n", _combinedTags._irregularFaceSizes);
    printf("       unordered verts  = %d\n", _combinedTags._unOrderedFaces);
    printf("       val-2 int verts  = %d\n", _combinedTags._interiorVal2Verts);

    if (printVerts) {
        Index const * indices = _indices;

        for (int i = 0; i < top._faceSize; ++i) {
            printf("        corner %d:\n", i);

            CornerTopology const & vTop = top.GetTopology(i);
            printf("            topology:  num faces  = %d, boundary = %d\n",
                    vTop.GetNumFaces(), vTop.IsBoundary());

            CornerSubset const & cSub = _corners[i];
            printf("            subset:    num faces  = %d, boundary = %d\n",
                    cSub._numFacesTotal, cSub._isBoundary);
            printf("                       num before = %d, num after = %d\n",
                    cSub._numFacesBefore, cSub._numFacesAfter);

            printf("            face-vert indices:\n");

            for (int j = 0, n = 0; j < vTop.GetNumFaces(); ++j) {
                printf("            face %d:  ", j);
                int S = vTop.GetFaceSize(j);
                for (int k = 0; k < S; ++k, ++n) {
                    printf("%3d", indices[n]);
                }
                printf("\n");
            }
            indices += vTop.GetNumFaceVertices();
        }
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
