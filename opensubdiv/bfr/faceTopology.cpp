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

#include "../bfr/faceTopology.h"
#include "../sdc/crease.h"


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {


//
//  Constructor needs the same Sdc scheme/options as the Factory to
//  support internal work -- may need to figure another way to assign
//  these if we later need a default constructor for some purpose...
//
FaceTopology::FaceTopology(Sdc::SchemeType schemeType,
                           Sdc::Options schemeOptions) :
    _schemeType(schemeType),
    _schemeOptions(schemeOptions),
    _regFaceSize(Sdc::SchemeTypeTraits::GetRegularFaceSize(schemeType)),
    _isInitialized(false) {

}

//
//  Main initialize/finalize used by base factory to delimit assignment:
//
void
FaceTopology::Initialize(int faceSize) {

    _faceSize = faceSize;

    _hasBoundaryVerts = false;
    _hasSharpVerts    = false;
    _hasSharpEdges    = false;
    _hasIncIrregFaces = (faceSize != _regFaceSize);
    _hasNonManCorners = false;
    _hasVal2IntVerts  = false;

    _isInitialized = true;
    _isFinalized   = false;

    _numFaceVertsTotal = 0;

    _vertexTopology.SetSize(faceSize);
    _faceInVertex.SetSize(faceSize);
}

void
FaceTopology::initializeSubsetInventory(CornerSubset cornerSubsets[]) const {

    //
    //  This is not done locally (in isolation) for each corner as there
    //  are some pathological cases where the inventory of one corner
    //  depends on one or more others:
    //
    int nVal3IntAdjTris = 0;

    for (int corner = 0; corner < _faceSize; ++corner) {
        VertexTopology const & V = _vertexTopology[corner];
        CornerSubset         & C = cornerSubsets[corner];

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
        if (V._commonFaceSize) {
            int S = V._commonFaceSize;

            if (!C._isBoundary) {
                if ((C._numFacesTotal == 3) && (V._commonFaceSize == 3)) {
                    nVerts += (++nVal3IntAdjTris == _faceSize);
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
            int cornerFace = _faceInVertex[corner];

            if (!C._isBoundary) {
                assert(C._numFacesTotal == V._numFaces);

                int nextFace = V.getFaceAfter(cornerFace, 2);
                if ((C._numFacesTotal == 3) && (V.getFaceSize(nextFace) == 3)) {
                    nVerts += (++nVal3IntAdjTris == _faceSize);
                } else {
                    for (int i = 2; i < C._numFacesTotal; ++i) {
                        int S = V.getFaceSize(nextFace);
                        nVerts += S - 2;
                        nextFace = V.getFaceNext(nextFace);
                    }
                    nVerts --;
                }
            } else {
                if (C._numFacesAfter) {
                    int nextFace = V.getFaceNext(cornerFace);
                    for (int i = 1; i < C._numFacesAfter; ++i) {
                        nextFace = V.getFaceNext(nextFace);
                        int S = V.getFaceSize(nextFace);
                        nVerts += S - 2;
                    }
                    nVerts ++;
                }
                if (C._numFacesBefore) {
                    int nextFace = V.getFaceBefore(cornerFace,
                                                   C._numFacesBefore);
                    for (int i = 0; i < C._numFacesBefore; ++i) {
                        int S = V.getFaceSize(nextFace);
                        nVerts += S - 2;
                        nextFace = V.getFaceNext(nextFace);
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
FaceTopology::InitializeVertexSubsets(Index const fvertIndices[]) {

    assert(_isFinalized);

    //  WIP - we eventually need the vertex indices to identify the
    //  adjacency/connectivity for unordered or non-manifold verts:
    if (_hasNonManCorners) {
        //assert(fvertIndices != 0);
        if (fvertIndices == 0) {
        }
    }

    //  Consider making this a member flag with other scheme/option
    //  related members:
    bool sharpenCorners =
            (_schemeOptions.GetVtxBoundaryInterpolation() ==
                Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);

    for (int i = 0; i < _faceSize; ++i) {
        VertexTopology & V = _vertexTopology[i];
        CornerSubset   & C = _cornerSubsets[i];

        //
        //  If the vertex topology was unordered (potentially non-manifold)
        //  this will have to be determined later from the face-vertex
        //  indices -- in which case, other FaceTopology members will need
        //  updating.
        //
        if (V._isOrdered) {
            C._isBoundary = V._isBoundary;

            C._numFacesTotal = V._numFaces;
            if (C._isBoundary) {
                C._numFacesBefore = _faceInVertex[i];
                C._numFacesAfter  = V._numFaces - 1 - _faceInVertex[i];
            } else {
                C._numFacesBefore = 0;
                C._numFacesAfter  = V._numFaces - 1;
            }

            C._isSharp = false;
            if (C._numFacesTotal == 1) {
                C._isSharp = sharpenCorners;
            }
        } else {
            //  Use a sharp corner for linear face for non-manifold for now:
            C._isBoundary     = true;
            C._numFacesTotal  = 1;
            C._numFacesBefore = 0;
            C._numFacesAfter  = 0;
            C._isSharp        = false;
        }

        C._numOuterFaces = -1;
        C._numOuterVerts = -1;
    }

    //
    //  Valence-2 interior vertices created the awkward situation where
    //  neighboring vertices in the ring of each corner fold over and
    //  overlap the base face -- not just at the valence-2 corner itself
    //  but its neighbors.  Tag the corners so that we can detect and
    //  deal with this situation appropriately:
    //
    if (_hasVal2IntVerts) {
    }

    initializeSubsetInventory(_cornerSubsets);
}

void
FaceTopology::Finalize() {

    //
    //  Inspect all corner vertex topologies -- accumulating the presence
    //  of irregular features for the face and assigning other internal
    //  members to help determine the limit surface:
    //
    //  WIP - potentially want to identify presence of degenerate faces
    //  below too, i.e. face size < 3.  A subclass may specify these in
    //  an ordered set and that would mess up some of the topological
    //  traversals.  In such case, we can initialize the vertex subset
    //  to excludes such faces.
    //
    assert(_isInitialized);

    for (int i = 0; i < _faceSize; ++i) {
        VertexTopology & vTop = _vertexTopology[i];
        assert(vTop._isFinalized);
        assert(vTop._numFaces > 0);

        _hasBoundaryVerts |=  vTop._isBoundary;
        _hasSharpVerts    |=  vTop._hasSharpVert;
        _hasSharpEdges    |=  vTop._hasSharpEdge;
        _hasIncIrregFaces |= (vTop._commonFaceSize != _regFaceSize);
        _hasNonManCorners |= !vTop._isOrdered;
        _hasVal2IntVerts  |= (vTop._numFaces == 2) && vTop._isInterior;

        _numFaceVertsTotal += vTop._numFaceVerts;
    }

    _isFinalized = true;
}

bool
FaceTopology::IsRegular(CornerSubset const cornerSubsets[]) const {

    if (cornerSubsets == 0) {
        cornerSubsets = &_cornerSubsets[0];
    }

    //  WIP - beware, these FaceToplogy members reflect the topology of
    //  the collection of vertices as a whole and not a subset -- so we
    //  may be excluding regular subets (optimize later)...
    //
    if (_hasIncIrregFaces || _hasSharpEdges    ||
                             _hasNonManCorners || _hasVal2IntVerts) {
        return false;
    }

    int regInteriorValence = (_regFaceSize == 4) ? 4 : 6;
    int regBoundaryValence = (regInteriorValence / 2);

    for (int i = 0; i < _faceSize; ++i) {
        CornerSubset const & subset = cornerSubsets[i];

        if (subset._numFacesTotal == regInteriorValence) {
            if (subset._isBoundary || subset._isSharp) return false;
        } else if (subset._numFacesTotal == regBoundaryValence) {
            if (!subset._isBoundary || subset._isSharp) return false;
        } else if (subset._numFacesTotal == 1) {
            if (!subset._isSharp) return false;
        } else {
            return false;
        }

        //  WIP - workaround to tagging defficiency for sharpened verts
        if (_vertexTopology[i]._hasSharpVert && !subset._isSharp) {
            return false;
        }
    }
    return true;
}

bool
FaceTopology::HasLimit() const {

    //  WIP - this will be greatly simplified once a "was boundary
    //  sharpened" tag is initialized per-vertex and a corresponding
    //  "has unsharpened boundaries" is added to the face.
    if (_schemeOptions.GetVtxBoundaryInterpolation() ==
                               Sdc::Options::VTX_BOUNDARY_NONE) {
        //
        //  With the "boundary none" case, a face with a boundary vertex
        //  only has a limit surface if all boundary edges incident its
        //  boundary vertices have been explicitly sharpened (inf-sharp):
        //
        if (!_hasBoundaryVerts) return true;

        //
        //  Boundary edges have been "unsharpened" to simplify processing,
        //  but we need to know if they had been explicitly sharpened
        //  for this purpose...
        //
        if (!_hasSharpEdges) {
            //if (!_hasSharpEdges) return false;
        } else {
            for (int i = 0; i < _faceSize; ++i) {
                if (_vertexTopology[i]._isBoundary) {
                    //  Return true if all boundary edges made inf-sharp
                    return true;
                }
            }
        }
    }
    return true;
}

void
FaceTopology::findFaceVaryingSubset(int corner,
        Index const          fvarIndices[],
        CornerSubset       & fvarSubset) const {

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
    VertexTopology const & vtxTop    = _vertexTopology[corner];
    CornerSubset const   & vtxSubset = _cornerSubsets[corner];

    bool vtxSubsetIsPeriodic = !vtxSubset._isBoundary;

    int cornerFace = _faceInVertex[corner];

    //
    //  Initialize as boundary until determined otherwise (periodic)
    //
    fvarSubset._isBoundary = true;

    fvarSubset._numFacesAfter = 0;
    fvarSubset._numFacesTotal = 0;
    fvarSubset._numFacesBefore = 0;

    fvarSubset._isSharp = vtxSubset._isSharp;

    //  Skip the following search if only one face:
    if (vtxSubset._numFacesTotal == 1) {
        fvarSubset._numFacesTotal = 1;
        return;
    }

    //
    //  Inspect/gather faces "after" (counter-clockwise order from)
    //  the corner face.  If all are included and the vtx subset is
    //  periodic, check the seam for the fvar subset.
    //
    int numFacesAfterToVisit = vtxSubset._numFacesAfter;
    if (numFacesAfterToVisit) {
        int thisFace = cornerFace;
        for (int i = 0; i < numFacesAfterToVisit; ++i) {
            int nextFace = vtxTop.getFaceNext(thisFace);

            if (vtxTop.getFaceVertexAtCorner(thisFace, fvarIndices) != 
                vtxTop.getFaceVertexAtCorner(nextFace, fvarIndices)) {
                break;
            }
            if (vtxTop.getFaceVertexTrailing(thisFace, fvarIndices) != 
                vtxTop.getFaceVertexLeading(nextFace, fvarIndices)) {
                break;
            }
            ++ fvarSubset._numFacesAfter;

            thisFace = nextFace;
        }
    }
    int numFacesAfterUnvisited = vtxSubset._numFacesAfter -
                                 fvarSubset._numFacesAfter;
    if (vtxSubsetIsPeriodic && (numFacesAfterUnvisited == 0)) {
        assert(vtxSubset._numFacesBefore == 0);
        int prevFace = vtxTop.getFacePrevious(cornerFace);

        if (vtxTop.getFaceVertexLeading(cornerFace, fvarIndices) == 
            vtxTop.getFaceVertexTrailing(prevFace, fvarIndices)) {
            fvarSubset._isBoundary = false;
        }
    }

    //
    //  Inspect/gather faces "before" (clockwise order from) the corner
    //  face.  Include any faces "after" in the periodic case that were
    //  interrupted by a discontinuity:
    //
    int numFacesBeforeToVisit = vtxSubset._numFacesBefore;
    if (vtxSubsetIsPeriodic) {
        numFacesBeforeToVisit += numFacesAfterUnvisited;
    }
    if (numFacesBeforeToVisit) {
        int thisFace = cornerFace;
        for (int i = 0; i < numFacesBeforeToVisit; ++i) {
            int prevFace = vtxTop.getFacePrevious(thisFace);

            if (vtxTop.getFaceVertexAtCorner(thisFace, fvarIndices) != 
                vtxTop.getFaceVertexAtCorner(prevFace, fvarIndices)) {
                break;
            }
            if (vtxTop.getFaceVertexLeading(thisFace, fvarIndices) != 
                vtxTop.getFaceVertexTrailing(prevFace, fvarIndices)) {
                break;
            }
            ++ fvarSubset._numFacesBefore;

            thisFace = prevFace;
        }
    }

    fvarSubset._numFacesTotal = 1 + fvarSubset._numFacesBefore +
                                    fvarSubset._numFacesAfter;
}

void
FaceTopology::sharpenFaceVaryingSubset(int corner,
        Index const    fvarIndices[],
        CornerSubset & fvarSubset) const {

    assert(!fvarSubset._isSharp);
    assert(fvarSubset._isBoundary);

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
    VertexTopology const & vtxTop  = _vertexTopology[corner];
    CornerSubset const & vtxSubset = _cornerSubsets[corner];

    int cornerFace = _faceInVertex[corner];

    Index fvarIndex = vtxTop.getFaceVertexAtCorner(cornerFace, fvarIndices);

    int fvarCount = vtxTop.getNumMatchingCornerIndices(fvarIndex, fvarIndices);

    bool fvarIsManifold = (fvarCount == fvarSubset._numFacesTotal);
    if (!fvarIsManifold) {
        fvarSubset._isSharp = true;
        return;
    }

    //
    //  Sharpen according to the face-varying interpolation option:
    //
    bool singleFaceSubset = (fvarSubset._numFacesTotal == 1);

    switch (_schemeOptions.GetFVarLinearInterpolation()) {
    case Sdc::Options::FVAR_LINEAR_NONE:
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_ONLY:
        fvarSubset._isSharp = singleFaceSubset;
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS1:
        fvarSubset._isSharp = singleFaceSubset;
        if (!fvarSubset._isSharp) {
            bool fvarDiffersAtVertex = (fvarCount != vtxTop._numFaces);
            bool moreThanTwoCornerFVars = fvarDiffersAtVertex &&
                vtxTop.moreThanTwoUniqueCornerIndices(fvarIndices);

            fvarSubset._isSharp = moreThanTwoCornerFVars;
        }
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2:
        fvarSubset._isSharp = singleFaceSubset;
        if (!fvarSubset._isSharp) {
            bool fvarDiffersAtVertex = (fvarCount != vtxTop._numFaces);
            bool moreThanTwoCornerFVars = fvarDiffersAtVertex &&
                vtxTop.moreThanTwoUniqueCornerIndices(fvarIndices);
            bool concaveCornerSubset = fvarDiffersAtVertex &&
                (fvarSubset._numFacesTotal == (vtxTop._numFaces-1));
            bool singleEdgeDartSubset =
                (fvarSubset._numFacesTotal == vtxTop._numFaces) &&
                (fvarSubset._isBoundary != vtxSubset._isBoundary);

            fvarSubset._isSharp = moreThanTwoCornerFVars ||
                                  concaveCornerSubset ||
                                  singleEdgeDartSubset;
        }
        break;

    case Sdc::Options::FVAR_LINEAR_BOUNDARIES:
        fvarSubset._isSharp = true;
        break;

    case Sdc::Options::FVAR_LINEAR_ALL:
        assert("Unexpected FVAR_LINEAR_ALL interpolation" == 0);
        break;

    default:
        assert("Unknown FVarLinearInterpolation value" == 0);
        break;
    }
}

bool
FaceTopology::IdentifyFaceVaryingSubsets(
        Index const  fvarCornerIndices[],
        CornerSubset fvarCornerSubsets[]) const {

    Index const * fvarIndices = fvarCornerIndices;

    bool fvarSubsetsAllMatch = true;

    for (int corner = 0; corner < _faceSize; ++corner) {
        VertexTopology const & vtxTop    = _vertexTopology[corner];
        CornerSubset const   & vtxSubset = _cornerSubsets[corner];

        CornerSubset & fvarSubset = fvarCornerSubsets[corner];

        //
        //  Seek the face-varying subset then determine its sharpness --
        //  initially inherited from the vertex subset (which takes
        //  precedence) but which may be applied to boundary subsets in
        //  other circumstances (typically the fvar interpolation option)
        //
        findFaceVaryingSubset(corner, fvarIndices, fvarSubset);

        if (fvarSubset._isBoundary && !fvarSubset._isSharp) {
            sharpenFaceVaryingSubset(corner, fvarIndices, fvarSubset);
        }

        bool fvarSubsetMatches =
                (fvarSubset._isBoundary     == vtxSubset._isBoundary) &&
                (fvarSubset._isSharp        == vtxSubset._isSharp) &&
                (fvarSubset._numFacesBefore == vtxSubset._numFacesBefore) &&
                (fvarSubset._numFacesAfter  == vtxSubset._numFacesAfter);

        fvarSubsetsAllMatch &= fvarSubsetMatches;

        fvarIndices += vtxTop._numFaceVerts;
    }

    if (fvarSubsetsAllMatch) {
        //  Copy the vertex subsets to initialize any other members:
        std::memcpy(fvarCornerSubsets, _cornerSubsets,
                    _faceSize * sizeof(CornerSubset));
    } else {
        initializeSubsetInventory(fvarCornerSubsets);
    }
    return fvarSubsetsAllMatch;
}

void
FaceTopology::GatherRegularPatchPoints4(
        CornerSubset const   faceSubsets[],
        Index        const   faceIndices[],
        Index                patchPoints[]) const {

    FaceTopology const & faceTopology = *this;

    assert(_regFaceSize == 4);

    if (faceSubsets == 0) faceSubsets = &faceTopology._cornerSubsets[0];

    //Index fvPhantom = -1;
    Index fvPhantom = faceIndices[0];

    //  WIP - currently passing in full set of indices for all face
    //  corners, but may gather them locally in future:
    Index const * fvIndices = &faceIndices[0];

    Index * P = patchPoints;
    for (int i = 0; i < 4; ++i) {
        VertexTopology const & vTop = faceTopology._vertexTopology[i];
        CornerSubset   const & cSub = faceSubsets[i];

        int faceCorner = faceTopology._faceInVertex[i];
        Index const *fvCorner = &fvIndices[faceCorner * 4];

        switch (i) {
        case 0:
            P[5] = fvCorner[0];
            if (!cSub._isBoundary) {
                int faceOpposite = (faceCorner + 2) & 3;
                Index const *fvOpposite = &fvIndices[faceOpposite * 4];
                P[4] = fvOpposite[1];
                P[0] = fvOpposite[2];
                P[1] = fvOpposite[3];
            } else {
                int faceOther = cSub._numFacesAfter ?
                                vTop.getFaceNext(faceCorner) :
                                vTop.getFacePrevious(faceCorner);
                Index const *fvOther = &fvIndices[faceOther * 4];
                P[4] = cSub._numFacesAfter  ? fvOther[3] : fvPhantom;
                P[0] = fvPhantom;
                P[1] = cSub._numFacesBefore ? fvOther[1] : fvPhantom;
            }
            break;
        case 1:
            P[6] = fvCorner[0];
            if (!cSub._isBoundary) {
                int faceOpposite = (faceCorner + 2) & 3;
                Index const *fvOpposite = &fvIndices[faceOpposite * 4];
                P[2] = fvOpposite[1];
                P[3] = fvOpposite[2];
                P[7] = fvOpposite[3];
            } else {
                int faceOther = cSub._numFacesAfter ?
                                vTop.getFaceNext(faceCorner) :
                                vTop.getFacePrevious(faceCorner);
                Index const *fvOther = &fvIndices[faceOther * 4];
                P[2] = cSub._numFacesAfter  ? fvOther[3] : fvPhantom;
                P[3] = fvPhantom;
                P[7] = cSub._numFacesBefore ? fvOther[1] : fvPhantom;
            }
            break;
        case 2:
            P[10] = fvCorner[0];
            if (!cSub._isBoundary) {
                int faceOpposite = (faceCorner + 2) & 3;
                Index const *fvOpposite = &fvIndices[faceOpposite * 4];
                P[11] = fvOpposite[1];
                P[15] = fvOpposite[2];
                P[14] = fvOpposite[3];
            } else {
                int faceOther = cSub._numFacesAfter ?
                                vTop.getFaceNext(faceCorner) :
                                vTop.getFacePrevious(faceCorner);
                Index const *fvOther = &fvIndices[faceOther * 4];
                P[11] = cSub._numFacesAfter  ? fvOther[3] : fvPhantom;
                P[15] = fvPhantom;
                P[14] = cSub._numFacesBefore ? fvOther[1] : fvPhantom;
            }
            break;
        case 3:
            P[9] = fvCorner[0];
            if (!cSub._isBoundary) {
                int faceOpposite = (faceCorner + 2) & 3;
                Index const *fvOpposite = &fvIndices[faceOpposite * 4];
                P[13] = fvOpposite[1];
                P[12] = fvOpposite[2];
                P[ 8] = fvOpposite[3];
            } else {
                int faceOther = cSub._numFacesAfter ?
                                vTop.getFaceNext(faceCorner) :
                                vTop.getFacePrevious(faceCorner);
                Index const *fvOther = &fvIndices[faceOther * 4];
                P[13] = cSub._numFacesAfter  ? fvOther[3] : fvPhantom;
                P[12] = fvPhantom;
                P[ 8] = cSub._numFacesBefore ? fvOther[1] : fvPhantom;
            }
            break;
        }
        fvIndices += vTop._numFaceVerts;
    }
}

void
FaceTopology::GatherRegularPatchPoints3(
        CornerSubset const   faceSubsets[],
        Index        const   faceIndices[],
        Index                patchPoints[]) const {

    assert(_regFaceSize == 3);

    if (faceSubsets || faceIndices || patchPoints) {
        assert("gatherRegularPatchPoints3() not yet supported" == 0);
    }
}

int
FaceTopology::GetNumControlVertices(CornerSubset const faceSubsets[]) const {

    if (faceSubsets == 0) faceSubsets = &_cornerSubsets[0];

    int nVerts = _faceSize;
    for (int i = 0; i < _faceSize; ++i) {
        nVerts += faceSubsets[i]._numOuterVerts;
    }
    return nVerts;
}

int
FaceTopology::GetNumControlFaces(CornerSubset const faceSubsets[]) const {

    if (faceSubsets == 0) faceSubsets = &_cornerSubsets[0];

    int nFaces = 1;
    for (int i = 0; i < _faceSize; ++i) {
        nFaces += faceSubsets[i]._numOuterFaces;
    }
    return nFaces;
}

int
FaceTopology::GatherControlVertexIndices(
        CornerSubset const faceSubsets[],
        Index        const faceIndices[],
        Index              cvIndices[]) const {

    if (faceSubsets == 0) faceSubsets = &_cornerSubsets[0];

    //
    //  Assign CV indices from the base face first:
    //
    int baseOffset = _vertexTopology[0].getFaceVertexOffset(_faceInVertex[0]);
    Index const * baseIndices = &faceIndices[baseOffset];
    std::memcpy(cvIndices, baseIndices, _faceSize * sizeof(Index));

    int nCVIndices = _faceSize;

    int nVal3IntAdjTris = 0;

    //
    //  Assign CV indices "local to" each corner:
    //
    Index const * cornerIndices = faceIndices;

    for (int corner = 0; corner < _faceSize; ++corner) {
        VertexTopology const & V = _vertexTopology[corner];
        CornerSubset const   & C = faceSubsets[corner];

        int cornerFace = _faceInVertex[corner];

        //
        //  Similar loops here to traverse the faces around each corner,
        //  so potential for some consolidation here...
        //
        int nCVIndicesBefore = nCVIndices;

        if (!C._isBoundary) {
            assert(C._numFacesTotal == V._numFaces);

            int numFaces = C._numFacesTotal - 2;
            int nextFace = V.getFaceAfter(cornerFace, 2);
            if ((numFaces == 1) && (V.getFaceSize(nextFace) == 3)) {
                if (++nVal3IntAdjTris == _faceSize) {
                    int fvOffset = V.getFaceVertexOffset(nextFace);

                    cvIndices[nCVIndices++] = cornerIndices[fvOffset + 1];
                }
            } else {
                for (int j = 0; j < numFaces; ++j) {
                    int S = V.getFaceSize(nextFace);
                    int fvOffset = V.getFaceVertexOffset(nextFace);

                    int M = (S - 2) - (j == (numFaces - 1));
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[nCVIndices++] = cornerIndices[fvOffset + k];
                    }
                    nextFace = V.getFaceNext(nextFace);
                }
            }
        } else {
            if (C._numFacesAfter) {
                //
                //  While the first face "after" is generally skipped, if it
                //  is the only one, we need to include its trailing edge:
                //
                int numFaces = C._numFacesAfter - 1;
                int nextFace = V.getFaceNext(cornerFace);
                for (int j = 0; j < numFaces; ++j) {
                    nextFace = V.getFaceNext(nextFace);

                    int S = V.getFaceSize(nextFace);
                    int fvOffset = V.getFaceVertexOffset(nextFace);

                    int M = (S - 2);
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[nCVIndices++] = cornerIndices[fvOffset + k];
                    }
                }
                cvIndices[nCVIndices++] =
                        V.getFaceVertexTrailing(nextFace, cornerIndices);
            }
            if (C._numFacesBefore) {
                int numFaces = C._numFacesBefore;
                int nextFace = V.getFaceBefore(cornerFace, C._numFacesBefore);
                for (int j = 0; j < numFaces; ++j) {
                    int S = V.getFaceSize(nextFace);
                    int fvOffset = V.getFaceVertexOffset(nextFace);

                    int M = (S - 2) - (j == (numFaces - 1));
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[nCVIndices++] = cornerIndices[fvOffset + k];
                    }
                    nextFace = V.getFaceNext(nextFace);
                }
            }
        }
        assert((nCVIndices - nCVIndicesBefore) == C._numOuterVerts);

        cornerIndices += V._numFaceVerts;
    }
    return nCVIndices;
}

int
FaceTopology::GatherControlFaceSizes(
        CornerSubset const faceSubsets[],
        int                faceSizes[]) const {

    if (faceSubsets == 0) faceSubsets = &_cornerSubsets[0];

    int nFaces = 1;
    int sumOfSizes = _faceSize;
    faceSizes[0] = _faceSize;

    for (int corner = 0; corner < _faceSize; ++corner) {
        VertexTopology const & V = _vertexTopology[corner];
        CornerSubset const   & C = faceSubsets[corner];

        int cornerFace = _faceInVertex[corner];

        if (!C._isBoundary) {
            int nextFace = V.getFaceAfter(cornerFace, 2);
            for (int i = 2; i < C._numFacesTotal; ++i) {
                int S = V.getFaceSize(nextFace);
                faceSizes[nFaces++] = S;
                sumOfSizes += S;

                nextFace = V.getFaceNext(nextFace);
            }
        } else {
            if (C._numFacesAfter > 1) {
                int nextFace = V.getFaceAfter(cornerFace, 2);
                for (int j = 1; j < C._numFacesAfter; ++j) {
                    int S = V.getFaceSize(nextFace);
                    faceSizes[nFaces++] = S;
                    sumOfSizes += S;

                    nextFace = V.getFaceNext(nextFace);
                }
            }
            if (C._numFacesBefore) {
                int nextFace = V.getFaceBefore(cornerFace, C._numFacesBefore);
                for (int j = 0; j < C._numFacesBefore; ++j) {
                    int S = V.getFaceSize(nextFace);
                    faceSizes[nFaces++] = S;
                    sumOfSizes += S;

                    nextFace = V.getFaceNext(nextFace);
                }
            }
        }
    }
    return sumOfSizes;
}

int
FaceTopology::GatherControlVertexSharpness(
        CornerSubset const faceSubsets[],
        int                cornerVerts[],
        float              vertSharpness[]) const {

    if (faceSubsets == 0) faceSubsets = &_cornerSubsets[0];

    int nSharpVerts = 0;

    for (int i = 0; i < _faceSize; ++i) {
        VertexTopology const & vTop = _vertexTopology[i];

        //  Sharpness of the CornerSubset takes precedence here:
        if (faceSubsets[i]._isSharp) {
            cornerVerts[nSharpVerts] = i;
            vertSharpness[nSharpVerts] = Sdc::Crease::SHARPNESS_INFINITE;
            ++ nSharpVerts;
        } else if (vTop._hasSharpVert) {
            cornerVerts[nSharpVerts] = i;
            vertSharpness[nSharpVerts] = vTop._vertSharpness;
            ++ nSharpVerts;
        }
    }
    return nSharpVerts;
}

int
FaceTopology::GatherControlEdgeSharpness(
        CornerSubset const faceSubsets[],
        int                edgeVertPairs[],
        float              edgeSharpness[]) const {

    if (faceSubsets == 0) faceSubsets = &_cornerSubsets[0];


    int nSharpEdges = 0;

    //
    //  For each corner, test the forward edge in the face and any
    //  interior edges local to the corner vertex:
    //
    int perimMax   = GetNumControlVertices(faceSubsets);
    int perimStart = _faceSize;

    for (int corner = 0; corner < _faceSize; ++corner) {
        VertexTopology const & V = _vertexTopology[corner];
        CornerSubset const   & C = faceSubsets[corner];

        if (!V._hasSharpEdge) {
            perimStart += C._numOuterVerts;
            continue;
        }

        int cornerFace = _faceInVertex[corner];

        //  Test the forward edge of the face:
        float sharpness = V._faceEdgeSharpness[2*cornerFace];
        if (sharpness > 0.0f) {
            *edgeVertPairs++ =  corner;
            *edgeVertPairs++ = (corner + 1) % _faceSize;
            *edgeSharpness++ = sharpness;
            nSharpEdges++;
        }

        //
        //  Inspect interior edges of the subset -- test sharpness of
        //  the trailing edge of the faces after/before the corner face.
        //
        //  Unfortunately we need the control vertex index at the end
        //  of the edge, and so we need to track the perimeter -- which
        //  requires the face sizes and may wrap around with tris...
        //
        int nextVert = perimStart;

        //  WIP - these blocks are similar enough to warrant merging
        if (!C._isBoundary) {
            int nextFace = V.getFaceNext(cornerFace);
            for (int i = 2; i < C._numFacesTotal; ++i) {
                sharpness = V._faceEdgeSharpness[2*nextFace + 1];
                if (sharpness > 0.0f) {
                    *edgeSharpness++ = sharpness;
                    *edgeVertPairs++ = corner;
                    *edgeVertPairs++ = (nextVert < perimMax)
                                     ? nextVert : _faceSize;
                    nSharpEdges++;
                }
                nextFace  = V.getFaceNext(nextFace);
                nextVert += V.getFaceSize(nextFace) - 2;
            }
        } else {
            if (C._numFacesAfter) {
                int nextFace = V.getFaceNext(cornerFace);
                for (int i = 1; i < C._numFacesAfter; ++i) {
                    sharpness = V._faceEdgeSharpness[2*nextFace + 1];
                    if (sharpness > 0.0f) {
                        *edgeSharpness++ = sharpness;
                        *edgeVertPairs++ = corner;
                        *edgeVertPairs++ = (nextVert < perimMax)
                                         ? nextVert : _faceSize;
                        nSharpEdges++;
                    }
                    nextFace  = V.getFaceNext(nextFace);
                    nextVert += V.getFaceSize(nextFace) - 2;
                }
            }
            if (C._numFacesBefore) {
                int nextFace = V.getFaceBefore(cornerFace, C._numFacesBefore);
                for (int i = 1; i < C._numFacesBefore; ++i) {
                    sharpness = V._faceEdgeSharpness[2*nextFace + 1];
                    if (sharpness > 0.0f) {
                        *edgeSharpness++ = sharpness;
                        *edgeVertPairs++ = corner;
                        *edgeVertPairs++ = (nextVert < perimMax)
                                         ? nextVert : _faceSize;
                        nSharpEdges++;
                    }
                    nextFace  = V.getFaceNext(nextFace);
                    nextVert += V.getFaceSize(nextFace) - 2;
                }
            }
        }
        perimStart += C._numOuterVerts;
    }
    return nSharpEdges;
}

int
FaceTopology::GatherControlFaceVertices(
        CornerSubset const faceSubsets[],
        int                numControlVertices,
        int                faceVertices[]) const {

    if (faceSubsets == 0) faceSubsets = &_cornerSubsets[0];

    //
    //  Assign face vertices for the first/base face:
    //
    for (int i = 0; i < _faceSize; ++i) {
        *faceVertices++ = i;
    }
    int nFaceVertices = _faceSize;

    //
    //  Assign face vertex indices "local to" each corner:
    //
    if (numControlVertices == 0) {
        numControlVertices = GetNumControlVertices(faceSubsets);
    }

    int startPerimOfCorner = _faceSize;
    for (int corner = 0; corner < _faceSize; ++corner) {
        VertexTopology const & V = _vertexTopology[corner];
        CornerSubset const   & C = faceSubsets[corner];

        int cornerFace = _faceInVertex[corner];

        //
        //  The interior case is simpler, the boundary needing more care:
        //
        if (!C._isBoundary) {
            assert(C._numFacesTotal == V._numFaces);
            int nextFace   = V.getFaceNext(cornerFace);
            int startPerimOfFace = startPerimOfCorner;

            int N = C._numFacesTotal - 2;
            for (int j = 0; j < N; ++j) {
                bool lastFace = (j == (N - 1));

                nextFace = V.getFaceNext(nextFace);
                int S = V.getFaceSize(nextFace);

                //
                //  Special cases:
                //      - the last face-vert of the last face here is the
                //        leading (?) edge of the corner
                //      - for the last corner only, the face-vert preceding
                //        the last will wrap around the perimeter
                //
                *faceVertices++ = corner;
                for (int k = 1; k < S - 2; ++k) {
                    *faceVertices++ = startPerimOfFace + k - 1;
                }

                int nextToLastPerimOfFace = startPerimOfFace + S - 3;
                if (nextToLastPerimOfFace == numControlVertices) {
                    nextToLastPerimOfFace = _faceSize;
                }
                *faceVertices++ = nextToLastPerimOfFace;

                int lastPerimOfFace = startPerimOfFace + S - 2;
                if (lastPerimOfFace == numControlVertices) {
                    lastPerimOfFace = _faceSize;
                }
                *faceVertices++ = (lastFace) ? ((corner+1) % _faceSize)
                                : lastPerimOfFace;

                nFaceVertices += S;

                startPerimOfFace += S - 2;
                startPerimOfCorner += S - 2;
            }
            startPerimOfCorner --;
        } else {
            if (C._numFacesAfter) {
                int nextFace   = V.getFaceNext(cornerFace);
                int startPerimOfFace = startPerimOfCorner;

                int N = C._numFacesAfter - 1;
                for (int j = 0; j < N; ++j) {
                    nextFace = V.getFaceNext(nextFace);
                    int S = V.getFaceSize(nextFace);

                    //  No special cases here
                    *faceVertices++ = corner;
                    for (int k = 1; k < S; ++k) {
                        *faceVertices++ = startPerimOfFace + k - 1;
                    }

                    nFaceVertices += S;

                    startPerimOfFace += S - 2;
                    startPerimOfCorner += S - 2;
                }
                startPerimOfCorner ++;
            }
            if (C._numFacesBefore) {
                //  Finding the first face here is a bit awkward -- and will
                //  be more so if the faces are unordered...
                assert(V._isOrdered);
                int nextFace = (cornerFace + V._numFaces - C._numFacesBefore);
                if (nextFace >= V._numFaces) {
                    nextFace -= V._numFaces;
                }
                int startPerimOfFace = startPerimOfCorner;

                int N = C._numFacesBefore;
                for (int j = 0; j < N; ++j) {
                    int S = V.getFaceSize(nextFace);
                    bool lastFace = (j == (N - 1));

                    //  Special cases are same as the interior case above
                    *faceVertices++ = corner;
                    for (int k = 1; k < S - 2; ++k) {
                        *faceVertices++ = startPerimOfFace + k - 1;
                    }

                    int nextToLastPerimOfFace = startPerimOfFace + S - 3;
                    if (nextToLastPerimOfFace == numControlVertices) {
                        nextToLastPerimOfFace = _faceSize;
                    }
                    *faceVertices++ = nextToLastPerimOfFace;

                    int lastPerimOfFace = startPerimOfFace + S - 2;
                    if (lastPerimOfFace == numControlVertices) {
                        lastPerimOfFace = _faceSize;
                    }
                    *faceVertices++ = (lastFace) ? ((corner+1) % _faceSize)
                                    : lastPerimOfFace;

                    nFaceVertices += S;

                    startPerimOfFace += S - 2;
                    startPerimOfCorner += S - 2;

                    nextFace = V.getFaceNext(nextFace);
                }
                startPerimOfCorner --;
            }
        }
    }
    return nFaceVertices;
}

namespace {
    struct SimpleHashBits {
        typedef unsigned long int_type;

        void Clear() { std::memset(this, 0, sizeof(*this)); }

        //  This generates fewest compiler issues about type aliasing...
        int_type GetInt() const {
            assert(sizeof(int_type) == sizeof(*this));
            int_type intVar;
            std::memcpy(&intVar, this, sizeof(*this));
            return intVar;
        }

        int_type v0Valence    :  7;
        int_type v1Valence    :  7;
        int_type v2Valence    :  7;
        int_type v3Valence    :  7;
        int_type v0IsBoundary :  1;
        int_type v1IsBoundary :  1;
        int_type v2IsBoundary :  1;
        int_type v3IsBoundary :  1;

        int_type v0IsSharp    :  1;
        int_type v1IsSharp    :  1;
        int_type v2IsSharp    :  1;
        int_type v3IsSharp    :  1;
        int_type v0FaceInRing :  6;
        int_type v1FaceInRing :  6;
        int_type v2FaceInRing :  6;
        int_type v3FaceInRing :  6;

        //  Possible approximation level here
        int_type unused       :  4;
    };
}

TopologyCache::Key
FaceTopology::ComputeTopologyKey(CornerSubset const faceSubsets[]) const {

    if (faceSubsets == 0) faceSubsets = &_cornerSubsets[0];

    //
    //  The Key computation is going to change significantly -- especially
    //  once sharpness values can be added -- though it will continue to
    //  support simple common cases with this simple bit assignment.  The
    //  features exclude:
    //
    //      - no incident irregular faces
    //      - no semi-sharp vertices
    //      - no sharp edges of any kind
    //
    //  and are otherwise limited to:
    //
    //      - interior valence up to 128 (7 bits)
    //      - boundary valence up to  64 (6 bits)
    //      - inf-sharp vertices
    //
    //  Dealing with incident irregular faces is unfortunate as the entire
    //  set of incident face sizes must somehow be encoded.  An exception
    //  this that's worth supporting (and easily achieved) is when the face
    //  size is constant but not regular, i.e. when Catmark is applied to a
    //  triangle mesh.  The bits already support triangular meshes for Loop,
    //  a bit just needs to be added to distinguish the scheme -- but a bit
    //  more work is needed in the code to support constant face sizes that
    //  are not regular.
    //
    TopologyCache::Key key;

    CornerSubset const * C = faceSubsets;
    if (_hasIncIrregFaces) {
        //  WIP - can still accept irregular but constant face size
        return key;
    }
    if (_hasSharpEdges) {
        return key;
    }
    //  WIP - improve tagging here, i.e. "has semi-sharp verts"
    if (_hasSharpVerts) {
        for (int i = 0; i < _faceSize; ++i) {
            if (!C[i]._isSharp) {
                if (_vertexTopology[i]._vertSharpness > 0.0) {
                    return key;
                }
            }
        }
    }

    //
    //  Reject valence higher than the supported maxima:
    //
    int const maxValInt = (1 << 7);
    int const maxValBnd = (1 << 6);

    for (int i = 0; i < _faceSize; ++i) {
        if (C[i]._isBoundary) {
            if (C[i]._numFacesTotal >= maxValBnd) return key;
        } else {
            if (C[i]._numFacesTotal >= maxValInt) return key;
        }
    }

    //
    //  Pack the corner subset topology into bits:
    //
    SimpleHashBits simpleBits;
    simpleBits.Clear();

    simpleBits.v0Valence    = C[0]._numFacesTotal;
    simpleBits.v0IsBoundary = C[0]._isBoundary;
    simpleBits.v0FaceInRing = C[0]._numFacesBefore;
    simpleBits.v0IsSharp    = C[0]._isSharp;

    simpleBits.v1Valence    = C[1]._numFacesTotal;
    simpleBits.v1IsBoundary = C[1]._isBoundary;
    simpleBits.v1FaceInRing = C[1]._numFacesBefore;
    simpleBits.v1IsSharp    = C[1]._isSharp;

    simpleBits.v2Valence    = C[2]._numFacesTotal;
    simpleBits.v2IsBoundary = C[2]._isBoundary;
    simpleBits.v2FaceInRing = C[2]._numFacesBefore;
    simpleBits.v2IsSharp    = C[2]._isSharp;

    if (_faceSize == 4) {
        simpleBits.v3Valence    = C[3]._numFacesTotal;
        simpleBits.v3IsBoundary = C[3]._isBoundary;
        simpleBits.v3FaceInRing = C[3]._numFacesBefore;
        simpleBits.v3IsSharp    = C[3]._isSharp;
    }

    key.hashBits = simpleBits.GetInt();
//key.hashBits = 0;
    return key;
}

void
FaceTopology::print(Index const faceIndices[], bool printVerts) const {

    FaceTopology const & f = *this;

    bool faceHasLimit  = f.HasLimit();
    bool faceIsRegular = f.IsRegular();

    printf("    FaceTopology:\n");
    if (!faceHasLimit) {
        printf("        has limit       = FALSE\n");
    }
    printf("        face size       = %d\n", _faceSize);
    printf("        is regular      = %d\n", faceIsRegular);
    if (!faceIsRegular) {
        printf("        has sharp verts = %d\n", f._hasSharpVerts);
        printf("        has sharp edges = %d\n", f._hasSharpEdges);
        printf("        inc irreg faces = %d\n", f._hasIncIrregFaces);
        printf("        non-man corners = %d\n", f._hasNonManCorners);
        printf("        val-2 int verts = %d\n", f._hasVal2IntVerts);
    }
    printf("        num-face-verts  = %d\n", f._numFaceVertsTotal);

    Index const * indices = faceIndices;

    for (int i = 0; printVerts && (i < _faceSize); ++i) {
        printf("        corner %d:\n", i);

        VertexTopology const & vTop = f._vertexTopology[i];
        printf("            topology:  num faces  = %d, boundary = %d\n",
                vTop._numFaces, vTop._isBoundary);

        CornerSubset const & cSub = f._cornerSubsets[i];
        printf("            subset:    num faces  = %d, boundary = %d\n",
                cSub._numFacesTotal, cSub._isBoundary);
        printf("                       num before = %d, num after = %d\n",
                cSub._numFacesBefore, cSub._numFacesAfter);

        printf("            face-vert indices:\n");

        for (int j = 0, n = 0; j < vTop._numFaces; ++j) {
            printf("            face %d:  ", j);
            int S = vTop.getFaceSize(j);
            for (int k = 0; k < S; ++k, ++n) {
                printf("%3d", indices[n]);
            }
            printf("\n");
        }
        indices += vTop._numFaceVerts;
    }
}

void
FaceTopology::printControlTopology(Index const faceIndices[]) const {

    printf("          FaceTopology properties:\n");
    printf("              has inc irreg faces = %d\n", _hasIncIrregFaces);
    printf("              has sharp verts     = %d\n", _hasSharpVerts);
    printf("              has sharp edges     = %d\n", _hasSharpEdges);

    int nVerts = GetNumControlVertices(0);
    int nFaces = GetNumControlFaces(0);

    int    S[nFaces];
    Index  P[_numFaceVertsTotal];
    Index *p = 0;

    printf("          FaceTopology topology:\n");

    int n = GatherControlVertexIndices(0, faceIndices, P);
    assert(n == nVerts);
    printf("              num control vertices = %d\n", nVerts);
    printf("              control vert indices:");
    p = P;
    for (int i = 0; i < _faceSize; ++i) {
        printf(" %3d", *p++);
    }
    printf("\n");
    if (nVerts > _faceSize) {
        printf("                                   ");
        for (int i = _faceSize; i < nVerts; ++i) {
            printf(" %3d", *p++);
        }
        printf("\n");
    }

    printf("              num control faces = %d\n", nFaces);
    int n1 = GatherControlFaceSizes(0, S);
    int n2 = GatherControlFaceVertices(0, nVerts, P);
    assert(n1 == n2);

    printf("              control face (size) verts:");
    p = P;
    printf(" (%d)", S[0]);
    for (int i = 0; i < _faceSize; ++i) {
        printf(" %3d", *p++);
    }
    printf("\n");
    for (int i = 1; i < nFaces; ++i) {
        printf("                                        ");
        printf(" (%d)", S[i]);
        for (int j = 0; j < S[i]; ++j) {
        printf(" %3d", *p++);
        }
        printf("\n");
    }

    //  Corner subsets may be sharpened, so don't just test topology:
    {
        int   cornerIndices[_faceSize];
        float vertSharpness[_faceSize];

        int nSharp = GatherControlVertexSharpness(0, cornerIndices,
                                                     vertSharpness);
        printf("              num sharp verts = %d\n", nSharp);
        for (int i = 0; i < nSharp; ++i) {
            printf("                              ");
            printf("  %d:  ", cornerIndices[i]);
            if (vertSharpness[i] < Sdc::Crease::SHARPNESS_INFINITE) {
                printf("%6.3f\n", vertSharpness[i]);
            } else {
                printf("inf\n");
            }
        }
    }

    if (_hasSharpEdges) {
        int   edgeVertPairs[nVerts * 2];
        float edgeSharpness[nVerts];

        int nSharp = GatherControlEdgeSharpness(0, edgeVertPairs,
                                                        edgeSharpness);
        printf("              num sharp edges = %d\n", nSharp);
        for (int i = 0; i < nSharp; ++i) {
            printf("                              ");
            printf(" (%d,%2d):  ", edgeVertPairs[i*2+0],
                                   edgeVertPairs[i*2+1]);
            if (edgeSharpness[i] < Sdc::Crease::SHARPNESS_INFINITE) {
                printf("%6.3f\n", edgeSharpness[i]);
            } else {
                printf("inf\n");
            }
        }
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
