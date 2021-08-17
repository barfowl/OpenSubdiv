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


//
//  Main initialization methods -- one for vertex topology and the other
//  for face-varying topology (a subset of the vertex topology):
//
void
SurfaceDescriptor::Initialize(Index const vtxIndices[]) {

    assert(_topology._isFinalized);

    //  WIP - we eventually need the vertex indices to identify the
    //  adjacency/connectivity for unordered or non-manifold verts:
    if (_topology._hasUnorderedVerts) {
        //assert(vtxIndices != 0);
        if (vtxIndices == 0) {
        }
    }

    //  Consider making this a member flag with other scheme/option
    //  related members:
    bool sharpenCorners =
            (_topology._schemeOptions.GetVtxBoundaryInterpolation() ==
                Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);

    _corners.SetSize(_topology._faceSize);

    for (int i = 0; i < _topology._faceSize; ++i) {
        VertexTopology const & V = _topology._vertexTopology[i];
        CornerSubset         & C = _corners[i];

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
                C._numFacesBefore = _topology._faceInVertex[i];
                C._numFacesAfter  = V._numFaces - 1 - _topology._faceInVertex[i];
            } else {
                C._numFacesBefore = 0;
                C._numFacesAfter  = V._numFaces - 1;
            }

            C._isSharp = V._isInfSharp;
            if (C._numFacesTotal == 1) {
                C._isSharp |= sharpenCorners;
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
    if (_topology._hasVal2IntVerts) {
    }

    initializeCornerInventory();

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

    CornerSubset const * vtxCorners = &vtxSurface._corners[0];

    bool fvarSubsetsAllMatch = true;

    int faceSize =  _topology._faceSize;

    _corners.SetSize(faceSize);

    Index const * fvarCornerIndices = fvarIndices;

    for (int corner = 0; corner < faceSize; ++corner) {
        VertexTopology const & vtxTop    = _topology._vertexTopology[corner];
        CornerSubset const   & vtxCorner = vtxCorners[corner];

        //
        //  Initialize the extent of the face-varying subset then determine
        //  its sharpness -- initially inherited from the vertex subset
        //  (which takes precedence) but which may be applied to boundary
        //  subsets in other circumstances (typically the fvar interpolation
        //  option)
        //
        CornerSubset & fvarCorner = _corners[corner];

        initializeFVarCorner(corner, vtxCorner, fvarCornerIndices);

        if (fvarCorner._isBoundary && !fvarCorner._isSharp) {
            sharpenFVarCorner(corner, vtxCorner, fvarCornerIndices);
        }

        bool fvarSubsetMatches =
                (fvarCorner._isBoundary     == vtxCorner._isBoundary) &&
                (fvarCorner._isSharp        == vtxCorner._isSharp) &&
                (fvarCorner._numFacesBefore == vtxCorner._numFacesBefore) &&
                (fvarCorner._numFacesAfter  == vtxCorner._numFacesAfter);

        fvarSubsetsAllMatch &= fvarSubsetMatches;

        fvarCornerIndices += vtxTop._numFaceVerts;
    }

    if (fvarSubsetsAllMatch) {
        //  Copy the vertex subsets to initialize any other members:
        std::memcpy(_corners, vtxCorners, faceSize * sizeof(CornerSubset));
    } else {
        initializeCornerInventory();
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
    //  WIP - beware, these FaceToplogy members reflect the topology of
    //  the collection of vertices as a whole and not those of the given
    //  subset -- so regular subets may be excluded here (optimize later)
    //
    if (_topology._hasIncIrregFaces  ||
        _topology._hasSharpEdges     || _topology._hasSemiSharpVerts ||
        _topology._hasUnorderedVerts || _topology._hasVal2IntVerts) {
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
SurfaceDescriptor::initializeCornerInventory() {

    //
    //  This is not done locally (in isolation) for each corner as there
    //  are some pathological cases where the inventory of one corner
    //  depends on one or more others:
    //
    int nVal3IntAdjTris = 0;

    for (int corner = 0; corner < _topology._faceSize; ++corner) {
        VertexTopology const & V = _topology._vertexTopology[corner];
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
        if (V._commonFaceSize) {
            int S = V._commonFaceSize;

            if (!C._isBoundary) {
                if ((C._numFacesTotal == 3) && (V._commonFaceSize == 3)) {
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
            int cornerFace = _topology._faceInVertex[corner];

            if (!C._isBoundary) {
                assert(C._numFacesTotal == V._numFaces);

                int nextFace = V.getFaceAfter(cornerFace, 2);
                if ((C._numFacesTotal == 3) && (V.getFaceSize(nextFace) == 3)) {
                    nVerts += (++nVal3IntAdjTris == _topology._faceSize);
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
SurfaceDescriptor::initializeFVarCorner(int corner,
        CornerSubset const & vtxCorner,
        Index        const   fvarIndices[]) {

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
    CornerSubset & fvarCorner = _corners[corner];

    VertexTopology const & vtxTop = _topology._vertexTopology[corner];

    bool vtxCornerIsPeriodic = !vtxCorner._isBoundary;

    int faceInVertex = _topology._faceInVertex[corner];

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
            int nextFace = vtxTop.getFaceNext(thisFace);

            if (vtxTop.getFaceVertexAtCorner(thisFace, fvarIndices) != 
                vtxTop.getFaceVertexAtCorner(nextFace, fvarIndices)) {
                break;
            }
            if (vtxTop.getFaceVertexTrailing(thisFace, fvarIndices) != 
                vtxTop.getFaceVertexLeading(nextFace, fvarIndices)) {
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
        int prevFace = vtxTop.getFacePrevious(faceInVertex);

        if (vtxTop.getFaceVertexLeading(faceInVertex, fvarIndices) == 
            vtxTop.getFaceVertexTrailing(prevFace, fvarIndices)) {
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
            int prevFace = vtxTop.getFacePrevious(thisFace);

            if (vtxTop.getFaceVertexAtCorner(thisFace, fvarIndices) != 
                vtxTop.getFaceVertexAtCorner(prevFace, fvarIndices)) {
                break;
            }
            if (vtxTop.getFaceVertexLeading(thisFace, fvarIndices) != 
                vtxTop.getFaceVertexTrailing(prevFace, fvarIndices)) {
                break;
            }
            ++ fvarCorner._numFacesBefore;

            thisFace = prevFace;
        }
    }

    fvarCorner._numFacesTotal = 1 + fvarCorner._numFacesBefore +
                                    fvarCorner._numFacesAfter;
}

void
SurfaceDescriptor::sharpenFVarCorner(int corner,
        CornerSubset const & vtxCorner,
        Index        const   fvarIndices[]) {

    CornerSubset & fvarCorner = _corners[corner];

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
    VertexTopology const & vtxTop = _topology._vertexTopology[corner];

    int faceInVertex = _topology._faceInVertex[corner];

    Index fvarIndex = vtxTop.getFaceVertexAtCorner(faceInVertex, fvarIndices);

    int fvarCount = vtxTop.getNumMatchingCornerIndices(fvarIndex, fvarIndices);

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
            bool fvarDiffersAtVertex = (fvarCount != vtxTop._numFaces);
            bool moreThanTwoCornerFVars = fvarDiffersAtVertex &&
                vtxTop.moreThanTwoUniqueCornerIndices(fvarIndices);

            fvarCorner._isSharp = moreThanTwoCornerFVars;
        }
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2:
        fvarCorner._isSharp = singleFaceSubset;
        if (!fvarCorner._isSharp) {
            bool fvarDiffersAtVertex = (fvarCount != vtxTop._numFaces);
            bool moreThanTwoCornerFVars = fvarDiffersAtVertex &&
                vtxTop.moreThanTwoUniqueCornerIndices(fvarIndices);
            bool concaveCornerSubset = fvarDiffersAtVertex &&
                (fvarCorner._numFacesTotal == (vtxTop._numFaces-1));
            bool singleEdgeDartSubset =
                (fvarCorner._numFacesTotal == vtxTop._numFaces) &&
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
    printf("        face size       = %d\n", top._faceSize);
    printf("        is regular      = %d\n", isRegular);
    if (!isRegular) {
        printf("        has inf-sharp verts  = %d\n", top._hasInfSharpVerts);
        printf("        has semi-sharp verts = %d\n", top._hasSemiSharpVerts);
        printf("        has any sharp edges  = %d\n", top._hasSharpEdges);
        printf("        has unsharp boundary = %d\n", top._hasUnSharpBound);
        printf("        inc irregular faces  = %d\n", top._hasIncIrregFaces);
        printf("        unordered verts      = %d\n", top._hasUnorderedVerts);
        printf("        val-2 interior verts = %d\n", top._hasVal2IntVerts);
    }
    printf("        num-face-verts  = %d\n", top._numFaceVertsTotal);

    if (printVerts) {
        Index const * indices = _indices;

        for (int i = 0; i < top._faceSize; ++i) {
            printf("        corner %d:\n", i);

            VertexTopology const & vTop = top._vertexTopology[i];
            printf("            topology:  num faces  = %d, boundary = %d\n",
                    vTop._numFaces, vTop._isBoundary);

            CornerSubset const & cSub = _corners[i];
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
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
