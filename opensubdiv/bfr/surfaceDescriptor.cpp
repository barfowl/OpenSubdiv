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
//  Minor methods supporting initialization:
//
void
SurfaceDescriptor::initialize(int faceSize, Index const indices[]) {

    assert(!_topology.IsUnsupported());

    _indices = indices;

    _corners.SetSize(faceSize);

    _combinedTag.Clear();

    _isInitialized = true;
    _isRegular     = false;
    _isFaceVarying = false;
    _matchesVertex = true;
}

bool
SurfaceDescriptor::isRegular() const {

    //
    //  Immediate reject features from the combined tags (semi-sharp
    //  vertices, any sharp edges, any irregular face sizes) before
    //  testing valence and topology at each corner:
    //
    if (_combinedTag.HasSharpEdges() ||
        _combinedTag.HasSemiSharpVertices() ||
        _combinedTag.HasIrregularFaceSizes()) {
        return false;
    }

    //
    //  If no boundaries, the interior case can be quickly determined:
    //
    if (!_combinedTag.HasBoundaryVertices()) {
        if (_combinedTag.HasInfSharpVertices()) return false;

        if (_topology._regFaceSize == 4) {
            //  Can use bitwise-OR here for reg valence of 4:
            return (_corners[0]._numFacesTotal |
                    _corners[1]._numFacesTotal |
                    _corners[2]._numFacesTotal |
                    _corners[3]._numFacesTotal) == 4;
        } else {
            return (_corners[0]._numFacesTotal == 6) &&
                   (_corners[1]._numFacesTotal == 6) &&
                   (_corners[2]._numFacesTotal == 6);
        }
    }

    //
    //  Test all corners for appropriate interior or boundary valence:
    //
    int regInteriorValence = (_topology._regFaceSize == 4) ? 4 : 6;
    int regBoundaryValence = (regInteriorValence / 2);

    for (int i = 0; i < _topology.GetFaceSize(); ++i) {
        CornerSubset const & corner = _corners[i];

        if (corner.IsSharp()) {
            if (corner._numFacesTotal != 1) return false;
        } else if (corner.IsBoundary()) {
            if (corner._numFacesTotal != regBoundaryValence) return false;
        } else {
            if (corner._numFacesTotal != regInteriorValence) return false;
        }
    }
    return true;
}

//
//  Main initialization methods -- one for vertex topology and the other
//  for face-varying topology (a subset of the vertex topology):
//
void
SurfaceDescriptor::InitializeVertex(Index const vtxIndices[]) {

    assert(_topology._isFinalized);

    //  Initialize members:
    initialize(_topology.GetFaceSize(), vtxIndices);

    //  Sharpen boundary vertices when warranted:
    bool applyVtxBoundaryInterpolation =
            (_topology._schemeOptions.GetVtxBoundaryInterpolation() ==
                Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);

    //
    //  Inspect each corner and initialize the subset for vertex topology:
    //
    for (int i = 0; i < _topology.GetFaceSize(); ++i) {
        CornerTopology const & cTop = GetCornerTopology(i);
        CornerSubset         & cSub = _corners[i];

        int cornerFace = cTop.GetFaceInVertex();

        //
        //  If the vertex topology was unordered (potentially non-manifold)
        //  the topology for the corners should have been amended with the
        //  neighboring faces, but the specific subset will not have been
        //  determined.
        //
        if (cTop.GetTag().IsOrdered()) {
            //  Boundary and sharpness bits copied from topology here:
            cSub._tag = cTop.GetTag();

            cSub._numFacesTotal = cTop.GetNumFaces();
            cSub._numFacesBefore = cSub.IsBoundary() ? cornerFace : 0;
            cSub._numFacesAfter = cSub._numFacesTotal - cSub._numFacesBefore -1;

            if (!cSub.IsSharp() && (cSub._numFacesTotal == 1)) {
                if (applyVtxBoundaryInterpolation) {
                    cSub.SetSharp(true);
                    cTop.ReviseSubsetTag(cSub._tag);
                }
            }
        } else {
            //  WIP - will need a forward/backward search here
            //      - use a sharp single-face corner for now
            cSub._numFacesTotal  = 1;
            cSub._numFacesBefore = 0;
            cSub._numFacesAfter  = 0;

            cSub._tag = cTop.GetTag();
            cSub.SetBoundary(true);
            cSub.SetSharp(true);

            cTop.ReviseSubsetTag(cSub._tag,
                                 cSub._numFacesBefore, cSub._numFacesAfter,
                                 _topology._regFaceSize);
        }
        _combinedTag.Combine(cSub._tag);
    }

    _isRegular = isRegular();
}

void
SurfaceDescriptor::InitializeFaceVarying(Index const fvarIndices[],
        SurfaceDescriptor const & vtxSurface) {

    assert(_topology._isFinalized);
    assert(&_topology == &vtxSurface._topology);

    //  Initialize members:
    initialize(_topology.GetFaceSize(), fvarIndices);

    _isFaceVarying = true;
    _matchesVertex = true;  // to be adjusted when mismatch detected below

    //
    //  Inspect each corner and initialize its face-varying subset relative
    //  to the corresponding vertex subset:
    //
    Index const * cornerIndices = fvarIndices;

    for (int corner = 0; corner < _topology.GetFaceSize(); ++corner) {
        CornerTopology const & cornerTop = GetCornerTopology(corner);

        CornerSubset const & vtxSub  = vtxSurface.GetCornerSubset(corner);
        CornerSubset       & fvarSub = _corners[corner];

        //
        //  Determine the extent of the fvar subset then determine its
        //  sharpness (according the local face-varying topology and the
        //  assigned interpolation option):
        //
        extendFVarSubset(fvarSub, vtxSub, cornerTop, cornerIndices);

        if (!fvarSub.IsSharp() && fvarSub.IsBoundary()) {
            sharpenFVarSubset(fvarSub, vtxSub, cornerTop, cornerIndices);
        }

        //
        //  If fvar subset matches vertex, all tags copied from vertex
        //  subset will apply, otherwise they will need to be revised to
        //  reflect the reduced extent of the fvar subset:
        //
        bool fvarExtentMatches =
                (fvarSub.IsBoundary()    == vtxSub.IsBoundary()) &&
                (fvarSub._numFacesBefore == vtxSub._numFacesBefore) &&
                (fvarSub._numFacesAfter  == vtxSub._numFacesAfter);

        bool fvarSubsetMatches = fvarExtentMatches &&
                (fvarSub.IsSharp() == vtxSub.IsSharp());

        if (!fvarExtentMatches) {
            cornerTop.ReviseSubsetTag(fvarSub._tag,
                             fvarSub._numFacesBefore, fvarSub._numFacesAfter,
                             _topology._regFaceSize);
        } else if (!fvarSubsetMatches) {
            cornerTop.ReviseSubsetTag(fvarSub._tag);
        }
        _combinedTag.Combine(fvarSub._tag);

        _matchesVertex &= fvarSubsetMatches;

        cornerIndices += cornerTop.GetNumFaceVertices();
    }

    _isRegular = isRegular();
}


//
//  Internal methods supporting face-varying initialization:
//
void
SurfaceDescriptor::extendFVarSubset(CornerSubset         & fvarSub,
                                    CornerSubset   const & vtxSub,
                                    CornerTopology const & cornerTop,
                                    Index          const   fvarIndices[]) {

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
    //  Initialize as boundary until determined otherwise (periodic)
    //
    fvarSub._numFacesAfter = 0;
    fvarSub._numFacesTotal = 0;
    fvarSub._numFacesBefore = 0;

    fvarSub._tag = vtxSub._tag;
    fvarSub.SetBoundary(true);

    //  Skip the following search if only one face:
    if (vtxSub._numFacesTotal == 1) {
        fvarSub._numFacesTotal = 1;
        return;
    }

    //
    //  Inspect/gather faces "after" (counter-clockwise order from)
    //  the corner face.  If all are included and the vtx subset is
    //  periodic, check the seam for the fvar subset.
    //
    int cornerFace = cornerTop.GetFaceInVertex();

    int numFacesAfterToVisit = vtxSub._numFacesAfter;
    if (numFacesAfterToVisit) {
        int thisFace = cornerFace;
        for (int i = 0; i < numFacesAfterToVisit; ++i) {
            int nextFace = cornerTop.GetFaceNext(thisFace);

            if (cornerTop.GetFaceVertexAtCorner(thisFace, fvarIndices) != 
                cornerTop.GetFaceVertexAtCorner(nextFace, fvarIndices)) {
                break;
            }
            if (cornerTop.GetFaceVertexTrailing(thisFace, fvarIndices) != 
                cornerTop.GetFaceVertexLeading(nextFace, fvarIndices)) {
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
        int prevFace = cornerTop.GetFacePrevious(cornerFace);

        if (cornerTop.GetFaceVertexLeading(cornerFace, fvarIndices) == 
            cornerTop.GetFaceVertexTrailing(prevFace, fvarIndices)) {
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
            int prevFace = cornerTop.GetFacePrevious(thisFace);

            if (cornerTop.GetFaceVertexAtCorner(thisFace, fvarIndices) != 
                cornerTop.GetFaceVertexAtCorner(prevFace, fvarIndices)) {
                break;
            }
            if (cornerTop.GetFaceVertexLeading(thisFace, fvarIndices) != 
                cornerTop.GetFaceVertexTrailing(prevFace, fvarIndices)) {
                break;
            }
            ++ fvarSub._numFacesBefore;

            thisFace = prevFace;
        }
    }

    fvarSub._numFacesTotal=fvarSub._numFacesBefore + fvarSub._numFacesAfter + 1;
}


//
//  Local utilities to deal with face-varying assignments at the corner:
//
namespace {
    int
    getNumMatchingCornerIndices(CornerTopology const & corner,
                                Index          const   indices[]) {

        //  WIP - streamline this to increment indices[] by face sizes

        int   cornerFace   = corner.GetFaceInVertex();
        Index indexToMatch = corner.GetFaceVertexAtCorner(cornerFace, indices);

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
SurfaceDescriptor::sharpenFVarSubset(CornerSubset         & fvarSub,
                                     CornerSubset   const & vtxSub,
                                     CornerTopology const & cTop,
                                     Index          const   fvarIndices[]) {

    //
    //  Sharpen if the face-varying topology is non-manifold (i.e. the fvar
    //  index at the corner occurs outside the subset):
    //
    int fvarCount = getNumMatchingCornerIndices(cTop, fvarIndices);

    if (fvarCount > fvarSub._numFacesTotal) {
        fvarSub.SetSharp(true);
        return;
    }

    //
    //  Sharpen according to the face-varying linear interpolation option:
    //
    bool isSharp = false;

    bool fvarIndexIsUnique = (fvarCount == cTop.GetNumFaces());

    switch (_topology._schemeOptions.GetFVarLinearInterpolation()) {
    case Sdc::Options::FVAR_LINEAR_NONE:
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_ONLY:
        //  Sharpen corners only:
        isSharp = (fvarSub._numFacesTotal == 1);
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS1:
        //  Sharpen corners and vertices with three or more fvar indices:
        isSharp = (fvarSub._numFacesTotal == 1);
        if (!fvarSub.IsSharp() && !fvarIndexIsUnique) {
            isSharp = moreThanTwoUniqueCornerIndices(cTop, fvarIndices);
        }
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2:
        //  Sharpen corners, vertices with three or more fvar indices (plus1),
        //  concave corners (two indices with the other unique to one face) and
        //  darts (one discontinuous edge of a periodic set of faces):
        isSharp = (fvarSub._numFacesTotal == 1);
        if (!fvarSub.IsSharp()) {
            if (!fvarIndexIsUnique) {
                isSharp =
                        moreThanTwoUniqueCornerIndices(cTop, fvarIndices) ||
                        (fvarSub._numFacesTotal == (cTop.GetNumFaces() - 1));
            } else {
                isSharp = !vtxSub.IsBoundary() &&
                        (fvarSub._numFacesTotal == cTop.GetNumFaces());
            }
        }
        break;

    case Sdc::Options::FVAR_LINEAR_BOUNDARIES:
        //  Sharpen all boundaries:
        isSharp = true;
        break;

    case Sdc::Options::FVAR_LINEAR_ALL:
        assert("Unexpected FVAR_LINEAR_ALL interpolation" == 0);
        break;

    default:
        assert("Unknown FVarLinearInterpolation value" == 0);
        break;
    }

    fvarSub.SetSharp(isSharp);
}

//
//  Miscellaneous methods for debugging:
//
void
SurfaceDescriptor::print(bool printVerts) const {

    CombinedTag const & tag = _combinedTag;

    printf("    FaceTopology:\n");
    printf("       face size       = %d\n", _topology.GetFaceSize());
    printf("       num-face-verts  = %d\n", _topology.GetNumFaceVertices());
    printf("    Properties:\n");
    printf("       is regular      = %d\n", IsRegular());
    printf("    Combined tags:\n");
    printf("       inf-sharp verts  = %d\n", tag.HasInfSharpVertices());
    printf("       semi-sharp verts = %d\n", tag.HasSemiSharpVertices());
    printf("       any sharp edges  = %d\n", tag.HasSharpEdges());
    printf("       unsharp boundary = %d\n", tag.HasNonSharpBoundary());
    printf("       irregular faces  = %d\n", tag.HasIrregularFaceSizes());
    printf("       unordered verts  = %d\n", tag.HasUnOrderedVertices());
    printf("       val-2 int verts  = %d\n", tag.HasInteriorVal2Vertices());

    if (printVerts) {
        Index const * indices = _indices;

        for (int i = 0; i < _topology.GetFaceSize(); ++i) {
            CornerTopology const & top = GetCornerTopology(i);
            CornerSubset   const & sub = GetCornerSubset(i);

            printf("        corner %d:\n", i);
            printf("            topology:  num faces  = %d, boundary = %d\n",
                    top.GetNumFaces(), top.GetTag().IsBoundary());
            printf("            subset:    num faces  = %d, boundary = %d\n",
                    sub._numFacesTotal, sub.IsBoundary());
            printf("                       num before = %d, num after = %d\n",
                    sub._numFacesBefore, sub._numFacesAfter);

            printf("            face-vert indices:\n");

            for (int j = 0, n = 0; j < top.GetNumFaces(); ++j) {
                printf("            face %d:  ", j);
                int S = top.GetFaceSize(j);
                for (int k = 0; k < S; ++k, ++n) {
                    printf("%3d", indices[n]);
                }
                printf("\n");
            }
            indices += top.GetNumFaceVertices();
        }
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
