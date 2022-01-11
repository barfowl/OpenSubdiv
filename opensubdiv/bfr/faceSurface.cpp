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

#include "../bfr/faceSurface.h"

#include <cstring>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Initialization utilities for both vertex and face-varying surfaces:
//
void
FaceSurface::preInitialize(FaceTopology const & faceTopology,
                           Index        const   faceIndices[]) {

    //
    //  Initialize members, allocate subsets for the corners and clear
    //  tags combining features of all corners:
    //
    _topology = &faceTopology;
    _indices  = faceIndices;

    _isFaceVarying = false;
    _matchesVertex = false;

    _corners.SetSize(GetFaceSize());

    _combinedTag.Clear();
}

void
FaceSurface::postInitialize() {

    //
    //  Determine if the surface is regular and if not, filter options
    //  that are not being used (to avoid them falsely indicating that
    //  two similar surfaces are different):
    //
    _isRegular = isRegular();

    _optionsInEffect = GetSdcOptionsAsAssigned();
    if (!_isRegular) {
        reviseSdcOptionsInEffect();
    }
}

//
//  Initializers for vertex and face-varying surfaces:
//
void
FaceSurface::Initialize(FaceTopology const & vtxTopology,
                        Index const          vtxIndices[]) {

    preInitialize(vtxTopology, vtxIndices);

    _isFaceVarying = false;

    //  WIP - we could reduce the subset by seeking delimiting inf-sharp
    //        edges, but not in the presence of a dart
    bool useInfSharpSubsets = _topology->GetTag().HasInfSharpEdges() &&
                             !_topology->GetTag().HasInfSharpDarts();

    //
    //  For each corner, identify the manifold subset containing the face
    //  and sharpen according to the vertex boundary interpolation option
    //  if warranted.  Meanwhile, accumulate the combined set of tags for
    //  all corners:
    //
    for (int corner = 0; corner < GetFaceSize(); ++corner) {
        CornerTopology const & vtxTop = GetCornerTopology(corner);
        CornerSubset         & vtxSub = _corners[corner];

        //  We can initialize the subset from the full topology in some
        //  cases, but others require search (non-manifold, unordered):
        CornerTag vtxTopTag = vtxTop.GetTag();
        if (vtxTopTag.IsOrdered()) {
            vtxTop.InitializeCompleteSubset(&vtxSub);
        } else if (!vtxTopTag.IsManifold() || vtxTopTag.IsBoundary()) {
            vtxTop.FindConnectedSubset(&vtxSub);
        } else {
            vtxTop.InitializeCompleteSubset(&vtxSub);
        }

        if (vtxSub.IsBoundary() && !vtxSub.IsSharp()) {
            sharpenBySdcVtxBoundaryInterpolation(&vtxSub, vtxTop);
        }
        if (useInfSharpSubsets && vtxTopTag.HasInfSharpEdges()) {
            //  WIP - potentially reduce to a smaller subset here
        }
        _combinedTag.Combine(vtxSub._tag);
    }
    postInitialize();
}

void
FaceSurface::Initialize(FaceSurface const  & vtxSurface,
                        Index const          fvarIndices[]) {

    preInitialize(*vtxSurface._topology, fvarIndices);

    _isFaceVarying = true;

    //
    //  For each corner, find the face-varying subset of the vertex subset
    //  and sharpen according to the face-varying interpolation option if
    //  warranted.  Meanwhile, accumulate the combined set of tags for all
    //  corners, and whether the face-varying topology matches the vertex
    //  for all corners:
    //
    for (int corner = 0; corner < GetFaceSize(); ++corner) {
        CornerTopology const & vtxTop  = GetCornerTopology(corner);
        CornerSubset const   & vtxSub  = vtxSurface.GetCornerSubset(corner);
        CornerSubset         & fvarSub = _corners[corner];

        vtxTop.FindFaceVaryingSubset(&fvarSub, fvarIndices, vtxSub);

        if (fvarSub.IsBoundary() && !fvarSub.IsSharp()) {
            sharpenBySdcFVarLinearInterpolation(&fvarSub, fvarIndices,
                    vtxSub, vtxTop);
        }
        _combinedTag.Combine(fvarSub._tag);

        _matchesVertex &= fvarSub.MatchesShapeOfSuperset(vtxSub);

        fvarIndices += vtxTop.GetNumFaceVertices();
    }
    postInitialize();
}

//
//  Minor methods supporting initialization:
//
bool
FaceSurface::isRegular() const {

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

        if (GetRegFaceSize() == 4) {
            //  Can use bitwise-OR here for reg valence of 4:
            return (_corners[0].GetNumFaces() |
                    _corners[1].GetNumFaces() |
                    _corners[2].GetNumFaces() |
                    _corners[3].GetNumFaces()) == 4;
        } else {
            return (_corners[0].GetNumFaces() == 6) &&
                   (_corners[1].GetNumFaces() == 6) &&
                   (_corners[2].GetNumFaces() == 6);
        }
    }

    //
    //  Test all corners for appropriate interior or boundary valence:
    //
    int regInteriorValence = (GetRegFaceSize() == 4) ? 4 : 6;
    int regBoundaryValence = (regInteriorValence / 2);

    for (int i = 0; i < GetFaceSize(); ++i) {
        CornerSubset const & corner = _corners[i];

        if (corner.IsSharp()) {
            if (corner.GetNumFaces() != 1) return false;
        } else if (corner.IsBoundary()) {
            if (corner.GetNumFaces() != regBoundaryValence) return false;
        } else {
            if (corner.GetNumFaces() != regInteriorValence) return false;
        }
    }
    return true;
}

void
FaceSurface::reviseSdcOptionsInEffect() {

    //
    //  "Override" (ignore, set to default) any options not affecting
    //  the shape of the limit surface.  The boundary and face-varying
    //  interpolation options are fixed/ignored for all cases.  Whether
    //  other options have an effect depends on the topology present.
    //
    //  This is done, in part, to make accurate comparisons between
    //  the topologies of two surfaces.  For example, the presence of
    //  differing creasing methods should not lead to two topologically
    //  identical surfaces with no creasing being considered different.
    //
    //  This is to be used on construction on irregular surfaces AFTER
    //  the combined tags have been determined.
    //
    assert(!_isRegular);

    CombinedTag const & tags = _combinedTag;

    Sdc::Options & options = _optionsInEffect;

    //  Boundary and face-varying interpolation fixed/ignored for all:
    options.SetVtxBoundaryInterpolation(Sdc::Options::VTX_BOUNDARY_EDGE_ONLY);
    options.SetFVarLinearInterpolation( Sdc::Options::FVAR_LINEAR_ALL);

    //  Crease-method ignored when no semi-sharp creasing:
    if (options.GetCreasingMethod() != Sdc::Options::CREASE_UNIFORM) {
        if (!tags.HasSemiSharpEdges() && !tags.HasSemiSharpVertices()) {
            options.SetCreasingMethod(Sdc::Options::CREASE_UNIFORM);
        }
    }

    //  Catmark triangle smoothing ignored if not Catmark with triangles:
    if (options.GetTriangleSubdivision() != Sdc::Options::TRI_SUB_CATMARK) {
        //  This is slightly stronger than necessary -- will keep the
        //  tri-smooth setting if Catmark and any non-quads:
        if ((GetSdcScheme() != Sdc::SCHEME_CATMARK) ||
                !tags.HasIrregularFaceSizes()) {
            options.SetTriangleSubdivision(Sdc::Options::TRI_SUB_CATMARK);
        }
    }

    //  Non-default values of any future options will warrant attention
}


//
//  Internal methods to apply the Sdc boundary interpolation options for
//  vertex and face-varying topology:
//
void
FaceSurface::sharpenBySdcVtxBoundaryInterpolation(CornerSubset * vtxSub,
        CornerTopology const & vtxTop) const {

    assert(vtxSub->IsBoundary() && !vtxSub->IsSharp());

    //
    //  Sharpen according to Sdc::Options::VtxBoundaryInterpolation:
    //
    //  Remember vertex boundary interpolation is applied based on the
    //  full topology of the vertex not a particular subset (e.g. we can
    //  have a smooth corner in a subset delimited by inf-sharp edges).
    //  And edges are all implicitly sharpened -- leaving only corners to
    //  be sharpened -- making the EDGE_ONLY and EDGE_AND_CORNER names
    //  somewhat misleading.
    //
    bool isSharp = false;

    switch (_topology->_schemeOptions.GetVtxBoundaryInterpolation()) {
    case Sdc::Options::VTX_BOUNDARY_NONE:
        //  Nothing to do, as the name suggests
        break;

    case Sdc::Options::VTX_BOUNDARY_EDGE_ONLY:
        //  Edges are implicitly sharpened -- nothing more to do
        break;

    case Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER:
        //  Edges are implicitly sharpened -- sharpen any corners
        isSharp = (vtxTop.GetNumFaces() == 1);
        break;

    default:
        assert("Unknown value for Sdc::Options::VtxBoundaryInterpolation" == 0);
        break;
    }

    if (isSharp) {
        vtxTop.SharpenSubset(vtxSub);
    }
}

namespace {
    //  Local function to support application of face-varying interpolation:
    bool
    moreThanTwoFVarIndicesAtCorner(CornerTopology const & topology,
                                   Index          const   fvarIndices[]) {

        Index iFirst  = fvarIndices[0];
        Index iSecond = -1;

        for (int corner = 1; corner < topology.GetNumFaces(); ++corner) {
            Index index = topology.GetFaceVertexAtCorner(corner, fvarIndices);
            if (index == iFirst) continue;
            if (index == iSecond) continue;
            if (iSecond >= 0) return true;
            iSecond = index;
        }
        return false;
    }
}

void
FaceSurface::sharpenBySdcFVarLinearInterpolation(CornerSubset * fvarSub,
        Index          const   fvarIndices[],
        CornerSubset   const & vtxSub,
        CornerTopology const & vtxTop) const {

    assert(fvarSub->IsBoundary() && !fvarSub->IsSharp());

    //
    //  Sharpen according to Sdc::Options::FVarLinearInterpolation:
    //
    //  Most cases are trivial, with the LINEAR_CORNERS_PLUS* cases being
    //  the only ones that require inspection outside the subset.
    //
    bool isSharp = false;

    switch (_topology->_schemeOptions.GetFVarLinearInterpolation()) {
    case Sdc::Options::FVAR_LINEAR_NONE:
        //  Nothing to do, as the name suggests
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_ONLY:
        //  Sharpen corners only:
        isSharp = (fvarSub->GetNumFaces() == 1);
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS1:
        //
        //  Sharpen corners and vertices with three or more fvar indices --
        //  and so three or more disjoint fvar regions:
        //
        isSharp = (fvarSub->GetNumFaces() == 1) ||
                  moreThanTwoFVarIndicesAtCorner(vtxTop, fvarIndices);
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2:
        //
        //  Sharpen corners, vertices with three or more fvar regions (see
        //  "plus1" above), concave corners or darts.
        //
        //  In other words, the only situations unsharpened are when either
        //  the fvar and vertex subsets exactly match, or there are two
        //  disjoint subsets that both have two or more faces.
        //
        isSharp = (fvarSub->GetNumFaces() == 1) ||
                  moreThanTwoFVarIndicesAtCorner(vtxTop, fvarIndices);
        if (!isSharp) {
            int numFacesLess = vtxSub.GetNumFaces() - fvarSub->GetNumFaces();
            if (numFacesLess > 0) {
                //  Sharpen if the only other potential subset is a corner
                isSharp = (numFacesLess == 1);
            } else {
                //  Sharpen if a dart created from a periodic vertex subset
                isSharp = !vtxSub.IsBoundary();
            }
        }
        break;

    case Sdc::Options::FVAR_LINEAR_BOUNDARIES:
        //  Sharpen all boundaries:
        isSharp = true;
        break;

    case Sdc::Options::FVAR_LINEAR_ALL:
        assert("Unexpected FVarLinearInterpolation == FVAR_LINEAR_ALL" == 0);
        break;

    default:
        assert("Unknown value for Sdc::Options::FVarLinearInterpolation" == 0);
        break;
    }

    if (isSharp) {
        vtxTop.SharpenSubset(fvarSub);
    }
}

//
//  Miscellaneous methods for debugging:
//
void
FaceSurface::print(bool printVerts) const {

    CombinedTag const & tag = _combinedTag;

    printf("    FaceTopology:\n");
    printf("       face size       = %d\n", _topology->GetFaceSize());
    printf("       num-face-verts  = %d\n", _topology->GetNumFaceVertices());
    printf("    Properties:\n");
    printf("       is regular      = %d\n", IsRegular());
    printf("    Combined tags:\n");
    printf("       inf-sharp verts  = %d\n", tag.HasInfSharpVertices());
    printf("       semi-sharp verts = %d\n", tag.HasSemiSharpVertices());
    printf("       inf-sharp edges  = %d\n", tag.HasInfSharpEdges());
    printf("       semi-sharp edges = %d\n", tag.HasSemiSharpEdges());
    printf("       inf-sharp darts  = %d\n", tag.HasInfSharpDarts());
    printf("       unsharp boundary = %d\n", tag.HasNonSharpBoundary());
    printf("       irregular faces  = %d\n", tag.HasIrregularFaceSizes());
    printf("       unordered verts  = %d\n", tag.HasUnOrderedVertices());

    if (printVerts) {
        Index const * indices = _indices;

        for (int i = 0; i < GetFaceSize(); ++i) {
            CornerTopology const & top = GetCornerTopology(i);
            CornerSubset   const & sub = GetCornerSubset(i);

            printf("        corner %d:\n", i);
            printf("            topology:  num faces  = %d, boundary = %d\n",
                    top.GetNumFaces(), top.GetTag().IsBoundary());
            printf("            subset:    num faces  = %d, boundary = %d\n",
                    sub.GetNumFaces(), sub.IsBoundary());
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
