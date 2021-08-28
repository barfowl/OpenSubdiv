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

#include "../bfr/regularPatchBuilder.h"


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Trivial constructor/destructor:
//
RegularPatchBuilder::RegularPatchBuilder(SurfaceDescriptor const & surface) :
        _surface(surface) {

    if (_surface.GetTopology()._faceSize == 4) {
        assert(_surface.GetTopology()._regFaceSize == 4);

        _patchSize = 16;
        _patchType = Far::PatchDescriptor::REGULAR;
    } else {
        assert(_surface.GetTopology()._faceSize == 3);
        assert(_surface.GetTopology()._regFaceSize == 3);

        _patchSize = 12;
        _patchType = Far::PatchDescriptor::LOOP;
    }
}

//
//  Methods to determine the boundary mask from assigned control point
//  indices -- which should have (-1) as indices for phantom points
//  (which will be replaced by a corner index later):
//
int
RegularPatchBuilder::GetBoundaryMask() const {

    assert(_patchSize == 16);

    CornerSubset const * C = _surface.GetSubsets();
    return ((C[0].IsBoundary() & (C[0]._numFacesBefore == 0)) << 0) |
           ((C[1].IsBoundary() & (C[1]._numFacesBefore == 0)) << 1) |
           ((C[2].IsBoundary() & (C[2]._numFacesBefore == 0)) << 2) |
           ((C[3].IsBoundary() & (C[3]._numFacesBefore == 0)) << 3);
}

//
//  Methods for gathering control vertices, faces, sharpness, etc. -- the
//  method to gather control vertex indices is for external use, while the
//  rest are internal:
//
int
RegularPatchBuilder::gatherPatchPoints4(Index patchPoints[]) const {

    //Index fvPhantom = -1;
    Index fvPhantom = _surface.GetIndices()[0];

    //  WIP - currently passing in full set of indices for all face
    //  corners, but may gather them locally in future:
    Index const * fvIndices = &_surface.GetIndices()[0];

    Index * P = patchPoints;
    for (int i = 0; i < 4; ++i) {
        CornerTopology const & cTop = _surface.GetCornerTopology(i);
        CornerSubset   const & cSub = _surface.GetCornerSubset(i);

        int faceCorner = cTop.GetFaceInVertex();
        Index const *fvCorner = &fvIndices[faceCorner * 4];

        switch (i) {
        case 0:
            P[5] = fvCorner[0];
            if (!cSub.IsBoundary()) {
                int faceOpposite = (faceCorner + 2) & 3;
                Index const *fvOpposite = &fvIndices[faceOpposite * 4];
                P[4] = fvOpposite[1];
                P[0] = fvOpposite[2];
                P[1] = fvOpposite[3];
            } else {
                int faceOther = cSub._numFacesAfter ?
                                cTop.GetFaceNext(faceCorner) :
                                cTop.GetFacePrevious(faceCorner);
                Index const *fvOther = &fvIndices[faceOther * 4];
                P[4] = cSub._numFacesAfter  ? fvOther[3] : fvPhantom;
                P[0] = fvPhantom;
                P[1] = cSub._numFacesBefore ? fvOther[1] : fvPhantom;
            }
            break;
        case 1:
            P[6] = fvCorner[0];
            if (!cSub.IsBoundary()) {
                int faceOpposite = (faceCorner + 2) & 3;
                Index const *fvOpposite = &fvIndices[faceOpposite * 4];
                P[2] = fvOpposite[1];
                P[3] = fvOpposite[2];
                P[7] = fvOpposite[3];
            } else {
                int faceOther = cSub._numFacesAfter ?
                                cTop.GetFaceNext(faceCorner) :
                                cTop.GetFacePrevious(faceCorner);
                Index const *fvOther = &fvIndices[faceOther * 4];
                P[2] = cSub._numFacesAfter  ? fvOther[3] : fvPhantom;
                P[3] = fvPhantom;
                P[7] = cSub._numFacesBefore ? fvOther[1] : fvPhantom;
            }
            break;
        case 2:
            P[10] = fvCorner[0];
            if (!cSub.IsBoundary()) {
                int faceOpposite = (faceCorner + 2) & 3;
                Index const *fvOpposite = &fvIndices[faceOpposite * 4];
                P[11] = fvOpposite[1];
                P[15] = fvOpposite[2];
                P[14] = fvOpposite[3];
            } else {
                int faceOther = cSub._numFacesAfter ?
                                cTop.GetFaceNext(faceCorner) :
                                cTop.GetFacePrevious(faceCorner);
                Index const *fvOther = &fvIndices[faceOther * 4];
                P[11] = cSub._numFacesAfter  ? fvOther[3] : fvPhantom;
                P[15] = fvPhantom;
                P[14] = cSub._numFacesBefore ? fvOther[1] : fvPhantom;
            }
            break;
        case 3:
            P[9] = fvCorner[0];
            if (!cSub.IsBoundary()) {
                int faceOpposite = (faceCorner + 2) & 3;
                Index const *fvOpposite = &fvIndices[faceOpposite * 4];
                P[13] = fvOpposite[1];
                P[12] = fvOpposite[2];
                P[ 8] = fvOpposite[3];
            } else {
                int faceOther = cSub._numFacesAfter ?
                                cTop.GetFaceNext(faceCorner) :
                                cTop.GetFacePrevious(faceCorner);
                Index const *fvOther = &fvIndices[faceOther * 4];
                P[13] = cSub._numFacesAfter  ? fvOther[3] : fvPhantom;
                P[12] = fvPhantom;
                P[ 8] = cSub._numFacesBefore ? fvOther[1] : fvPhantom;
            }
            break;
        }
        fvIndices += cTop.GetNumFaceVertices();
    }
    return 16;
}

int
RegularPatchBuilder::gatherPatchPoints3(Index patchPoints[]) const {

    if (patchPoints) {
        assert("gatherRegularPatchPoints3() not yet supported" == 0);
    }
    return 12;
}

int
RegularPatchBuilder::GatherControlVertexIndices(Index cvIndices[]) const {

    return (_patchSize == 16) ? gatherPatchPoints4(cvIndices) :
                                gatherPatchPoints3(cvIndices);
}


//
//  Methods for debugging...
//
void
RegularPatchBuilder::print() const {

    assert(_surface.GetIndices() != 0);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
