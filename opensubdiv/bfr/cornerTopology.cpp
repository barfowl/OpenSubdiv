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

#include "../sdc/crease.h"
#include "../bfr/cornerTopology.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Main pre-initialize and post-finalize methods used to delimit assignment
//  by clients (who use the base class initialize and finalize methods, which
//  now do very little in light of these):
//
void
CornerTopology::Initialize(int faceSize) {

    _tag.Clear();

    _commonFaceSize = faceSize;
    _numFaceVerts   = 0;

    _vTop._isInitialized = false;
}

void
CornerTopology::Finalize(int regFaceSize, int faceInVertex) {

    //
    //  Test and transfer tags:
    //
    assert(_vTop._isFinalized);

    _tag._unOrderedFaces  = !_vTop.IsOrdered();

    _tag._boundaryVerts = _vTop.IsBoundary();

    _tag._interiorVal2Verts = !_vTop.IsBoundary() && (_vTop._numFaces == 2);

    //
    //  Deal with face sizes and number of face-vertices first:
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

    _tag._irregularFaceSizes = (_commonFaceSize != regFaceSize);

    //
    //  Deal with vertex sharpness -- simply assign tags
    //
    _tag._infSharpVerts  = Sdc::Crease::IsInfinite(_vTop._vertSharpness);
    _tag._semiSharpVerts = (_vTop._vertSharpness > 0) && !_tag._infSharpVerts;

    //
    //  Deal with edge sharpness:
    //
    _tag._boundaryNonSharp = _tag._boundaryVerts;

    _tag._anySharpEdges = _vTop.HasEdgeSharpness();
    if (_tag._anySharpEdges) {
        int numSharpness = _vTop._numFaces * 2;
        if (_tag._boundaryVerts) {
            int last = numSharpness - 1;
            _tag._boundaryNonSharp =
                    !Sdc::Crease::IsInfinite(_vTop._faceEdgeSharpness[0]) ||
                    !Sdc::Crease::IsInfinite(_vTop._faceEdgeSharpness[last]);

            _vTop._faceEdgeSharpness[0]    = 0.0f;
            _vTop._faceEdgeSharpness[last] = 0.0f;
        }

        //  WIP - note we need to look for 3 or more inf-sharp edges (only
        //  one for a boundary) to tag this vertex as inf-sharp as a result

        //  Ignore assigned edge sharpness if all zero:
        _tag._anySharpEdges = false;
        for (int i = 0; i < numSharpness; ++i) {
            if (_vTop._faceEdgeSharpness[i] > 0.0f) {
                _tag._anySharpEdges = true;
                break;
            }
        }
    }

    _faceInRing = faceInVertex;
}

//
//  Method to connect unordered faces to allow for topological
//  traversals of the incident faces:
//
void
CornerTopology::ConnectUnOrderedFaces(Index const fvIndices[]) {

    assert(fvIndices);
}

//
//  Method to revise the tags for a subset of the corner, which may no
//  longer include properties that trigger exceptional behavior:
//
void
CornerTopology::ReviseSubsetTag(CornerTag & subsetTag) const {

    //  Adjust simple bits for change in boundary or sharpness:
    if (subsetTag.IsBoundary()) {
        subsetTag._interiorVal2Verts = false;
    }
    if (subsetTag.IsInfSharp()) {
        subsetTag._semiSharpVerts = false;
    }
    subsetTag._nonManifoldVerts = false;
}

void
CornerTopology::ReviseSubsetTag(CornerTag & subsetTag,
        int numFacesBefore, int numFacesAfter, int regFaceSize) const {

    //  Adjust simple bits for change in boundary or sharpness:
    ReviseSubsetTag(subsetTag);

    //
    //  There are two cases to deal with:
    //      - possibility of irregular faces within the subset
    //      - possibility of sharp edges within the subset
    //  Both are subject to conditions that can quickly reject
    //  the full iteration through the faces of the subset.
    //
    int  numFacesInSubset = numFacesBefore + 1 + numFacesAfter;
    bool numFacesIsFewer  = (numFacesInSubset < GetNumFaces());

    if (subsetTag._irregularFaceSizes) {
        if (numFacesIsFewer) {
            if (subsetTag._unCommonFaceSizes) {
                subsetTag._irregularFaceSizes = false;

                //  Search for faces with irregular size:
                int face = GetFaceBefore(numFacesBefore);

                for (int i = 0; i < numFacesInSubset; ++i) {
                    if (GetFaceSize(face) != regFaceSize) {
                        subsetTag._irregularFaceSizes = true;
                        break;
                    }
                    face = GetFaceNext(face);
                }
            } else {
                subsetTag._irregularFaceSizes = true;
            }
        }
    }

    if (subsetTag._anySharpEdges) {
        if (numFacesIsFewer || !_tag.IsBoundary()) {
            if (numFacesInSubset > 1) {
                subsetTag._anySharpEdges = false;

                //  Search for faces whose leading edges were sharpened,
                //  skipping the first face of a boundary subset (whose
                //  leading edge is a boundary):
                int face = GetFaceBefore(numFacesBefore);

                bool isBoundary = subsetTag.IsBoundary();
                if (isBoundary) face = GetFaceNext(face);

                for (int i = isBoundary; i < numFacesInSubset; ++i) {
                    if (GetFaceEdgeSharpness(face,0) > 0.0f) {
                        subsetTag._anySharpEdges = true;
                        break;
                    }
                    face = GetFaceNext(face);
                }
            } else {
                subsetTag._anySharpEdges = false;
            }
        }
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
