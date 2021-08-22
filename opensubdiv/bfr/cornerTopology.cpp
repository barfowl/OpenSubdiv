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

    _tags.Clear();

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

    _tags._unOrderedFaces  = !_vTop.IsOrdered();

    _tags._boundaryVerts = _vTop.IsBoundary();
    if (_tags._boundaryVerts) {
        _tags._boundaryCorners = (_vTop._numFaces == 1);
    } else {
        _tags._interiorVal2Verts = (_vTop._numFaces == 2);
    }

    //
    //  Deal with face sizes and number of face-vertices first:
    //
    _tags._unCommonFaceSizes = !_vTop.HasCommonFaceSize();
    if (_tags._unCommonFaceSizes) {
        //  WIP - consider testing for degenerate faces (< 3) here and tag
        //      - may want to convert sizes to offsets here to combine
        _numFaceVerts = _vTop._faceSizeOffsets[_vTop._numFaces];
        _commonFaceSize = 0;
    } else {
        _numFaceVerts = _vTop._numFaces * _commonFaceSize;
    }

    _tags._irregularFaceSizes = (_commonFaceSize != regFaceSize);

    //
    //  Deal with vertex sharpness -- simply assign tags
    //
    _tags._infSharpVerts  = Sdc::Crease::IsInfinite(_vTop._vertSharpness);
    _tags._semiSharpVerts = (_vTop._vertSharpness > 0) && !_tags._infSharpVerts;

    //
    //  Deal with edge sharpness:
    //
    _tags._boundaryNonSharp = _tags._boundaryVerts;

    _tags._anySharpEdges = _vTop.HasEdgeSharpness();
    if (_tags._anySharpEdges) {
        int numSharpness = _vTop._numFaces * 2;
        if (_tags._boundaryVerts) {
            int last = numSharpness - 1;
            _tags._boundaryNonSharp =
                    !Sdc::Crease::IsInfinite(_vTop._faceEdgeSharpness[0]) ||
                    !Sdc::Crease::IsInfinite(_vTop._faceEdgeSharpness[last]);

            _vTop._faceEdgeSharpness[0]    = 0.0f;
            _vTop._faceEdgeSharpness[last] = 0.0f;
        }

        //  WIP - note we need to look for 3 or more inf-sharp edges (only
        //  one for a boundary) to tag this vertex as inf-sharp as a result

        //  Ignore assigned edge sharpness if all zero:
        _tags._anySharpEdges = false;
        for (int i = 0; i < numSharpness; ++i) {
            if (_vTop._faceEdgeSharpness[i] > 0.0f) {
                _tags._anySharpEdges = true;
                break;
            }
        }
    }

    _faceInRing = faceInVertex;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
