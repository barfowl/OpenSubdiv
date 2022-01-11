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

#include "../bfr/vertexTopology.h"
#include "../sdc/crease.h"

#include <cstring>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Main initialize/finalize methods used by clients to delimit the
//  assignment (most work is now handled by the containing class):
//
void
VertexTopology::Initialize(int numFaces) {

    assert(numFaces > 0);
    _numFaces = numFaces;

    _vertSharpness = 0.0f;

    _isOrdered  = false;
    _isBoundary = false;

    _hasFaceSizes     = true;
    _hasEdgeSharpness = false;
    _wasFaceSizesSet  = false;

    _isInitialized = true;
    _isFinalized   = false;
}

void
VertexTopology::Finalize() {

    assert(_isInitialized);

    //
    //  Should test for errors here and fail, e.g.:
    //      - whether face sizes common not set (don't rely on default)
    //      - face sizes expected but not present
    //      - edge sharpness expected but not present
    //
    assert(_wasFaceSizesSet);

    if (_hasFaceSizes)     assert(_faceSizeOffsets.GetSize() > 0);
    if (_hasEdgeSharpness) assert(_faceEdgeSharpness.GetSize() > 0);

    //  Convert the N face sizes to N+1 offsets and assign face-verts:
    if (_hasFaceSizes) {
        //  WIP - worth testing if all same size and ignoring if so
        int sum = 0;
        for (int i = 0; i < _numFaces; ++i) {
            //  WIP - test face size for degenerate (< 3) here and tag
            //      - may want to defer this conversion for this reason
            int nextSum = sum + _faceSizeOffsets[i];
            _faceSizeOffsets[i] = sum;
            sum = nextSum;
        }
        _faceSizeOffsets[_numFaces] = sum;
    }

    _isFinalized = true;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
