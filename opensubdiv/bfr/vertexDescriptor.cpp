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

#include "../bfr/vertexDescriptor.h"
#include "../bfr/limits.h"
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
bool
VertexDescriptor::Initialize(int numFaces) {

    //  Mark invalid if too many or too few incident faces specified:
    if (numFaces > Limits::MaxValence()) {
        _numFaces = (short) Limits::MaxValence();
        _isValid  = false;
    } else {
        _numFaces = (short) numFaces;
        _isValid  = (numFaces > 0);
    }

    //  Initialize all other members regardless of the above:
    _vertSharpness = 0.0f;

    _isOrdered  = false;
    _isBoundary = false;

    _hasFaceSizes     = true;
    _hasEdgeSharpness = false;
    _wasFaceSizesSet  = false;

    _isInitialized = true;
    _isFinalized   = false;

    return _isValid;
}

bool
VertexDescriptor::Finalize() {

    //
    //  Test for a number of possible errors here and fail, e.g.:
    //      - invalid or uninitialized
    //      - whether face sizes common not set (don't rely on default)
    //      - face sizes expected but not present
    //      - edge sharpness expected but not present
    //
    //  WIP - failure could set an error code for inspection
    //
    if (!_isValid || !_isInitialized) return false;

    if (!_wasFaceSizesSet) return false;

    if (_hasFaceSizes && (_faceSizeOffsets.GetSize() == 0)) return false;

    //  Convert the N face sizes to N+1 offsets (total face-vertices last):
    if (_hasFaceSizes) {
        int sum = 0;
        for (int i = 0; i < _numFaces; ++i) {
            int nextSum = sum + _faceSizeOffsets[i];
            _faceSizeOffsets[i] = sum;
            sum = nextSum;
        }
        _faceSizeOffsets[_numFaces] = sum;
    }

    _isFinalized = true;

    return true;
}

//
//  Internal methods for resizing local buffers:
//
void
VertexDescriptor::initFaceSizes() {

    _faceSizeOffsets.SetSize(_numFaces + 1);
    std::memset(_faceSizeOffsets, 0, (_numFaces + 1) * sizeof(int));
    _hasFaceSizes = true;
}

void
VertexDescriptor::initEdgeSharpness() {

    _faceEdgeSharpness.SetSize(_numFaces * 2);
    std::memset(_faceEdgeSharpness, 0, (_numFaces * 2) * sizeof(float));
    _hasEdgeSharpness = true;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
