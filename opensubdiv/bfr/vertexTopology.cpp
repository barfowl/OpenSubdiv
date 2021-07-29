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

#include "../bfr/vertexTopology.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Main initialize/finalize methods used by clients to delimit assignment:
//
void
VertexTopology::Initialize(int numFaces) {

    _isInitialized = true;
    _isFinalized   = false;

    _isOrdered    = false;
    _isBoundary   = false;
    _isInterior   = false;
    _hasSharpVert = false;
    _hasSharpEdge = false;

    _numFaces = numFaces;
    _commonFaceSize = 0;
    _numFaceVerts = 0;
}

void
VertexTopology::Finalize() {

    assert(_isInitialized);

    //
    //  WIP - most of this is likely moving to a class that either
    //  is-a or has-a VertexTopology instance.  A little more tagging
    //  will be done for val-2 verts and degenerate faces.
    //
    if (_commonFaceSize) {
        _numFaceVerts = _numFaces * _commonFaceSize;
    } else {
        assert(_faceSizeOffsets.GetSize() > 0);

        //  Convert N face sizes to N+1 offsets and assign face-verts:
        int sum = 0;
        for (int i = 0; i < _numFaces; ++i) {
            //  WIP - test face size for degenerate (< 3) here and tag
            int nextSum = sum + _faceSizeOffsets[i];
            _faceSizeOffsets[i] = sum;
            sum = nextSum;
        }
        _faceSizeOffsets[_numFaces] = sum;

        _numFaceVerts = _faceSizeOffsets[_numFaces];
    }

    if (_hasSharpEdge) {
        int numSharpness = _numFaces * 2;
        if (_isBoundary) {
            //  WIP - detect if boundary was sharpened before clearing
            _faceEdgeSharpness[0] = 0.0f;
            _faceEdgeSharpness[numSharpness-1] = 0.0f;
        }

        //  Ignore assigned edge sharpness if all zero:
        _hasSharpEdge = false;
        for (int i = 0; i < numSharpness; ++i) {
            if (_faceEdgeSharpness[i] > 0.0f) {
                _hasSharpEdge = true;
                break;
            }
        }
    }

    _isFinalized = true;
}

bool
VertexTopology::moreThanTwoUniqueCornerIndices(Index const indices[]) const {

    //
    //  This is primarily used for face-varying indices -- where any
    //  more than three unique values is irrelant:

    //  WIP - potentially streamline this to increment indices[] by
    //  face sizes, especially when face-size is constant

    Index index1 = getFaceVertexAtCorner(0, indices);
    Index index2 = -1;

    for (int i = 1; i < _numFaces; ++i) {
        Index index = getFaceVertexAtCorner(i, indices);
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

int
VertexTopology::getNumMatchingCornerIndices(Index indexToMatch,
                                            Index const indices[]) const {

    //  WIP - streamline this to increment indices[] by face sizes

    int numMatches = 0;
    for (int i = 0; i < _numFaces; ++i) {
        if (getFaceVertexAtCorner(i, indices) == indexToMatch) {
            numMatches ++;
        }
    }
    return numMatches;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
