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

#include <cstdio>

#include "../bfr/faceDescriptors.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {


//
//  Support for the RegularFaceDescriptor:
//
void
RegularFaceDescriptor::Initialize(int patchSize) {

    switch (patchSize) {
    case  3:  _faceSize = 3; break;
    case  4:  _faceSize = 4; break;
    case 12:  _faceSize = 3; break;
    case 16:  _faceSize = 4; break;
    default:  _faceSize = 0; break;
    }

    _patchSize = patchSize;

    _boundaryMask    = 0;
    _boundaryMaskSet = false;

    _numCVs = 0;

    _finalized = false;
}

bool
RegularFaceDescriptor::Finalize() {

    if (!_boundaryMaskSet) {
        _boundaryMask = 0;
        assert(!_boundaryMaskSet);
    }

    if (_boundaryMask == 0) {
        _numCVs = _patchSize;
    } else {
        _numCVs = 0;
        for (int i = 0; i < _patchSize; ++i) {
            _numCVs += (_patchPoints[i] >= 0);
        }
    }

    _finalized = true;

    return true;
}


//
//  Support for the ManifoldFaceDescriptor:
//
void
ManifoldFaceDescriptor::Initialize(int faceSize, bool constFaceSize) {

    _faceSize = faceSize;
    _constFaceSize = constFaceSize ? _faceSize : 0;

    _corners.SetSize(_faceSize);

    for (int i = 0; i < _faceSize; ++i) {
        _corners[i]._numIncFaces = 0;
        _corners[i]._vertexIndex = -1;
        _corners[i]._isBoundary  = false;
        _corners[i]._faceInRing  = 0;
    }

    _numVerts     = 0;
    _numFaces     = 0;
    _numFaceVerts = 0;

    _finalized = false;
}

bool
ManifoldFaceDescriptor::Finalize() {

    //
    //  Compute members for internal use:
    //
    _numVerts     = _faceSize;
    _numFaces     = 1;
    _numFaceVerts = _faceSize;

    for (int i = 0; i < _faceSize; ++i) {
        Corner & C = _corners[i];

        int N = C._numIncFaces;
        int M = C._faceInRing;

        assert(_constFaceSize);
        int S = _constFaceSize;

        if (C._isBoundary) {
            C._numLocalVerts = 0;
            C._numLocalVerts += (M == (N-1)) ? 0 : ((N-M-2)*(S-2)+1);
            C._numLocalVerts += (M ==   0  ) ? 0 : ((M-1)*(S-2)+(S-3));

            C._numLocalFaces = N - 2 + (M == (N - 1));
        } else {
            C._numLocalVerts = (N - 2) * (S - 2) - 1;

            C._numLocalFaces = N - 2;
        }

        _numVerts     += C._numLocalVerts;
        _numFaces     += C._numLocalFaces;
        _numFaceVerts += C._numLocalFaces * S;
    }
    _finalized = true;
    return true;
}


//
//  Support for the temporary/obsolete NonManifoldFaceDescriptor:
//
void
NonManifoldFaceDescriptor::Initialize() {

    _data.faceSizes.clear();
    _data.faceVerts.clear();
    _data.faceFVarValues.clear();
    _data.creaseIndices.clear();
    _data.creaseSharpness.clear();
    _data.cornerIndices.clear();
    _data.cornerSharpness.clear();
    _data.faceHoles.clear();

    _topology.numVertices = 0;
    _topology.numFaces    = 0;

    _topology.numVertsPerFace        = 0;
    _topology.vertIndicesPerFace     = 0;
    _topology.creaseVertexIndexPairs = 0;
    _topology.creaseWeights          = 0;
    _topology.cornerVertexIndices    = 0;
    _topology.cornerWeights          = 0;
    _topology.holeIndices            = 0;

    _controlVertices.clear();

    _finalized = false;
}

void
NonManifoldFaceDescriptor::Finalize() {

    int nVerts = (int) _controlVertices.size();
    int nFaces = (int) _data.faceSizes.size();

    _topology.numVertices = nVerts;
    _topology.numFaces    = nFaces;

    _topology.numVertsPerFace    = &_data.faceSizes[0];
    _topology.vertIndicesPerFace = &_data.faceVerts[0];

    _topology.numCreases = _data.creaseSharpness.size();
    if (_topology.numCreases) {
        _topology.creaseVertexIndexPairs = &_data.creaseIndices[0];
        _topology.creaseWeights          = &_data.creaseSharpness[0];
    } else {
        _topology.creaseVertexIndexPairs = 0;
        _topology.creaseWeights          = 0;
    }

    _topology.numCorners = _data.cornerSharpness.size();
    if (_topology.numCorners) {
        _topology.cornerVertexIndices = &_data.cornerIndices[0];
        _topology.cornerWeights       = &_data.cornerSharpness[0];
    } else {
        _topology.cornerVertexIndices = 0;
        _topology.cornerWeights       = 0;
    }

    _topology.numHoles = nFaces - 1;
    _data.faceHoles.resize(nFaces - 1);
    for (int i = 1; i < nFaces; ++i) {
        _data.faceHoles[i-1] = i;
    }
    _topology.holeIndices = (nFaces > 1) ? &_data.faceHoles[0] : 0;

    _topology.isLeftHanded = false;

    _finalized = true;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
