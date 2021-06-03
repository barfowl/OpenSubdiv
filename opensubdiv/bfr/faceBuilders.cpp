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

#include "../bfr/faceBuilders.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {
namespace internal {


//
//  Support for the RegularFaceBuilder:
//
RegularFaceBuilder::PatchType
RegularFaceBuilder::GetPatchType() const {

    switch (_patchSize) {
        case  3: return Far::PatchDescriptor::TRIANGLES;
        case  4: return Far::PatchDescriptor::QUADS;
        case 12: return Far::PatchDescriptor::LOOP;
        case 16: return Far::PatchDescriptor::REGULAR;
        default:
            assert("Invalid patch size for regular patch" == 0);
            return Far::PatchDescriptor::NON_PATCH;
    }
}

namespace {
    //
    //  Local utilities for dealing with arrays of indices for the Regular
    //  descriptor/builder -- supporting retrieval/filterning of both vertex
    //  and face-varying indices:
    //
    inline int
    getPatchIndices(int pSize, int const pSrc[], int boundary, int pDst[]) {

        if (boundary == 0) {
            std::memcpy(pDst, pSrc, pSize * sizeof(int));
        } else {
            int phantom = (pSize == 16) ? pSrc[5] : pSrc[4];
            for (int i = 0; i < pSize; ++i) {
                pDst[i] = (pSrc[i] < 0) ? phantom : pSrc[i];
            }
        }
        return pSize;
    }

    inline int
    getFaceIndices(int pSize, int const pSrc[], int fSize, int fDst[]) {

        if (pSize <= 4) {
            std::memcpy(fDst, pSrc, fSize * sizeof(int));
        } else if (pSize == 16) {
            fDst[0] = pSrc[ 5];
            fDst[1] = pSrc[ 6];
            fDst[2] = pSrc[10];
            fDst[3] = pSrc[ 9];
        } else {
            fDst[0] = pSrc[4];
            fDst[1] = pSrc[5];
            fDst[2] = pSrc[8];
        }
        return fSize;
    }
}

int
RegularFaceBuilder::GetFVarPatchPointIndices(int indices[]) const {

    return getPatchIndices(_patchSize, _fvarPatchPoints, _boundaryMask, indices);
}
int
RegularFaceBuilder::GetPatchPointIndices(int indices[]) const {

    return getPatchIndices(_patchSize, _patchPoints, _boundaryMask, indices);
/*
    if (_boundaryMask == 0) {
        std::memcpy(patchPoints, _patchPoints, _patchSize * sizeof(int));
    } else {
        int phantom = (_patchSize == 16) ? _patchPoints[5] : _patchPoints[4];
        for (int i = 0; i < _patchSize; ++i) {
            patchPoints[i] = (_patchPoints[i] < 0) ? phantom : _patchPoints[i];
        }
    }
    return _patchSize;
*/
}

int
RegularFaceBuilder::GetFaceFVarValueIndices(int indices[]) const {

    return getFaceIndices(_patchSize, _fvarPatchPoints, _faceSize, indices);
}
int
RegularFaceBuilder::GetFaceVertexIndices(int indices[]) const {

    return getFaceIndices(_patchSize, _patchPoints, _faceSize, indices);
/*
    if (_patchSize <= 4) {
        std::memcpy(fVerts, _patchPoints, _faceSize * sizeof(int));
    } else if (_patchSize == 16) {
        fVerts[0] = _patchPoints[ 5];
        fVerts[1] = _patchPoints[ 6];
        fVerts[2] = _patchPoints[10];
        fVerts[3] = _patchPoints[ 9];
    } else {
        fVerts[0] = _patchPoints[4];
        fVerts[1] = _patchPoints[5];
        fVerts[2] = _patchPoints[8];
    }
    return _faceSize;
*/
}

int
RegularFaceBuilder::GetControlVertexIndices(int cVerts[]) const {

    if (_patchSize == 4) {
        std::memcpy(cVerts, _patchPoints, 4 * sizeof(int));
        return 4;
    }

    //  WIP - move these to statics within the derived RegularFaceDescriptor
    //      - also, adjust triangular patch ordering to match Manifold
    int const order16[] = { 5,6,10,9, 4,0,1,2,3,7,11,15,14,13,12,8 };
    int const order12[] = { 4,5, 8,   7,3,0,1,2,6, 9,11,10 };

    int const * order = (_patchSize == 16) ? order16 : order12;

    if (_boundaryMask == 0) {
        for (int i = 0; i < _patchSize; ++i) {
            cVerts[i] = _patchPoints[order[i]];
        }
        return _patchSize;
    } else {
        int numCVs = 0;
        for (int i = 0; i < _patchSize; ++i) {
            int patchPointIndex = _patchPoints[order[i]];
            if (patchPointIndex >= 0) {
                cVerts[numCVs++] = patchPointIndex;
            }
        }
        return numCVs;
    }
}

int
RegularFaceBuilder::GetLocalPatchPoints(int localPatchPoints[]) const {

    if (_patchSize <= 4) {
        localPatchPoints[0] = 0;
        localPatchPoints[1] = 1;
        localPatchPoints[2] = 2;
        if (_patchSize > 3) {
            localPatchPoints[3] = 3;
        }
        return _patchSize;
    }

    if (_boundaryMask == 0) {
        int const local16[] = { 5,6,7,8, 4,0,1,9, 15,3,2,10, 14,13,12,11 };
        int const local12[] = { 5,6,7, 4,0,1,8, 3,2,9, 11,10 };

        int const * local = (_patchSize == 16) ? local16 : local12;

        std::memcpy(localPatchPoints, local, _patchSize * sizeof(int));
        return _patchSize;
    } else {
        //  WIP - move these to statics within the derived RegularFaceDescriptor
        int const order16[] = { 5,6,10,9, 4,0,1,2,3,7,11,15,14,13,12,8 };
        int const order12[] = { 4,5, 8,   7,3,0,1,2,6, 9,11,10 };

        int const * order = (_patchSize == 16) ? order16 : order12;

        int numCVerts = 0;
        for (int i = 0; i < _patchSize; ++i) {
            int index = order[i];
            localPatchPoints[index] = (_patchPoints[index] < 0) ? 0 : numCVerts++;
        }
        return _patchSize;
    }
}


//
//  Support for the ManifoldFaceBuilder:
//
int
ManifoldFaceBuilder::GetFaceVertexIndices(int fVerts[]) const {

    for (int i = 0; i < _faceSize; ++i) {
        fVerts[i] = _corners[i]._vertexIndex;
    }
    return _faceSize;
}

int
ManifoldFaceBuilder::GetFaceFVarValueIndices(int fvarValues[]) const {

    if (!_fvarIndicesSet) return -1;

    for (int i = 0; i < _faceSize; ++i) {
        fvarValues[i] = _corners[i]._fvarValueIndex;
    }
    return _faceSize;
}

int
ManifoldFaceBuilder::GetControlVertexIndices(int cVerts[], int rotation) const {

    //  Assign CV indices from the base face -- accounting for rotation:
    int numCVerts = 0;
    for (int i = 0; i < _faceSize; ++i) {
        Corner const & C = _corners[(rotation + i) % _faceSize];

        cVerts[numCVerts++] = C._vertexIndex;
    }

    //  Assign CV indices from each corner -- accounting for rotation:
    for (int i = 0; i < _faceSize; ++i) {
        Corner const & C = _corners[(rotation + i) % _faceSize];

        int N = C._numIncFaces;
        int M = C._faceInRing;

        assert(_constFaceSize);
        int S = _constFaceSize ? _constFaceSize : C._incFaceSizes[i];

        //  Identify the start of this corner's local subset of the ring:
        int leadingFace = 2;
        if (C._isBoundary) {
            if (M == (N - 1)) {
                leadingFace = 0;
            } else if (M == (N - 2)) {
                leadingFace = N;
            } else {
                leadingFace = M + 2;
            }
        }
        assert(S);
        int localStart = leadingFace * (S - 2);

        //
        //  Gather the contiguous or dis-joint subset from the ring:
        //
        if (C._faceInRing == 0) {
            Index const * localVerts = &C._ringVertIndices[localStart];
            for (int j = 0; j < C._numLocalVerts; ++j) {
                cVerts[numCVerts++] = localVerts[j];
            }
            /*
            int const * src = &C._ringVertIndices[localStart];
            int       * dst = &cVerts[numCVerts];
            std::memcpy(dst, src, C._numLocalVerts * sizeof(int));
            numCVerts += C._numLocalVerts;
            */
        } else {
            //  Need the ring size here -- consider computing this during
            //  finalization and store as member given the required loop:
            assert(S);
            int ringSize = N * (S - 2) + C._isBoundary;

            Index const * ringVerts = &C._ringVertIndices[0];
            int ringIndex = localStart;
            for (int j = 0; j < C._numLocalVerts; ++j) {
                cVerts[numCVerts++] = ringVerts[ringIndex++];
                if (ringIndex == ringSize) ringIndex = 0;
            }
        }
    }
    return numCVerts;
}

int
ManifoldFaceBuilder::GetLocalFaceVertices(int faceVerts[], int faceSizes[]) const {

    //
    //  Assign the base face and its size as the first face:
    //
    int numFaces = 1;
    faceSizes[0] = _faceSize;

    int numFaceVerts = 0;
    for (int i = 0; i < _faceSize; ++i) {
        faceVerts[numFaceVerts++] = i;
    }

    int cvCount = GetNumControlVertices();

    //
    //  Assign faces "local" to each corner -- most will consist of the
    //  corner vertex and remaining vertices from the surrouding ring:
    //
    int cvStart = _faceSize;
    for (int cornerIndex = 0; cornerIndex < _faceSize; ++cornerIndex) {
        Corner const & C = _corners[cornerIndex];

        int N = C._numIncFaces;
        int M = C._faceInRing;

        //
        //  The base face and the face immediately following it in the ring
        //  are not considered "local" to this corner, so skip those two
        //  faces (or only one if the face is on a boundary) and identify
        //  the first face of the ring to be assigned:
        //
        int localStart = M + 2;
        if (localStart >= N) {
            localStart = C._isBoundary ? 0 : (localStart - N);
        }

        int cvIndex = cvStart;
        for (int localFace = 0; localFace < C._numLocalFaces; ++localFace) {
            bool lastFace = (localFace == (C._numLocalFaces - 1));

            int ringFace = localStart + localFace;
            if (ringFace == N) ringFace = 0;

            assert(_constFaceSize);
            int S = _constFaceSize ? _constFaceSize : C._incFaceSizes[ringFace];
            faceSizes[numFaces++] = S;

            //
            //  Assign starting corner vertex followed by all vertices of the
            //  incident face except the last two -- which in general will 
            //  both be from the ring, but will differ at the end of the ring:
            //
            faceVerts[numFaceVerts++] = cornerIndex;

            //  Advance CV index when starting the leading face of a boundary:
            if (C._isBoundary && (ringFace == 0) && (M < (N - 1))) {
                cvIndex++;
            }

            if (S == 4) {
                faceVerts[numFaceVerts++] = cvIndex++;
            } else if (S > 4) {
                for (int k = 0; k < (S - 3); ++k) {
                    faceVerts[numFaceVerts++] = cvIndex++;
                }
            }

            //
            //  The last face of the ring will share an edge with the base
            //  face (unless that edge is a boundary), and the vertex that
            //  precedes the edge may wrap to the next corner:
            //
            if (lastFace && (!C._isBoundary || (M > 0))) {
                if (cornerIndex == (_faceSize - 1)) {
                    faceVerts[numFaceVerts++] = _faceSize;
                    faceVerts[numFaceVerts++] = 0;
                } else {
                    faceVerts[numFaceVerts++] = cvIndex;
                    faceVerts[numFaceVerts++] = cornerIndex + 1;
                }
            } else {
                faceVerts[numFaceVerts++] = cvIndex++;
                if (cvIndex == cvCount) cvIndex = _faceSize;
                faceVerts[numFaceVerts++] = cvIndex;
            }
        }
        cvStart += C._numLocalVerts;
    }
    return numFaceVerts;
}

TopologyCache::Key
ManifoldFaceBuilder::ComputeTopologyKey() const {

    //
    //  The Key computation is going to change significantly -- especially
    //  once sharpness values can be added.
    //
    //  Even without any explicit sharpness values assigned, dealing with
    //  incident irregular faces is a pain (the valence of a corner alone
    //  is not enough as the irregular faces must be identified in each
    //  ring).
    //
    //  It's also unclear "who" should compute the keys...  Either the
    //  FaceBuilders know enough about the key structure to do so (as is
    //  the case here), or only the TopologyCache knows and is required
    //  to make generic queries of the FaceBuilders to construct them.
    //
    //  For now, we only compute keys for manifold interior faces that do
    //  not involve any irregular incident faces and that have a reasonably
    //  low valence (that will fit in 8 bits):
    //
    TopologyCache::Key key;
    if ((_faceSize > 4) || (_constFaceSize == 0) ||
        _corners[0]._isBoundary || (_corners[0]._numIncFaces > 255) ||
        _corners[1]._isBoundary || (_corners[1]._numIncFaces > 255) ||
        _corners[2]._isBoundary || (_corners[2]._numIncFaces > 255) ||
        _corners[3]._isBoundary || (_corners[3]._numIncFaces > 255)) {
        key.hashBits = 0;
    } else {
        key.hashBits  = _corners[0]._numIncFaces <<  0;
        key.hashBits |= _corners[1]._numIncFaces <<  8;
        key.hashBits |= _corners[2]._numIncFaces << 16;
        if (_faceSize == 4) {
            key.hashBits |= _corners[3]._numIncFaces << 24;
        }
//key.hashBits = 0;
    }
    return key;
}

} // end namespace internal
} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
