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

#include "../bfr/irregularPatchBuilder.h"
#include "../far/topologyDescriptor.h"
#include "../far/topologyRefiner.h"
#include "../bfr/patchTree.h"
#include "../bfr/patchTreeFactory.h"

#include <cstring>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Local namespace with utilities to help deal with complications arising
//  from the presence of valence-2 interior vertices.
//
//  Interior vertices with valence-2 significantly complicate construction
//  of the control hull due to the way that incident faces overlap with
//  the base face. The builder identifies the presence of such vertices on
//  construction, records the status as a member and makes use of these
//  utilities as needed.
//
//  WIP - more work is needed to support val-2 interior vertices
//      - some cases of laminar faces are not correctly represented
//      - will likely construct the control vertices differently
//
namespace val2 {
    //
    //  Simple query if a FaceVertexSubset is val-2 interior:
    //
    bool
    subsetIsVal2Interior(FaceVertexSubset const & corner) {
        return (corner.GetNumFaces() == 2) && !corner.IsBoundary();
    }

    //
    //  Count the number of val-2 interior corners that follow the given
    //  corner (ignore if this corner is val-2 interior):
    //
    int
    getFaceOverlap(FaceSurface const & surface, int corner) {

        FaceVertexSubset const * corners = &surface.GetCornerSubset(0);

        int numOverlap = 0;
        if (!subsetIsVal2Interior(corners[corner])) {
            int numCorners = surface.GetFaceSize();
            for (int i = 1; i < numCorners; ++i, ++numOverlap) {
                if (!subsetIsVal2Interior(corners[(corner+i) % numCorners])) {
                    break;
                }
            }
        }
        return numOverlap;
    }
}


//
//  Trivial constructor -- initializes members related to the control hull:
//
IrregularPatchBuilder::IrregularPatchBuilder(
        FaceSurface const & surfaceDescription, Options options) :
            _surface(surfaceDescription),
            _options(options) {

    initializeControlHullInventory();
}

//
//  The IrregularPatchBuilder assembles a control hull for the base face
//  from the topology information given for each corner of the face.  It
//  first initializes the number of control vertices and faces required,
//  along with the contributions of each from the corners of the face.
//
//  What should be a relatively straightforward task is unfortunately
//  complicated by special cases -- typically involving pathologically
//  low valence (e.g. valence-2 interior vertices) that cause adjacent
//  corner faces to overlap with the face itself.
//
void
IrregularPatchBuilder::initializeControlHullInventory() {

    //
    //  Determine if we have any val-2 interior vertices -- which wreak
    //  havoc on the control hull perimeter as it overlaps the faces:
    //
    int faceSize = _surface.GetFaceSize();

    _hasVal2IntCorners = false;
    for (int corner = 0; corner < faceSize; ++corner) {
        FaceVertexSubset const & cSub = _surface.GetCornerSubset(corner);
        _hasVal2IntCorners |= (cSub.GetNumFaces() == 2) && !cSub.IsBoundary();
    }

    //
    //  Iterate through the corners to identify the vertices, faces and
    //  face-vertices that contribute to the collective control hull:
    //
    int numVal3IntAdjTris = 0;
    int numVal2IntCorners = 0;
    int numSrcFaceIndices = 0;

    _cornerControlInfo.SetSize(faceSize);

    _numControlVerts     = faceSize;
    _numControlFaces     = 1;
    _numControlFaceVerts = faceSize;

    for (int corner = 0; corner < faceSize; ++corner) {
        FaceVertex       const & cTop = _surface.GetCornerTopology(corner);
        FaceVertexSubset const & cSub = _surface.GetCornerSubset(corner);

        //
        //  Need to keep track of corners at and adjacent to valence-2
        //  interior corners to detect and avoid overlaps:
        //
        int numVal2Overlap = _hasVal2IntCorners ?
                             val2::getFaceOverlap(_surface, corner) : 0;

        //
        //  Inspect faces after the corner face first -- dealing with a few
        //  special cases for interior vertices of low valence -- followed
        //  by those faces before the corner face:
        //
        CornerControl & cControl = _cornerControlInfo[corner];
        cControl.Clear();

        int numCornerFaceVerts = 0;

        if (cSub._numFacesAfter) {
            int nextFace = cTop.GetFaceNext(cTop.GetFace());

            if (cSub.IsBoundary()) {
                //  Boundary -- no special cases:
                for (int i = 1; i < cSub._numFacesAfter; ++i) {
                    nextFace = cTop.GetFaceNext(nextFace);
                    int S = cTop.GetFaceSize(nextFace);

                    cControl.numVerts  += S - 2;
                    numCornerFaceVerts += S;
                }
                cControl.numFaces = cSub._numFacesAfter - 1;
                //  Include unshared vertex of trailing edge
                cControl.numVerts ++;
            } else if (cSub._numFacesTotal == 2) {
                //  Interior, valence-2 -- special case:
                if (++numVal2IntCorners == faceSize) {
                    cControl.singleCommonFace = true;
                    cControl.numFaces  = 1;
                    numCornerFaceVerts = faceSize;
                }
            } else if ((cSub._numFacesTotal == 3) &&
                    (cTop.GetFaceSize(cTop.GetFaceAfter(2)) == 3)) {
                //  Interior, valence-3, adjacent triangle -- special case:
                if (++numVal3IntAdjTris == faceSize) {
                    cControl.singleCommonVert = true;
                    cControl.numVerts = 1;
                }
                cControl.numFaces  = 1;
                numCornerFaceVerts = 3;
            } else if (cSub._numFacesTotal > 2) {
                //  Interior -- general case:
                for (int i = 2; i < cSub._numFacesTotal; ++i) {
                    nextFace = cTop.GetFaceNext(nextFace);
                    int S = cTop.GetFaceSize(nextFace);

                    cControl.numVerts  += S - 2;
                    numCornerFaceVerts += S;
                }
                cControl.numFaces = cSub._numFacesTotal - 2;
                //  Exclude vertex shared with/contributed by next corner
                cControl.numVerts --;
            }
        }
        if (cSub._numFacesBefore) {
            assert(cSub.IsBoundary());
            int nextFace = cTop.GetFaceFirst(cSub);

            for (int i = 0; i < cSub._numFacesBefore; ++i) {
                int S = cTop.GetFaceSize(nextFace);
                nextFace = cTop.GetFaceNext(nextFace);

                cControl.numVerts  += S - 2;
                numCornerFaceVerts += S;
            }
            cControl.numFaces += cSub._numFacesBefore;
            //  Exclude vertex shared with/contributed by next corner
            cControl.numVerts --;
        }

        //  Account for the overlap with valence-2 interior vertices:
        if (numVal2Overlap) {
            assert(cControl.numVerts >= numVal2Overlap);
            cControl.numVerts -= numVal2Overlap;
        }

        //  Assign the contributions for this corner:
        cControl.nextPerimeterVert = _numControlVerts;
        cControl.nextSrcFaceIndex  = numSrcFaceIndices;

        _numControlVerts     += cControl.numVerts;
        _numControlFaces     += cControl.numFaces;
        _numControlFaceVerts += numCornerFaceVerts;

        numSrcFaceIndices += cTop.GetNumFaceVertices();
    }
}

//
//  Methods for gathering control vertices, faces, sharpness, etc. -- the
//  method to gather control vertex indices is for external use, while the
//  rest are internal:
//
int
IrregularPatchBuilder::GatherControlVertexIndices(Index cvIndices[]) const {

    //
    //  Assign CV indices from the base face first:
    //
    int N = _surface.GetFaceSize();

    Index const * faceIndices = _surface.GetIndices();

    FaceVertex const & cTop0 = _surface.GetCornerTopology(0);
    int baseOffset = cTop0.GetFaceIndexOffset(cTop0.GetFace());

    Index const * baseIndices = &faceIndices[baseOffset];
    std::memcpy(cvIndices, baseIndices, N * sizeof(Index));

    int numIndices = N;

    //
    //  Assign vertex indices identified as contributed by each corner:
    //
    for (int corner = 0; corner < N; ++corner) {
        CornerControl const & cControl = _cornerControlInfo[corner];

        if (cControl.numVerts == 0) continue;

        FaceVertex       const & cTop = _surface.GetCornerTopology(corner);
        FaceVertexSubset const & cSub = _surface.GetCornerSubset(corner);

        Index const * srcIndices = faceIndices + cControl.nextSrcFaceIndex;

        //  Special case with all val-3 interior triangles:
        if (cControl.singleCommonVert) {
            assert(!cSub.IsBoundary() && (cSub._numFacesTotal == 3) &&
                   (cTop.GetFaceSize(cTop.GetFaceAfter(2)) == 3));
            int fvOffset = cTop.GetFaceIndexOffset(cTop.GetFaceAfter(2));

            cvIndices[numIndices++] = srcIndices[fvOffset + 1];
            continue;
        }

        //  Detect and consider valence-2 interior overlap:
        int nVal2Overlap = _hasVal2IntCorners ?
                           val2::getFaceOverlap(_surface, corner) : 0;

        //
        //  Deal with the faces after the base faces first, followed by
        //  those that precede it:
        //
        if (cSub._numFacesAfter) {
            //  Be careful not to skip the last face entirely if it is the
            //  only one of a boundary as we need its trailing edge:
            int numFaces = cSub._numFacesAfter - 1;
            int nextFace = cTop.GetFaceAfter(1);
            for (int j = 0; j < numFaces; ++j) {
                nextFace = cTop.GetFaceNext(nextFace);

                int S = cTop.GetFaceSize(nextFace);
                int fvOffset = cTop.GetFaceIndexOffset(nextFace);

                int L = ((j < (numFaces-1)) || cSub.IsBoundary()) ?
                        0 : (1 + nVal2Overlap);
                int M = (S - 2) - L;
                for (int k = 1; k <= M; ++k) {
                    cvIndices[numIndices++] = srcIndices[fvOffset + k];
                }
            }
            //  Include trailing edge for boundary before crossing the gap:
            if (cSub.IsBoundary()) {
                cvIndices[numIndices++] =
                    cTop.GetFaceIndexTrailing(nextFace, srcIndices);
            }
        }
        if (cSub._numFacesBefore) {
            int numFaces = cSub._numFacesBefore;
            int nextFace = cTop.GetFaceFirst(cSub);
            for (int j = 0; j < numFaces; ++j) {
                int S = cTop.GetFaceSize(nextFace);
                int fvOffset = cTop.GetFaceIndexOffset(nextFace);

                int L = (j < (numFaces-1)) ? 0 : (1 + nVal2Overlap);

                int M = (S - 2) - L;
                for (int k = 1; k <= M; ++k) {
                    cvIndices[numIndices++] = srcIndices[fvOffset + k];
                }
                nextFace = cTop.GetFaceNext(nextFace);
            }
        }
    }
    assert(numIndices == _numControlVerts);
    return numIndices;
}

int
IrregularPatchBuilder::gatherControlFaceSizes(int faceSizes[]) const {

    //
    //  If all control faces are a common size, fill array and return:
    //
    int baseFaceSize = _surface.GetFaceSize();

    if (!_surface.GetTag().HasUnCommonFaceSizes()) {
        std::fill(faceSizes, faceSizes + _numControlFaces, baseFaceSize);
        return _numControlFaces;
    }

    //
    //  Start with the base face first, return if that's all:
    //
    faceSizes[0] = baseFaceSize;

    if (_numControlFaces == 1) return 1;

    //
    //  Otherwise, traverse all corners and populate control face sizes:
    //
    int nFaces = 1;

    for (int corner = 0; corner < _surface.GetFaceSize(); ++corner) {
        CornerControl const & cControl = _cornerControlInfo[corner];

        if (cControl.numFaces == 0) continue;

        FaceVertex       const & cTop = _surface.GetCornerTopology(corner);
        FaceVertexSubset const & cSub = _surface.GetCornerSubset(corner);

        //
        //  If the subset has a common face size, this is trivial:
        //
        if (!cSub._tag.HasUnCommonFaceSizes()) {
            int N = cControl.numFaces;
            for (int i = 0; i < N; ++i) {
                faceSizes[nFaces++] = baseFaceSize;
            }
        } else if (cControl.singleCommonFace) {
            //
            //  Special case of a single adjacent laminar face:
            //
            assert(cControl.numFaces == 1);
            assert(cSub._numFacesTotal == 2);

            faceSizes[nFaces++] = _surface.GetFaceSize();
        } else {
            //
            //  The general case - identify face sizes after and before:
            //
            if (cSub._numFacesAfter > 1) {
                int face = cTop.GetFaceAfter(2);
                for (int i = 1; i < cSub._numFacesAfter; ++i) {
                    faceSizes[nFaces++] = cTop.GetFaceSize(face);

                    face = cTop.GetFaceNext(face);
                }
            }
            if (cSub._numFacesBefore) {
                int face = cTop.GetFaceFirst(cSub);
                for (int i = 0; i < cSub._numFacesBefore; ++i) {
                    faceSizes[nFaces++] = cTop.GetFaceSize(face);

                    face = cTop.GetFaceNext(face);
                }
            }
        }
    }
    return _numControlFaces;
}

int
IrregularPatchBuilder::gatherControlVertexSharpness(
        int vertIndices[], float vertSharpness[]) const {

    int nSharpVerts = 0;
    for (int i = 0; i < _surface.GetFaceSize(); ++i) {
        FaceVertexSubset const & cSub = _surface.GetCornerSubset(i);

        if (cSub._tag.IsInfSharp()) {
            vertSharpness[nSharpVerts] = Sdc::Crease::SHARPNESS_INFINITE;
            vertIndices[nSharpVerts++] = i;
        } else if (cSub._tag.IsSemiSharp()) {
            vertSharpness[nSharpVerts] = (cSub._localSharpness > 0.0f) ? 
                        cSub._localSharpness :
                        _surface.GetCornerTopology(i).GetVertexSharpness();
            vertIndices[nSharpVerts++] = i;
        }
    }
    return nSharpVerts;
}

int
IrregularPatchBuilder::gatherControlEdgeSharpness(
        int edgeVertPairs[], float edgeSharpness[]) const {

    //
    //  First test the forward edge of each corner of the face (avoid
    //  including redundant inf-sharp boundary edges):
    //
    int nSharpEdges = 0;

    int faceSize = _surface.GetFaceSize();

    for (int corner = 0; corner < faceSize; ++corner) {
        FaceVertex       const & cTop = _surface.GetCornerTopology(corner);
        FaceVertexSubset const & cSub = _surface.GetCornerSubset(corner);

        if (cSub._tag.HasSharpEdges() &&
                (!cSub.IsBoundary() || cSub._numFacesBefore)) {
            int   cornerFace = cTop.GetFace();
            float sharpness  = cTop.GetFaceEdgeSharpness(cornerFace, 0);
            if (Sdc::Crease::IsSharp(sharpness)) {
                *edgeSharpness++ = sharpness;
                *edgeVertPairs++ =  corner;
                *edgeVertPairs++ = (corner + 1) % faceSize;
                nSharpEdges++;
            }
        }
    }

    //
    //  For each corner, test any interior edges connected to vertices
    //  on the perimeter:
    //
    for (int corner = 0; corner < faceSize; ++corner) {
        CornerControl const & cControl = _cornerControlInfo[corner];

        if (cControl.numFaces == 0) continue;

        FaceVertex       const & cTop = _surface.GetCornerTopology(corner);
        FaceVertexSubset const & cSub = _surface.GetCornerSubset(corner);

        if (!cSub._tag.HasSharpEdges()) continue;

        int cornerFace = cTop.GetFace();

        //
        //  Inspect interior edges of the subset -- test sharpness of
        //  the trailing edge of the faces after/before the corner face.
        //
        //  Unfortunately we need the control vertex index at the end
        //  of the edge, and so we need to track the perimeter -- which
        //  requires the face sizes and may wrap around with tris. The
        //  need to increment the perimeter also requires dealing with
        //  the after and before faces separately (vs iterating over
        //  the entire subset):
        //
        int maxVert  = _numControlVerts;
        int nextVert = cControl.nextPerimeterVert;

        if (cSub._numFacesAfter) {
            int nextFace = cTop.GetFaceNext(cornerFace);
            for (int i = 1; i < cSub._numFacesAfter; ++i) {
                float sharpness = cTop.GetFaceEdgeSharpness(nextFace, 1);
                if (Sdc::Crease::IsSharp(sharpness)) {
                    *edgeSharpness++ = sharpness;
                    *edgeVertPairs++ = corner;
                    *edgeVertPairs++ = (nextVert < maxVert)
                                     ? nextVert : faceSize;
                    nSharpEdges++;
                }
                nextFace  = cTop.GetFaceNext(nextFace);
                nextVert += cTop.GetFaceSize(nextFace) - 2;
            }
            nextVert += cSub.IsBoundary();
        }
        if (cSub._numFacesBefore) {
            int nextFace = cTop.GetFaceFirst(cSub);
            for (int i = 1; i < cSub._numFacesBefore; ++i) {
                nextVert += cTop.GetFaceSize(nextFace) - 2;
                float sharpness = cTop.GetFaceEdgeSharpness(nextFace, 1);
                if (Sdc::Crease::IsSharp(sharpness)) {
                    *edgeSharpness++ = sharpness;
                    *edgeVertPairs++ = corner;
                    *edgeVertPairs++ = (nextVert < maxVert)
                                     ? nextVert : faceSize;
                    nSharpEdges++;
                }
                nextFace  = cTop.GetFaceNext(nextFace);
            }
        }
    }
    return nSharpEdges;
}

int
IrregularPatchBuilder::gatherControlFaceVertices(int faceVertices[]) const {

    //
    //  Assign face-vertices for the first/base face:
    //
    int * nextFaceVert = faceVertices;

    int faceSize = _surface.GetFaceSize();

    for (int i = 0; i < faceSize; ++i) {
        *nextFaceVert++ = i;
    }

    //
    //  Assign face-vertex indices for faces "local to" each corner:
    //
    for (int corner = 0; corner < faceSize; ++corner) {
        CornerControl const & cControl = _cornerControlInfo[corner];

        if (cControl.numFaces == 0) continue;

        FaceVertex       const & cTop = _surface.GetCornerTopology(corner);
        FaceVertexSubset const & cSub = _surface.GetCornerSubset(corner);

        int nVal2Overlap = _hasVal2IntCorners ?
                           val2::getFaceOverlap(_surface, corner) : 0;

        //
        //  Special case for single adjacent laminar face:
        //
        if (cControl.singleCommonFace) {
            assert(_hasVal2IntCorners && val2::subsetIsVal2Interior(cSub));
            //  Must be the special case of the reversed/laminar base face:
            for (int j = faceSize - 1; j >= 0; --j) {
                *nextFaceVert ++ = j;
            }
            continue;
        }

        //
        //  WIP - the following blocks for faces before and after the base
        //        face can probably be merged into one iteration -- provided
        //        the transition across the boundary discontinuity is handled
        //        properly (reset next face, adjust next vertex, etc.)
        //
        int nextVert = cControl.nextPerimeterVert;

        if (cSub._numFacesAfter > 1) {
            int nextFace = cTop.GetFaceAfter(2);

            int N = cSub._numFacesAfter - 1;
            for (int j = 0; j < N; ++j) {
                int S = cTop.GetFaceSize(nextFace);

                if (cSub.IsBoundary()) {
                    getControlFaceVertices(nextFaceVert, S, corner, nextVert);
                } else {
                    getControlFaceVertices(nextFaceVert, S, corner, nextVert,
                            (j == (N - 1)), nVal2Overlap);
                }

                nextFaceVert  += S;
                nextVert      += S - 2;
                nextFace       = cTop.GetFaceNext(nextFace);
            }
        }
        if (cSub._numFacesAfter && cSub.IsBoundary()) {
            nextVert ++;
        }
        if (cSub._numFacesBefore) {
            int nextFace = cTop.GetFaceFirst(cSub);

            int N = cSub._numFacesBefore;
            for (int j = 0; j < N; ++j) {
                int S = cTop.GetFaceSize(nextFace);

                getControlFaceVertices(nextFaceVert, S, corner, nextVert,
                        (j == (N - 1)), nVal2Overlap);

                nextFaceVert += S;
                nextVert     += S - 2;
                nextFace      = cTop.GetFaceNext(nextFace);
            }
        }
    }
    assert((nextFaceVert - faceVertices) == _numControlFaceVerts);
    return _numControlFaceVerts;
}


//
//  Methods to gather the local face-vertices for a particular incident
//  face of a corner.  The first is the trivial case -- where all vertices
//  other than the initial corner vertex lie on the perimeter and do not
//  wrap around.  The second deals handles more of the complications and
//  is used in all cases where the trivial method cannot be applied.
//
void
IrregularPatchBuilder::getControlFaceVertices(int fVerts[], int numFVerts,
        int corner, int nextPerimeterVert) const {

    *fVerts++ = corner;
    for (int i = 1; i < numFVerts; ++i) {
        *fVerts++ = nextPerimeterVert + i - 1;
    }
}

void
IrregularPatchBuilder::getControlFaceVertices(int fVerts[], int numFVerts,
        int corner, int nextPerimeterVert, bool lastFace,
        int val2IntOverlap) const {

    int S = numFVerts;
    int N = _surface.GetFaceSize();

    //
    //  The pathological case where the incident face overlaps with the
    //  base face due to valence-2 interior vertices:
    //
    if (lastFace && val2IntOverlap) {
        *fVerts++ = corner;

        for (int i = 0; i < S - 2 - val2IntOverlap; ++i) {
            *fVerts++ = ((nextPerimeterVert + i) < _numControlVerts)
                      ? (nextPerimeterVert + i) : N;
        }

        for (int i = val2IntOverlap; i >= 0; --i) {
            *fVerts++ = (corner + 1 + i) % N;
        }
        return;
    }

    //
    //  The typical case:  the corner vertex first, followed by vertices
    //  on the perimeter -- potentially wrapping around the perimeter to
    //  the first corner or the base face:
    //
    //      - for the last corner only, the face-vert preceding the last
    //        will wrap around the perimeter
    //      - for all corners, the last face-vert of the last face wraps
    //        around the face to the next corner
    //
    *fVerts++ = corner;

    for (int i = 1; i < S - 2; ++i) {
        *fVerts++ = nextPerimeterVert + i - 1;
    }

    int nextToLastPerimOfFace = nextPerimeterVert + S - 3;
    if (nextToLastPerimOfFace == _numControlVerts) {
        nextToLastPerimOfFace = N;
    }
    *fVerts++ = nextToLastPerimOfFace;

    int lastPerimOfFace = nextPerimeterVert + S - 2;
    if (lastPerimOfFace == _numControlVerts) {
        lastPerimOfFace = N;
    }
    *fVerts++ = lastFace ? ((corner + 1) % N) : lastPerimOfFace;
}


//
//  The main build/assembly method to create a PatchTree:
//
IrregularPatchBuilder::IrregPatchType const *
IrregularPatchBuilder::Build() {

    //
    //  The purpose here is to build a PatchTree -- whose factory
    //  requires a Far::TopologyRefiner.
    //
    //  For now, the quickest way to a Far::TopologyRefiner is via a
    //  Far::TopologyDescriptor, which simply refers to pre-allocated
    //  topology arrays -- which will be allocated on the stack for
    //  typical cases nor requiring much memory.
    //
    //  Use of the Far::TopologyDescriptor can be eliminated by defining
    //  a factory to create a Far::TopologyDescriptor directly from an
    //  instance of FaceTopology, but the benefits relative to the cost
    //  of creating the PatchTree may not be worth it.
    //
    int numVerts     = _numControlVerts;
    int numFaces     = _numControlFaces;
    int numFaceVerts = _numControlFaceVerts;
    int numCorners   = _surface.GetFaceSize();
    int numCreases   = _numControlVerts;

    //  Allocate and partition stack buffers for the topology arrays:
    int numInts   = numFaces + numFaceVerts + numCorners + numCreases*2;
    int numFloats = numCorners + numCreases;

    Vtr::internal::StackBuffer<int, 256,true> intBuffer(numInts);
    Vtr::internal::StackBuffer<float,64,true> floatBuffer(numFloats);

    int * faceSizes     = intBuffer;
    int * faceVerts     = faceSizes     + numFaces;
    int * cornerIndices = faceVerts     + numFaceVerts;
    int * creaseIndices = cornerIndices + numCorners;

    float * cornerWeights = floatBuffer;
    float * creaseWeights = cornerWeights + numCorners;

    //  Gather face sizes, face vertices and optional vertex or edge sharpness:
    gatherControlFaceSizes(faceSizes);
    gatherControlFaceVertices(faceVerts);

    numCorners = _surface.GetTag().HasSharpVertices() ?
                 gatherControlVertexSharpness(cornerIndices, cornerWeights) : 0;
    numCreases = _surface.GetTag().HasSharpEdges() ?
                 gatherControlEdgeSharpness(creaseIndices, creaseWeights) : 0;

    //  Declare a TopologyDescriptor to reference the data gathered above:
    Far::TopologyDescriptor topDescriptor;

    topDescriptor.numVertices = numVerts;
    topDescriptor.numFaces    = numFaces;

    topDescriptor.numVertsPerFace    = faceSizes;
    topDescriptor.vertIndicesPerFace = faceVerts;

    if (numCorners) {
        topDescriptor.numCorners          = numCorners;
        topDescriptor.cornerVertexIndices = cornerIndices;
        topDescriptor.cornerWeights       = cornerWeights;
    }

    if (numCreases) {
        topDescriptor.numCreases             = numCreases;
        topDescriptor.creaseVertexIndexPairs = creaseIndices;
        topDescriptor.creaseWeights          = creaseWeights;
    }

    //  Construct a TopologyRefiner in order to create a PatchTree:
    typedef Far::TopologyDescriptor Descriptor;
    typedef Far::TopologyRefinerFactory<Descriptor> RefinerFactory;

    RefinerFactory::Options refinerOptions;
    refinerOptions.schemeType    = _surface.GetSdcScheme();
    refinerOptions.schemeOptions = _surface.GetSdcOptionsInEffect();
    // WIP - enable for debugging
    //refinerOptions.validateFullTopology = true;

    Far::TopologyRefiner * refiner =
            RefinerFactory::Create(topDescriptor, refinerOptions);

    //  Create the PatchTree:
    PatchTreeFactory::Options patchTreeOptions;
    patchTreeOptions.includeInteriorPatches = false;
    patchTreeOptions.maxPatchDepthSharp  = _options.sharpLevel;
    patchTreeOptions.maxPatchDepthSmooth = _options.smoothLevel;
    patchTreeOptions.useDoublePrecision  = _options.doublePrecision;

    PatchTree const * patchTree =
            PatchTreeFactory::Create(*refiner, patchTreeOptions);

    assert(patchTree->GetNumControlPoints() == _numControlVerts);

    delete refiner;
    return patchTree;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
