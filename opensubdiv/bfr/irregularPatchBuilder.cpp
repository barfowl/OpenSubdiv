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

#include "../bfr/irregularPatchBuilder.h"

#include "../far/topologyDescriptor.h"
#include "../far/topologyRefiner.h"
#include "../far/patchTree.h"
#include "../far/patchTreeFactory.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Trivial constructor/destructor:
//
IrregularPatchBuilder::IrregularPatchBuilder(
        SurfaceDescriptor const & surface, Options options) :
            _surface(surface),
            _options(options) {

    initializeControlCounts();
}

void
IrregularPatchBuilder::initializeControlCounts() {

    //
    //  This is not done locally (in isolation) for each corner as there
    //  are some pathological cases where the inventory of one corner
    //  depends on one or more others:
    //
    //  WIP - this now matters only to the IrregularPatchBuilder and so
    //  may be worth moving there.
    //
    int N = _surface.GetFaceSize();

    int nVal3IntAdjTris = 0;

    _cornerControlVerts.SetSize(N);
    _cornerControlFaces.SetSize(N);

    _numControlVerts = N;
    _numControlFaces = 1;

    for (int corner = 0; corner < N; ++corner) {
        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset   const & cSub = _surface.GetCornerSubset(corner);

        //
        //  WIP - the counting and gathering of the "after" faces may be
        //  better handled as follows:
        //      - include "trailing" edge/vert of first "after" face
        //      - include interior and "trailing" edge/vert (i.e. S - 2)
        //        of all other "after" faces
        //  * the gathering (of both control verts and control face-verts)
        //  may benefit from this more than the simple counting here...
        //

        int nVerts = 0;
        if (cTop.GetCommonFaceSize()) {
            int S = cTop.GetCommonFaceSize();

            if (!cSub.IsBoundary()) {
                if ((cSub._numFacesTotal == 3) && (S == 3)) {
                    nVerts += (++nVal3IntAdjTris == N);
                } else {
                    nVerts += (cSub._numFacesTotal - 2) * (S - 2) - 1;
                }
            } else {
                if (cSub._numFacesAfter) {
                    nVerts += (cSub._numFacesAfter - 1) * (S - 2) + 1;
                }
                if (cSub._numFacesBefore) {
                    nVerts += cSub._numFacesBefore * (S - 2) - 1;
                }
            }
        } else {
            int cornerFace = cTop.GetFaceInVertex();

            if (!cSub.IsBoundary()) {
                assert(cSub._numFacesTotal == cTop.GetNumFaces());

                int nextFace = cTop.GetFaceAfter(2);
                if ((cSub._numFacesTotal == 3) &&
                    (cTop.GetFaceSize(nextFace) == 3)) {
                    nVerts += (++nVal3IntAdjTris == N);
                } else {
                    for (int i = 2; i < cSub._numFacesTotal; ++i) {
                        int S = cTop.GetFaceSize(nextFace);
                        nVerts += S - 2;
                        nextFace = cTop.GetFaceNext(nextFace);
                    }
                    nVerts --;
                }
            } else {
                if (cSub._numFacesAfter) {
                    int nextFace = cTop.GetFaceNext(cornerFace);
                    for (int i = 1; i < cSub._numFacesAfter; ++i) {
                        nextFace = cTop.GetFaceNext(nextFace);
                        int S = cTop.GetFaceSize(nextFace);
                        nVerts += S - 2;
                    }
                    nVerts ++;
                }
                if (cSub._numFacesBefore) {
                    int nextFace = cTop.GetFaceBefore(cSub._numFacesBefore);
                    for (int i = 0; i < cSub._numFacesBefore; ++i) {
                        int S = cTop.GetFaceSize(nextFace);
                        nVerts += S - 2;
                        nextFace = cTop.GetFaceNext(nextFace);
                    }
                    nVerts --;
                }
            }
        }
        _cornerControlVerts[corner] = nVerts;
        _numControlVerts += nVerts;

        //
        //  Enumerating the exterior control faces is far more simple --
        //  though complications dealing with val-2 interior verts and
        //  other degeneracies are likely to change this:
        //
        int nFaces = 0;
        if (!cSub.IsBoundary()) {
            nFaces += cSub._numFacesTotal - 2;
        } else {
            nFaces += cSub._numFacesAfter ? (cSub._numFacesAfter - 1) : 0;
            nFaces += cSub._numFacesBefore;
        }
        _cornerControlFaces[corner] = nFaces;
        _numControlFaces += nFaces;
    }
}


//
//  Initialization of buffers used for cache keys and assembly:
//

//
//  Computation of the cache key:
//
namespace {
    struct SimpleHashBits {
        typedef unsigned long int_type;

        void Clear() { std::memset(this, 0, sizeof(*this)); }

        //  This generates fewest compiler issues about type aliasing...
        int_type GetInt() const {
            assert(sizeof(int_type) == sizeof(*this));
            int_type intVar;
            std::memcpy(&intVar, this, sizeof(*this));
            return intVar;
        }

        int_type v0Valence    :  7;
        int_type v1Valence    :  7;
        int_type v2Valence    :  7;
        int_type v3Valence    :  7;
        int_type v0IsBoundary :  1;
        int_type v1IsBoundary :  1;
        int_type v2IsBoundary :  1;
        int_type v3IsBoundary :  1;

        int_type v0IsSharp    :  1;
        int_type v1IsSharp    :  1;
        int_type v2IsSharp    :  1;
        int_type v3IsSharp    :  1;
        int_type v0FaceInRing :  6;
        int_type v1FaceInRing :  6;
        int_type v2FaceInRing :  6;
        int_type v3FaceInRing :  6;

        //  Possible approximation level here
        int_type unused       :  4;
    };
}

TopologyCache::Key
IrregularPatchBuilder::ComputeTopologyKey() const {

    //
    //  The Key computation is going to change significantly -- especially
    //  once sharpness values can be added -- though it will continue to
    //  support simple common cases with this simple bit assignment.
    //
    //  The features for simple hashing exclude:
    //
    //      - any incident irregular faces
    //      - any semi-sharp vertices
    //      - any sharp edges of any kind (semi-sharp or inf-sharp)
    //
    //  and are otherwise limited to:
    //
    //      - interior valence up to 128 (7 bits)
    //      - boundary valence up to  64 (6 bits)
    //      - inf-sharp vertices
    //
    TopologyCache::Key key;

    CombinedTag combinedTag = _surface.GetTag();
    if (combinedTag.HasSharpEdges() ||
        combinedTag.HasSemiSharpVertices() ||
        combinedTag.HasIrregularFaceSizes()) {
        return key;
    }

    //
    //  Reject valence higher than the supported maxima:
    //
    int const maxValInt = (1 << 7);
    int const maxValBnd = (1 << 6);

    CornerSubset const * subsets = _surface.GetSubsets();

    for (int i = 0; i < _surface.GetFaceSize(); ++i) {
        if (subsets[i].IsBoundary()) {
            if (subsets[i]._numFacesTotal >= maxValBnd) return key;
        } else {
            if (subsets[i]._numFacesTotal >= maxValInt) return key;
        }
    }

    //
    //  Pack the corner subset topology into bits:
    //
    SimpleHashBits simpleBits;
    simpleBits.Clear();

    simpleBits.v0Valence    = subsets[0]._numFacesTotal;
    simpleBits.v0IsBoundary = subsets[0].IsBoundary();
    simpleBits.v0FaceInRing = subsets[0]._numFacesBefore;
    simpleBits.v0IsSharp    = subsets[0].IsSharp();

    simpleBits.v1Valence    = subsets[1]._numFacesTotal;
    simpleBits.v1IsBoundary = subsets[1].IsBoundary();
    simpleBits.v1FaceInRing = subsets[1]._numFacesBefore;
    simpleBits.v1IsSharp    = subsets[1].IsSharp();

    simpleBits.v2Valence    = subsets[2]._numFacesTotal;
    simpleBits.v2IsBoundary = subsets[2].IsBoundary();
    simpleBits.v2FaceInRing = subsets[2]._numFacesBefore;
    simpleBits.v2IsSharp    = subsets[2].IsSharp();

    if (_surface.GetFaceSize() == 4) {
        simpleBits.v3Valence    = subsets[3]._numFacesTotal;
        simpleBits.v3IsBoundary = subsets[3].IsBoundary();
        simpleBits.v3FaceInRing = subsets[3]._numFacesBefore;
        simpleBits.v3IsSharp    = subsets[3].IsSharp();
    }

    key.hashBits = simpleBits.GetInt();
//key.hashBits = 0;
    return key;
}


//
//  Search and update of a TopologyCache:
//
IrregularPatchBuilder::IrregPatchType const *
IrregularPatchBuilder::Find(TopologyCache & topologyCache,
        bool & patchIsNew, bool & patchIsCached) {

    //
    //  If cache key is not valid, just create and return:
    //
    TopologyCache::Key patchKey = ComputeTopologyKey();
    if (!patchKey.IsValid()) {
        patchIsCached = false;
        patchIsNew = true;
        return Build();
    }

    //
    //  If found in the cache, just return:
    //
    IrregPatchType const * patch = topologyCache.Find(patchKey);
    if (patch) {
        patchIsNew    = false;
        patchIsCached = false;
        return patch;
    }

    //
    //  Create a new patch and add to the cache -- but beware of the race
    //  condition: a patch with the same key may have been added while this
    //  one was being built, so be sure to return that one if the case:
    //
    patch = Build();

    IrregPatchType const * patchWithKey = topologyCache.Add(patchKey, patch);
    if (patchWithKey == patch) {
        patchIsNew    = true;
        patchIsCached = true;
        return patch;
    }

    //  Another patch was added, delete this one and return that one:
    delete patch;
    patch = patchWithKey;

    patchIsNew    = false;
    patchIsCached = true;
    return patch;
}

//
//  The main build/assembly method to create a PatchTree:
//
IrregularPatchBuilder::IrregPatchType const *
IrregularPatchBuilder::Build() {

    //
    //  Old notes from when this functionality was part of the Factory...
    //
    //  Gather all topology data the given topology container.  This will
    //  be gathered on the stack as much as possible and referenced by a
    //  Far::TopologyDescriptor -- an intermediate step towards creating
    //  the irregular PatchTree.
    //
    //  WIP - the Far::TopologyDescriptor can be eliminated by defining
    //  a factory to create a Far::TopologyDescriptor directly from an
    //  instance of FaceTopology. Some of this intermediate buffering can
    //  also be eliminated in that case.
    //
    int vertCount  = _numControlVerts;
    int faceCount  = _numControlFaces;
    int fVertCount = 0;

    Vtr::internal::StackBuffer<int, 64,true> faceSizes(faceCount);
    if (_surface.GetTag().HasIrregularFaceSizes()) {
        fVertCount = gatherControlFaceSizes(faceSizes);
    } else {
        int regFaceSize = _surface.GetRegFaceSize();
        fVertCount = faceCount * regFaceSize;
        std::fill(&faceSizes[0], &faceSizes[faceCount], regFaceSize);
    }

    Vtr::internal::StackBuffer<int,256,true> faceVerts(fVertCount);
    gatherControlFaceVertices(faceVerts);

    //  Gather sharpness for corner vertices:
    int faceSize = _surface.GetFaceSize();
    Vtr::internal::StackBuffer<float,8,true> cornerWeights(faceSize);
    Vtr::internal::StackBuffer<Index,8,true> cornerIndices(faceSize);

    int nSharpVerts = gatherControlVertexSharpness(cornerIndices,
                                                   cornerWeights);

    //  Gather sharpness for edges:
    Vtr::internal::StackBuffer<float,8,true>  creaseWeights(vertCount);
    Vtr::internal::StackBuffer<Index,16,true> creaseIndices(vertCount * 2);

    int nSharpEdges = 0;
    if (_surface.GetTag().HasSharpEdges()) {
        nSharpEdges = gatherControlEdgeSharpness(creaseIndices,
                                                 creaseWeights);
    }

    //
    //  Declare a TopologyDescriptor to reference the data gathered above:
    //
    Far::TopologyDescriptor topDescriptor;

    topDescriptor.numVertices = vertCount;
    topDescriptor.numFaces    = faceCount;

    topDescriptor.numVertsPerFace    = faceSizes;
    topDescriptor.vertIndicesPerFace = faceVerts;

    if (nSharpVerts) {
        topDescriptor.numCorners          = nSharpVerts;
        topDescriptor.cornerVertexIndices = cornerIndices;
        topDescriptor.cornerWeights       = cornerWeights;
    }

    if (nSharpEdges) {
        topDescriptor.numCreases             = nSharpEdges;
        topDescriptor.creaseVertexIndexPairs = creaseIndices;
        topDescriptor.creaseWeights          = creaseWeights;
    }

    //
    //  Important:
    //      Override the scheme options for boundary interpolation: all
    //  corners have already been explicitly sharpened where necessary,
    //  so do not allow the user options assigned to the mesh to sharpen
    //  those that do not warrant it (e.g. sharpening a corner for a
    //  subset that should stay smooth):
    //
    Sdc::Options localSchemeOptions = _surface.GetSchemeOptions();
    localSchemeOptions.SetVtxBoundaryInterpolation(
                                Sdc::Options::VTX_BOUNDARY_EDGE_ONLY);

    //  Construct a TopologyRefiner in order to create a PatchTree:
    typedef Far::TopologyDescriptor Descriptor;
    typedef Far::TopologyRefinerFactory<Descriptor> RefinerFactory;

    RefinerFactory::Options refinerOptions;
    refinerOptions.schemeType = _surface.GetSchemeType();
    refinerOptions.schemeOptions = localSchemeOptions;
    refinerOptions.validateFullTopology = true;  // WIP - remove when stable

    Far::TopologyRefiner * refiner =
            RefinerFactory::Create(topDescriptor, refinerOptions);

    //  Create the PatchTree:
    Far::PatchTreeFactory::Options patchTreeOptions;
    patchTreeOptions.maxPatchDepthSharp = _options.sharpLevel;
    patchTreeOptions.maxPatchDepthSmooth = _options.smoothLevel;
    patchTreeOptions.includeInteriorPatches = false;

    Far::PatchTree const * patchTree =
            Far::PatchTreeFactory::Create(*refiner, patchTreeOptions);

    assert(patchTree->GetNumControlPoints() == vertCount);

    delete refiner;
    return patchTree;
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

    CornerTopology const & cTop0 = _surface.GetCornerTopology(0);
    int baseOffset = cTop0.GetFaceVertexOffset(cTop0.GetFaceInVertex());

    Index const * baseIndices = &_surface.GetIndices()[baseOffset];
    std::memcpy(cvIndices, baseIndices, N * sizeof(Index));

    int nCVIndices = N;

    int nVal3IntAdjTris = 0;

    //
    //  Assign CV indices "local to" each corner:
    //
    Index const * cornerIndices = _surface.GetIndices();

    for (int corner = 0; corner < N; ++corner) {
        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset const   & cSub = _surface.GetCornerSubset(corner);

        int cornerFace = cTop.GetFaceInVertex();

        //
        //  Similar loops here to traverse the faces around each corner,
        //  so potential for some consolidation here...
        //
        int nCVIndicesBefore = nCVIndices;

        if (!cSub.IsBoundary()) {
            assert(cSub._numFacesTotal == cTop.GetNumFaces());

            int numFaces = cSub._numFacesTotal - 2;
            int nextFace = cTop.GetFaceAfter(2);
            if ((numFaces == 1) && (cTop.GetFaceSize(nextFace) == 3)) {
                if (++nVal3IntAdjTris == N) {
                    int fvOffset = cTop.GetFaceVertexOffset(nextFace);

                    cvIndices[nCVIndices++] = cornerIndices[fvOffset + 1];
                }
            } else {
                for (int j = 0; j < numFaces; ++j) {
                    int S = cTop.GetFaceSize(nextFace);
                    int fvOffset = cTop.GetFaceVertexOffset(nextFace);

                    int M = (S - 2) - (j == (numFaces - 1));
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[nCVIndices++] = cornerIndices[fvOffset + k];
                    }
                    nextFace = cTop.GetFaceNext(nextFace);
                }
            }
        } else {
            if (cSub._numFacesAfter) {
                //
                //  While the first face "after" is generally skipped, if it
                //  is the only one, we need to include its trailing edge:
                //
                int numFaces = cSub._numFacesAfter - 1;
                int nextFace = cTop.GetFaceNext(cornerFace);
                for (int j = 0; j < numFaces; ++j) {
                    nextFace = cTop.GetFaceNext(nextFace);

                    int S = cTop.GetFaceSize(nextFace);
                    int fvOffset = cTop.GetFaceVertexOffset(nextFace);

                    int M = (S - 2);
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[nCVIndices++] = cornerIndices[fvOffset + k];
                    }
                }
                cvIndices[nCVIndices++] =
                        cTop.GetFaceVertexTrailing(nextFace, cornerIndices);
            }
            if (cSub._numFacesBefore) {
                int numFaces = cSub._numFacesBefore;
                int nextFace = cTop.GetFaceBefore(cSub._numFacesBefore);
                for (int j = 0; j < numFaces; ++j) {
                    int S = cTop.GetFaceSize(nextFace);
                    int fvOffset = cTop.GetFaceVertexOffset(nextFace);

                    int M = (S - 2) - (j == (numFaces - 1));
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[nCVIndices++] = cornerIndices[fvOffset + k];
                    }
                    nextFace = cTop.GetFaceNext(nextFace);
                }
            }
        }
        assert((nCVIndices - nCVIndicesBefore) == _cornerControlVerts[corner]);

        cornerIndices += cTop.GetNumFaceVertices();
    }
    assert(nCVIndices == _numControlVerts);
    return nCVIndices;
}

int
IrregularPatchBuilder::gatherControlFaceSizes(
        int faceSizes[]) const {

    int nFaces = 1;
    int sumOfSizes = _surface.GetFaceSize();
    faceSizes[0] = _surface.GetFaceSize();

    for (int corner = 0; corner < _surface.GetFaceSize(); ++corner) {
        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset const   & cSub = _surface.GetCornerSubset(corner);

        if (!cSub.IsBoundary()) {
            int nextFace = cTop.GetFaceAfter(2);
            for (int i = 2; i < cSub._numFacesTotal; ++i) {
                int S = cTop.GetFaceSize(nextFace);
                faceSizes[nFaces++] = S;
                sumOfSizes += S;

                nextFace = cTop.GetFaceNext(nextFace);
            }
        } else {
            if (cSub._numFacesAfter > 1) {
                int nextFace = cTop.GetFaceAfter(2);
                for (int j = 1; j < cSub._numFacesAfter; ++j) {
                    int S = cTop.GetFaceSize(nextFace);
                    faceSizes[nFaces++] = S;
                    sumOfSizes += S;

                    nextFace = cTop.GetFaceNext(nextFace);
                }
            }
            if (cSub._numFacesBefore) {
                int nextFace = cTop.GetFaceBefore(cSub._numFacesBefore);
                for (int j = 0; j < cSub._numFacesBefore; ++j) {
                    int S = cTop.GetFaceSize(nextFace);
                    faceSizes[nFaces++] = S;
                    sumOfSizes += S;

                    nextFace = cTop.GetFaceNext(nextFace);
                }
            }
        }
    }
    return sumOfSizes;
}

int
IrregularPatchBuilder::gatherControlVertexSharpness(
        int vertIndices[], float vertSharpness[]) const {

    int nSharpVerts = 0;

    for (int i = 0; i < _surface.GetFaceSize(); ++i) {
        CornerTopology const & cTop = _surface.GetCornerTopology(i);
        CornerSubset   const & cSub = _surface.GetSubsets()[i];

        if (cSub._tag.IsInfSharp() || cSub._tag.IsSemiSharp()) {
            vertIndices[nSharpVerts] = i;
            vertSharpness[nSharpVerts] = cSub._tag.IsInfSharp()
                                       ? Sdc::Crease::SHARPNESS_INFINITE
                                       : cTop.GetVertexSharpness();
            ++ nSharpVerts;
        }
    }
    return nSharpVerts;
}

int
IrregularPatchBuilder::gatherControlEdgeSharpness(
        int edgeVertPairs[], float edgeSharpness[]) const {

    //
    //  For each corner, test the forward edge in the face and any
    //  interior edges local to the corner vertex:
    //
    int faceSize = _surface.GetFaceSize();

    int perimMax   = GetNumControlVertices();
    int perimStart = faceSize;

    int nSharpEdges = 0;

    for (int corner = 0; corner < faceSize; ++corner) {
        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset const   & cSub = _surface.GetCornerSubset(corner);

        if (!cTop.GetTag().HasSharpEdges()) {
            perimStart += _cornerControlVerts[corner];
            continue;
        }

        int cornerFace = cTop.GetFaceInVertex();

        //  Test the forward edge of the face:
        float sharpness = cTop.GetFaceEdgeSharpness(cornerFace, 0);
        if (sharpness > 0.0f) {
            *edgeVertPairs++ =  corner;
            *edgeVertPairs++ = (corner + 1) % faceSize;
            *edgeSharpness++ = sharpness;
            nSharpEdges++;
        }

        //
        //  Inspect interior edges of the subset -- test sharpness of
        //  the trailing edge of the faces after/before the corner face.
        //
        //  Unfortunately we need the control vertex index at the end
        //  of the edge, and so we need to track the perimeter -- which
        //  requires the face sizes and may wrap around with tris...
        //
        int nextVert = perimStart;

        //  WIP - these blocks are similar enough to warrant merging
        if (!cSub.IsBoundary()) {
            int nextFace = cTop.GetFaceNext(cornerFace);
            for (int i = 2; i < cSub._numFacesTotal; ++i) {
                sharpness = cTop.GetFaceEdgeSharpness(nextFace, 1);
                if (sharpness > 0.0f) {
                    *edgeSharpness++ = sharpness;
                    *edgeVertPairs++ = corner;
                    *edgeVertPairs++ = (nextVert < perimMax)
                                     ? nextVert : faceSize;
                    nSharpEdges++;
                }
                nextFace  = cTop.GetFaceNext(nextFace);
                nextVert += cTop.GetFaceSize(nextFace) - 2;
            }
        } else {
            if (cSub._numFacesAfter) {
                int nextFace = cTop.GetFaceNext(cornerFace);
                for (int i = 1; i < cSub._numFacesAfter; ++i) {
                    sharpness = cTop.GetFaceEdgeSharpness(nextFace, 1);
                    if (sharpness > 0.0f) {
                        *edgeSharpness++ = sharpness;
                        *edgeVertPairs++ = corner;
                        *edgeVertPairs++ = (nextVert < perimMax)
                                         ? nextVert : faceSize;
                        nSharpEdges++;
                    }
                    nextFace  = cTop.GetFaceNext(nextFace);
                    nextVert += cTop.GetFaceSize(nextFace) - 2;
                }
            }
            if (cSub._numFacesBefore) {
                int nextFace = cTop.GetFaceBefore(cSub._numFacesBefore);
                for (int i = 1; i < cSub._numFacesBefore; ++i) {
                    sharpness = cTop.GetFaceEdgeSharpness(nextFace, 1);
                    if (sharpness > 0.0f) {
                        *edgeSharpness++ = sharpness;
                        *edgeVertPairs++ = corner;
                        *edgeVertPairs++ = (nextVert < perimMax)
                                         ? nextVert : faceSize;
                        nSharpEdges++;
                    }
                    nextFace  = cTop.GetFaceNext(nextFace);
                    nextVert += cTop.GetFaceSize(nextFace) - 2;
                }
            }
        }
        perimStart += _cornerControlVerts[corner];
    }
    return nSharpEdges;
}

int
IrregularPatchBuilder::gatherControlFaceVertices(int faceVertices[]) const {

    //
    //  Assign face vertices for the first/base face:
    //
    int faceSize = _surface.GetFaceSize();

    for (int i = 0; i < faceSize; ++i) {
        *faceVertices++ = i;
    }
    int nFaceVertices = faceSize;

    //
    //  Assign face vertex indices "local to" each corner:
    //
    int numControlVertices = _numControlVerts;

    int startPerimOfCorner = faceSize;
    for (int corner = 0; corner < faceSize; ++corner) {
        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset const   & cSub = _surface.GetCornerSubset(corner);

        int cornerFace = cTop.GetFaceInVertex();

        //
        //  The interior case is simpler, the boundary needing more care:
        //
        if (!cSub.IsBoundary()) {
            assert(cSub._numFacesTotal == cTop.GetNumFaces());
            int nextFace = cTop.GetFaceNext(cornerFace);
            int startPerimOfFace = startPerimOfCorner;

            int N = cSub._numFacesTotal - 2;
            for (int j = 0; j < N; ++j) {
                bool lastFace = (j == (N - 1));

                nextFace = cTop.GetFaceNext(nextFace);
                int S = cTop.GetFaceSize(nextFace);

                //
                //  Special cases:
                //      - the last face-vert of the last face here is the
                //        leading (?) edge of the corner
                //      - for the last corner only, the face-vert preceding
                //        the last will wrap around the perimeter
                //
                *faceVertices++ = corner;
                for (int k = 1; k < S - 2; ++k) {
                    *faceVertices++ = startPerimOfFace + k - 1;
                }

                int nextToLastPerimOfFace = startPerimOfFace + S - 3;
                if (nextToLastPerimOfFace == numControlVertices) {
                    nextToLastPerimOfFace = faceSize;
                }
                *faceVertices++ = nextToLastPerimOfFace;

                int lastPerimOfFace = startPerimOfFace + S - 2;
                if (lastPerimOfFace == numControlVertices) {
                    lastPerimOfFace = faceSize;
                }
                *faceVertices++ = (lastFace) ? ((corner+1) % faceSize)
                                : lastPerimOfFace;

                nFaceVertices += S;

                startPerimOfFace += S - 2;
                startPerimOfCorner += S - 2;
            }
            startPerimOfCorner --;
        } else {
            if (cSub._numFacesAfter) {
                int nextFace = cTop.GetFaceNext(cornerFace);
                int startPerimOfFace = startPerimOfCorner;

                int N = cSub._numFacesAfter - 1;
                for (int j = 0; j < N; ++j) {
                    nextFace = cTop.GetFaceNext(nextFace);
                    int S = cTop.GetFaceSize(nextFace);

                    //  No special cases here
                    *faceVertices++ = corner;
                    for (int k = 1; k < S; ++k) {
                        *faceVertices++ = startPerimOfFace + k - 1;
                    }

                    nFaceVertices += S;

                    startPerimOfFace += S - 2;
                    startPerimOfCorner += S - 2;
                }
                startPerimOfCorner ++;
            }
            if (cSub._numFacesBefore) {
                int nextFace = cTop.GetFaceBefore(cSub._numFacesBefore);
                int startPerimOfFace = startPerimOfCorner;

                int N = cSub._numFacesBefore;
                for (int j = 0; j < N; ++j) {
                    int S = cTop.GetFaceSize(nextFace);
                    bool lastFace = (j == (N - 1));

                    //  Special cases are same as the interior case above
                    *faceVertices++ = corner;
                    for (int k = 1; k < S - 2; ++k) {
                        *faceVertices++ = startPerimOfFace + k - 1;
                    }

                    int nextToLastPerimOfFace = startPerimOfFace + S - 3;
                    if (nextToLastPerimOfFace == numControlVertices) {
                        nextToLastPerimOfFace = faceSize;
                    }
                    *faceVertices++ = nextToLastPerimOfFace;

                    int lastPerimOfFace = startPerimOfFace + S - 2;
                    if (lastPerimOfFace == numControlVertices) {
                        lastPerimOfFace = faceSize;
                    }
                    *faceVertices++ = (lastFace) ? ((corner+1) % faceSize)
                                    : lastPerimOfFace;

                    nFaceVertices += S;

                    startPerimOfFace += S - 2;
                    startPerimOfCorner += S - 2;

                    nextFace = cTop.GetFaceNext(nextFace);
                }
                startPerimOfCorner --;
            }
        }
    }
    return nFaceVertices;
}

//
//  Methods for debugging...
//
void
IrregularPatchBuilder::print() const {

    int faceSize = _surface.GetFaceSize();

    CombinedTag tag = _surface.GetTag();

    printf("IrregularPatchBuilder control hull:\n");
    printf("    Combined surface tags:\n");
    printf("        has inc irreg faces  = %d\n", tag.HasIrregularFaceSizes());
    printf("        has inf-sharp verts  = %d\n", tag.HasInfSharpVertices());
    printf("        has semi-sharp verts = %d\n", tag.HasSemiSharpVertices());
    printf("        has any sharp edges  = %d\n", tag.HasSharpEdges());

    int nVerts = _numControlVerts;
    int nFaces = _numControlFaces;

    printf("    Topology:\n");
    printf("        num control verts = %3d\n", nVerts);
    printf("        num control faces = %3d\n", nFaces);

    printf("    Indices:\n");

    int    S[nFaces];
    Index  P[_surface.GetNumIndices()];
    Index *p = 0;

    int n = GatherControlVertexIndices(P);
    assert(n == nVerts);
    printf("        control verts:");
    p = P;
    for (int i = 0; i < faceSize; ++i) {
        printf(" %3d", *p++);
    }
    printf("\n");
    if (nVerts > faceSize) {
        printf("                      ");
        for (int i = faceSize; i < nVerts; ++i) {
            printf(" %3d", *p++);
        }
        printf("\n");
    }

    int n1 = gatherControlFaceSizes(S);
    int n2 = gatherControlFaceVertices(P);
    assert(n1 == n2);

    printf("        control faces:");
    p = P;
    for (int i = 0; i < faceSize; ++i) {
        printf(" %3d", *p++);
    }
    printf("    (%d)\n", S[0]);
    for (int i = 1; i < nFaces; ++i) {
        printf("                      ");
        for (int j = 0; j < S[i]; ++j) {
            printf(" %3d", *p++);
        }
        printf("    (%d)\n", S[i]);
    }

    //  Corner subsets may be sharpened, so don't just test topology:
    printf("    Sharpness:\n");
    {
        int   cornerIndices[faceSize];
        float vertSharpness[faceSize];

        int nSharp = gatherControlVertexSharpness(cornerIndices, vertSharpness);
        printf("        num sharp verts = %3d\n", nSharp);
        for (int i = 0; i < nSharp; ++i) {
            printf("                       ");
            printf("  %d:  ", cornerIndices[i]);
            if (vertSharpness[i] < Sdc::Crease::SHARPNESS_INFINITE) {
                printf("%6.3f\n", vertSharpness[i]);
            } else {
                printf("inf\n");
            }
        }
    }

    if (tag.HasSharpEdges()) {
        int   edgeVertPairs[nVerts * 2];
        float edgeSharpness[nVerts];

        int nSharp = gatherControlEdgeSharpness(edgeVertPairs, edgeSharpness);
        printf("        num sharp edges = %3d\n", nSharp);
        for (int i = 0; i < nSharp; ++i) {
            printf("                       ");
            printf(" (%d,%2d):  ", edgeVertPairs[i*2+0],
                                   edgeVertPairs[i*2+1]);
            if (edgeSharpness[i] < Sdc::Crease::SHARPNESS_INFINITE) {
                printf("%6.3f\n", edgeSharpness[i]);
            } else {
                printf("inf\n");
            }
        }
    } else {
        printf("        num sharp edges = %3d\n", 0);
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
