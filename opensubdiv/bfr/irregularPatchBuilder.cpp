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
            _topology(surface._topology),
            _options(options) {

    int faceSize = _topology._faceSize;

    _numControlVerts = faceSize;
    _numControlFaces = 1;
    for (int i = 0; i < faceSize; ++i) {
        _numControlVerts += _surface._corners[i]._numOuterVerts;
        _numControlFaces += _surface._corners[i]._numOuterFaces;
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
    //  support simple common cases with this simple bit assignment.  The
    //  features exclude:
    //
    //      - no incident irregular faces
    //      - no semi-sharp vertices
    //      - no sharp edges of any kind
    //
    //  and are otherwise limited to:
    //
    //      - interior valence up to 128 (7 bits)
    //      - boundary valence up to  64 (6 bits)
    //      - inf-sharp vertices
    //
    //  Dealing with incident irregular faces is unfortunate as the entire
    //  set of incident face sizes must somehow be encoded.  An exception
    //  this that's worth supporting (and easily achieved) is when the face
    //  size is constant but not regular, i.e. when Catmark is applied to a
    //  triangle mesh.  The bits already support triangular meshes for Loop,
    //  a bit just needs to be added to distinguish the scheme -- but a bit
    //  more work is needed in the code to support constant face sizes that
    //  are not regular.
    //
    TopologyCache::Key key;

    CornerSubset const * C = _surface._corners;
    if (_topology._hasIncIrregFaces) {
        //  WIP - the subset may not include the irregular faces
        return key;
    }
    if (_topology._hasSharpEdges) {
        //  WIP - the subset may not include the sharp edges
        return key;
    }
    if (_topology._hasSemiSharpVerts) {
        //  WIP - the subset may not include the semi-sharp verts
        return key;
    }

    //
    //  Reject valence higher than the supported maxima:
    //
    int const maxValInt = (1 << 7);
    int const maxValBnd = (1 << 6);

    for (int i = 0; i < _topology._faceSize; ++i) {
        if (C[i]._isBoundary) {
            if (C[i]._numFacesTotal >= maxValBnd) return key;
        } else {
            if (C[i]._numFacesTotal >= maxValInt) return key;
        }
    }

    //
    //  Pack the corner subset topology into bits:
    //
    SimpleHashBits simpleBits;
    simpleBits.Clear();

    simpleBits.v0Valence    = C[0]._numFacesTotal;
    simpleBits.v0IsBoundary = C[0]._isBoundary;
    simpleBits.v0FaceInRing = C[0]._numFacesBefore;
    simpleBits.v0IsSharp    = C[0]._isSharp;

    simpleBits.v1Valence    = C[1]._numFacesTotal;
    simpleBits.v1IsBoundary = C[1]._isBoundary;
    simpleBits.v1FaceInRing = C[1]._numFacesBefore;
    simpleBits.v1IsSharp    = C[1]._isSharp;

    simpleBits.v2Valence    = C[2]._numFacesTotal;
    simpleBits.v2IsBoundary = C[2]._isBoundary;
    simpleBits.v2FaceInRing = C[2]._numFacesBefore;
    simpleBits.v2IsSharp    = C[2]._isSharp;

    if (_topology._faceSize == 4) {
        simpleBits.v3Valence    = C[3]._numFacesTotal;
        simpleBits.v3IsBoundary = C[3]._isBoundary;
        simpleBits.v3FaceInRing = C[3]._numFacesBefore;
        simpleBits.v3IsSharp    = C[3]._isSharp;
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
    if (_topology._hasIncIrregFaces) {
        fVertCount = gatherControlFaceSizes(faceSizes);
    } else {
        fVertCount = faceCount * _topology._regFaceSize;
        std::fill(&faceSizes[0], &faceSizes[faceCount], _topology._regFaceSize);
    }

    Vtr::internal::StackBuffer<int,256,true> faceVerts(fVertCount);
    gatherControlFaceVertices(faceVerts);

    //  Gather sharpness for corner vertices:
    int faceSize = _topology.GetFaceSize();
    Vtr::internal::StackBuffer<float,8,true> cornerWeights(faceSize);
    Vtr::internal::StackBuffer<Index,8,true> cornerIndices(faceSize);

    int nSharpVerts = gatherControlVertexSharpness(cornerIndices,
                                                   cornerWeights);

    //  Gather sharpness for edges:
    Vtr::internal::StackBuffer<float,8,true>  creaseWeights(vertCount);
    Vtr::internal::StackBuffer<Index,16,true> creaseIndices(vertCount * 2);

    int nSharpEdges = 0;
    if (_topology._hasSharpEdges) {
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
    Sdc::Options localSchemeOptions = _topology._schemeOptions;
    localSchemeOptions.SetVtxBoundaryInterpolation(
                                Sdc::Options::VTX_BOUNDARY_EDGE_ONLY);

    //  Construct a TopologyRefiner in order to create a PatchTree:
    typedef Far::TopologyDescriptor Descriptor;
    typedef Far::TopologyRefinerFactory<Descriptor> RefinerFactory;

    RefinerFactory::Options refinerOptions;
    refinerOptions.schemeType = _topology._schemeType;
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
    int N = _topology._faceSize;

    int baseOffset = _topology._vertexTopology[0].getFaceVertexOffset(
                        _topology._faceInVertex[0]);

    Index const * baseIndices = &_surface._indices[baseOffset];
    std::memcpy(cvIndices, baseIndices, N * sizeof(Index));

    int nCVIndices = N;

    int nVal3IntAdjTris = 0;

    //
    //  Assign CV indices "local to" each corner:
    //
    Index const * cornerIndices = _surface._indices;

    for (int corner = 0; corner < N; ++corner) {
        VertexTopology const & V = _topology._vertexTopology[corner];
        CornerSubset const   & C = _surface._corners[corner];

        int cornerFace = _topology._faceInVertex[corner];

        //
        //  Similar loops here to traverse the faces around each corner,
        //  so potential for some consolidation here...
        //
        int nCVIndicesBefore = nCVIndices;

        if (!C._isBoundary) {
            assert(C._numFacesTotal == V._numFaces);

            int numFaces = C._numFacesTotal - 2;
            int nextFace = V.getFaceAfter(cornerFace, 2);
            if ((numFaces == 1) && (V.getFaceSize(nextFace) == 3)) {
                if (++nVal3IntAdjTris == N) {
                    int fvOffset = V.getFaceVertexOffset(nextFace);

                    cvIndices[nCVIndices++] = cornerIndices[fvOffset + 1];
                }
            } else {
                for (int j = 0; j < numFaces; ++j) {
                    int S = V.getFaceSize(nextFace);
                    int fvOffset = V.getFaceVertexOffset(nextFace);

                    int M = (S - 2) - (j == (numFaces - 1));
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[nCVIndices++] = cornerIndices[fvOffset + k];
                    }
                    nextFace = V.getFaceNext(nextFace);
                }
            }
        } else {
            if (C._numFacesAfter) {
                //
                //  While the first face "after" is generally skipped, if it
                //  is the only one, we need to include its trailing edge:
                //
                int numFaces = C._numFacesAfter - 1;
                int nextFace = V.getFaceNext(cornerFace);
                for (int j = 0; j < numFaces; ++j) {
                    nextFace = V.getFaceNext(nextFace);

                    int S = V.getFaceSize(nextFace);
                    int fvOffset = V.getFaceVertexOffset(nextFace);

                    int M = (S - 2);
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[nCVIndices++] = cornerIndices[fvOffset + k];
                    }
                }
                cvIndices[nCVIndices++] =
                        V.getFaceVertexTrailing(nextFace, cornerIndices);
            }
            if (C._numFacesBefore) {
                int numFaces = C._numFacesBefore;
                int nextFace = V.getFaceBefore(cornerFace, C._numFacesBefore);
                for (int j = 0; j < numFaces; ++j) {
                    int S = V.getFaceSize(nextFace);
                    int fvOffset = V.getFaceVertexOffset(nextFace);

                    int M = (S - 2) - (j == (numFaces - 1));
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[nCVIndices++] = cornerIndices[fvOffset + k];
                    }
                    nextFace = V.getFaceNext(nextFace);
                }
            }
        }
        assert((nCVIndices - nCVIndicesBefore) == C._numOuterVerts);

        cornerIndices += V._numFaceVerts;
    }
    assert(nCVIndices == _numControlVerts);
    return nCVIndices;
}

int
IrregularPatchBuilder::gatherControlFaceSizes(
        int faceSizes[]) const {

    int nFaces = 1;
    int sumOfSizes = _topology._faceSize;
    faceSizes[0] = _topology._faceSize;

    for (int corner = 0; corner < _topology._faceSize; ++corner) {
        VertexTopology const & V = _topology._vertexTopology[corner];
        CornerSubset const   & C = _surface._corners[corner];

        int cornerFace = _topology._faceInVertex[corner];

        if (!C._isBoundary) {
            int nextFace = V.getFaceAfter(cornerFace, 2);
            for (int i = 2; i < C._numFacesTotal; ++i) {
                int S = V.getFaceSize(nextFace);
                faceSizes[nFaces++] = S;
                sumOfSizes += S;

                nextFace = V.getFaceNext(nextFace);
            }
        } else {
            if (C._numFacesAfter > 1) {
                int nextFace = V.getFaceAfter(cornerFace, 2);
                for (int j = 1; j < C._numFacesAfter; ++j) {
                    int S = V.getFaceSize(nextFace);
                    faceSizes[nFaces++] = S;
                    sumOfSizes += S;

                    nextFace = V.getFaceNext(nextFace);
                }
            }
            if (C._numFacesBefore) {
                int nextFace = V.getFaceBefore(cornerFace, C._numFacesBefore);
                for (int j = 0; j < C._numFacesBefore; ++j) {
                    int S = V.getFaceSize(nextFace);
                    faceSizes[nFaces++] = S;
                    sumOfSizes += S;

                    nextFace = V.getFaceNext(nextFace);
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

    for (int i = 0; i < _topology._faceSize; ++i) {
        VertexTopology const & vTop = _topology._vertexTopology[i];

        //  Sharpness of the CornerSubset takes precedence here:
        if (_surface._corners[i]._isSharp) {
            vertIndices[nSharpVerts] = i;
            vertSharpness[nSharpVerts] = Sdc::Crease::SHARPNESS_INFINITE;
            ++ nSharpVerts;
        } else if (vTop._isSemiSharp) {
            vertIndices[nSharpVerts] = i;
            vertSharpness[nSharpVerts] = vTop._vertSharpness;
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
    int faceSize = _topology._faceSize;

    int perimMax   = GetNumControlVertices();
    int perimStart = faceSize;

    int nSharpEdges = 0;

    for (int corner = 0; corner < faceSize; ++corner) {
        VertexTopology const & V = _topology._vertexTopology[corner];
        CornerSubset const   & C = _surface._corners[corner];

        if (!V._hasSharpEdge) {
            perimStart += C._numOuterVerts;
            continue;
        }

        int cornerFace = _topology._faceInVertex[corner];

        //  Test the forward edge of the face:
        float sharpness = V._faceEdgeSharpness[2*cornerFace];
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
        if (!C._isBoundary) {
            int nextFace = V.getFaceNext(cornerFace);
            for (int i = 2; i < C._numFacesTotal; ++i) {
                sharpness = V._faceEdgeSharpness[2*nextFace + 1];
                if (sharpness > 0.0f) {
                    *edgeSharpness++ = sharpness;
                    *edgeVertPairs++ = corner;
                    *edgeVertPairs++ = (nextVert < perimMax)
                                     ? nextVert : faceSize;
                    nSharpEdges++;
                }
                nextFace  = V.getFaceNext(nextFace);
                nextVert += V.getFaceSize(nextFace) - 2;
            }
        } else {
            if (C._numFacesAfter) {
                int nextFace = V.getFaceNext(cornerFace);
                for (int i = 1; i < C._numFacesAfter; ++i) {
                    sharpness = V._faceEdgeSharpness[2*nextFace + 1];
                    if (sharpness > 0.0f) {
                        *edgeSharpness++ = sharpness;
                        *edgeVertPairs++ = corner;
                        *edgeVertPairs++ = (nextVert < perimMax)
                                         ? nextVert : faceSize;
                        nSharpEdges++;
                    }
                    nextFace  = V.getFaceNext(nextFace);
                    nextVert += V.getFaceSize(nextFace) - 2;
                }
            }
            if (C._numFacesBefore) {
                int nextFace = V.getFaceBefore(cornerFace, C._numFacesBefore);
                for (int i = 1; i < C._numFacesBefore; ++i) {
                    sharpness = V._faceEdgeSharpness[2*nextFace + 1];
                    if (sharpness > 0.0f) {
                        *edgeSharpness++ = sharpness;
                        *edgeVertPairs++ = corner;
                        *edgeVertPairs++ = (nextVert < perimMax)
                                         ? nextVert : faceSize;
                        nSharpEdges++;
                    }
                    nextFace  = V.getFaceNext(nextFace);
                    nextVert += V.getFaceSize(nextFace) - 2;
                }
            }
        }
        perimStart += C._numOuterVerts;
    }
    return nSharpEdges;
}

int
IrregularPatchBuilder::gatherControlFaceVertices(int faceVertices[]) const {

    //
    //  Assign face vertices for the first/base face:
    //
    int faceSize = _topology._faceSize;

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
        VertexTopology const & V = _topology._vertexTopology[corner];
        CornerSubset const   & C = _surface._corners[corner];

        int cornerFace = _topology._faceInVertex[corner];

        //
        //  The interior case is simpler, the boundary needing more care:
        //
        if (!C._isBoundary) {
            assert(C._numFacesTotal == V._numFaces);
            int nextFace   = V.getFaceNext(cornerFace);
            int startPerimOfFace = startPerimOfCorner;

            int N = C._numFacesTotal - 2;
            for (int j = 0; j < N; ++j) {
                bool lastFace = (j == (N - 1));

                nextFace = V.getFaceNext(nextFace);
                int S = V.getFaceSize(nextFace);

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
            if (C._numFacesAfter) {
                int nextFace   = V.getFaceNext(cornerFace);
                int startPerimOfFace = startPerimOfCorner;

                int N = C._numFacesAfter - 1;
                for (int j = 0; j < N; ++j) {
                    nextFace = V.getFaceNext(nextFace);
                    int S = V.getFaceSize(nextFace);

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
            if (C._numFacesBefore) {
                //  Finding the first face here is a bit awkward -- and will
                //  be more so if the faces are unordered...
                assert(V._isOrdered);
                int nextFace = (cornerFace + V._numFaces - C._numFacesBefore);
                if (nextFace >= V._numFaces) {
                    nextFace -= V._numFaces;
                }
                int startPerimOfFace = startPerimOfCorner;

                int N = C._numFacesBefore;
                for (int j = 0; j < N; ++j) {
                    int S = V.getFaceSize(nextFace);
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

                    nextFace = V.getFaceNext(nextFace);
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

    int faceSize = _topology._faceSize;

    printf("IrregularPatchBuilder control hull:\n");
    printf("    Properties:\n");
    printf("        has inc irreg faces  = %d\n", _topology._hasIncIrregFaces);
    printf("        has inf-sharp verts  = %d\n", _topology._hasInfSharpVerts);
    printf("        has semi-sharp verts = %d\n", _topology._hasSemiSharpVerts);
    printf("        has any sharp edges  = %d\n", _topology._hasSharpEdges);

    int nVerts = _numControlVerts;
    int nFaces = _numControlFaces;

    printf("    Topology:\n");
    printf("        num control verts = %3d\n", nVerts);
    printf("        num control faces = %3d\n", nFaces);

    printf("    Indices:\n");

    int    S[nFaces];
    Index  P[_topology._numFaceVertsTotal];
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

    if (_topology._hasSharpEdges) {
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
