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
#include "../bfr/hash.h"
#include "../far/topologyDescriptor.h"
#include "../far/topologyRefiner.h"
#include "../far/patchTree.h"
#include "../far/patchTreeFactory.h"

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
    //  Simple query if a CornerSubset is val-2 interior:
    //
    bool
    subsetIsVal2Interior(CornerSubset const & corner) {
        return (corner.GetNumFaces() == 2) && !corner.IsBoundary();
    }

    //
    //  Count the number of val-2 interior corners that follow the given
    //  corner (ignore if this corner is val-2 interior):
    //
    int
    getFaceOverlap(FaceSurface const & surface, int corner) {

        CornerSubset const * corners = &surface.GetCornerSubset(0);

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
    //  This process is not done locally for each corner as there are
    //  rare but legitimate cases where the inventory of one corner
    //  depends on one or more others:
    //
    int N = _surface.GetFaceSize();

    //
    //  First iterate through the corners to get the number of control
    //  faces -- while also identifying val-2 and other features that
    //  complicate dealing with control vertices:
    //
    int nVal2IntCorners = 0;

    _numControlFaces = 1;
    _cornerCFaceCount.SetSize(N);

    for (int corner = 0; corner < N; ++corner) {
        CornerSubset   const & cSub = _surface.GetCornerSubset(corner);

        int nFaces = 0;
        if (cSub.IsBoundary()) {
            nFaces += cSub._numFacesAfter ? (cSub._numFacesAfter - 1) : 0;
            nFaces += cSub._numFacesBefore;
        } else if (cSub._numFacesTotal > 2) {
            assert(cSub._numFacesBefore == 0);
            nFaces += cSub._numFacesTotal - 2;
        } else {
            //  If all corners val-2 interior, we have an extra face:
            nFaces += (++nVal2IntCorners == N);
        }
        _numControlFaces += nFaces;
        _cornerCFaceCount[corner] = nFaces;
    }

    _hasVal2IntCorners = (nVal2IntCorners > 0);

    //
    //  Now iterate through the corners to get the number of control
    //  vertices -- taking into account complications identified above:
    //
    int nVal3IntAdjTris   = 0;

    _numControlVerts = N;
    _cornerCVertStart.SetSize(N+1);
    _cornerCVertStart[0] = N;

    for (int corner = 0; corner < N; ++corner) {
        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset   const & cSub = _surface.GetCornerSubset(corner);

        //
        //  Need to keep track of corners at and adjacent to valence-2
        //  interior corners to detect and avoid overlaps:
        //
        int nVal2Overlap = _hasVal2IntCorners ?
                           val2::getFaceOverlap(_surface, corner) : 0;

        //
        //  If all incident faces share a common face size, we can use
        //  formulae to determine contributions -- otherwise inspection
        //  if relevant incident faces is required:
        //
        int nVerts = 0;
        if (cTop.GetCommonFaceSize()) {
            int S = cTop.GetCommonFaceSize();

            if (!cSub.IsBoundary()) {
                if ((cSub._numFacesTotal == 3) && (S == 3)) {
                    nVerts += (++nVal3IntAdjTris == N);
                } else if (cSub._numFacesTotal > 2) {
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
                int nextFace = cTop.GetFaceAfter(2);
                if ((cSub._numFacesTotal == 3) &&
                    (cTop.GetFaceSize(nextFace) == 3)) {
                    nVerts += (++nVal3IntAdjTris == N);
                } else if (cSub._numFacesTotal > 2) {
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
        if (nVal2Overlap) {
            assert(nVerts >= nVal2Overlap);
            nVerts -= nVal2Overlap;
        }

        _numControlVerts += nVerts;
        _cornerCVertStart[corner+1] = _numControlVerts;
    }
}


//
//  Computation of the cache key:
//
bool
IrregularPatchBuilder::packTopologyKey(TopologyCache::Key * key) const {

    //
    //  Keep the bitfield struct local in scope unless needed elsewhere:
    //
    struct KeyBits {
        typedef TopologyCache::Key::IntType IntType;

        //  Bits for general options:
        IntType subdScheme   :  2;
        IntType subdOptions  :  2;  // Encoded Sdc::Options - UNUSED
        IntType sharpLevel   :  4;
        IntType smoothLevel  :  4;

        //  Bits for features of the 3 or 4 corner vertices:
        IntType v0Valence    :  6;
        IntType v1Valence    :  6;
        IntType v2Valence    :  6;
        IntType v3Valence    :  6;
        IntType v0IsBoundary :  1;
        IntType v1IsBoundary :  1;
        IntType v2IsBoundary :  1;
        IntType v3IsBoundary :  1;

        IntType v0IsSharp    :  1;
        IntType v1IsSharp    :  1;
        IntType v2IsSharp    :  1;
        IntType v3IsSharp    :  1;
        IntType v0FaceInRing :  5;
        IntType v1FaceInRing :  5;
        IntType v2FaceInRing :  5;
        IntType v3FaceInRing :  5;

        //
        //  Methods for initializing and accessing as IntType:
        //
        void Clear() { std::memset(this, 0, sizeof(*this)); }

        IntType GetAsInt() const {
            //  This generates fewest compiler issues about type aliasing
            IntType intVar;
            std::memcpy(&intVar, this, sizeof(*this));
            return intVar;
        }

        //
        //  Static methods to determine if bitfields can be used:
        //
        static bool InteriorValenceFits(int n) { return n < (1 << 6); }
        static bool BoundaryValenceFits(int n) { return n < (1 << 5); }

        //  A place holder if future Sdc::Options inhibit use of bitfields:
        static bool OptionsInhibitUsage(Sdc::Options) { return false; }
    };
    assert(sizeof(KeyBits::IntType) == sizeof(KeyBits));

    //
    //  Quickly test if the topology can be packed into bitfields, or
    //  if hashing must be used.  Bitfields cannot be used when the
    //  following features are present:
    //
    //      - any sharp edges of any kind (semi-sharp or inf-sharp)
    //      - any semi-sharp vertices (inf-sharp is 1-bit per corner)
    //      - any incident irregular faces
    //
    //  These can quickly be determined by inspecting the topology tags.
    //  Two other situations are:
    //
    //      - any vertex with valence too high (more than ~6 bits)
    //      - any Sdc::Option that cannot be encoded (in theory only)
    //
    //  The former must inspect the valence of each face-vertex, while
    //  the latter requires inspecting the Sdc::Options -- and this is
    //  called out more as a future possibility...
    //
    //  In theory, if certain Sdc::Options impact the limit surface,
    //  they might need to be encoded, or might not be able to be fully
    //  encoded in future.  This is not currently the case in practice:
    //  boundary interpolation options are essentially unused as the
    //  boundary conditions are explicitly applied; the creasing method
    //  can be ignored here because creases cannot be packed; and the
    //  Catmark triangle subdivision option can be ignored because the
    //  presence of any irregular faces cannot be packed.
    //

    //  Immediate rejection of bitfields:
    CombinedTag combinedTag = _surface.GetTag();
    if (combinedTag.HasSharpEdges() ||
        combinedTag.HasSemiSharpVertices() ||
        combinedTag.HasIrregularFaceSizes()) {
        return false;
    }

    //  Conditional rejection of bitfields for high valence:
    CornerSubset const * subsets = _surface.GetSubsets();

    for (int i = 0; i < _surface.GetFaceSize(); ++i) {
        int valence = subsets[i]._numFacesTotal;
        if (subsets[i].IsBoundary()) {
            if (!KeyBits::BoundaryValenceFits(valence)) return false;
        } else {
            if (!KeyBits::InteriorValenceFits(valence)) return false;
        }
    }

    //  Conditional rejection of bitfields for specific Sdc::Options:
    if (KeyBits::OptionsInhibitUsage(_surface.GetSdcOptionsInEffect())) {
        return false;
    }

    //
    //  Pack the topology of each CornerSubset into bitfields:
    //
    KeyBits keyBits;
    keyBits.Clear();

    keyBits.subdScheme  = _surface.GetSdcScheme();
    keyBits.subdOptions = 0;
    keyBits.sharpLevel  = _options.sharpLevel;
    keyBits.smoothLevel = _options.smoothLevel;

    keyBits.v0Valence    = subsets[0]._numFacesTotal;
    keyBits.v0IsBoundary = subsets[0].IsBoundary();
    keyBits.v0FaceInRing = subsets[0]._numFacesBefore;
    keyBits.v0IsSharp    = subsets[0].IsSharp();

    keyBits.v1Valence    = subsets[1]._numFacesTotal;
    keyBits.v1IsBoundary = subsets[1].IsBoundary();
    keyBits.v1FaceInRing = subsets[1]._numFacesBefore;
    keyBits.v1IsSharp    = subsets[1].IsSharp();

    keyBits.v2Valence    = subsets[2]._numFacesTotal;
    keyBits.v2IsBoundary = subsets[2].IsBoundary();
    keyBits.v2FaceInRing = subsets[2]._numFacesBefore;
    keyBits.v2IsSharp    = subsets[2].IsSharp();

    if (_surface.GetFaceSize() == 4) {
        keyBits.v3Valence    = subsets[3]._numFacesTotal;
        keyBits.v3IsBoundary = subsets[3].IsBoundary();
        keyBits.v3FaceInRing = subsets[3]._numFacesBefore;
        keyBits.v3IsSharp    = subsets[3].IsSharp();
    }

    //  Assign the bitfields to the resulting TopologyKey:
    key->SetFormat(TopologyCache::Key::BITFIELDS);
    key->SetValue(keyBits.GetAsInt());

    return true;
}

bool
IrregularPatchBuilder::hashTopologyKey(TopologyCache::Key * key) const {

    //  WIP - quickly disable hashed caching for debugging, profiling...
    //if (key) return false;

    //
    //  Hashing topology descriptions into 64-bit uints is currently
    //  working as planned.  A few improvements are possible here (e.g.
    //  reducing the number of Hash() calls to fewer or even a single
    //  buffer), but improvements targeting performance should first
    //  assess if they are worthwhile.
    //
    //  The entire topology is hashed in five parts:
    //      - a "header" with some summary information
    //      - an array of topological descriptions of each face-vertex
    //      - an array of sizes of all control faces (when not constant)
    //      - arrays for indices and sharpness of vertices (when present)
    //      - arrays for indices and sharpness of edges (when present)
    //
    TopologyCache::Key::IntType topHash = 0;

    CombinedTag tags = _surface.GetTag();

    //  First, the header:
    struct TopHeader {
        Sdc::SchemeType subdScheme;
        Sdc::Options    subdOptions;

        unsigned int sharpLevel    :  4;
        unsigned int smoothLevel   :  4;
        unsigned int faceSize      : 15;
        unsigned int hasIrregSizes :  1;

        int numVerts;
        int numFaces;
        int numSharpVerts;
        int numSharpEdges;
    };

    TopHeader topHeader;
    std::memset(&topHeader, 0, sizeof(TopHeader));

    topHeader.subdScheme    = _surface.GetSdcScheme();
    topHeader.subdOptions   = _surface.GetSdcOptionsInEffect();
    topHeader.sharpLevel    = _options.sharpLevel;
    topHeader.smoothLevel   = _options.smoothLevel;
    topHeader.faceSize      = _surface.GetFaceSize();
    topHeader.hasIrregSizes = tags.HasIrregularFaceSizes();
    topHeader.numVerts      = _numControlVerts;
    topHeader.numFaces      = _numControlFaces;
    topHeader.numSharpVerts = countSharpControlVertices();
    topHeader.numSharpEdges = countSharpControlEdges();

    topHash = internal::Hash64(&topHeader, sizeof(TopHeader), topHash);

    //  Second, the topology of the corner vertices:
    if (true) {
        int count = topHeader.faceSize;

        struct CornerInfo {
            unsigned int valence    : 15;
            unsigned int isBoundary :  1;
            unsigned int faceInRing : 15;
            unsigned int isSharp    :  1;
        };

        Vtr::internal::StackBuffer<CornerInfo,8,true> cArray(count);
        std::memset(cArray, 0, count * sizeof(CornerInfo));

        for (int i = 0; i < count; ++i) {
            CornerInfo         & cInfo   = cArray[i];
            CornerSubset const & cSubset = _surface.GetCornerSubset(i);

            cInfo.valence    = cSubset._numFacesTotal;
            cInfo.isBoundary = cSubset.IsBoundary();
            cInfo.faceInRing = cSubset._numFacesBefore;
            cInfo.isSharp    = cSubset.IsSharp();
        }

        topHash = internal::Hash64(cArray, count * sizeof(CornerInfo), topHash);
    }

    Vtr::internal::StackBuffer<int,16,true>  iArray;
    Vtr::internal::StackBuffer<float,8,true> fArray;

    //  Third, if necessary, the sizes of incident faces:
    if (topHeader.hasIrregSizes) {
        int nFaces = _numControlFaces;
        iArray.SetSize(nFaces);

        //  Remember, return value is sum of all face sizes
        gatherControlFaceSizes(iArray);

        topHash = internal::Hash64(iArray, nFaces * sizeof(int), topHash);
    }

    //  Fourth, if necessary, arrays for sharp vertices:
    if (topHeader.numSharpVerts) {
        int nVerts = topHeader.numSharpVerts;
        iArray.SetSize(nVerts);
        fArray.SetSize(nVerts);

        int nSharp = gatherControlVertexSharpness(iArray, fArray);
        assert(nSharp == nVerts);

        topHash = internal::Hash64(iArray, nVerts * sizeof(int),   topHash);
        topHash = internal::Hash64(fArray, nVerts * sizeof(float), topHash);
    }

    //  Fifth, if necessary, arrays for sharp edges:
    if (topHeader.numSharpEdges) {
        int nEdges = topHeader.numSharpEdges;
        iArray.SetSize(nEdges * 2);
        fArray.SetSize(nEdges);

        int nSharp = gatherControlEdgeSharpness(iArray, fArray);
        assert(nSharp == nEdges);

        topHash = internal::Hash64(iArray, 2 * nEdges * sizeof(int),   topHash);
        topHash = internal::Hash64(fArray,     nEdges * sizeof(float), topHash);
    }

    //  Assign the bitfields to the resulting TopologyKey:
    key->SetFormat(TopologyCache::Key::HASHED);
    key->SetValue(topHash);

    return true;
}

TopologyCache::Key
IrregularPatchBuilder::computeTopologyKey() const {

    TopologyCache::Key key;

    //
    //  Dispatch the different key encoding strategies here:
    //
    if (packTopologyKey(&key)) {
        return key;
    } else if (hashTopologyKey(&key)) {
        return key;
    } else {
        return TopologyCache::Key();
    }
}

//
//  Search and update of a TopologyCache:
//
IrregularPatchBuilder::IrregPatchType const *
IrregularPatchBuilder::Find(TopologyCache * topologyCachePtr,
        bool * patchIsNewPtr, bool * patchIsCachedPtr) {

    TopologyCache & topologyCache = *topologyCachePtr;

    bool & patchIsNew    = *patchIsNewPtr;
    bool & patchIsCached = *patchIsCachedPtr;

    //
    //  If cache key is not valid, just create and return:
    //
    TopologyCache::Key patchKey = computeTopologyKey();
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

    //  The rare case where another thread has added a patch with the same
    //  key/topology while this one was being built -- so delete the patch
    //  created here and return the one added to the cache elsewhere:
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
    fVertCount = gatherControlFaceSizes(faceSizes);

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
        nSharpEdges = gatherControlEdgeSharpness(creaseIndices, creaseWeights);
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

    //  Construct a TopologyRefiner in order to create a PatchTree:
    typedef Far::TopologyDescriptor Descriptor;
    typedef Far::TopologyRefinerFactory<Descriptor> RefinerFactory;

    RefinerFactory::Options refinerOptions;
    refinerOptions.schemeType    = _surface.GetSdcScheme();
    refinerOptions.schemeOptions = _surface.GetSdcOptionsInEffect();
    refinerOptions.validateFullTopology = true;  // WIP - remove when stable

    Far::TopologyRefiner * refiner =
            RefinerFactory::Create(topDescriptor, refinerOptions);

    //  Create the PatchTree:
    Far::PatchTreeFactory::Options patchTreeOptions;
    patchTreeOptions.includeInteriorPatches = false;
    patchTreeOptions.maxPatchDepthSharp  = _options.sharpLevel;
    patchTreeOptions.maxPatchDepthSmooth = _options.smoothLevel;
    patchTreeOptions.useDoublePrecision  = _options.doublePrecision;
    patchTreeOptions.useStencilTables    = _options.stencilTables;

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

    int numIndices = N;

    //
    //  Assign CV indices "local to" each corner:
    //
    Index const * faceVertIndices = _surface.GetIndices();

    for (int corner = 0; corner < N; ++corner) {
        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset const   & cSub = _surface.GetCornerSubset(corner);

        //  Skip this corner if no control vertices to contribute:
        if (getNumControlVertices(corner) == 0) {
            faceVertIndices += cTop.GetNumFaceVertices();
            continue;
        }

        int nVal2Overlap = _hasVal2IntCorners ?
                           val2::getFaceOverlap(_surface, corner) : 0;

        //
        //  Similar loops here to traverse the faces around each corner,
        //  so potential for some consolidation here...
        //
        if (!cSub.IsBoundary()) {
            int numFaces = cSub._numFacesTotal - 2;
            int nextFace = cTop.GetFaceAfter(2);
            if ((numFaces == 1) && (cTop.GetFaceSize(nextFace) == 3)) {
                //  Special case for val-3 adjacent triangle:
                int fvOffset = cTop.GetFaceVertexOffset(nextFace);

                cvIndices[numIndices++] = faceVertIndices[fvOffset + 1];
            } else {
                for (int j = 0; j < numFaces; ++j) {
                    int S = cTop.GetFaceSize(nextFace);
                    int fvOffset = cTop.GetFaceVertexOffset(nextFace);

                    int L = (j < (numFaces-1)) ? 0 : (1 + nVal2Overlap);
                    int M = (S - 2) - L;
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[numIndices++] = faceVertIndices[fvOffset + k];
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
                int nextFace = cTop.GetFaceAfter(1);
                for (int j = 0; j < numFaces; ++j) {
                    nextFace = cTop.GetFaceNext(nextFace);

                    int S = cTop.GetFaceSize(nextFace);
                    int fvOffset = cTop.GetFaceVertexOffset(nextFace);

                    int M = (S - 2);
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[numIndices++] = faceVertIndices[fvOffset + k];
                    }
                }
                cvIndices[numIndices++] =
                        cTop.GetFaceVertexTrailing(nextFace, faceVertIndices);
            }
            if (cSub._numFacesBefore) {
                int numFaces = cSub._numFacesBefore;
                int nextFace = cTop.GetFaceBefore(cSub._numFacesBefore);
                for (int j = 0; j < numFaces; ++j) {
                    int S = cTop.GetFaceSize(nextFace);
                    int fvOffset = cTop.GetFaceVertexOffset(nextFace);

                    int L = (j < (numFaces-1)) ? 0 : (1 + nVal2Overlap);
                    int M = (S - 2) - L;
                    for (int k = 1; k <= M; ++k) {
                        cvIndices[numIndices++] = faceVertIndices[fvOffset + k];
                    }
                    nextFace = cTop.GetFaceNext(nextFace);
                }
            }
        }
        assert(numIndices == _cornerCVertStart[corner+1]);

        faceVertIndices += cTop.GetNumFaceVertices();
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
        return _numControlFaces * baseFaceSize;
    }

    //
    //  Start with the base face first, return if that's all:
    //
    faceSizes[0] = baseFaceSize;

    if (_numControlFaces == 1) return baseFaceSize;

    //
    //  Otherwise, travers all corners and populate control face sizes:
    //
    int nFaces = 1;
    int sumOfSizes = faceSizes[0];

    for (int corner = 0; corner < _surface.GetFaceSize(); ++corner) {
        if (getNumControlFaces(corner) == 0) continue;

        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset const   & cSub = _surface.GetCornerSubset(corner);

        //
        //  If the subset has a common face size, populate and continue:
        //
        if (!cSub._tag.HasUnCommonFaceSizes()) {
            int N = getNumControlFaces(corner);
            for (int i = 0; i < N; ++i) {
                faceSizes[nFaces++] = baseFaceSize;
            }
            sumOfSizes += N * baseFaceSize;
            continue;
        }

        //
        //  Otherwise, we need to gather sizes of specific faces:
        //
        if (!cSub.IsBoundary()) {
            if (cSub._numFacesTotal > 2) {
                int nextFace = cTop.GetFaceAfter(2);
                for (int i = 2; i < cSub._numFacesTotal; ++i) {
                    int S = cTop.GetFaceSize(nextFace);
                    faceSizes[nFaces++] = S;
                    sumOfSizes += S;

                    nextFace = cTop.GetFaceNext(nextFace);
                }
            } else {
                assert(getNumControlFaces(corner) == 1);
                faceSizes[nFaces++] = _surface.GetFaceSize();
                sumOfSizes += _surface.GetFaceSize();
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

    bool assigning = (vertIndices != 0) && (vertSharpness != 0);

    int nSharpVerts = 0;

    for (int i = 0; i < _surface.GetFaceSize(); ++i) {
        CornerTopology const & cTop = _surface.GetCornerTopology(i);
        CornerSubset   const & cSub = _surface.GetSubsets()[i];

        if (cSub._tag.IsInfSharp() || cSub._tag.IsSemiSharp()) {
            if (assigning) {
                vertIndices[nSharpVerts] = i;
                vertSharpness[nSharpVerts] = cSub._tag.IsInfSharp()
                                           ? Sdc::Crease::SHARPNESS_INFINITE
                                           : cTop.GetVertexSharpness();
            }
            ++ nSharpVerts;
        }
    }
    return nSharpVerts;
}

int
IrregularPatchBuilder::countSharpControlVertices() const {

    return gatherControlVertexSharpness(0, 0);
}

int
IrregularPatchBuilder::gatherControlEdgeSharpness(
        int edgeVertPairs[], float edgeSharpness[]) const {

    bool assigning = (edgeVertPairs != 0) && (edgeSharpness != 0);

    //
    //  First test the forward edge of each corner of the face (avoid
    //  including redundant inf-sharp boundary edges):
    //
    int nSharpEdges = 0;

    int faceSize = _surface.GetFaceSize();

    for (int corner = 0; corner < faceSize; ++corner) {
        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset const   & cSub = _surface.GetCornerSubset(corner);

        if (cSub._tag.HasSharpEdges() &&
                (!cSub.IsBoundary() || cSub._numFacesBefore)) {
            int   cornerFace = cTop.GetFaceInVertex();
            float sharpness  = cTop.GetFaceEdgeSharpness(cornerFace, 0);
            if (Sdc::Crease::IsSharp(sharpness)) {
                if (assigning) {
                    *edgeSharpness++ = sharpness;
                    *edgeVertPairs++ =  corner;
                    *edgeVertPairs++ = (corner + 1) % faceSize;
                }
                nSharpEdges++;
            }
        }
    }

    //
    //  For each corner, test any interior edges connected to vertices
    //  on the perimeter:
    //
    for (int corner = 0; corner < faceSize; ++corner) {
        if (getNumControlFaces(corner) == 0) continue;

        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset const   & cSub = _surface.GetCornerSubset(corner);

        if (!cSub._tag.HasSharpEdges()) continue;

        int cornerFace = cTop.GetFaceInVertex();

        //
        //  Inspect interior edges of the subset -- test sharpness of
        //  the trailing edge of the faces after/before the corner face.
        //
        //  Unfortunately we need the control vertex index at the end
        //  of the edge, and so we need to track the perimeter -- which
        //  requires the face sizes and may wrap around with tris...
        //
        int maxVert  = _numControlVerts;
        int nextVert = getNextControlVertex(corner);

        //  WIP - these blocks are similar enough to warrant merging
        if (!cSub.IsBoundary()) {
            int nextFace = cTop.GetFaceNext(cornerFace);
            for (int i = 2; i < cSub._numFacesTotal; ++i) {
                float sharpness = cTop.GetFaceEdgeSharpness(nextFace, 1);
                if (Sdc::Crease::IsSharp(sharpness)) {
                    if (assigning) {
                        *edgeSharpness++ = sharpness;
                        *edgeVertPairs++ = corner;
                        *edgeVertPairs++ = (nextVert < maxVert)
                                         ? nextVert : faceSize;
                    }
                    nSharpEdges++;
                }
                nextFace  = cTop.GetFaceNext(nextFace);
                nextVert += cTop.GetFaceSize(nextFace) - 2;
            }
        } else {
            if (cSub._numFacesAfter) {
                int nextFace = cTop.GetFaceNext(cornerFace);
                for (int i = 1; i < cSub._numFacesAfter; ++i) {
                    float sharpness = cTop.GetFaceEdgeSharpness(nextFace, 1);
                    if (Sdc::Crease::IsSharp(sharpness)) {
                        if (assigning) {
                            *edgeSharpness++ = sharpness;
                            *edgeVertPairs++ = corner;
                            *edgeVertPairs++ = (nextVert < maxVert)
                                             ? nextVert : faceSize;
                        }
                        nSharpEdges++;
                    }
                    nextFace  = cTop.GetFaceNext(nextFace);
                    nextVert += cTop.GetFaceSize(nextFace) - 2;
                }
                nextVert ++;
            }
            if (cSub._numFacesBefore) {
                int nextFace = cTop.GetFaceBefore(cSub._numFacesBefore);
                for (int i = 1; i < cSub._numFacesBefore; ++i) {
                    nextVert += cTop.GetFaceSize(nextFace) - 2;
                    float sharpness = cTop.GetFaceEdgeSharpness(nextFace, 1);
                    if (Sdc::Crease::IsSharp(sharpness)) {
                        if (assigning) {
                            *edgeSharpness++ = sharpness;
                            *edgeVertPairs++ = corner;
                            *edgeVertPairs++ = (nextVert < maxVert)
                                             ? nextVert : faceSize;
                        }
                        nSharpEdges++;
                    }
                    nextFace  = cTop.GetFaceNext(nextFace);
                }
            }
        }
    }
    return nSharpEdges;
}

int
IrregularPatchBuilder::countSharpControlEdges() const {

    return gatherControlEdgeSharpness(0, 0);
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
        if (getNumControlFaces(corner) == 0) continue;

        CornerTopology const & cTop = _surface.GetCornerTopology(corner);
        CornerSubset const   & cSub = _surface.GetCornerSubset(corner);

        int nVal2Overlap = _hasVal2IntCorners ?
                           val2::getFaceOverlap(_surface, corner) : 0;

        //
        //  WIP - the following blocks for faces before and after the base
        //        face can probably be merged into one iteration -- provided
        //        the transition across the boundary discontinuity is handled
        //        properly (reset next face, adjust next vertex, etc.)
        //
        int nextVert = getNextControlVertex(corner);

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
        } else if (_hasVal2IntCorners && val2::subsetIsVal2Interior(cSub)) {
            //  Must be the special case of the reversed/laminar base face:
            for (int j = faceSize - 1; j >= 0; --j) {
                *nextFaceVert ++ = j;
            }
        }
        if (cSub.IsBoundary() && cSub._numFacesAfter) {
            nextVert ++;
        }
        if (cSub._numFacesBefore) {
            int nextFace = cTop.GetFaceBefore(cSub._numFacesBefore);

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
    return (nextFaceVert - faceVertices);
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
    printf("        has inf-sharp darts  = %d\n", tag.HasInfSharpDarts());
    printf("        has val-2 int verts  = %d\n", _hasVal2IntCorners);

    int nVerts = _numControlVerts;
    int nFaces = _numControlFaces;

    printf("    Topology:\n");
    printf("        num control verts = %3d\n", nVerts);
    printf("            exterior verts:");
    for (int i = 0; i < faceSize; ++i) {
        printf(" %3d", getNumControlVertices(i));
    }
    printf("\n");
    printf("            perimeter base:");
    for (int i = 0; i < faceSize; ++i) {
        if (getNextControlVertex(i) < _numControlVerts) {
            printf(" %3d", getNextControlVertex(i));
        } else {
            printf("   -");
        }
    }
    printf("\n");
    printf("        num control faces = %3d\n", nFaces);
    printf("            exterior faces:");
    for (int i = 0; i < faceSize; ++i) {
        printf(" %3d", getNumControlFaces(i));
    }
    printf("\n");

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
        printf("            perimeter:");
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
    printf(" (%d)", S[0]);
    for (int i = 0; i < faceSize; ++i) {
        printf(" %3d", *p++);
    }
    printf("\n");
    for (int i = 1; i < nFaces; ++i) {
        printf("                      ");
        printf(" (%d)", S[i]);
        for (int j = 0; j < S[i]; ++j) {
            printf(" %3d", *p++);
        }
        printf("\n");
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
