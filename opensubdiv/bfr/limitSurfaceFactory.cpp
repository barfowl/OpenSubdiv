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

#include "../far/topologyRefinerFactory.h"
#include "../far/topologyDescriptor.h"
#include "../far/patchTreeFactory.h"

#include "../bfr/limitSurfaceFactory.h"
#include "../bfr/limitSurface.h"
#include "../bfr/topologyCache.h"
#include "../bfr/faceTopology.h"

#include <map>
#include <cstdio>


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {
//
//  DEBUG - some static variables to keep track of a few things...
//
//#define _BFR_DEBUG_TOP_TYPE_STATS
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
static int __numLinearPatches    = 0;
static int __numRegularPatches   = 0;
static int __numIrregularPatches = 0;
static int __numIrregularCreated = 0;
#endif

//
//  Main constructor and destructor:
//
LimitSurfaceFactory::LimitSurfaceFactory(
    Sdc::SchemeType schemeType,
    Sdc::Options    schemeOptions,
    Options         limitOptions,
    int             numFaces,
    int             numFVarTopologies) :
        _schemeType(schemeType),
        _schemeOptions(schemeOptions),
        _limitOptions(limitOptions),
        _topologyCache(0),
        _numFaces(numFaces),
        _numFVarTopologies(numFVarTopologies) {

    //  Initialize members dependent on mesh topology:
    _regFaceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(_schemeType);

    _linearScheme =
        (Sdc::SchemeTypeTraits::GetLocalNeighborhoodSize(_schemeType) == 0);

    _linearFVarInterp = _linearScheme || (_numFVarTopologies == 0) ||
                       (_schemeOptions.GetFVarLinearInterpolation() ==
                                 Sdc::Options::FVAR_LINEAR_ALL);

    _testBoundaryLimit = !_linearScheme &&
                       (_schemeOptions.GetVtxBoundaryInterpolation() ==
                                 Sdc::Options::VTX_BOUNDARY_NONE);

    //  Assign the topology cache -- externally or to an internal instance:
    if (_limitOptions.ExternalTopologyCache()) {
        _topologyCache = _limitOptions.ExternalTopologyCache();
    } else if (!_limitOptions.DisableTopologyCache()) {
        _topologyCache = new TopologyCache();
    }
}

LimitSurfaceFactory::~LimitSurfaceFactory() {

//  DEBUG - report and reset inventory:
bool debug = false;
if (debug) {
printf("LimitSurfaceFactory destructor:\n");
printf(    " _numFaces            = %6d\n", _numFaces);
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
printf(    "__numLinearPatches    = %6d\n", __numLinearPatches);
printf(    "__numRegularPatches   = %6d\n", __numRegularPatches);
printf(    "__numIrregularPatches = %6d\n", __numIrregularPatches);
printf(    "__numIrregularCreated = %6d\n", __numIrregularCreated);
#endif
if (_topologyCache) {
printf(    " _topologyCache size  = %6d\n", (int) _topologyCache->Size());
} else {
printf(    " _topologyCache size  = %6d (disabled)\n", 0);
}
}
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numLinearPatches    = 0;
__numRegularPatches   = 0;
__numIrregularPatches = 0;
__numIrregularCreated = 0;
#endif

    if (_limitOptions.ExternalTopologyCache() == 0) delete _topologyCache;
}

//
//  The "has limit surface" query for a face is a bit more complicated than
//  may be expected... There are two cases when a face will not have a limit
//  surface:
//
//      - the face is a hole
//      - the boundary interpolation option VTX_BOUNDARY_NONE is assigned:
//          - in which case some, not all, boundary faces have no surface
//
//  Dealing with holes is trivial.  But dealing with boundary faces when
//  "boundary none" is set (which is rarely used) is awkward.  It is not
//  enough to test if a face is on the boundary, i.e. one of its vertices
//  is on a boundary, and return false.  If a boundary face has all of its
//  incident boundary edges (i.e. including all boundary edges incident all
//  of its vertices) then the boundary face has a limit.
//
//  This potentially requires that a full topological description of the
//  face, including all explicitly assigned edge sharpness, be provided.
//
bool
LimitSurfaceFactory::FaceHasLimitSurface(Index faceIndex) const {

    if (isFaceHole(faceIndex)) return false;

    if (_testBoundaryLimit) {
        FaceTopology faceTopology(_schemeType, _schemeOptions);

        if (!populateFaceTopology(faceIndex, faceTopology)) {
            return false;
        }
        if (faceTopology._hasUnorderedVerts) {
            //  WIP - more here for potentially non-manifold vertices
            //      - need to gather indices to identify boundaries
        }
        return faceTopology._hasUnSharpBound ? false : true;
    }
    return true;
}


//
//  Methods supporting construction of linear, regular and irregular patches:
//
void
LimitSurfaceFactory::assignLinearEvaluator(LimitSurface::Evaluator & eval,
        Index faceIndex, int fvarIndex) const {

    //  Use a regular patch if faces is degenerate:
    int faceSize  = getFaceSize(faceIndex);

    int patchSize = (faceSize < 3) ? _regFaceSize : faceSize;

    //  Initialize instance members from the associated irregular patch:
    eval._param = Parameterization(_schemeType, patchSize);

    eval._isRegular = (patchSize == _regFaceSize);
    eval._isLinear  = true;

    eval._regPatchType = (_regFaceSize == 4)
                       ?  Far::PatchDescriptor::QUADS
                       :  Far::PatchDescriptor::TRIANGLES;
    eval._regPatchParam.Clear();

    eval._numControlPoints = patchSize;
    eval._numPatchPoints   = patchSize;

    //  Assign control points from face-vertices:
    eval._controlPoints.SetSize(eval._numControlPoints);
    int * points = &eval._controlPoints[0];

    int count = 0;
    if (fvarIndex < 0) {
        count = getFaceVertexIndices(faceIndex, points);
    } else {
        count = getFaceFVarValueIndices(faceIndex, points, fvarIndex);
    }
    assert(count == faceSize);

    //  Fill in missing indices for a degenerate face:
    if (faceSize < patchSize) {
        for (int i = faceSize; i < patchSize; ++i) {
            points[i] = points[i % faceSize];
        }
    }

    eval._isValid = true;
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numLinearPatches ++;
#endif
}

void
LimitSurfaceFactory::assignRegularEvaluator(LimitSurface::Evaluator & eval,
        FaceTopology const & faceTopology,
        Index        const   faceIndices[],
        CornerSubset const   faceSubsets[]) const {

    if (faceSubsets == 0) {
        faceSubsets = faceTopology._cornerSubsets;
    }

    //
    //  Assign the topological fields of the patch first:
    //
    eval._param = Parameterization(_schemeType, _regFaceSize);

    eval._isRegular = true;
    eval._isLinear  = false;

    int patchSize = 0;
    int patchBoundaryMask = 0;
    if (_regFaceSize == 4) {
        patchSize = 16;

        CornerSubset const * corner = faceSubsets;
        patchBoundaryMask =
            ((corner[0]._isBoundary & (corner[0]._numFacesBefore == 0)) << 0) |
            ((corner[1]._isBoundary & (corner[1]._numFacesBefore == 0)) << 1) |
            ((corner[2]._isBoundary & (corner[2]._numFacesBefore == 0)) << 2) |
            ((corner[3]._isBoundary & (corner[3]._numFacesBefore == 0)) << 3);

        eval._regPatchType = Far::PatchDescriptor::REGULAR;
    } else {
        patchSize = 12;

        eval._regPatchType = Far::PatchDescriptor::LOOP;
    }
    eval._regPatchParam.Set(0, 0, 0, 0, 0, patchBoundaryMask, 0, true);

    eval._numControlPoints = patchSize;
    eval._numPatchPoints   = patchSize;

    //
    //  Now gather the patch control points from FaceTopology and indices:
    //
    eval._controlPoints.SetSize(patchSize);
    int * P = &eval._controlPoints[0];
    if (_regFaceSize == 4) {
        faceTopology.GatherRegularPatchPoints4(faceSubsets, faceIndices, P);
    } else {
        faceTopology.GatherRegularPatchPoints3(faceSubsets, faceIndices, P);
    }

    eval._isValid = true;
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numRegularPatches ++;
#endif
}

void
LimitSurfaceFactory::assignIrregularEvaluator(LimitSurface::Evaluator & eval,
        FaceTopology const & faceTopology,
        Index        const   faceIndices[],
        CornerSubset const   faceSubsets[]) const {

    if (faceSubsets == 0) faceSubsets = faceTopology._cornerSubsets;

//bool debug = faceTopology._hasIncIrregFaces || faceTopology._hasSharpEdges;
//if (debug) faceTopology.printControlTopology(faceIndices);

    //
    //  Identify the patch -- retrieved from the cache or newly constructed:
    //
    bool patchIsNew    = false;
    bool patchIsCached = false;
    Far::PatchTree const * patch = findIrregularPatch(
                faceTopology, faceSubsets, patchIsNew, patchIsCached);

    //
    //  Assign the topological fields of the patch first:
    //
    eval._param = Parameterization(_schemeType, faceTopology.GetFaceSize());

    eval._isRegular = false;
    eval._isLinear  = false;

    eval._irregPatch = patch;
    eval._irregOwner = patchIsNew && !patchIsCached;

    eval._numControlPoints = patch->GetNumControlPoints();
    eval._numPatchPoints   = patch->GetNumPointsTotal();

    //
    //  Now gather the patch control points from FaceTopology and indices:
    //
    eval._controlPoints.SetSize(eval._numControlPoints);
    faceTopology.GatherControlVertexIndices(faceSubsets, faceIndices,
                                            &eval._controlPoints[0]);

    eval._isValid = true;
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numIrregularPatches ++;
__numIrregularCreated += patchIsNew;
#endif
}

void
LimitSurfaceFactory::copyNonLinearEvaluator(LimitSurface::Evaluator & dstEval,
        LimitSurface::Evaluator const & srcEval,
        FaceTopology const &            faceTopology,
        Index        const              dstIndices[],
        CornerSubset const              dstSubsets[]) const {

    //  Should be creating a linear patch directly rather than copying:
    assert(!srcEval._isLinear);

    //
    //  Assign the topological fields of the patch first:
    //
    dstEval._param = srcEval._param;

    dstEval._isLinear  = false;
    dstEval._isRegular = srcEval._isRegular;

    dstEval._numControlPoints = srcEval._numControlPoints;
    dstEval._numPatchPoints   = srcEval._numPatchPoints;

    dstEval._controlPoints.SetSize(srcEval._numControlPoints);

    //
    //  Assign regular vs irregular fields and gather control accordingly:
    //
    if (dstEval._isRegular) {
        dstEval._regPatchType  = srcEval._regPatchType;
        dstEval._regPatchParam = srcEval._regPatchParam;

        if (_regFaceSize == 4) {
            faceTopology.GatherRegularPatchPoints4(dstSubsets, dstIndices,
                                                  &dstEval._controlPoints[0]);
        } else {
            faceTopology.GatherRegularPatchPoints3(dstSubsets, dstIndices,
                                                  &dstEval._controlPoints[0]);
        }
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numRegularPatches ++;
#endif
    } else {
        dstEval._irregPatch = srcEval._irregPatch;
        dstEval._irregOwner = false;

        faceTopology.GatherControlVertexIndices(dstSubsets, dstIndices,
                                               &dstEval._controlPoints[0]);
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numIrregularPatches ++;
__numIrregularCreated += dstEval._irregOwner;
#endif
    }

    dstEval._isValid = true;
}

IrregPatchPtr
LimitSurfaceFactory::findIrregularPatch(
        FaceTopology const & faceTopology,
        CornerSubset const   faceSubsets[],
        bool               & patchIsNew,
        bool               & patchIsCached) const {

    //
    //  Identify the irregular patch from the cache if specified:
    //
    patchIsNew    = false;
    patchIsCached = false;

    IrregPatchPtr      patch = 0;
    TopologyCache::Key patchKey;

    //  Try to retrieve the patch from the cache:
    if (_topologyCache) {
        patchKey = faceTopology.ComputeTopologyKey(faceSubsets);
        if (patchKey.IsValid()) {
            patch = _topologyCache->Find(patchKey);
            patchIsCached = (patch != 0);
        }
    }

    //  Create a new patch (no cache, no valid key or not cached):
    if (patch == 0) {
        patch = buildIrregularPatch(faceTopology, faceSubsets);
        patchIsNew = true;
    }

    //  Update the cache with the new patch when necessary:
    if (patchKey.IsValid() && !patchIsCached) {
        //  Beware the race condition when adding to the cache:
        IrregPatchPtr patchAdded = _topologyCache->Add(patchKey,patch);
        if (patchAdded != patch) {
            delete patch;
            patch = patchAdded;
        }
        patchIsCached = true;
    }
    return patch;
}

//
//  WIP - the construction of the irregular patch, i.e. the Far::PatchTree,
//  will be moved elsewhere.  The Factory doesn't need to know the details
//  of the IrregPatchType -- only that is support some minimal interface.
//  So construction will be moved to some kind of IrregPatchBuilder that
//  will assemble it from the FaceTopology and anything else required (i.e.
//  options related to approximation passed to the Factory as Options).
//
IrregPatchPtr
LimitSurfaceFactory::buildIrregularPatch(
        FaceTopology const & faceTopology,
        CornerSubset const   faceSubsets[]) const {

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
    int vertCount  = faceTopology.GetNumControlVertices(faceSubsets);
    int faceCount  = faceTopology.GetNumControlFaces(faceSubsets);
    int fVertCount = 0;

    Vtr::internal::StackBuffer<int, 64,true> faceSizes(faceCount);
    if (faceTopology._hasIncIrregFaces) {
        fVertCount = faceTopology.GatherControlFaceSizes(faceSubsets,
                                                         faceSizes);
    } else {
        fVertCount = faceCount * _regFaceSize;
        std::fill(&faceSizes[0], &faceSizes[faceCount], _regFaceSize);
    }

    Vtr::internal::StackBuffer<int,256,true> faceVerts(fVertCount);
    faceTopology.GatherControlFaceVertices(faceSubsets, vertCount, faceVerts);

    //  Gather sharpness for corner vertices:
    int faceSize = faceTopology.GetFaceSize();
    Vtr::internal::StackBuffer<float,8,true> cornerWeights(faceSize);
    Vtr::internal::StackBuffer<Index,8,true> cornerIndices(faceSize);

    int nSharpVerts = faceTopology.GatherControlVertexSharpness(faceSubsets,
                        cornerIndices, cornerWeights);

    //  Gather sharpness for edges:
    Vtr::internal::StackBuffer<float,8,true>  creaseWeights(vertCount);
    Vtr::internal::StackBuffer<Index,16,true> creaseIndices(vertCount * 2);

    int nSharpEdges = 0;
    if (faceTopology._hasSharpEdges) {
        nSharpEdges = faceTopology.GatherControlEdgeSharpness(faceSubsets,
                        creaseIndices, creaseWeights);
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
    Sdc::Options localSchemeOptions = _schemeOptions;
    localSchemeOptions.SetVtxBoundaryInterpolation(
                                Sdc::Options::VTX_BOUNDARY_EDGE_ONLY);

    //  Construct a TopologyRefiner in order to create a PatchTree:
    typedef Far::TopologyDescriptor Descriptor;
    typedef Far::TopologyRefinerFactory<Descriptor> RefinerFactory;

    RefinerFactory::Options refinerOptions;
    refinerOptions.schemeType = _schemeType;
    refinerOptions.schemeOptions = localSchemeOptions;
    refinerOptions.validateFullTopology = true;  // WIP - remove when stable

    Far::TopologyRefiner * refiner =
            RefinerFactory::Create(topDescriptor, refinerOptions);

    //  Create the PatchTree:
    Far::PatchTreeFactory::Options patchTreeOptions;
    patchTreeOptions.maxPatchDepthSharp = _limitOptions.MaxLevelPrimary();
    patchTreeOptions.maxPatchDepthSmooth = _limitOptions.MaxLevelSecondary();
    patchTreeOptions.includeInteriorPatches = false;

    IrregPatchPtr patchTree =
            Far::PatchTreeFactory::Create(*refiner, patchTreeOptions);

    assert(patchTree->GetNumControlPoints() == vertCount);

    delete refiner;
    return patchTree;
}


//
//  Methods to deal with topology assembly and inspection:
//
bool
LimitSurfaceFactory::populateFaceTopology(Index faceIndex,
        FaceTopology & faceTopology) const {

    int N = getFaceSize(faceIndex);

    faceTopology.Initialize(N);

    for (int i = 0; i < N; ++i) {
        faceTopology._faceInVertex[i] = populateFaceCornerTopology(
                faceIndex, i, faceTopology._vertexTopology[i]);

        //  Subclass returning negative here indicates unsupported features
        //  or some other kind of failure:
        if (faceTopology._faceInVertex[i] < 0) return false;
    }

    faceTopology.Finalize();

    //  WIP - eventually need face-vert indices here to fully initialize
    //  manifold subsets at non-manifold vertices:
    faceTopology.InitializeVertexSubsets();

    //  Debugging output:
    bool debugFace  = false;
    if (debugFace) {
        bool debugVerts = false;

        Vtr::internal::StackBuffer<Index,1024,true> faceIndices;
        faceIndices.SetSize(faceTopology._numFaceVertsTotal);
        gatherFaceTopologyIndices(faceIndex, faceTopology, faceIndices);

        printf("    populateFaceTopology(face = %d):\n", faceIndex);
        faceTopology.print(faceIndices, debugVerts);
    }
    return true;
}

int
LimitSurfaceFactory::gatherFaceTopologyIndices(
        Index                faceIndex,
        FaceTopology const & faceTopology,
        Index                faceTopologyIndices[],
        int                  fvarIndex) const {

    int faceSize = faceTopology.GetFaceSize();

    Index * indices  = faceTopologyIndices;
    int     nIndices = 0;

    for (int i = 0; i < faceSize; ++i) {
        int numFaceVerts = (fvarIndex < 0) ?
                getFaceCornerVertexIndices(faceIndex, i, indices) :
                getFaceCornerFVarValueIndices(faceIndex, i, indices, fvarIndex);

        //  WIP - what should behavior be when not getting expected number?
        assert(numFaceVerts == faceTopology._vertexTopology[i]._numFaceVerts);

        indices  += numFaceVerts;
        nIndices += numFaceVerts;
    }
    return nIndices;
}

//
//  Main method to populate an instance of LimitSurface:
//
//  Notes on presence/absence of a limit surface:
//      Unfortunately it is not trivial to detect when a face does not
//  have an associated limit surface.  There are two cases when a face
//  will not have a limit surface:
//
//      - the face is a hole
//      - boundary interpolation option "none" is assigned:
//          - in which case some, not all, boundary faces have no limit
//
//  Dealing with holes is trivial.  But dealing with boundary faces when
// "boundary none" is set (not very often) is difficult.  It's not enough
//  to test if a face is a boundary -- if a boundary face has all of its
//  incident boundary edges (i.e. all boundary edges incident to all of
//  its face-vertices) then the boundary face has a limit surface.
//
//  So a near full description -- sharpness included -- of each face-vertex
//  is necessary. Given that is the case, we might as well forge ahead and
//  simply gather all information for the base face, and -- only in the
//  case of "boundary none" being set -- abort if a boundary vertex with
//  unsharpened boundary edges is encountered.
//
//  WIP - deal with above with assisted tagging of the vertex topology,
//  i.e. when first inspected, detect if a boundary vertex was explicitly
//  sharpened.
//
bool
LimitSurfaceFactory::Populate(LimitSurface & s,
        Index baseFace,
        EvaluatorOptions evalOptions) const {

    //
    //  Clear and re-initialize the existing instance before re-populating.
    //
    s.clear();
    s.initialize(_numFVarTopologies);

    s._faceIndex = baseFace;

    //
    //  Make sure we have a limit surface before proceeding:
    //
    if (isFaceHole(baseFace)) return false;

    if (_testBoundaryLimit) {
        //  WIP - factor this later to avoid repeated topology gathering
        if (!FaceHasLimitSurface(baseFace)) return false;
    }

    //
    //  Determine if we need to gather the topological neighborhood of the
    //  face -- which is required for any non-linear limit surface:
    //
    int faceSize = getFaceSize(baseFace);

    bool isFaceDegenerate = (faceSize < 3);

    bool hasNonLinearVtxEvaluator  = evalOptions.CreateVertexEvaluator() &&
                                     !_linearScheme && !isFaceDegenerate;
    bool hasNonLinearFVarEvaluator = evalOptions.GetNumFVarEvaluators() &&
                                     !_linearFVarInterp && !isFaceDegenerate;

    bool needTopology = hasNonLinearVtxEvaluator || hasNonLinearFVarEvaluator;

    //
    //  The main "buffers" for face topology and control vertex indices --
    //  initialize below only when necessary:
    //
    typedef Vtr::internal::StackBuffer<Index,96,true> IndexBuffer;

    FaceTopology faceTopology(_schemeType, _schemeOptions);
    IndexBuffer  faceIndices;

    if (needTopology) {
        if (!populateFaceTopology(baseFace, faceTopology)) {
            return false;
        }

        //  It may additionally be necessary to gather control vertex indices
        //  to identify the topology around vertices that did not specify an
        //  ordering to their incident faces (possibly non-manifold).  Do so
        //  to resolve this, and also gather them here if needed otherwise so
        //  that we don't have to test later if they were already gathered:
        //  
        bool needVertexIndices = hasNonLinearVtxEvaluator ||
                                 faceTopology._hasUnorderedVerts;
        if (needVertexIndices) {
            faceIndices.SetSize(faceTopology._numFaceVertsTotal);
            gatherFaceTopologyIndices(baseFace, faceTopology, faceIndices);

            if (faceTopology._hasUnorderedVerts) {
                //faceTopology.ResolveUnorderedCornerTopology(faceIndices);
            }
        }

        //  WIP - this will be removed once all cases are supported
        if (faceTopology.IsUnsupported()) {
            hasNonLinearVtxEvaluator  = false;
            hasNonLinearFVarEvaluator = false;
        }
    }

    //  Assign a parameterization (reverting to regular when degenerate)
    //  then assign the varying Evaluator first (trivial) followed by the
    //  vertex Evaluator and face-varying Evaluators last.
    //
    //  It is important to process the vertex Evaluator before the
    //  face-varying Evaluators as its patch representation may be
    //  shared by them, and the buffer used to gather control point
    //  indices can then also be re-used for face-varying.
    //
    if (!isFaceDegenerate) {
        s.parameterize(Parameterization(_schemeType, faceSize));
    } else {
        s.parameterize(Parameterization(_schemeType, _regFaceSize));
    }

    if (evalOptions.CreateVaryingEvaluator()) {
        assignLinearEvaluator(s._varEval, baseFace);
    }

    if (evalOptions.CreateVertexEvaluator()) {
        if (!hasNonLinearVtxEvaluator) {
            assignLinearEvaluator(s._vtxEval, baseFace);
        } else if (faceTopology.IsRegular()) {
            assignRegularEvaluator(s._vtxEval, faceTopology, faceIndices);
        } else {
            assignIrregularEvaluator(s._vtxEval, faceTopology, faceIndices);
        }
    }

    if (evalOptions.GetNumFVarEvaluators()) {
        Vtr::internal::StackBuffer<CornerSubset,8,true> fvarCorners(faceSize);

        int         numSpecified   = evalOptions.GetNumFVarEvaluators();
        int const * fvarsSpecified = evalOptions.GetFVarEvaluatorIndices();

        for (int i = 0; i < numSpecified; ++i) {
            int fvarIndex = fvarsSpecified ? fvarsSpecified[i] : i;
            if (fvarIndex >= _numFVarTopologies) continue;

            LimitSurface::Evaluator & fvarEval = s._fvarEval[fvarIndex];

            if (!hasNonLinearFVarEvaluator) {
                assignLinearEvaluator(fvarEval, baseFace, fvarIndex);
                continue;
            }

            //  Recall we can re-use the index buffer for face-varying:
            gatherFaceTopologyIndices(baseFace,
                    faceTopology, faceIndices, fvarIndex);

            bool fvarMatches = faceTopology.IdentifyFaceVaryingSubsets(
                                                faceIndices, fvarCorners);

            if (fvarMatches && s._vtxEval._isValid) {
                copyNonLinearEvaluator(fvarEval, s._vtxEval, faceTopology,
                                       faceIndices, fvarCorners);
            } else if (faceTopology.IsRegular(fvarCorners)) {
                assignRegularEvaluator(fvarEval, faceTopology,
                                       faceIndices, fvarCorners);
            } else {
                assignIrregularEvaluator(fvarEval, faceTopology,
                                         faceIndices, fvarCorners);
            }
        }
    }
    return true;
}

LimitSurface *
LimitSurfaceFactory::Create(Index baseFace,
        EvaluatorOptions evalOptions) const {

    //
    //  Avoid allocation if face trivially has no limit (a hole).
    //  Still need to return 0 if face has no limit surface due to
    //  more complex conditions (e.g. unsharpened boundary faces):
    //
    if (isFaceHole(baseFace)) return 0;

    LimitSurface * limitSurface = new LimitSurface();

    if (!Populate(*limitSurface, baseFace, evalOptions)) {
        delete limitSurface;
        return 0;
    }
    return limitSurface;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
