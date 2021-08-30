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
#include "../bfr/surfaceDescriptor.h"
#include "../bfr/regularPatchBuilder.h"
#include "../bfr/irregularPatchBuilder.h"

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
static int __numLinearPatches     = 0;
static int __numRegularPatches    = 0;
static int __numIrregularPatches  = 0;
static int __numIrregularUncached = 0;
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

    _testTriangleLimit = !_linearScheme && (_regFaceSize == 3);

    //  Assign the topology cache -- externally or to an internal instance:
    if (_limitOptions.ExternalTopologyCache()) {
        _topologyCache = _limitOptions.ExternalTopologyCache();
    } else if (!_limitOptions.DisableTopologyCache()) {
        _topologyCache = new TopologyCache();
    }
}

LimitSurfaceFactory::~LimitSurfaceFactory() {

#ifdef _BFR_DEBUG_TOP_TYPE_STATS
//  DEBUG - report and reset inventory:
printf("LimitSurfaceFactory destructor:\n");
printf("     _numFaces             = %6d\n", _numFaces);
printf("\n");
printf("    __numLinearPatches     = %6d\n", __numLinearPatches);
printf("    __numRegularPatches    = %6d\n", __numRegularPatches);
printf("    __numIrregularPatches  = %6d\n", __numIrregularPatches);
if (_topologyCache) {
printf("\n");
printf("    __numIrregularUncached = %6d\n", __numIrregularUncached);
printf("    num irregular in cache = %6d\n", (int)_topologyCache->Size());
}
__numLinearPatches     = 0;
__numRegularPatches    = 0;
__numIrregularPatches  = 0;
__numIrregularUncached = 0;
#endif

    if (_limitOptions.ExternalTopologyCache() == 0) delete _topologyCache;
}

//
//  Notes on presence/absence of a limit surface...
//
//  Unfortunately it is not trivial to detect when a face does not have
//  an associated limit surface.  There are a few cases when a face will
//  not have a limit surface -- divided into simple and complex cases:
//
//      - simple:
//          - the face is a hole
//          - the face is degenerate (< 3 edges)
//      - complex:
//          - boundary interpolation option "none" is assigned:
//              - in which case some, not all, boundary faces have no limit
//          - Loop subdivision is applied to non-triangles
//
//  The simple cases are, as the name suggests, simple.  But the complex
//  cases require a greater inspection of the topological neighborhood of
//  the face.
//
//  With boundary faces when "boundary none" is set (not very often) it is
//  not enough to test if a face is a boundary -- if a boundary face has all
//  of its incident boundary edges (i.e. all boundary edges incident to all
//  of its face-vertices) then the boundary face has a limit surface.  This
//  requires a complete topological description of each corner of the face.
//
//  Similarly, the case of Loop subdivision in the presence of non-triangles
//  required determining if any corner of the face has an incidendent face
//  that is not a triangle.
//
//  The method here inspects a corner at a time and tries to reject a face
//  without a limit surface as soon as possible. But most cases are going to
//  require inspection of all corners -- and that same inspection is likely
//  to be applied later when constructing the limit.
//
bool
LimitSurfaceFactory::FaceHasLimitSurface(Index faceIndex) const {

    int faceSize = getFaceSize(faceIndex);

    if (faceSize < 3) return false;

    if (isFaceHole(faceIndex)) return false;

    bool inspectTopology = _testBoundaryLimit || _testTriangleLimit;
    if (inspectTopology) {
        CornerTopology   cTop;
        VertexTopology & vTop = cTop.GetVertexTopology();

        for (int i = 0; i < faceSize; ++i) {
            //  Have the subclass load VertexTopology and finalize:
            cTop.Initialize(faceSize);

            int faceInRing = populateFaceCornerTopology(faceIndex, i, vTop);
            if (faceInRing < 0) return false;

            cTop.Finalize(_regFaceSize, faceInRing);

            //  Inspect the tag tod reject cases with no limit surface:
            CornerTag cTag = cTop.GetTag();

            if (_testBoundaryLimit) {
                if (cTag.IsUnOrdered()) {
                    //  WIP - more needed here to fully resolve topology
                    //      - need to gather indices to identify boundaries
                }
                if (cTag.HasNonSharpBoundary()) return false;
            }
            if (_testTriangleLimit) {
                if (cTag.HasIrregularFaceSizes()) return false;
            }
        }
    }
    return true;
}


//
//  Methods supporting construction of linear, regular and irregular patches:
//
void
LimitSurfaceFactory::assignLinearEvaluator(LimitSurface::Evaluator & eval,
        Index faceIndex, int fvarIndex) const {

    //  Initialize instance members from the associated irregular patch:
    int faceSize  = getFaceSize(faceIndex);

    eval._param = Parameterization(_schemeType, faceSize);

    eval._isRegular = (faceSize == _regFaceSize);
    eval._isLinear  = true;

    eval._regPatchType = (_regFaceSize == 4)
                       ?  Far::PatchDescriptor::QUADS
                       :  Far::PatchDescriptor::TRIANGLES;
    eval._regPatchParam.Clear();

    //
    //  Finally, gather patch control points from the appropriate indices:
    //
    eval._numControlPoints = faceSize;
    eval._numPatchPoints   = faceSize;

    eval._controlPoints.SetSize(eval._numControlPoints);
    int count = 0;
    if (fvarIndex < 0) {
        count = getFaceVertexIndices(faceIndex, &eval._controlPoints[0]);
    } else {
        count = getFaceFVarValueIndices(faceIndex, &eval._controlPoints[0],
                                                   fvarIndex);
    }
    //  If subclass fails to get indices, Evaluator will remain invalid
    if (count < faceSize) return;

    eval._isValid = true;
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numLinearPatches ++;
#endif
}

void
LimitSurfaceFactory::assignRegularEvaluator(
        LimitSurface::Evaluator & eval,
        SurfaceDescriptor const & surface) const {

    //
    //  Assign the parameterization and discriminants first:
    //
    eval._param = Parameterization(_schemeType, _regFaceSize);

    eval._isRegular = true;
    eval._isLinear  = false;

    //
    //  Assemble the regular patch:
    //
    RegularPatchBuilder builder(surface);

    eval._regPatchType = builder.GetPatchType();
    eval._regPatchParam.Set(0, 0, 0, 0, 0, builder.GetBoundaryMask(), 0, true);

    //
    //  Gather the patch control points from the given indices:
    //
    eval._numControlPoints = builder.GetNumControlVertices();
    eval._numPatchPoints   = eval._numControlPoints;

    eval._controlPoints.SetSize(eval._numControlPoints);
    builder.GatherControlVertexIndices(&eval._controlPoints[0]);

    eval._isValid = true;
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numRegularPatches ++;
#endif
}

void
LimitSurfaceFactory::assignIrregularEvaluator(
        LimitSurface::Evaluator & eval,
        SurfaceDescriptor const & surface) const {

    //
    //  Assign the parameterization and discriminants first:
    //
    eval._param = Parameterization(_schemeType, surface.GetFaceSize());

    eval._isRegular = false;
    eval._isLinear  = false;

    //
    //  Construct a new irregular patch or identify one from the cache:
    //
    IrregularPatchBuilder::Options buildOptions;
    buildOptions.sharpLevel  = _limitOptions.MaxLevelPrimary();
    buildOptions.smoothLevel = _limitOptions.MaxLevelSecondary();

    IrregularPatchBuilder builder(surface, buildOptions);

//bool debug = surface._topology.GetTag().HasIrregularFaceSizes() ||
//             surface._topology.GetTag().HasSharpEdges();
//if (debug) builder.print();

    if (_topologyCache == 0) {
        eval._irregPatch = builder.Build();
        eval._irregOwner = true;
    } else {
        bool isNew    = false;
        bool isCached = false;
        eval._irregPatch = builder.Find(*_topologyCache, isNew, isCached);
        eval._irregOwner = isNew && !isCached;
    }

    //
    //  Gather the patch control points from the given indices:
    //
    eval._numControlPoints = eval._irregPatch->GetNumControlPoints();
    eval._numPatchPoints   = eval._irregPatch->GetNumPointsTotal();

    eval._controlPoints.SetSize(eval._numControlPoints);
    builder.GatherControlVertexIndices(&eval._controlPoints[0]);

    eval._isValid = true;
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numIrregularPatches  ++;
__numIrregularUncached += eval._irregOwner;
#endif
}

void
LimitSurfaceFactory::copyNonLinearEvaluator(
        LimitSurface::Evaluator       & dstEval,
        LimitSurface::Evaluator const & srcEval,
        SurfaceDescriptor const       & surface) const {

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
    //  Assign regular/irregular fields and gather control points:
    //
    if (dstEval._isRegular) {
        dstEval._regPatchType  = srcEval._regPatchType;
        dstEval._regPatchParam = srcEval._regPatchParam;

        RegularPatchBuilder builder(surface);
        assert(builder.GetNumControlVertices() == dstEval._numControlPoints);

        builder.GatherControlVertexIndices(&dstEval._controlPoints[0]);
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numRegularPatches ++;
#endif
    } else {
        dstEval._irregPatch = srcEval._irregPatch;
        dstEval._irregOwner = false;

        IrregularPatchBuilder builder(surface);
        assert(builder.GetNumControlVertices() == dstEval._numControlPoints);

        builder.GatherControlVertexIndices(&dstEval._controlPoints[0]);
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numIrregularPatches  ++;
__numIrregularUncached += dstEval._irregOwner;
#endif
    }

    dstEval._isValid = true;
}


//
//  Methods to deal with topology assembly and inspection:
//
bool
LimitSurfaceFactory::gatherFaceNeighborhoodTopology(Index faceIndex,
        FaceTopology & faceTopology) const {

    int N = getFaceSize(faceIndex);

    faceTopology.Initialize(N);

    for (int i = 0; i < N; ++i) {
        CornerTopology & cornerTop = faceTopology.GetTopology(i);
        VertexTopology & vertexTop = cornerTop.GetVertexTopology();

        cornerTop.Initialize(N);

        //  Subclass returning negative here indicates unsupported features
        //  or some other kind of failure:
        int faceInRing = populateFaceCornerTopology(faceIndex, i, vertexTop);
        if (faceInRing < 0) return false;

        cornerTop.Finalize(_regFaceSize, faceInRing);
    }

    faceTopology.Finalize();

    return true;
}

int
LimitSurfaceFactory::gatherFaceNeighborhoodIndices(Index faceIndex,
        FaceTopology const & faceTopology,
        Index                controlIndices[],
        int                  fvarIndex) const {

    int faceSize = faceTopology.GetFaceSize();

    Index * indices  = controlIndices;
    int     nIndices = 0;

    for (int i = 0; i < faceSize; ++i) {
        int numFaceVerts = (fvarIndex < 0) ?
                getFaceCornerVertexIndices(faceIndex, i, indices) :
                getFaceCornerFVarValueIndices(faceIndex, i, indices, fvarIndex);

        if (numFaceVerts != faceTopology.GetNumFaceVertices(i)) {
            return -1;
        }

        indices  += numFaceVerts;
        nIndices += numFaceVerts;
    }
    return nIndices;
}

//
//  Main method to populate an instance of LimitSurface:
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

    //  Make sure we have a limit surface before proceeding:
    //  WIP - factor this later to avoid repeated topology inspection
    if (!FaceHasLimitSurface(baseFace)) {
        return false;
    }

    //
    //  Determine if we need to gather the topological neighborhood --
    //  required for any non-linear limit surface:
    //
    //  WIP - consider splitting the rest of this into two separate methods:
    //        one to initialize the linear Evaluators and the other for the
    //        non-linear Evaluators
    //      - that will separate all the topological analysis from the
    //        simpler linear cases and make both much clearer
    //
    bool hasNonLinearVtxEvaluator  = evalOptions.CreateVertexEvaluator() &&
                                     !_linearScheme;
    bool hasNonLinearFVarEvaluator = evalOptions.GetNumFVarEvaluators() &&
                                     !_linearFVarInterp;

    bool needTopology = hasNonLinearVtxEvaluator || hasNonLinearFVarEvaluator;

    //
    //  Local "buffers" for the face topology, control vertex indices,
    //  and complete description of the limit surface:
    //
    typedef Vtr::internal::StackBuffer<Index,72,true> IndexBuffer;

    FaceTopology faceTopology(_schemeType, _schemeOptions);

    SurfaceDescriptor vtxSurface(faceTopology);
    IndexBuffer       vtxIndices;

    if (needTopology) {
        if (!gatherFaceNeighborhoodTopology(baseFace, faceTopology)) {
            return false;
        }

        vtxIndices.SetSize(faceTopology._numFaceVertsTotal);
        if (gatherFaceNeighborhoodIndices(baseFace, faceTopology,
                    vtxIndices, -1) < 0) {
            return false;
        }

        if (faceTopology.GetTag().HasUnOrderedVertices()) {
            //  WIP - use indices to resolve unordered topology
            //faceTopology.ResolveUnorderedCornerTopology(vtxIndices);
        }

        vtxSurface.InitializeVertex(vtxIndices);

        //  WIP - revert to linear for temporarily unsupported cases:
        if (faceTopology.IsUnsupported()) {
            hasNonLinearVtxEvaluator  = false;
            hasNonLinearFVarEvaluator = false;
        }

        //  WIP - debugging
        bool debugFaceTopology = false;
        if (debugFaceTopology) {
            printf("SurfaceDescriptor(face = %d):\n", baseFace);
            vtxSurface.print();
        }
    }

    //  Assign a parameterization then assign the varying Evaluator first
    //  (trivial) followed by the vertex Evaluator and face-varying
    //  Evaluators last.
    //
    //  It is important to process the vertex Evaluator before the face-
    //  varying Evaluators as its patch representation may be shared by
    //  them, and the buffer used to gather control point indices can then
    //  also be re-used for face-varying.
    //
    int faceSize = getFaceSize(baseFace);

    s.parameterize(Parameterization(_schemeType, faceSize));

    if (evalOptions.CreateVaryingEvaluator()) {
        assignLinearEvaluator(s._varEval, baseFace, -1);
    }

    if (evalOptions.CreateVertexEvaluator()) {
        if (!hasNonLinearVtxEvaluator) {
            assignLinearEvaluator(s._vtxEval, baseFace, -1);
        } else if (vtxSurface.IsRegular()) {
            assignRegularEvaluator(s._vtxEval, vtxSurface);
        } else {
            assignIrregularEvaluator(s._vtxEval, vtxSurface);
        }
    }

    if (evalOptions.GetNumFVarEvaluators()) {
        //  We can re-use the vertex index buffer at this point:
        IndexBuffer & fvarIndices = vtxIndices;

        int         numSpecified   = evalOptions.GetNumFVarEvaluators();
        int const * fvarsSpecified = evalOptions.GetFVarEvaluatorIndices();

        for (int i = 0; i < numSpecified; ++i) {
            int fvarID = fvarsSpecified ? fvarsSpecified[i] : i;
            if (fvarID >= _numFVarTopologies) continue;

            LimitSurface::Evaluator & fvarEval = s._fvarEval[fvarID];

            if (!hasNonLinearFVarEvaluator) {
                assignLinearEvaluator(fvarEval, baseFace, fvarID);
                continue;
            }

            //  Skip if subclass fails to gather indices for given fvarID
            if (gatherFaceNeighborhoodIndices(baseFace, faceTopology,
                    fvarIndices, fvarID) < 0) {
                continue;
            }

            //  Detect matching topology or regular and dispatch accordingly:
            SurfaceDescriptor fvarSurface(faceTopology);

            fvarSurface.InitializeFaceVarying(fvarIndices, vtxSurface);

            if (fvarSurface.MatchesVertexTopology() && s._vtxEval._isValid) {
                copyNonLinearEvaluator(fvarEval, s._vtxEval, fvarSurface);
            } else if (fvarSurface.IsRegular()) {
                assignRegularEvaluator(fvarEval, fvarSurface);
            } else {
                assignIrregularEvaluator(fvarEval, fvarSurface);
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
