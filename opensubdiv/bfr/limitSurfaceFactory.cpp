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

        if (!gatherFaceNeighborhoodTopology(faceIndex, faceTopology)) {
            return false;
        }
        if (faceTopology.GetTag().HasUnOrderedVertices()) {
            //  WIP - more here for potentially non-manifold vertices
            //      - need to gather indices to identify boundaries
        }
        return faceTopology.GetTag().HasNonSharpBoundary() ? false : true;
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

    //
    //  Finally, gather patch control points from the appropriate indices:
    //
    eval._numControlPoints = patchSize;
    eval._numPatchPoints   = patchSize;

    eval._controlPoints.SetSize(eval._numControlPoints);
    int count = 0;
    if (fvarIndex < 0) {
        count = getFaceVertexIndices(faceIndex, &eval._controlPoints[0]);
    } else {
        count = getFaceFVarValueIndices(faceIndex, &eval._controlPoints[0], fvarIndex);
    }
    //  This premature return leaves the Evaluator invalid:
    if (count < faceSize) return;

    //  Fill in missing indices for a degenerate face:
    if (faceSize < patchSize) {
        int * points = &eval._controlPoints[0];
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

        //  WIP - what should behavior be when not getting expected number?
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
    //  declare but only initialize below when necessary:
    //
    typedef Vtr::internal::StackBuffer<Index,72,true> IndexBuffer;

    FaceTopology faceTopology(_schemeType, _schemeOptions);

    SurfaceDescriptor vtxSurface(faceTopology);
    IndexBuffer       vtxIndices;

    if (needTopology) {
        if (!gatherFaceNeighborhoodTopology(baseFace, faceTopology)) {
            return false;
        }

        //  It may additionally be necessary to gather control vertex indices
        //  to identify the topology around vertices that did not specify an
        //  ordering to their incident faces (possibly non-manifold).  Do so
        //  to resolve this, and also gather them here if needed otherwise so
        //  that we don't have to test later if they were already gathered:
        //  
        bool needVertexIndices = hasNonLinearVtxEvaluator ||
                                 faceTopology.GetTag().HasUnOrderedVertices();
        if (needVertexIndices) {
            vtxIndices.SetSize(faceTopology._numFaceVertsTotal);
            if (gatherFaceNeighborhoodIndices(baseFace, faceTopology,
                    vtxIndices, -1) < 0) {
                return false;
            }

            if (faceTopology.GetTag().HasUnOrderedVertices()) {
                //faceTopology.ResolveUnorderedCornerTopology(vtxIndices);
            }

            vtxSurface.InitializeVertex(vtxIndices);
        }

        bool debugFaceTopology = false;
        if (debugFaceTopology) {
            printf("SurfaceDescriptor(face = %d):\n", baseFace);
            vtxSurface.print();
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
