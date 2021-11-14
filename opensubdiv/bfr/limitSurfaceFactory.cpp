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
//  Methods for LimitSurfaceFactory::Evaluators (used by Options):
//
void
LimitSurfaceFactory::Evaluators::CreateFVarEvaluator(int fvarID) {

    _fvarEvalCount = 1;

    _fvarEvalIDsDynamic.clear();
    _fvarEvalIDs = &_fvarEvalIDsStatic[0];

    _fvarEvalIDs[0] = fvarID;
}

void
LimitSurfaceFactory::Evaluators::CreateFVarEvaluators(
        int count, int const fvarIDs[]) {

    _fvarEvalCount = count;

    if (count > (int)(sizeof(_fvarEvalIDsStatic) / sizeof(int))) {
        _fvarEvalIDsDynamic.resize(count);
        _fvarEvalIDs = &_fvarEvalIDsDynamic[0];
    } else {
        _fvarEvalIDsDynamic.clear();
        _fvarEvalIDs = &_fvarEvalIDsStatic[0];
    }

    if (fvarIDs) {
        std::memcpy(_fvarEvalIDs, fvarIDs, _fvarEvalCount * sizeof(int));
    } else {
        for (int i = 0; i < count; ++i) {
            _fvarEvalIDs[i] = i;
        }
    }
}

//
//  Main constructor and destructor:
//
LimitSurfaceFactory::LimitSurfaceFactory(
    Sdc::SchemeType schemeType,
    Sdc::Options    schemeOptions,
    Options         limitOptions) :
        _schemeType(schemeType),
        _schemeOptions(schemeOptions),
        _limitOptions(limitOptions),
        _topologyCache(0) {

    //  Override the topology cache if options require it:
    if (_limitOptions.DisableTopologyCache()) {
        _topologyCache = 0;
    } else if (_limitOptions.ExternalTopologyCache()) {
        _topologyCache = _limitOptions.ExternalTopologyCache();
    }

    //  Initialize members dependent on subdivision topology:
    _regFaceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(_schemeType);

    _linearScheme =
        (Sdc::SchemeTypeTraits::GetLocalNeighborhoodSize(_schemeType) == 0);

    _linearFVarInterp = _linearScheme ||
                       (_schemeOptions.GetFVarLinearInterpolation() ==
                                 Sdc::Options::FVAR_LINEAR_ALL);

    //  Initialize members related to the "face has limit" test:
    _rejectSmoothBoundariesForLimit = !_linearScheme &&
                       (_schemeOptions.GetVtxBoundaryInterpolation() ==
                                 Sdc::Options::VTX_BOUNDARY_NONE);

    _rejectIrregularFacesForLimit = !_linearScheme && (_regFaceSize == 3);

    _testNeighborhoodForLimit = _rejectSmoothBoundariesForLimit ||
                                _rejectIrregularFacesForLimit;
}

void
LimitSurfaceFactory::assignInternalTopologyCache(TopologyCache * cache) {

    if (!_limitOptions.DisableTopologyCache() && (_topologyCache == 0)) {
        _topologyCache = cache;
    }
}


LimitSurfaceFactory::~LimitSurfaceFactory() {

#ifdef _BFR_DEBUG_TOP_TYPE_STATS
//  DEBUG - report and reset inventory:
printf("LimitSurfaceFactory destructor:\n");
printf("    __numLinearPatches     = %6d\n", __numLinearPatches);
printf("    __numRegularPatches    = %6d\n", __numRegularPatches);
printf("    __numIrregularPatches  = %6d\n", __numIrregularPatches);
if (_topologyCache) {
printf("\n");
printf("    __numIrregularUncached = %6d\n", __numIrregularUncached);
}
__numLinearPatches     = 0;
__numRegularPatches    = 0;
__numIrregularPatches  = 0;
__numIrregularUncached = 0;
#endif
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
inline bool
LimitSurfaceFactory::faceHasLimitLocal(Index faceIndex, int faceSize) const {

    return (faceSize >= 3) && !isFaceHole(faceIndex);
}

bool
LimitSurfaceFactory::faceHasLimitNeighborhood(Index faceIndex,
    FaceTopology const * faceTopology) const {

    assert(_testNeighborhoodForLimit);

    //
    //  If the full neighborhood topology is available, use it, otherwise
    //  gather and inspect each corner in turn:
    //
    if (faceTopology) {
        CombinedTag tag = faceTopology->GetTag();
        if ((_rejectSmoothBoundariesForLimit && tag.HasNonSharpBoundary()) ||
            (_rejectIrregularFacesForLimit   && tag.HasIrregularFaceSizes())) {
            return false;
        }
    } else {
        CornerTopology   cTop;
        VertexTopology & vTop = cTop.GetVertexTopology();

        int faceSize = getFaceSize(faceIndex);
        for (int i = 0; i < faceSize; ++i) {
            //  Have the subclass load VertexTopology and finalize:
            cTop.Initialize(faceSize);

            int faceInRing = populateFaceVertexTopology(faceIndex, i, vTop);
            if (faceInRing < 0) return false;

            cTop.Finalize(_regFaceSize, faceInRing);

            //  Inspect the tag tod reject cases with no limit surface:
            CornerTag cTag = cTop.GetTag();

            if (_rejectSmoothBoundariesForLimit) {
                if (cTag.IsUnOrdered()) {
                    //  WIP - more needed here to fully resolve topology
                    //      - need to gather indices to identify boundaries
                }
                if (cTag.HasNonSharpBoundary()) return false;
            }
            if (_rejectIrregularFacesForLimit) {
                if (cTag.HasIrregularFaceSizes()) return false;
            }
        }
    }
    return true;
}

bool
LimitSurfaceFactory::FaceHasLimitSurface(Index faceIndex) const {

    return faceHasLimitLocal(faceIndex, getFaceSize(faceIndex)) &&
        (!_testNeighborhoodForLimit || faceHasLimitNeighborhood(faceIndex, 0));
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

    int boundaryMask = builder.GetPatchParamBoundaryMask();

    eval._regPatchType = builder.GetPatchType();
    eval._regPatchParam.Set(0, 0, 0, 0, 0, boundaryMask, 0, true);

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
        int faceInRing = populateFaceVertexTopology(faceIndex, i, vertexTop);
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
                getFaceVertexIncidentFaceVertexIndices(faceIndex, i,
                        indices) :
                getFaceVertexIncidentFaceFVarValueIndices(faceIndex, i,
                        indices, fvarIndex);

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
LimitSurfaceFactory::populateLinearEvaluators(LimitSurface & s,
        Index faceIndex,
        Evaluators const & evalOptions) const {

    if (evalOptions.CreateVaryingEvaluator()) {
        assignLinearEvaluator(s._varEval, faceIndex, -1);
    }

    if (_linearScheme && evalOptions.CreateVertexEvaluator()) {
        assignLinearEvaluator(s._vtxEval, faceIndex, -1);
    }

    if (_linearFVarInterp) {
        int numFVarEvaluators = evalOptions.GetNumFVarEvaluators();
        for (int i = 0; i < numFVarEvaluators; ++i) {
            assignLinearEvaluator(s._fvarEval[i], faceIndex,
                                  evalOptions.GetFVarEvaluatorID(i));
        }
    }
    return true;
}

bool
LimitSurfaceFactory::populateNonLinearEvaluators(LimitSurface & s,
        Index faceIndex,
        Evaluators const & evalOptions) const {

    typedef Vtr::internal::StackBuffer<Index,72,true> IndexBuffer;

    bool vtxIsNonLinear = evalOptions.CreateVertexEvaluator() &&
                          !_linearScheme;

    bool fvarIsNonLinear = evalOptions.GetNumFVarEvaluators() &&
                           !_linearFVarInterp;

    //
    //  Three steps are required to get the full description of a limit
    //  surface for the face:
    //
    //      - gathering the topological description of the neighborhood
    //      - gathering vertex indices for the neighborhood
    //      - using the indices to resolve any unordered topology
    //
    //  The limit surface for face-varying data is a topological subset
    //  of the vertex data, and so the vertex surface description is then
    //  used to initialize face-varying limit surfaces.
    //
    FaceTopology faceTopology(_schemeType, _schemeOptions);
    if (!gatherFaceNeighborhoodTopology(faceIndex, faceTopology)) {
        return false;
    }

    IndexBuffer vtxIndices;
    vtxIndices.SetSize(faceTopology._numFaceVertsTotal);
    if (gatherFaceNeighborhoodIndices(faceIndex, faceTopology,
                vtxIndices, -1) < 0) {
        return false;
    }

    if (faceTopology.HasUnOrderedCorners()) {
        faceTopology.ResolveUnOrderedCorners(vtxIndices);
    }

    if (_testNeighborhoodForLimit &&
                !faceHasLimitNeighborhood(faceIndex, &faceTopology)) {
        return false;
    }

    //
    //  Process the limit surface for vertex topology first -- all of
    //  the face-varying surfaces depend on it and will make use of its
    //  description when they are processed:
    //
    SurfaceDescriptor vtxSurface(faceTopology, vtxIndices);

    if (vtxIsNonLinear) {
        //  WIP - revert to linear for temporarily unsupported cases:
        if (faceTopology.IsUnsupported()) {
            assignLinearEvaluator(s._vtxEval, faceIndex, -1);
        } else 

        if (vtxSurface.IsRegular()) {
            assignRegularEvaluator(s._vtxEval, vtxSurface);
        } else {
            assignIrregularEvaluator(s._vtxEval, vtxSurface);
        }
    }

    //
    //  Process the limit surface for face-varying topologies -- all
    //  of which are potentially distinct.  Use the description of the
    //  vertex surface along with the face-varying indices assigned to
    //  determine the appropriate topological subset, then classify
    //  and assign the Evaluator:
    //
    if (fvarIsNonLinear) {
        //  We can re-use the vertex index buffer for face-varying indices:
        IndexBuffer & fvarIndices = vtxIndices;

        int numFVarEvaluators = evalOptions.GetNumFVarEvaluators();
        for (int i = 0; i < numFVarEvaluators; ++i) {
            LimitSurface::Evaluator & fvarEval = s._fvarEval[i];

            int fvarID = evalOptions.GetFVarEvaluatorID(i);

            //  WIP - revert to linear for temporarily unsupported cases:
            if (faceTopology.IsUnsupported()) {
                assignLinearEvaluator(fvarEval, faceIndex, fvarID);
                continue;
            }

            //  Skip if subclass fails to gather indices for given fvarID
            if (gatherFaceNeighborhoodIndices(faceIndex, faceTopology,
                    fvarIndices, fvarID) < 0) {
                continue;
            }

            //  Detect matching topology or regular and dispatch accordingly:
            SurfaceDescriptor fvarSurface(faceTopology, fvarIndices, vtxSurface);

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

bool
LimitSurfaceFactory::Populate(LimitSurface & s,
        Index faceIndex,
        Evaluators const & evalOptions) const {

    //
    //  Clear and re-initialize the existing instance before re-populating.
    //
    int faceSize = getFaceSize(faceIndex);
    int numFVarEvaluators = evalOptions.GetNumFVarEvaluators();

    s.clear();
    s.initialize(numFVarEvaluators);

    s._faceIndex = faceIndex;

    s.parameterize(Parameterization(_schemeType, faceSize));

    //  Quickly reject faces with no limit (typically holes) -- some cases
    //  require full topological inspection and will be rejected later:
    if (!faceHasLimitLocal(faceIndex, faceSize)) {
        return false;
    }

    //  Determine if we have any non-linear cases to deal with -- which
    //  require gathering and inspection of the full neighborhood around
    //  the given face:
    bool hasNonLinearEvaluators =
                (evalOptions.CreateVertexEvaluator() && !_linearScheme) ||
                (numFVarEvaluators && !_linearFVarInterp);

    bool hasLinearEvaluators =
                 evalOptions.CreateVaryingEvaluator() ||
                (evalOptions.CreateVertexEvaluator() && _linearScheme) ||
                (numFVarEvaluators && _linearFVarInterp);

    if (hasNonLinearEvaluators || _testNeighborhoodForLimit) {
        if (!populateNonLinearEvaluators(s, faceIndex, evalOptions)) {
            return false;
        }
    }
    if (hasLinearEvaluators) {
        if (!populateLinearEvaluators(s, faceIndex, evalOptions)) {
            return false;
        }
    }
    return true;
}

LimitSurface *
LimitSurfaceFactory::Create(Index faceIndex,
        Evaluators const & evalOptions) const {

    //
    //  Avoid allocation if face trivially has no limit (a hole).
    //  Still need to return 0 if face has no limit surface due to
    //  more complex conditions (e.g. unsharpened boundary faces):
    //
    if (isFaceHole(faceIndex)) return 0;

    LimitSurface * limitSurface = new LimitSurface();

    if (!Populate(*limitSurface, faceIndex, evalOptions)) {
        delete limitSurface;
        return 0;
    }
    return limitSurface;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
