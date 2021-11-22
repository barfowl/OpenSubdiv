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
//  Definition of the private/nested SurfaceSet class:
//
//  This class (really a struct) encapsulates a clients specification of
//  a set of multiple surfaces and their intended interpolation types
//  (vertex, varying, and face-varying).  The multiple public creation
//  methods to request common subsets of surfaces all populate an instance
//  of SurfaceSet for internal use.
//
//  WIP - while this appears to be very similar to the public Evaluators
//        class, it includes Surface* members destined for assignment
//      - the public Evaluators class is expected to be deprecated while
//        SurfaceSet will continue to serve the Factory internally.
//
class LimitSurfaceFactory::SurfaceSet {
public:
    SurfaceSet() : numSurfs(0), numFVarSurfs(0),
                   vtxSurf(0), varSurf(0),
                   fvarSurfs(0), fvarSurfPtrs(0), fvarIDs(0) { }

public:
    //  Assignment to member variable is intended to be explicit:
    int numSurfs;
    int numFVarSurfs;

    Surface  * vtxSurf;
    Surface  * varSurf;
    Surface  * fvarSurfs;
    Surface ** fvarSurfPtrs;
    int const  * fvarIDs;

    void InitializeSurfaces() const {
        if (vtxSurf) vtxSurf->reinitialize();
        if (varSurf) varSurf->reinitialize();
        for (int i = 0; i < numFVarSurfs; ++i) {
            GetFVarSurface(i)->reinitialize();
        }
    }

public:
    //  Access to member variables is preferred through these methods,
    //  which may require a little more logic than expected:
    int GetNumSurfaces() const { return numSurfs; }

    bool      HasVertexSurface() const { return (vtxSurf != 0); }
    Surface * GetVertexSurface() const { return vtxSurf; }

    bool      HasVaryingSurface() const { return (varSurf != 0); }
    Surface * GetVaryingSurface() const { return varSurf; }

    bool      HasFVarSurfaces()       const { return numFVarSurfs > 0; }
    int       GetNumFVarSurfaces()    const { return numFVarSurfs; }
    int       GetFVarSurfaceID(int i) const { return fvarIDs ? fvarIDs[i] : i; }
    Surface * GetFVarSurface(int i)   const {
        //  Note that FVar Surfaces may be specified either as an
        //  array of Surfaces or an array of Surface pointers:
        return fvarSurfs ? (fvarSurfs + i) : fvarSurfPtrs[i];
    }
};


//
//  Main constructor and destructor:
//
LimitSurfaceFactory::LimitSurfaceFactory(
    Sdc::SchemeType schemeType,
    Sdc::Options    schemeOptions,
    Options         limitOptions) :
        _schemeType(schemeType),
        _schemeOptions(schemeOptions),
        _limitOptions(limitOptions) {

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

inline TopologyCache *
LimitSurfaceFactory::getTopologyCache() const {

    if (_limitOptions.ExternalTopologyCache()) {
        return _limitOptions.ExternalTopologyCache();
    } else if (!_limitOptions.DisableTopologyCache()) {
        return getInternalTopologyCache();
    }
    return 0;
}

LimitSurfaceFactory::~LimitSurfaceFactory() {

#ifdef _BFR_DEBUG_TOP_TYPE_STATS
//  DEBUG - report and reset inventory:
printf("LimitSurfaceFactory destructor:\n");
printf("    __numLinearPatches     = %6d\n", __numLinearPatches);
printf("    __numRegularPatches    = %6d\n", __numRegularPatches);
printf("    __numIrregularPatches  = %6d\n", __numIrregularPatches);
if (getTopologyCache()) {
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

Parameterization
LimitSurfaceFactory::GetFaceParameterization(Index faceIndex) const {

    return Parameterization(_schemeType, getFaceSize(faceIndex));
}

//
//  Methods supporting construction of linear, regular and irregular patches:
//
void
LimitSurfaceFactory::assignLinearSurface(Surface & surf,
        Index faceIndex, int fvarIndex) const {

    //  Initialize instance members from the associated irregular patch:
    int faceSize  = getFaceSize(faceIndex);

    surf._param = Parameterization(_schemeType, faceSize);

    surf._isRegular = (faceSize == _regFaceSize);
    surf._isLinear  = true;

    surf._regPatchType = (_regFaceSize == 4)
                       ?  Far::PatchDescriptor::QUADS
                       :  Far::PatchDescriptor::TRIANGLES;
    surf._regPatchParam.Clear();

    //
    //  Finally, gather patch control points from the appropriate indices:
    //
    surf._numControlPoints = faceSize;
    surf._numPatchPoints   = faceSize;

    surf._controlPoints.SetSize(surf._numControlPoints);
    int count = 0;
    if (fvarIndex < 0) {
        count = getFaceVertexIndices(faceIndex, &surf._controlPoints[0]);
    } else {
        count = getFaceFVarValueIndices(faceIndex, &surf._controlPoints[0],
                                                   fvarIndex);
    }
    //  If subclass fails to get indices, Surface will remain invalid
    if (count < faceSize) return;

    surf._isValid = true;
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numLinearPatches ++;
#endif
}

void
LimitSurfaceFactory::assignRegularSurface(Surface & surf,
        SurfaceDescriptor const & descriptor) const {

    //
    //  Assign the parameterization and discriminants first:
    //
    surf._param = Parameterization(_schemeType, _regFaceSize);

    surf._isRegular = true;
    surf._isLinear  = false;

    //
    //  Assemble the regular patch:
    //
    RegularPatchBuilder builder(descriptor);

    int boundaryMask = builder.GetPatchParamBoundaryMask();

    surf._regPatchType = builder.GetPatchType();
    surf._regPatchParam.Set(0, 0, 0, 0, 0, boundaryMask, 0, true);

    //
    //  Gather the patch control points from the given indices:
    //
    surf._numControlPoints = builder.GetNumControlVertices();
    surf._numPatchPoints   = surf._numControlPoints;

    surf._controlPoints.SetSize(surf._numControlPoints);
    builder.GatherControlVertexIndices(&surf._controlPoints[0]);

    surf._isValid = true;
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numRegularPatches ++;
#endif
}

void
LimitSurfaceFactory::assignIrregularSurface(Surface & surf,
        SurfaceDescriptor const & descriptor) const {

    //
    //  Assign the parameterization and discriminants first:
    //
    surf._param = Parameterization(_schemeType, descriptor.GetFaceSize());

    surf._isRegular = false;
    surf._isLinear  = false;

    //
    //  Construct a new irregular patch or identify one from the cache:
    //
    IrregularPatchBuilder::Options buildOptions;
    buildOptions.sharpLevel  = _limitOptions.MaxLevelPrimary();
    buildOptions.smoothLevel = _limitOptions.MaxLevelSecondary();

    IrregularPatchBuilder builder(descriptor, buildOptions);

    TopologyCache * topologyCache = getTopologyCache();
    if (topologyCache == 0) {
        surf._irregPatch = builder.Build();
        surf._irregOwner = true;
    } else {
        bool isNew    = false;
        bool isCached = false;
        surf._irregPatch = builder.Find(*topologyCache, isNew, isCached);
        surf._irregOwner = isNew && !isCached;
    }

    //
    //  Gather the patch control points from the given indices:
    //
    surf._numControlPoints = surf._irregPatch->GetNumControlPoints();
    surf._numPatchPoints   = surf._irregPatch->GetNumPointsTotal();

    surf._controlPoints.SetSize(surf._numControlPoints);
    builder.GatherControlVertexIndices(&surf._controlPoints[0]);

    surf._isValid = true;
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numIrregularPatches  ++;
__numIrregularUncached += surf._irregOwner;
#endif
}

void
LimitSurfaceFactory::copyNonLinearSurface(
        Surface                 & dstSurf,
        Surface const           & srcSurf,
        SurfaceDescriptor const & descriptor) const {

    //  Should be creating a linear patch directly rather than copying:
    assert(!srcSurf._isLinear);

    //
    //  Assign the topological fields of the patch first:
    //
    dstSurf._param = srcSurf._param;

    dstSurf._isLinear  = false;
    dstSurf._isRegular = srcSurf._isRegular;

    dstSurf._numControlPoints = srcSurf._numControlPoints;
    dstSurf._numPatchPoints   = srcSurf._numPatchPoints;

    dstSurf._controlPoints.SetSize(srcSurf._numControlPoints);

    //
    //  Assign regular/irregular fields and gather control points:
    //
    if (dstSurf._isRegular) {
        dstSurf._regPatchType  = srcSurf._regPatchType;
        dstSurf._regPatchParam = srcSurf._regPatchParam;

        RegularPatchBuilder builder(descriptor);
        assert(builder.GetNumControlVertices() == dstSurf._numControlPoints);

        builder.GatherControlVertexIndices(&dstSurf._controlPoints[0]);
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numRegularPatches ++;
#endif
    } else {
        dstSurf._irregPatch = srcSurf._irregPatch;
        dstSurf._irregOwner = false;

        IrregularPatchBuilder builder(descriptor);
        assert(builder.GetNumControlVertices() == dstSurf._numControlPoints);

        builder.GatherControlVertexIndices(&dstSurf._controlPoints[0]);
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numIrregularPatches  ++;
__numIrregularUncached += dstSurf._irregOwner;
#endif
    }

    dstSurf._isValid = true;
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
//  Main internal methods to populate set of limit Surfaces:
//
bool
LimitSurfaceFactory::populateAllSurfaces(Index faceIndex,
        SurfaceSet & surfaces) const {

    //  Abort if no Surfaces are specified to populate:
    if (surfaces.GetNumSurfaces() == 0) {
        return false;
    }

    //
    //  Be sure to re-initialize all Surfaces up-front, rather than
    //  deferring it to the assignment of each.  A failure of any one
    //  surface may leave others unvisited -- leaving it unchanged
    //  from previous use.
    //
    surfaces.InitializeSurfaces();

    //  Quickly reject faces with no limit (typically holes) -- some cases
    //  require full topological inspection and will be rejected later:
    if (!faceHasLimitLocal(faceIndex, getFaceSize(faceIndex))) {
        return false;
    }

    //  Determine if we have any non-linear cases to deal with -- which
    //  require gathering and inspection of the full neighborhood around
    //  the given face:
    int numFVarSurfaces = surfaces.GetNumFVarSurfaces();

    bool hasNonLinearSurfaces =
                (surfaces.HasVertexSurface() && !_linearScheme) ||
                (numFVarSurfaces && !_linearFVarInterp);

    bool hasLinearSurfaces =
                 surfaces.HasVaryingSurface() ||
                (surfaces.HasVertexSurface() && _linearScheme) ||
                (numFVarSurfaces && _linearFVarInterp);

    if (hasNonLinearSurfaces || _testNeighborhoodForLimit) {
        if (!populateNonLinearSurfaces(faceIndex, surfaces)) {
            return false;
        }
    }
    if (hasLinearSurfaces) {
        if (!populateLinearSurfaces(faceIndex, surfaces)) {
            return false;
        }
    }
    return true;
}

bool
LimitSurfaceFactory::populateLinearSurfaces(Index faceIndex,
        SurfaceSet & surfaces) const {

    if (surfaces.HasVaryingSurface()) {
        assignLinearSurface(*surfaces.GetVaryingSurface(), faceIndex, -1);
    }

    if (_linearScheme && surfaces.HasVertexSurface()) {
        assignLinearSurface(*surfaces.GetVertexSurface(), faceIndex, -1);
    }

    if (_linearFVarInterp) {
        int numFVarSurfaces = surfaces.GetNumFVarSurfaces();
        for (int i = 0; i < numFVarSurfaces; ++i) {
            assignLinearSurface(*surfaces.GetFVarSurface(i), faceIndex,
                                 surfaces.GetFVarSurfaceID(i));
        }
    }
    return true;
}

bool
LimitSurfaceFactory::populateNonLinearSurfaces(Index faceIndex,
        SurfaceSet & surfaces) const {

    typedef Vtr::internal::StackBuffer<Index,72,true> IndexBuffer;

    bool vtxIsNonLinear  = surfaces.HasVertexSurface() && !_linearScheme;
    bool fvarIsNonLinear = surfaces.HasFVarSurfaces()  && !_linearFVarInterp;

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

    bool vtxSurfIsValid = false;
    if (vtxIsNonLinear) {
        Surface & vtxSurf = *surfaces.GetVertexSurface();

        //  WIP - revert to linear for temporarily unsupported cases:
        if (faceTopology.IsUnsupported()) {
            assignLinearSurface(vtxSurf, faceIndex, -1);
        } else 

        if (vtxSurface.IsRegular()) {
            assignRegularSurface(vtxSurf, vtxSurface);
        } else {
            assignIrregularSurface(vtxSurf, vtxSurface);
        }
        vtxSurfIsValid = vtxSurf.IsValid();
    }

    //
    //  Process the limit surface for face-varying topologies -- all
    //  of which are potentially distinct.  Use the description of the
    //  vertex surface along with the face-varying indices assigned to
    //  determine the appropriate topological subset, then classify
    //  and assign the Surface:
    //
    if (fvarIsNonLinear) {
        //  We can re-use the vertex index buffer for face-varying indices:
        IndexBuffer & fvarIndices = vtxIndices;

        int numFVarSurfaces = surfaces.GetNumFVarSurfaces();
        for (int i = 0; i < numFVarSurfaces; ++i) {
            Surface & fvarSurf = *surfaces.GetFVarSurface(i);
            int       fvarID   =  surfaces.GetFVarSurfaceID(i);

            //  WIP - revert to linear for temporarily unsupported cases:
            if (faceTopology.IsUnsupported()) {
                assignLinearSurface(fvarSurf, faceIndex, fvarID);
                continue;
            }

            //  Skip if subclass fails to gather indices for given fvarID
            if (gatherFaceNeighborhoodIndices(faceIndex, faceTopology,
                    fvarIndices, fvarID) < 0) {
                continue;
            }

            //  Detect matching topology or regular and dispatch accordingly:
            SurfaceDescriptor fvarSurface(faceTopology, fvarIndices, vtxSurface);

            if (fvarSurface.MatchesVertexTopology() && vtxSurfIsValid) {
                Surface & vtxSurf = *surfaces.GetVertexSurface();
                copyNonLinearSurface(fvarSurf, vtxSurf, fvarSurface);
            } else if (fvarSurface.IsRegular()) {
                assignRegularSurface(fvarSurf, fvarSurface);
            } else {
                assignIrregularSurface(fvarSurf, fvarSurface);
            }
        }
    }
    return true;
}

//
//  Public creation methods for instances of Surface:
//
bool
LimitSurfaceFactory::CreateVertexSurface(Index faceIndex,
        Surface * vtxSurface) const {

    assert(vtxSurface);
    //
    //  This can be streamlined in future (no need to use full SurfaceSet):
    //
    SurfaceSet surfaces;

    surfaces.vtxSurf  = vtxSurface;
    surfaces.numSurfs = 1;

    return populateAllSurfaces(faceIndex, surfaces);
}

bool
LimitSurfaceFactory::CreateVaryingSurface(Index faceIndex,
        Surface * varSurface) const {

    assert(varSurface);
    //
    //  This can be streamlined in future (no need to use full SurfaceSet):
    //
    SurfaceSet surfaces;

    surfaces.varSurf  = varSurface;
    surfaces.numSurfs = 1;

    return populateAllSurfaces(faceIndex, surfaces);
}

bool
LimitSurfaceFactory::CreateFaceVaryingSurface(Index faceIndex,
        Surface * fvarSurface, int fvarID) const {

    assert(fvarSurface);
    //
    //  This can be streamlined in future (no need to use full SurfaceSet):
    //
    SurfaceSet surfaces;

    surfaces.fvarSurfs    =  fvarSurface;
    surfaces.fvarIDs      = &fvarID;
    surfaces.numSurfs     = 1;
    surfaces.numFVarSurfs = 1;

    return populateAllSurfaces(faceIndex, surfaces);
}

bool
LimitSurfaceFactory::CreateSurfaces(Index faceIndex,
        Surface * vtxSurface,
        Surface * varSurface,
        Surface * fvarSurfaces,
        int       fvarCount,
        int const fvarIDs[]) const {

    SurfaceSet surfaces;

    surfaces.vtxSurf   = vtxSurface;
    surfaces.varSurf   = varSurface;
    surfaces.fvarSurfs = fvarSurfaces;
    surfaces.fvarIDs   = &fvarIDs[0];

    surfaces.numFVarSurfs = fvarCount;
    surfaces.numSurfs     = fvarCount + (vtxSurface != 0) + (varSurface != 0);

    return populateAllSurfaces(faceIndex, surfaces);
}

Surface *
LimitSurfaceFactory::CreateVertexSurface(Index faceIndex) const {

    Surface * s = new Surface();

    if (CreateVertexSurface(faceIndex, s)) return s;

    delete s;
    return 0;
}

Surface *
LimitSurfaceFactory::CreateVaryingSurface(Index faceIndex) const {

    Surface * s = new Surface();

    if (CreateVaryingSurface(faceIndex, s)) return s;

    delete s;
    return 0;
}

Surface *
LimitSurfaceFactory::CreateFaceVaryingSurface(Index faceIndex, int fvID) const {

    Surface * s = new Surface();

    if (CreateFaceVaryingSurface(faceIndex, s, fvID)) return s;

    delete s;
    return 0;
}

//
//  Public creation methods for an instance of LimitSurface:
//
bool
LimitSurfaceFactory::Populate(LimitSurface & s,
        Index faceIndex,
        Evaluators const & evalOptions) const {

    //
    //  Clear and re-initialize the LimitSurface before re-populating:
    //
    int faceSize = getFaceSize(faceIndex);
    int numFVarEvaluators = evalOptions.GetNumFVarEvaluators();

    s.clear();
    s.initialize(numFVarEvaluators);

    s._faceIndex = faceIndex;

    s.parameterize(Parameterization(_schemeType, faceSize));

    //  Nothing more to do if no Evaluator specified:
    int numEvaluators = evalOptions.GetNumEvaluators();
    if (numEvaluators == 0) return false;

    //
    //  Convert the LimitSurface and specified Evaluators to a SurfaceSet
    //  to be populated:
    //
    SurfaceSet surfaceSet;

    if (evalOptions.CreateVertexEvaluator())  surfaceSet.vtxSurf = &s._vtxEval;
    if (evalOptions.CreateVaryingEvaluator()) surfaceSet.varSurf = &s._varEval;
    if (numFVarEvaluators) {
        surfaceSet.fvarSurfs = &s._fvarEval[0];
        surfaceSet.fvarIDs   = evalOptions.GetFVarEvaluatorIDs();
    }
    surfaceSet.numFVarSurfs = numFVarEvaluators;
    surfaceSet.numSurfs     = numEvaluators;

    return populateAllSurfaces(faceIndex, surfaceSet);
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

bool
LimitSurfaceFactory::Populate(LimitSurface & instance, Index faceIndex) const {

    return Populate(instance, faceIndex, _limitOptions.GetEvaluators());
}

LimitSurface *
LimitSurfaceFactory::Create(Index faceIndex) const {

    return Create(faceIndex, _limitOptions.GetEvaluators());
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
