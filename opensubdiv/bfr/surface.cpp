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

#include "../bfr/surface.h"
#include "../bfr/surfaceData.h"
#include "../bfr/patchTree.h"
#include "../far/patchParam.h"
#include "../far/patchDescriptor.h"
#include "../far/patchBasis.h"

#include <cassert>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Internal utilities for dealing with "points" -- floating point tuples:
//
//  Note that serious performance degradation occurred switching to an
//  arbitrary size tuple vs the old <T,U> interface.  An optimization for
//  3D points was included to preserve previous performance, but more work
//  is needed here to specialize for other small tuples.
//
namespace {
    template <typename REAL>
    inline void
    pointClear(REAL p[], int size) {

        if (size == 3) {
            p[0] = 0.0f;
            p[1] = 0.0f;
            p[2] = 0.0f;
        } else {
            for (int i = 0; i < size; ++i) {
                p[i] = 0.0f;
            }
        }
    }

    template <typename REAL>
    inline void
    pointCopy(REAL p[], int size, REAL const src[]) {

        if (size == 3) {
            p[0] = src[0];
            p[1] = src[1];
            p[2] = src[2];
        } else {
            for (int i = 0; i < size; ++i) {
                p[i] = src[i];
            }
        }
    }

    template <typename REAL>
    inline void
    pointSet(REAL p[], int size, REAL const src[], REAL w) {

        if (size == 3) {
            p[0] = w * src[0];
            p[1] = w * src[1];
            p[2] = w * src[2];
        } else {
            for (int i = 0; i < size; ++i) {
                p[i] = w * src[i];
            }
        }
    }

    template <typename REAL>
    inline void
    pointAdd(REAL p[], int size, REAL const src[], REAL w) {

        if (size == 3) {
            p[0] += w * src[0];
            p[1] += w * src[1];
            p[2] += w * src[2];
        } else {
            for (int i = 0; i < size; ++i) {
                p[i] += w * src[i];
            }
        }
    }
}


//
//  Constructor for the Surface -- defers to the constructor for its full
//  set of member variables, but marks the precision as double when needed
//  (as a specialization here):
//
template <typename REAL>
Surface<REAL>::Surface() : _data() {

    //  Surface<> should not be adding members outside its SurfaceData:
    assert(sizeof(*this) == sizeof(internal::SurfaceData));
}

template <>
Surface<double>::Surface() : _data() {

    _data.setDouble(true);
}


//
//  Simple internal utilities:
//
template <typename REAL>
inline internal::IrregularPatchType const &
Surface<REAL>::getIrregPatch() const {

    return _data.getIrregPatch();
}

template <typename REAL>
int
Surface<REAL>::GetNumPatchPoints() const {

    if (isRegular()) {
        return GetNumControlPoints();
    } else if (isLinear()) {
        return 2 * GetNumControlPoints() + 1;
    } else {
        return getIrregPatch().GetNumPointsTotal();
    }
}

template <typename REAL>
int
Surface<REAL>::GetControlPointIndices(Index cvs[]) const {

    std::memcpy(cvs, _data.getCVIndices(), _data.getNumCVs() * sizeof(Index));
    return _data.getNumCVs();
}


//
//  Methods for gathering and computing control and patch points:
//
template <typename REAL>
void
Surface<REAL>::GatherControlPoints(PointBuffer const & meshPoints,
        REAL * controlPoints, int controlPointStride) const {

    if (controlPointStride == 0) controlPointStride = meshPoints.size;

    Index const * index = _data.getCVIndices();
    for (int i = 0; i < GetNumControlPoints(); ++i) {
        REAL const * meshPoint = meshPoints.data + meshPoints.stride * index[i];
        REAL       * controlPoint = controlPoints + controlPointStride * i;

        pointCopy(controlPoint, meshPoints.size, meshPoint);
    }
}

template <typename REAL>
void
Surface<REAL>::computeLinearPatchPoints(REAL * patchPoints,
        int pointSize, int pointStride) const {

    //
    //  Following the N control points, compute patch points for the
    //  midpoint of the face followed by the midpoint of the N edges:
    //
    int N = GetNumControlPoints();

    REAL * facePoint = patchPoints + pointStride * N;
    REAL   facePointWeight = 1.0f / (REAL) N;
    pointClear(facePoint, pointSize);

    for (int i = 0; i < N; ++i) {
        int iNext = (i < (N - 1)) ? (i + 1) : 0;

        REAL * v0Point = patchPoints + pointStride * i;
        REAL * v1Point = patchPoints + pointStride * iNext;

        REAL * edgePoint = patchPoints + pointStride * (N + 1 + i);
        pointSet<REAL>(edgePoint, pointSize, v0Point, 0.5f);
        pointAdd<REAL>(edgePoint, pointSize, v1Point, 0.5f);

        pointAdd(facePoint, pointSize, v0Point, facePointWeight);
    }
}

template <typename REAL>
void
Surface<REAL>::computeIrregularPatchPoints(REAL * patchPoints,
        int pointSize, int pointStride) const {

    //
    //  An "irregular patch" may be represented by a regular patch in
    //  some cases, so be sure there are patch points to compute:
    //
    internal::IrregularPatchType const & irregPatch = getIrregPatch();

    int numControlPoints = GetNumControlPoints();
    int numPatchPoints   = irregPatch.GetNumPointsTotal();

    if (numPatchPoints == numControlPoints) return;

    //
    //  Apply the coefficient matrix to compute any patch points
    //  in addition to the control points gathered previously:
    //
    REAL const * matrixRow       = irregPatch.GetStencilMatrix<REAL>();
    int          matrixRowStride = numControlPoints;

    REAL * patchPoint = patchPoints + pointStride * numControlPoints;

    for (int i = numControlPoints; i < numPatchPoints; ++i) {
        pointClear(patchPoint, pointSize);

        REAL * controlPoint = patchPoints;
        for (int j = 0; j < numControlPoints; ++j) {
            pointAdd(patchPoint, pointSize, controlPoint,matrixRow[j]);
            controlPoint += pointStride;
        }

        patchPoint += pointStride;
        matrixRow  += matrixRowStride;
    }
}


//
//  Evaluation methods accessing the local data for a simple regular patch:
//
template <typename REAL>
void
Surface<REAL>::evalRegularPatchBasis(REAL u, REAL v,
        REAL wP[],   REAL wDu[],  REAL wDv[],
        REAL wDuu[], REAL wDuv[], REAL wDvv[]) const {

    Far::PatchParam patchParam;
    patchParam.Set(0, 0, 0, 0, 0, getRegPatchMask(), 0, true);

    Far::internal::EvaluatePatchBasisNormalized(
        getRegPatchType(), patchParam, u, v, wP, wDu, wDv, wDuu, wDuv, wDvv);
}

template <typename REAL>
int
Surface<REAL>::evalRegularPatchStencils(REAL u, REAL v,
        REAL sP[],   REAL sDu[],  REAL sDv[],
        REAL sDuu[], REAL sDuv[], REAL sDvv[]) const {

    //
    //  The control points of a regular patch are always the full set
    //  of points required by a patch, i.e. phantom points will have an
    //  entry of some kind (a duplicate).  For example, for an isolated
    //  quad, its regular patch still has 16 control points.  So we can
    //  return the basis weights as stencil weights for all cases.
    //
    Far::PatchParam patchParam;
    patchParam.Set(0, 0, 0, 0, 0, getRegPatchMask(), 0, true);

    Far::internal::EvaluatePatchBasisNormalized(
        getRegPatchType(), patchParam, u, v, sP, sDu, sDv, sDuu, sDuv, sDvv);

    return GetNumControlPoints();
}

template <typename REAL>
void
Surface<REAL>::evalRegularPatch(REAL u, REAL v, PointBuffer const & points,
    REAL * P, REAL * Du, REAL * Dv, REAL * Duu, REAL * Duv, REAL * Dvv) const {

    //
    //  Regular basis evaluation simply returns weights for use with
    //  the entire set of patch control points:
    //
    bool eval1stDerivs = (Du && Dv);
    bool eval2ndDerivs = eval1stDerivs && (Duu && Duv && Dvv);

    REAL wP[20], wDu[20], wDv[20], wDuu[20], wDuv[20], wDvv[20];
    if (!eval1stDerivs) {
        evalRegularPatchBasis(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        evalRegularPatchBasis(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        evalRegularPatchBasis(u, v, wP, wDu, wDv, wDuu, wDuv, wDvv);
    }

    //
    //  Apply each successive control point to all derivatives at once,
    //  rather than computing each derivate independently:
    //
    int numPatchPoints = GetNumControlPoints();

    REAL const * patchPoint = points.data;

    pointSet(P, points.size, patchPoint, wP[0]);
    if (eval1stDerivs) {
        pointSet(Du, points.size, patchPoint, wDu[0]);
        pointSet(Dv, points.size, patchPoint, wDv[0]);
        if (eval2ndDerivs) {
            pointSet(Duu, points.size, patchPoint, wDuu[0]);
            pointSet(Duv, points.size, patchPoint, wDuv[0]);
            pointSet(Dvv, points.size, patchPoint, wDvv[0]);
        }
    }
    for (int i = 1; i < numPatchPoints; ++i) {
        patchPoint += points.stride;

        pointAdd(P, points.size, patchPoint, wP[i]);
        if (eval1stDerivs) {
            pointAdd(Du, points.size, patchPoint, wDu[i]);
            pointAdd(Dv, points.size, patchPoint, wDv[i]);
            if (eval2ndDerivs) {
                pointAdd(Duu, points.size, patchPoint, wDuu[i]);
                pointAdd(Duv, points.size, patchPoint, wDuv[i]);
                pointAdd(Dvv, points.size, patchPoint, wDvv[i]);
            }
        }
    }
}

//
//  Evaluation methods accessing the PatchTree for irregular patches:
//
template <typename REAL>
typename Surface<REAL>::PatchPointArray
Surface<REAL>::evalIrregularPatchBasis(REAL u, REAL v,
        REAL wP[],   REAL wDu[],  REAL wDv[],
        REAL wDuu[], REAL wDuv[], REAL wDvv[]) const {

    Parameterization param = GetParameterization();
    REAL uv[2] = { u, v };
    int subFace = param.HasSubFaces() ?
                  param.ConvertCoordToNormalizedSubFace(uv, uv) : 0;

    internal::IrregularPatchType const & irregPatch = getIrregPatch();
    int subPatchIndex = irregPatch.FindSubPatch(uv[0], uv[1], subFace);
    assert(subPatchIndex >= 0);

    irregPatch.EvalSubPatchBasis(subPatchIndex, uv[0], uv[1],
                                 wP, wDu, wDv, wDuu, wDuv, wDvv);

    return irregPatch.GetSubPatchPoints(subPatchIndex);
}

template <typename REAL>
int
Surface<REAL>::evalIrregularPatchStencils(REAL u, REAL v,
        REAL sP[],   REAL sDu[],  REAL sDv[],
        REAL sDuu[], REAL sDuv[], REAL sDvv[]) const {

    Parameterization param = GetParameterization();
    REAL uv[2] = { u, v };
    int subFace = param.HasSubFaces() ?
                  param.ConvertCoordToNormalizedSubFace(uv, uv) : 0;

    internal::IrregularPatchType const & irregPatch = getIrregPatch();
    int subPatchIndex = irregPatch.FindSubPatch(uv[0], uv[1], subFace);
    assert(subPatchIndex >= 0);

    return irregPatch.EvalSubPatchStencils(
            subPatchIndex, uv[0], uv[1], sP, sDu, sDv, sDuu, sDuv, sDvv);
}

template <typename REAL>
void
Surface<REAL>::evalIrregularPatch(REAL u, REAL v, PointBuffer const & points,
    REAL * P, REAL * Du, REAL * Dv, REAL * Duu, REAL * Duv, REAL * Dvv) const {

    //
    //  Non-linear irregular basis evaluation returns both the weights
    //  and the corresponding points of a sub-patch defined by a subset
    //  of the given patch points:
    //
    bool eval1stDerivs = (Du && Dv);
    bool eval2ndDerivs = eval1stDerivs && (Duu && Duv && Dvv);

    REAL wP[20], wDu[20], wDv[20], wDuu[20], wDuv[20], wDvv[20];
    PatchPointArray subPatchPoints;

    if (!eval1stDerivs) {
        subPatchPoints = evalIrregularPatchBasis(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        subPatchPoints = evalIrregularPatchBasis(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        subPatchPoints = evalIrregularPatchBasis(u, v, wP, wDu, wDv,
                                                       wDuu, wDuv, wDvv);
    }

    //
    //  Apply each successive control point to all derivatives at once,
    //  rather than computing each derivate independently:
    //
    int numPatchPoints = subPatchPoints.size();

    REAL const * patchPoint = points.data + points.stride * subPatchPoints[0];

    pointSet(P, points.size, patchPoint, wP[0]);
    if (eval1stDerivs) {
        pointSet(Du, points.size, patchPoint, wDu[0]);
        pointSet(Dv, points.size, patchPoint, wDv[0]);
        if (eval2ndDerivs) {
            pointSet(Duu, points.size, patchPoint, wDuu[0]);
            pointSet(Duv, points.size, patchPoint, wDuv[0]);
            pointSet(Dvv, points.size, patchPoint, wDvv[0]);
        }
    }
    for (int i = 1; i < numPatchPoints; ++i) {
        patchPoint = points.data + points.stride * subPatchPoints[i];

        pointAdd(P, points.size, patchPoint, wP[i]);
        if (eval1stDerivs) {
            pointAdd(Du, points.size, patchPoint, wDu[i]);
            pointAdd(Dv, points.size, patchPoint, wDv[i]);
            if (eval2ndDerivs) {
                pointAdd(Duu, points.size, patchPoint, wDuu[i]);
                pointAdd(Duv, points.size, patchPoint, wDuv[i]);
                pointAdd(Dvv, points.size, patchPoint, wDvv[i]);
            }
        }
    }
}

//
//  Supporting methods for the N-sided quadrangulated linear patch:
//
namespace {
    //
    //  For stencils, there are four unique weights derived from the
    //  four bilinear weights of the sub-face.  Given these weights as
    //  input for a sub-face with origin at base point P, the resulting
    //  weights are associated with the N base points as follows:
    //
    //      w[0] = the point at the origin (P)
    //      w[1] = the point following P
    //      w[2] = the N-3 points not adjacent to P (contributing to center)
    //      w[3] = the point preceding P
    //
    template <typename REAL>
    inline void
    transformLinearQuadWeightsToStencil(REAL w[4], int N) {

        REAL wOrigin = w[0];
        REAL wNext   = w[1] * 0.5f;
        REAL wCenter = w[2] / (REAL)N;
        REAL wPrev   = w[3] * 0.5f;

        w[0] = wCenter + wNext + wPrev + wOrigin;
        w[1] = wCenter + wNext;
        w[2] = wCenter;
        w[3] = wCenter + wPrev;
    }

    template <typename REAL>
    inline void
    scaleWeights4(REAL w[4], REAL derivScale) {
        if (w) {
            w[0] *= derivScale;
            w[1] *= derivScale;
            w[2] *= derivScale;
            w[3] *= derivScale;
        }
    }
}

template <typename REAL>
int
Surface<REAL>::evalMultiLinearPatchBasis(REAL u, REAL v,
        REAL wP[4],   REAL wDu[4],  REAL wDv[4],
        REAL wDuu[4], REAL wDuv[4], REAL wDvv[4]) const {

    Parameterization param = GetParameterization();
    assert(param.GetType() == Parameterization::QUAD_SUBFACES);

    REAL uv[2] = { u, v };
    int subFace = param.HasSubFaces() ?
                  param.ConvertCoordToNormalizedSubFace(uv, uv) : 0;

    //  WIP - Prefer to eval Linear basis directly, i.e.:
    //
    //      Far::internal::EvalBasisLinear(u, v, wP, wDu, wDv);
    //
    //  but this internal Far function is sometimes optimized out, causing
    //  link errors.  Need to fix in Far with explicit instantiation...
    Far::internal::EvaluatePatchBasisNormalized(Far::PatchDescriptor::QUADS,
            Far::PatchParam(), uv[0], uv[1], wP, wDu, wDv, wDuu, wDuv, wDvv);

    //  Scale weights for derivatives (only mixed partial of 2nd is non-zero):
    scaleWeights4<REAL>(wDu, 2.0f);
    scaleWeights4<REAL>(wDv, 2.0f);

    scaleWeights4<REAL>(wDuv, 4.0f);

    return subFace;
}

template <typename REAL>
int
Surface<REAL>::evalMultiLinearPatchStencils(REAL u, REAL v,
        REAL sP[],   REAL sDu[],  REAL sDv[],
        REAL sDuu[], REAL sDuv[], REAL sDvv[]) const {

    //
    //  Linear evaluation of irregular N-sided faces (usually for varying
    //  or linear face-varying cases) quadrangulates the face implicitly
    //  as part of evaluation.  The control point that is the origin of
    //  the sub-face is returned along with the weights adjusted to apply
    //  to the full set of control points:
    //
    bool eval1stDerivs = (sDu && sDv);
    bool eval2ndDerivs = eval1stDerivs && (sDuu && sDuv && sDvv);

    REAL wP[4], wDu[4], wDv[4], wDuu[4], wDuv[4], wDvv[4];

    int iOrigin = -1;
    if (!eval1stDerivs) {
        iOrigin = evalMultiLinearPatchBasis(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        iOrigin = evalMultiLinearPatchBasis(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        iOrigin = evalMultiLinearPatchBasis(u, v, wP, wDu, wDv,
                                                      wDuu, wDuv, wDvv);
    }

    //
    //  Transform the four linear weights to four unique stencil weights:
    //
    int numControlPoints = GetNumControlPoints();

    transformLinearQuadWeightsToStencil(wP, numControlPoints);
    if (eval1stDerivs) {
        transformLinearQuadWeightsToStencil(wDu, numControlPoints);
        transformLinearQuadWeightsToStencil(wDv, numControlPoints);
        if (sDuv) {
            transformLinearQuadWeightsToStencil(wDuv, numControlPoints);
        }
    }

    //
    //  Assign the N stencil weights from the four unique values:
    //
    int iNext = (iOrigin + 1) % numControlPoints;
    int iPrev = (iOrigin + numControlPoints - 1) % numControlPoints;

    for (int i = 0; i < numControlPoints; ++i) {
        int wIndex = 2;
        if (i == iOrigin) {
            wIndex = 0;
        } else if (i == iNext) {
            wIndex = 1;
        } else if (i == iPrev) {
            wIndex = 3;
        }

        sP[i] = wP[wIndex];
        if (eval1stDerivs) {
            sDu[i] = wDu[wIndex];
            sDv[i] = wDv[wIndex];
            if (eval2ndDerivs) {
                sDuu[i] = 0.0f;
                sDuv[i] = wDuv[wIndex];
                sDvv[i] = 0.0f;
            }
        }
    }
    return numControlPoints;
}

template <typename REAL>
void
Surface<REAL>::evalMultiLinearPatch(REAL u, REAL v, PointBuffer const &points,
    REAL * P, REAL * Du, REAL * Dv, REAL * Duu, REAL * Duv, REAL * Dvv) const {

    //
    //  Linear evaluation of irregular N-sided faces (usually for varying
    //  or linear face-varying cases) evaluates one of N locally subdivided
    //  quad faces:
    //
    bool eval1stDerivs = (Du && Dv);
    bool eval2ndDerivs = eval1stDerivs && (Duu && Duv && Dvv);

    REAL wP[4], wDu[4], wDv[4], wDuu[4], wDuv[4], wDvv[4];

    int subQuad = -1;
    if (!eval1stDerivs) {
        subQuad = evalMultiLinearPatchBasis(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        subQuad = evalMultiLinearPatchBasis(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        subQuad = evalMultiLinearPatchBasis(u, v, wP, wDu, wDv,
                                                  wDuu, wDuv, wDvv);
    }

    //
    //  Identify the four points of the quad sub-face:
    //
    int N = GetNumControlPoints();

    int quadIndices[4];
    quadIndices[0] = subQuad;
    quadIndices[1] = N + 1 + subQuad;
    quadIndices[2] = N;
    quadIndices[3] = N + 1 + (subQuad + N - 1) % N;

    //
    //  Apply each successive control point to all derivatives at once,
    //  rather than computing each derivate independently:
    //
    int numPatchPoints = 4;

    REAL const * patchPoint = points.data + points.stride * quadIndices[0];

    pointSet(P, points.size, patchPoint, wP[0]);
    if (eval1stDerivs) {
        pointSet(Du, points.size, patchPoint, wDu[0]);
        pointSet(Dv, points.size, patchPoint, wDv[0]);
        if (eval2ndDerivs) {
            pointSet(Duu, points.size, patchPoint, wDuu[0]);
            pointSet(Duv, points.size, patchPoint, wDuv[0]);
            pointSet(Dvv, points.size, patchPoint, wDvv[0]);
        }
    }
    for (int i = 1; i < numPatchPoints; ++i) {
        patchPoint = points.data + points.stride * quadIndices[i];

        pointAdd(P, points.size, patchPoint, wP[i]);
        if (eval1stDerivs) {
            pointAdd(Du, points.size, patchPoint, wDu[i]);
            pointAdd(Dv, points.size, patchPoint, wDv[i]);
            if (eval2ndDerivs) {
                pointAdd(Duu, points.size, patchPoint, wDuu[i]);
                pointAdd(Duv, points.size, patchPoint, wDuv[i]);
                pointAdd(Dvv, points.size, patchPoint, wDvv[i]);
            }
        }
    }
}

//
//  Main public evaluation and stencil evaluation methods -- which
//  simply dispatches the method for the appropriate surface type:
//
template <typename REAL>
void
Surface<REAL>::Evaluate(REAL const uv[2], PointBuffer const & points,
    REAL * P, REAL * Du, REAL * Dv, REAL * Duu, REAL * Duv, REAL * Dvv) const{

    if (isRegular()) {
        evalRegularPatch(uv[0], uv[1], points, P, Du, Dv, Duu, Duv, Dvv);
    } else if (isLinear()) {
        evalMultiLinearPatch(uv[0], uv[1], points, P, Du, Dv, Duu, Duv, Dvv);
    } else {
        evalIrregularPatch(uv[0], uv[1], points, P, Du, Dv, Duu, Duv, Dvv);
    }
}

template <typename REAL>
int
Surface<REAL>::EvaluateStencils(REAL const uv[2],
                  REAL sP[],   REAL sDu[],  REAL sDv[],
                  REAL sDuu[], REAL sDuv[], REAL sDvv[]) const {

    if (isRegular()) {
        return evalRegularPatchStencils(uv[0], uv[1],
                                        sP, sDu, sDv, sDuu, sDuv, sDvv);
    } else if (isLinear()) {
        return evalMultiLinearPatchStencils(uv[0], uv[1],
                                            sP, sDu, sDv, sDuu, sDuv, sDvv);
    } else {
        return evalIrregularPatchStencils(uv[0], uv[1],
                                          sP, sDu, sDv, sDuu, sDuv, sDvv);
    }
}

//
//  Public methods to apply stencils:
//
template <typename REAL>
void
Surface<REAL>::ApplyStencil(REAL const s[],
        PointBuffer const & meshPoints, REAL result[]) const {

    Index const * index = _data.getCVIndices();

    REAL const * meshPoint = meshPoints.data + meshPoints.stride * index[0];
    pointSet(result, meshPoints.size, meshPoint, s[0]);

    for (int i = 1; i < GetNumControlPoints(); ++i) {
        meshPoint = meshPoints.data + meshPoints.stride * index[i];
        pointAdd(result, meshPoints.size, meshPoint, s[i]);
    }
}

template <typename REAL>
void
Surface<REAL>::ApplyStencilGathered(REAL const s[],
        PointBuffer const & controlPoints, REAL result[]) const {

    REAL const * controlPoint = controlPoints.data;
    pointSet(result, controlPoints.size, controlPoint, s[0]);

    for (int i = 1; i < GetNumControlPoints(); ++i) {
        controlPoint += controlPoints.stride;
        pointAdd(result, controlPoints.size, controlPoint, s[i]);
    }
}


//
//  Explicitly instantiate Surface<> implementations for float and double:
//
template class Surface<float>;
template class Surface<double>;

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
