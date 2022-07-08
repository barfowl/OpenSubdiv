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

        std::memcpy(controlPoint, meshPoint, meshPoints.size * sizeof(REAL));
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
    pointSet<REAL>(facePoint, pointSize, patchPoints, 0.0f);

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
    //  Apply the stencil coefficient matrix to compute any additional
    //  patch points from the control points already assigned:
    //
    PointBuffer controlPoints(patchPoints, pointSize, pointStride);

    REAL const * stencilMatrix = irregPatch.GetStencilMatrix<REAL>();
    int          stencilStride = numControlPoints;
    REAL const * stencil       = stencilMatrix;

    REAL * patchPoint = patchPoints + pointStride * numControlPoints;

    for (int i = numControlPoints; i < numPatchPoints; ++i) {
        combinePoints(controlPoints, numControlPoints, 0, stencil, patchPoint);

        stencil    += stencilStride;
        patchPoint += pointStride;
    }
}

template <typename REAL>
inline int
Surface<REAL>::assignWeights(REAL * const deriv[6], int wSize, REAL wBuffer[],
                             REAL * wDeriv[6]) const {

    std::memset(wDeriv, 0, 6 * sizeof(REAL*));

    wDeriv[0] = wBuffer;
    if (deriv[1] && deriv[2]) {
        wDeriv[1] = wDeriv[0] + wSize;
        wDeriv[2] = wDeriv[1] + wSize;
        if (deriv[3] && deriv[4] && deriv[5]) {
            wDeriv[3] = wDeriv[2] + wSize;
            wDeriv[4] = wDeriv[3] + wSize;
            wDeriv[5] = wDeriv[4] + wSize;
            return 6;
        }
        return 3;
    }
    return 1;
}



//
//  Evaluation methods accessing the local data for a simple regular patch:
//
template <typename REAL>
void
Surface<REAL>::evalRegularBasis(REAL const uv[2], REAL * wDeriv[]) const {

    Far::PatchParam patchParam;
    patchParam.Set(0, 0, 0, 0, 0, getRegPatchMask(), 0, true);

    Far::internal::EvaluatePatchBasisNormalized(
        getRegPatchType(), patchParam, uv[0], uv[1],
        wDeriv[0], wDeriv[1], wDeriv[2], wDeriv[3], wDeriv[4], wDeriv[5]);
}

template <typename REAL>
int
Surface<REAL>::evalRegularStencils(REAL const uv[2], REAL * sDeriv[]) const {

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
        getRegPatchType(), patchParam, uv[0], uv[1],
        sDeriv[0], sDeriv[1], sDeriv[2], sDeriv[3], sDeriv[4], sDeriv[5]);

    return GetNumControlPoints();
}

template <typename REAL>
void
Surface<REAL>::evalRegularDerivs(REAL const uv[2],
        PointBuffer const & patchPoints, REAL * deriv[]) const {

    //
    //  Regular basis evaluation simply returns weights for use with
    //  the entire set of patch control points.
    //
    //  Assign weights for requested derivatives, evaluate and apply:
    //
    REAL   wBuffer[6 * 20];
    REAL * wDeriv[6];

    int numDerivs = assignWeights(deriv, 20, wBuffer, wDeriv);

    evalRegularBasis(uv, wDeriv);

    int numPoints = GetNumControlPoints();
    if (numDerivs == 1) {
        combinePoints(patchPoints, numPoints, 0, wDeriv[0], deriv[0]);
    } else {
        combinePoints(patchPoints, numPoints, 0, wDeriv, deriv);
    }
}

//
//  Evaluation methods accessing the PatchTree for irregular patches:
//
template <typename REAL>
typename Surface<REAL>::IndexArray
Surface<REAL>::evalIrregularBasis(REAL const UV[2], REAL * wDeriv[]) const {

    Parameterization param = GetParameterization();
    REAL uv[2] = { UV[0], UV[1] };
    int subFace = param.HasSubFaces() ?
                  param.ConvertCoordToNormalizedSubFace(uv, uv) : 0;

    internal::IrregularPatchType const & irregPatch = getIrregPatch();
    int subPatchIndex = irregPatch.FindSubPatch(uv[0], uv[1], subFace);
    assert(subPatchIndex >= 0);

    irregPatch.EvalSubPatchBasis(subPatchIndex, uv[0], uv[1],
            wDeriv[0], wDeriv[1], wDeriv[2], wDeriv[3], wDeriv[4], wDeriv[5]);

    return irregPatch.GetSubPatchPoints(subPatchIndex);
}

template <typename REAL>
int
Surface<REAL>::evalIrregularStencils(REAL const UV[2], REAL * sDeriv[]) const {

    Parameterization param = GetParameterization();
    REAL uv[2] = { UV[0], UV[1] };
    int subFace = param.HasSubFaces() ?
                  param.ConvertCoordToNormalizedSubFace(uv, uv) : 0;

    internal::IrregularPatchType const & irregPatch = getIrregPatch();
    int subPatchIndex = irregPatch.FindSubPatch(uv[0], uv[1], subFace);
    assert(subPatchIndex >= 0);

    return irregPatch.EvalSubPatchStencils(
            subPatchIndex, uv[0], uv[1],
            sDeriv[0], sDeriv[1], sDeriv[2], sDeriv[3], sDeriv[4], sDeriv[5]);
}

template <typename REAL>
void
Surface<REAL>::evalIrregularDerivs(REAL const uv[2],
        PointBuffer const & patchPoints, REAL * deriv[]) const {

    //
    //  Non-linear irregular basis evaluation returns both the weights
    //  and the corresponding points of a sub-patch defined by a subset
    //  of the given patch points.
    //
    //  Assign weights for requested derivatives, evaluate and apply:
    //
    REAL   wBuffer[6 * 20];
    REAL * wDeriv[6];

    int numDerivs = assignWeights(deriv, 20, wBuffer, wDeriv);

    IndexArray indices = evalIrregularBasis(uv, wDeriv);

    int numPoints = indices.size();
    if (numDerivs == 1) {
        combinePoints(patchPoints, numPoints, &indices[0], wDeriv[0], deriv[0]);
    } else {
        combinePoints(patchPoints, numPoints, &indices[0], wDeriv, deriv);
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
Surface<REAL>::evalMultiLinearBasis(REAL const UV[2], REAL *wDeriv[]) const {

    Parameterization param = GetParameterization();
    assert(param.GetType() == Parameterization::QUAD_SUBFACES);

    REAL uv[2];
    int subFace = param.ConvertCoordToNormalizedSubFace(UV, uv);

    //  WIP - Prefer to eval Linear basis directly, i.e.:
    //
    //      Far::internal::EvalBasisLinear(u, v, wP, wDu, wDv);
    //
    //  but this internal Far function is sometimes optimized out, causing
    //  link errors.  Need to fix in Far with explicit instantiation...
    Far::internal::EvaluatePatchBasisNormalized(Far::PatchDescriptor::QUADS,
            Far::PatchParam(), uv[0], uv[1],
            wDeriv[0], wDeriv[1], wDeriv[2], wDeriv[3], wDeriv[4], wDeriv[5]);

    //  Scale weights for derivatives (only mixed partial of 2nd is non-zero):
    scaleWeights4<REAL>(wDeriv[1], 2.0f);
    scaleWeights4<REAL>(wDeriv[2], 2.0f);

    scaleWeights4<REAL>(wDeriv[4], 4.0f);

    return subFace;
}

template <typename REAL>
int
Surface<REAL>::evalMultiLinearStencils(REAL const uv[2], REAL *sDeriv[]) const {

    //
    //  Linear evaluation of irregular N-sided faces evaluates one of N
    //  locally subdivided quad faces and identifes that sub-face -- also
    //  the origin vertex of the quad. The basis weights are subsequently
    //  transformed into the four unique values that are then assigned to
    //  the N vertices of the face.
    //
    //  Assign weights for requested stencils and evaluate:
    //
    REAL   wBuffer[6 * 4];
    REAL * wDeriv[6];

    int numDerivs = assignWeights(sDeriv, 4, wBuffer, wDeriv);

    int iOrigin = evalMultiLinearBasis(uv, wDeriv);

    //
    //  Transform the four linear weights to four unique stencil weights:
    //
    int numControlPoints = GetNumControlPoints();

    transformLinearQuadWeightsToStencil(wDeriv[0], numControlPoints);
    if (numDerivs > 1) {
        transformLinearQuadWeightsToStencil(wDeriv[1], numControlPoints);
        transformLinearQuadWeightsToStencil(wDeriv[2], numControlPoints);
        if (numDerivs > 3) {
            transformLinearQuadWeightsToStencil(wDeriv[4], numControlPoints);
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

        sDeriv[0][i] = wDeriv[0][wIndex];
        if (numDerivs > 1) {
            sDeriv[1][i] = wDeriv[1][wIndex];
            sDeriv[2][i] = wDeriv[2][wIndex];
            if (numDerivs > 3) {
                sDeriv[3][i] = 0.0f;
                sDeriv[4][i] = wDeriv[4][wIndex];
                sDeriv[5][i] = 0.0f;
            }
        }
    }
    return numControlPoints;
}

template <typename REAL>
void
Surface<REAL>::evalMultiLinearDerivs(REAL const uv[],
        PointBuffer const & patchPoints, REAL * deriv[]) const {

    //
    //  Linear evaluation of irregular N-sided faces evaluates one of N
    //  locally subdivided quad faces and identifes that sub-face.
    //
    //  Assign weights for requested derivatives and evaluate:
    //
    REAL   wBuffer[6 * 4];
    REAL * wDeriv[6];

    int numDerivs = assignWeights(deriv, 4, wBuffer, wDeriv);

    int subQuad = evalMultiLinearBasis(uv, wDeriv);

    //
    //  Identify the patch points for the sub-face and interpolate:
    //
    int N = GetNumControlPoints();

    int quadIndices[4];
    quadIndices[0] = subQuad;
    quadIndices[1] = N + 1 + subQuad;
    quadIndices[2] = N;
    quadIndices[3] = N + 1 + (subQuad + N - 1) % N;

    if (numDerivs == 1) {
        combinePoints(patchPoints, 4, quadIndices, wDeriv[0], deriv[0]);
    } else {
        combinePoints(patchPoints, 4, quadIndices, wDeriv, deriv);
    }
}


//
//  Internal methods for combining control/patch points:
//
template <typename REAL>
void
Surface<REAL>::combinePoints(PointBuffer const & points,
                             int numIndices, int const indices[], 
                             REAL const weights[], REAL * result) const {

    int s = points.size;

    if (indices == 0) {
        //
        //  Combination uses the first N control points:
        //
        REAL const * p = points.data;
        pointSet(result, s, p, weights[0]);

        for (int i = 1; i < numIndices; ++i) {
            p += points.stride;
            pointAdd(result, s, p, weights[i]);
        }
    } else {
        //
        //  Combination uses an arbitrary subset of the patch points:
        //
        REAL const * p = points.data + points.stride * indices[0];
        pointSet(result, s, p, weights[0]);

        for (int i = 1; i < numIndices; ++i) {
            p = points.data + points.stride * indices[i];
            pointAdd(result, s, p, weights[i]);
        }
    }
}

template <typename REAL>
void
Surface<REAL>::combinePoints(PointBuffer const & points,
                             int numIndices, int const indices[], 
                             REAL * const wDeriv[], REAL * deriv[]) const {

    //  WIP - note that we currently assume 3 or 6 derivatives here...
    assert(deriv[1]);
    bool has2ndDerivs = (deriv[5] != 0);

    int s = points.size;

    //
    //  Apply each successive control point to all derivatives at once,
    //  rather than computing each derivate independently:
    //
    REAL const * p = indices ? (points.data + points.stride * indices[0]) :
                                points.data;

    pointSet(deriv[0], s, p, wDeriv[0][0]);
    pointSet(deriv[1], s, p, wDeriv[1][0]);
    pointSet(deriv[2], s, p, wDeriv[2][0]);
    if (has2ndDerivs) {
        pointSet(deriv[3], s, p, wDeriv[3][0]);
        pointSet(deriv[4], s, p, wDeriv[4][0]);
        pointSet(deriv[5], s, p, wDeriv[5][0]);
    }

    for (int i = 1; i < numIndices; ++i) {
        p = indices ? (points.data + points.stride * indices[i]) :
                      (p + points.stride);

        pointAdd(deriv[0], s, p, wDeriv[0][i]);
        pointAdd(deriv[1], s, p, wDeriv[1][i]);
        pointAdd(deriv[2], s, p, wDeriv[2][i]);
        if (has2ndDerivs) {
            pointAdd(deriv[3], s, p, wDeriv[3][i]);
            pointAdd(deriv[4], s, p, wDeriv[4][i]);
            pointAdd(deriv[5], s, p, wDeriv[5][i]);
        }
    }
}


//
//  Public methods to apply stencils:
//
template <typename REAL>
void
Surface<REAL>::ApplyStencil(REAL const stencil[],
        PointBuffer const & meshPoints, REAL result[]) const {

    combinePoints(meshPoints, GetNumControlPoints(), _data.getCVIndices(),
                              stencil, result);
}

template <typename REAL>
void
Surface<REAL>::ApplyStencilGathered(REAL const stencil[],
        PointBuffer const & controlPoints, REAL result[]) const {

    combinePoints(controlPoints, GetNumControlPoints(), 0,
                                 stencil, result);
}


//
//  Explicitly instantiate Surface<> implementations for float and double:
//
template class Surface<float>;
template class Surface<double>;

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
