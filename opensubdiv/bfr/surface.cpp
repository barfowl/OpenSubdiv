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

//
//  Evaluation methods accessing the PatchTree for irregular patches:
//
template <typename REAL>
int
Surface<REAL>::getNumIrregPatchPoints() const {

    return getIrregPatch().GetNumPointsTotal();
}

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

//
//  Evaluation methods for the N-sided quadrangulated linear patch:
//
//  Rather than computing a set of weights for potentially large N, basis
//  evaluation computes basis functions on the containing bilinear sub-face
//  (only 4 weights) and transforms them to a set of 4 unique weights to
//  be used for all N base points.
//
namespace {
    //
    //  Regardless of N, there are four unique weights derived from the
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
    transformSubFaceWeightsToBase(int N, REAL w[4], REAL derivScale) {

        REAL wOrigin = w[0];
        REAL wNext   = w[1] * 0.5f;
        REAL wCenter = w[2] / (REAL)N;
        REAL wPrev   = w[3] * 0.5f;

        w[0] = wCenter + wNext + wPrev + wOrigin;
        w[1] = wCenter + wNext;
        w[2] = wCenter;
        w[3] = wCenter + wPrev;

        if (derivScale > 0.0f) {
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

    int numControlPoints = GetNumControlPoints();

    transformSubFaceWeightsToBase<REAL>(numControlPoints, wP, 1.0f);
    if (wDu) {
        transformSubFaceWeightsToBase<REAL>(numControlPoints, wDu, 2.0f);
    }
    if (wDv) {
        transformSubFaceWeightsToBase<REAL>(numControlPoints, wDv, 2.0f);
    }
    if (wDuu) {
        //  Basis weights will be and should remain zero for this 2nd deriv
    }
    if (wDuv) {
        transformSubFaceWeightsToBase<REAL>(numControlPoints, wDuv, 4.0f);
    }
    if (wDvv) {
        //  Basis weights will be and should remain zero for this 2nd deriv
    }
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

    int numControlPoints = GetNumControlPoints();

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

//
//  Main public stencil evaluation method -- simply dispatches the stencil
//  evaluation methods for the three types above:
//
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
//  Access to PatchTree's stencils to compute patch points:
//
template <typename REAL>
REAL const *
Surface<REAL>::getIrregPatchPointMatrix() const {

    internal::IrregularPatchType const & irregPatch = getIrregPatch();
    return irregPatch.GetStencilMatrix<REAL>();
}


//
//  Explicitly instantiate Surface<> implementations for float and double:
//
template class Surface<float>;
template class Surface<double>;

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
