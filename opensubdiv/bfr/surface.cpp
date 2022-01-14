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
#include "../far/stencilTable.h"
#include "../far/patchBasis.h"
#include "../far/patchTree.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Surface methods to use for construction and re-initialization:
//
void
Surface::initialize() {

    _numControlPoints = 0;
    _numPatchPoints   = 0;

    _isValid   = false;
    _isRegular = true;
    _isLinear  = false;
    _useDouble = false;

    _irregOwner = false;
    _irregPatch = 0;
}

void
Surface::clear() {

    if (_irregOwner) {
        delete _irregPatch;
        _irregPatch = 0;
    }
}

//
//  Evaluation methods accessing the local data for a simple regular patch:
//
template <typename REAL>
void
Surface::evalRegularPatchBasis(REAL u, REAL v,
        REAL wP[],   REAL wDu[],  REAL wDv[],
        REAL wDuu[], REAL wDuv[], REAL wDvv[]) const {

    Far::internal::EvaluatePatchBasisNormalized(
        _regPatchType, _regPatchParam, u, v, wP, wDu, wDv, wDuu, wDuv, wDvv);
}

template <typename REAL>
int
Surface::evalRegularPatchStencils(REAL u, REAL v,
        REAL sP[],   REAL sDu[],  REAL sDv[],
        REAL sDuu[], REAL sDuv[], REAL sDvv[]) const {

    //
    //  The control vertices of a regular patch are always the full set
    //  of points required by a patch, i.e. phantom points will have an
    //  entry of some kind (a duplicate).  For example, for an isolated
    //  quad, its regular patch still has 16 control vertices.  So we can
    //  return the basis weights as stencil weights for all cases.
    //
    Far::internal::EvaluatePatchBasisNormalized(
        _regPatchType, _regPatchParam, u, v, sP, sDu, sDv, sDuu, sDuv, sDvv);

    return _numControlPoints;
}

//
//  Evaluation methods accessing the Far::PatchTree for irregular patches:
//
template <typename REAL>
ConstIndexArray
Surface::evalIrregularPatchBasis(REAL u, REAL v,
        REAL wP[],   REAL wDu[],  REAL wDv[],
        REAL wDuu[], REAL wDuv[], REAL wDvv[]) const {

    int subFace = 0;
    if (_param.GetType() == Parameterization::QPOLY) {
        //  Quadrangulated faces internally use a Ptex parameterization
        _param.ConvertUvToPtex(u, v, &u, &v, &subFace);
    }

    int subPatchIndex = _irregPatch->FindSubPatch(u, v, subFace);
    assert(subPatchIndex >= 0);

    _irregPatch->EvalSubPatchBasis(subPatchIndex,u, v,
                                   wP, wDu, wDv, wDuu, wDuv, wDvv);

    return _irregPatch->GetSubPatchPoints(subPatchIndex);
}

template <typename REAL>
int
Surface::evalIrregularPatchStencils(REAL u, REAL v,
        REAL sP[],   REAL sDu[],  REAL sDv[],
        REAL sDuu[], REAL sDuv[], REAL sDvv[]) const {

    assert(_irregPatch->SupportsStencilEval());

    int subFace = 0;
    if (_param.GetType() == Parameterization::QPOLY) {
        //  Quadrangulated faces internally use a Ptex parameterization
        _param.ConvertUvToPtex(u, v, &u, &v, &subFace);
    }

    int subPatchIndex = _irregPatch->FindSubPatch(u, v, subFace);
    assert(subPatchIndex >= 0);

    return _irregPatch->EvalSubPatchStencils<REAL>(subPatchIndex, u, v, sP,
                                                   sDu, sDv, sDuu, sDuv, sDvv);
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
Surface::evalMultiLinearPatchBasis(REAL u, REAL v,
        REAL wP[4],   REAL wDu[4],  REAL wDv[4],
        REAL wDuu[4], REAL wDuv[4], REAL wDvv[4]) const {

    assert(_param.GetType() == Parameterization::QPOLY);

    int subFace = 0;
    _param.ConvertUvToPtex(u, v, &u, &v, &subFace);

    //  WIP - Prefer to eval Linear basis directly, i.e.:
    //
    //      Far::internal::EvalBasisLinear(u, v, wP, wDu, wDv);
    //
    //  but this internal Far function is sometimes optimized out, causing
    //  link errors.  Need to fix in Far with explicit instantiation...
    Far::internal::EvaluatePatchBasisNormalized(Far::PatchDescriptor::QUADS,
            Far::PatchParam(), u, v, wP, wDu, wDv, wDuu, wDuv, wDvv);

    transformSubFaceWeightsToBase<REAL>(_numControlPoints, wP, 1.0f);
    if (wDu) {
        transformSubFaceWeightsToBase<REAL>(_numControlPoints, wDu, 2.0f);
    }
    if (wDv) {
        transformSubFaceWeightsToBase<REAL>(_numControlPoints, wDv, 2.0f);
    }
    if (wDuu) {
        //  Basis weights will be and should remain zero for this 2nd deriv
    }
    if (wDuv) {
        transformSubFaceWeightsToBase<REAL>(_numControlPoints, wDuv, 4.0f);
    }
    if (wDvv) {
        //  Basis weights will be and should remain zero for this 2nd deriv
    }
    return subFace;
}

template <typename REAL>
int
Surface::evalMultiLinearPatchStencils(REAL u, REAL v,
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
        iOrigin = evalMultiLinearPatchBasis<REAL>(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        iOrigin = evalMultiLinearPatchBasis<REAL>(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        iOrigin = evalMultiLinearPatchBasis<REAL>(u, v, wP, wDu, wDv,
                                                        wDuu, wDuv, wDvv);
    }

    int iNext = (iOrigin + 1) % _numControlPoints;
    int iPrev = (iOrigin + _numControlPoints - 1) % _numControlPoints;

    for (int i = 0; i < _numControlPoints; ++i) {
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
    return _numControlPoints;
}

//
//  Main public stencil evaluation method -- simply dispatches the stencil
//  evaluation methods for the three types above:
//
template <typename REAL>
int
Surface::EvaluateStencils(double u, double v,
                  REAL sP[],   REAL sDu[],  REAL sDv[],
                  REAL sDuu[], REAL sDuv[], REAL sDvv[]) const {

    if (_isRegular) {
        return evalRegularPatchStencils<REAL>((REAL)u, (REAL)v,
                                        sP, sDu, sDv, sDuu, sDuv, sDvv);
    } else if (_isLinear) {
        return evalMultiLinearPatchStencils<REAL>((REAL)u, (REAL)v,
                                            sP, sDu, sDv, sDuu, sDuv, sDvv);
    } else {
        return evalIrregularPatchStencils<REAL>((REAL)u, (REAL)v,
                                          sP, sDu, sDv, sDuu, sDuv, sDvv);
    }
}


//
//  Access to PatchTree's stencils to compute patch points:
//
template <typename REAL>
REAL const *
Surface::getIrregPatchStencilMatrix() const {

    return _irregPatch->GetStencilMatrix<REAL>();
}


//  WIP - use of StencilTables will eventually be removed
bool
Surface::irregPatchNeedsStencilTable() const {

    return _irregPatch->UsesStencilTable();
}

Far::StencilTableReal<float> const *
Surface::getIrregPatchStencilTable() const {

    return _irregPatch->GetStencilTable();
}

//
//  Explicit instantiation for supporting multiple precision:
//
//  Regular patch basis and stencil evaluation:
template void
Surface::evalRegularPatchBasis<float>(float u, float v,
        float wP[],   float wDu[],  float wDv[],
        float wDuu[], float wDuv[], float wDvv[]) const;
template void
Surface::evalRegularPatchBasis<double>(double u, double v,
        double wP[],   double wDu[],  double wDv[],
        double wDuu[], double wDuv[], double wDvv[]) const;

template int
Surface::evalRegularPatchStencils<float>(float u, float v,
        float sP[],   float sDu[],  float sDv[],
        float sDuu[], float sDuv[], float sDvv[]) const;
template int
Surface::evalRegularPatchStencils<double>(double u, double v,
        double sP[],   double sDu[],  double sDv[],
        double sDuu[], double sDuv[], double sDvv[]) const;

//  Irregular patch basis and stencil evaluation:
template ConstIndexArray
Surface::evalIrregularPatchBasis<float>(float u, float v,
        float wP[],   float wDu[],  float wDv[],
        float wDuu[], float wDuv[], float wDvv[]) const;
template ConstIndexArray
Surface::evalIrregularPatchBasis<double>(double u, double v,
        double wP[],   double wDu[],  double wDv[],
        double wDuu[], double wDuv[], double wDvv[]) const;

template int
Surface::evalIrregularPatchStencils<float>(float u, float v,
        float sP[],   float sDu[],  float sDv[],
        float sDuu[], float sDuv[], float sDvv[]) const;
template int
Surface::evalIrregularPatchStencils<double>(double u, double v,
        double sP[],   double sDu[],  double sDv[],
        double sDuu[], double sDuv[], double sDvv[]) const;

//  Multi-linear patch basis and stencil evaluation:
template int
Surface::evalMultiLinearPatchBasis<float>(float u, float v,
        float wP[4],   float wDu[4],  float wDv[4],
        float wDuu[4], float wDuv[4], float wDvv[4]) const;
template int
Surface::evalMultiLinearPatchBasis<double>(double u, double v,
        double wP[4],   double wDu[4],  double wDv[4],
        double wDuu[4], double wDuv[4], double wDvv[4]) const;

template int
Surface::evalMultiLinearPatchStencils<float>(float u, float v,
        float sP[],   float sDu[],  float sDv[],
        float sDuu[], float sDuv[], float sDvv[]) const;
template int
Surface::evalMultiLinearPatchStencils<double>(double u, double v,
        double sP[],   double sDu[],  double sDv[],
        double sDuu[], double sDuv[], double sDvv[]) const;

//  Top-level stencil evaluation:
template int
Surface::EvaluateStencils<float>(double u, double v,
        float sP[],   float sDu[],  float sDv[],
        float sDuu[], float sDuv[], float sDvv[]) const;
template int
Surface::EvaluateStencils<double>(double u, double v,
        double sP[],   double sDu[],  double sDv[],
        double sDuu[], double sDuv[], double sDvv[]) const;

//  Irregular stencil matrix access:
template float  const *
Surface::getIrregPatchStencilMatrix<float>() const;
template double const *
Surface::getIrregPatchStencilMatrix<double>() const;

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
