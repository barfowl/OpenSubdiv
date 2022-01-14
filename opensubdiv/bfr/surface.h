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

#ifndef OPENSUBDIV3_BFR_SURFACE_H
#define OPENSUBDIV3_BFR_SURFACE_H

#include "../version.h"

#include "../bfr/parameterization.h"
#include "../bfr/types.h"
#include "../far/stencilTable.h"
#include "../far/patchDescriptor.h"
#include "../far/patchParam.h"
#include "../vtr/stackBuffer.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
    class PatchTree;
}

namespace Bfr {

//
//  The Surface class encapsulates the limit surface of a face for any
//  of the data interpolation types (vertex, varying and face-varying)
//  and provides the public interface for its evaluation.
//
//  Instances of Surface are created or initialized by a subclass of the
//  SurfaceFactory. Since existing instances can be re-initialized, they
//  should be tested for validity after such re-initialization.  Surface
//  is also non-copyable, so care should be taken when managing multiple
//  surfaces declared for initialization by a factory.
//
//  All Surfaces are assigned a Parameterization based on the subdivision
//  scheme and the size of the face, which can then be used for evaluation
//  and tessellation of the surface.
//
class Surface {
private:  // non-copyable:
    Surface(Surface const &);
    Surface & operator=(Surface const &);

public:
    Surface() { initialize(); }
    ~Surface() { clear(); }

public:
    bool IsValid() const { return _isValid; }

    Parameterization const & GetParameterization() const { return _param; }

    int GetFaceSize() const  { return _param.GetFaceSize(); }

    //
    //  A Surface is evaluated by preparing a set of "patch points"
    //  required for subsequent evaluation methods.  The patch points
    //  consist of a subset of the control vertices of the mesh in the
    //  neighborhood of the face plus any additional points derived from
    //  them that may be required to represent the limit surface as one
    //  or more parametric patches.
    //
    int GetNumPatchPoints() const { return _numPatchPoints; }

    template <class T, class U>
    void PreparePatchPointValues(T const & meshVertices,
                                 U       & patchPoints) const;

    template <class T, class U>
    void Evaluate(double u, double v, T const & patchPoints, U * P) const;

    template <class T, class U>
    void Evaluate(double u, double v, T const & patchPoints, U * P,
                                      U * Du, U * Dv) const;
    template <class T, class U>
    void Evaluate(double u, double v, T const & patchPoints, U * P,
                                      U * Du,  U * Dv,
                                      U * Duu, U * Duv, U * Dvv) const;

    //
    //  The "control vertices" identify the subset of vertices of the
    //  mesh that contribute to the limit surface of the face (use of
    //  "vertex" here is intended to emphasize their presence in the
    //  mesh, unlike patch "points" which may be derived from them).
    //  They will be a subset of the patch points and are intended for
    //  combination with "limit stencils" that can be evaluated below.
    //
    //  Control vertices can be "gathered" into a local buffer for
    //  various purposes (e.g. repeated evaluation, computation of
    //  bounding box, etc.) and stencils can be optionally applied to
    //  control vertices in this form.
    //
    int GetNumControlVertices() const { return _numControlPoints; }

    ConstIndexArray GetControlVertexIndices() const;

    template <typename REAL>
    int EvaluateStencils(double u, double v, REAL sP[]) const;

    template <typename REAL>
    int EvaluateStencils(double u, double v, REAL sP[],
                         REAL sDu[], REAL sDv[]) const;

    template <typename REAL>
    int EvaluateStencils(double u, double v, REAL sP[],
                         REAL sDu[],  REAL sDv[],
                         REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;

    template <typename REAL, class T, class U>
    void ApplyStencil(REAL const sD[], T const & meshVerts, U * D) const;

    //  Convenience methods to gather control vertices and apply stencil
    //  to the resulting local array of control vertices:
    template <class T, class U>
    void GatherControlVertexValues(T const & meshVerts, U & cVerts) const;

    template <typename REAL, class T, class U>
    void ApplyStencilGathered(REAL const sD[], T const & cVerts, U * D) const;

private:
    //  Evaluation applying weighted combinations of client type <T>:
    template <typename REAL, class T, class U>
    void evaluate(double u, double v, T const & patchPoints, U * P,
                                      U * Du,  U * Dv,
                                      U * Duu, U * Duv, U * Dvv) const;

    template <typename REAL, class T, class U>
    void evalRegularPatch(REAL u, REAL v, T const & patchPoints,
            U * P, U * Du, U * Dv, U * Duu, U * Dvu, U * Dvv) const;
    template <typename REAL, class T, class U>
    void evalIrregularPatch(REAL u, REAL v, T const & patchPoints,
            U * P, U * Du, U * Dv, U * Duu, U * Dvu, U * Dvv) const;
    template <typename REAL, class T, class U>
    void evalMultiLinearPatch(REAL u, REAL v, T const & patchPoints,
            U * P, U * Du, U * Dv, U * Duu, U * Dvu, U * Dvv) const;

    //  Evaluation of basis functions and contributing points of internal
    //  patch (implicit for a regular patch):
    template <typename REAL>
    void evalRegularPatchBasis(REAL u, REAL v, REAL wP[],
        REAL wDu[], REAL wDv[], REAL wDuu[], REAL wDuv[], REAL wDvv[]) const;
    template <typename REAL>
    ConstIndexArray evalIrregularPatchBasis(REAL u, REAL v, REAL wP[],
        REAL wDu[], REAL wDv[], REAL wDuu[], REAL wDuv[], REAL wDvv[]) const;
    template <typename REAL>
    int evalMultiLinearPatchBasis(REAL u, REAL v, REAL wP[],
        REAL wDu[], REAL wDv[], REAL wDuu[], REAL wDuv[], REAL wDvv[]) const;

    //  Evaluation of limit stencils:
    template <typename REAL>
    int evalRegularPatchStencils(REAL u, REAL v, REAL sP[],
        REAL sDu[], REAL sDv[], REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;
    template <typename REAL>
    int evalIrregularPatchStencils(REAL u, REAL v, REAL sP[],
        REAL sDu[], REAL sDv[], REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;
    template <typename REAL>
    int evalMultiLinearPatchStencils(REAL u, REAL v, REAL sP[], 
        REAL sDu[], REAL sDv[], REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;

    //  Access stencils to compute patch point values of client type <T>:
    template <typename REAL, class T>
    void applyIrregPatchStencils(T & patchPoints) const;

    template <typename REAL>
    REAL const * getIrregPatchStencilMatrix() const;

    //  WIP - use of StencilTable is likely to be replaced
    bool irregPatchNeedsStencilTable() const;
    Far::StencilTableReal<float> const * getIrregPatchStencilTable() const;

private:
    friend class SurfaceFactory;

    void clear();
    void initialize();
    void reinitialize() { if (_isValid) clear(), initialize(); }

private:
    typedef Far::PatchTree const * IrregPatchPtr;

    Parameterization _param;

    Vtr::internal::StackBuffer<Index,20,true> _controlPoints;

    int _numControlPoints;
    int _numPatchPoints;

    unsigned int _isValid   : 1;
    unsigned int _isRegular : 1;
    unsigned int _isLinear  : 1;
    unsigned int _useDouble : 1;

    //  WIP - consider a union here for the reg/irreg members:
    unsigned int _irregOwner : 1;
    IrregPatchPtr _irregPatch;

    Far::PatchDescriptor::Type _regPatchType;
    Far::PatchParam            _regPatchParam;
};

//
//  Inline methods and templates for gathering control points:
//
inline ConstIndexArray
Surface::GetControlVertexIndices() const {
    return ConstIndexArray(&_controlPoints[0], (int)_controlPoints.GetSize());
}

template <class T, class U>
void
Surface::GatherControlVertexValues(T const & meshPoints,
                                   U       & controlPoints) const {
    for (int i = 0; i < _numControlPoints; ++i) {
        //  WIP - cannot guarantee that type T is copyable here, so must
        //        use Clear() and AddWithWeight():
        controlPoints[i].Clear();
        controlPoints[i].AddWithWeight(meshPoints[_controlPoints[i]], 1.0f);
    }
}

template <typename REAL, class T>
void
Surface::applyIrregPatchStencils(T & patchPoints) const {

    REAL const * stencilWeights = getIrregPatchStencilMatrix<REAL>();
    int          stencilStride  = _numControlPoints;

    for (int i = _numControlPoints; i < _numPatchPoints; ++i) {
        patchPoints[i].Clear();
        for (int j = 0; j < _numControlPoints; ++j) {
            patchPoints[i].AddWithWeight(patchPoints[j],
                                         stencilWeights[j]);
        }
        stencilWeights += stencilStride;
    }
}

template <class T, class U>
void
Surface::PreparePatchPointValues(T const & meshPoints,
                                 U       & patchPoints) const {

    GatherControlVertexValues(meshPoints, patchPoints);

    if (_numPatchPoints > _numControlPoints) {
        //  Apply the patch point stencils to compute remaining patch
        //  points from those gathered above from the control points:
        if (irregPatchNeedsStencilTable()) {
            //  WIP - use of the StencilTable will eventually be removed
            getIrregPatchStencilTable()->UpdateValues(
                patchPoints, patchPoints, GetNumControlVertices());
        } else if (_useDouble) {
            applyIrregPatchStencils<double>(patchPoints);
        } else {
            applyIrregPatchStencils<float>(patchPoints);
        }
    }
}

//
//  Evaluation method templates:
//
template <typename REAL, class T, class U>
void
Surface::evalRegularPatch(REAL u, REAL v, T const & patchPoints,
                          U * P,   U * Du,  U * Dv,
                          U * Duu, U * Duv, U * Dvv) const {
    //
    //  Regular basis evaluation simply returns weights for use with
    //  the entire set of patch control points:
    //
    bool eval1stDerivs = (Du && Dv);
    bool eval2ndDerivs = eval1stDerivs && (Duu && Duv && Dvv);

    REAL wP[20], wDu[20], wDv[20], wDuu[20], wDuv[20], wDvv[20];
    if (!eval1stDerivs) {
        evalRegularPatchBasis<REAL>(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        evalRegularPatchBasis<REAL>(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        evalRegularPatchBasis<REAL>(u, v, wP, wDu, wDv, wDuu, wDuv, wDvv);
    }

    P->Clear();
    if (eval1stDerivs) {
        Du->Clear();
        Dv->Clear();
        if (eval2ndDerivs) {
            Duu->Clear();
            Duv->Clear();
            Dvv->Clear();
        }
    }

    for (int i = 0; i < _numControlPoints; ++i) {
        P->AddWithWeight(patchPoints[i], wP[i]);
        if (eval1stDerivs) {
            Du->AddWithWeight(patchPoints[i], wDu[i]);
            Dv->AddWithWeight(patchPoints[i], wDv[i]);
            if (eval2ndDerivs) {
                Duu->AddWithWeight(patchPoints[i], wDuu[i]);
                Duv->AddWithWeight(patchPoints[i], wDuv[i]);
                Dvv->AddWithWeight(patchPoints[i], wDvv[i]);
            }
        }
    }
}

template <typename REAL, class T, class U>
void
Surface::evalIrregularPatch(REAL u, REAL v, T const & patchPoints,
                            U * P,   U * Du,  U * Dv,
                            U * Duu, U * Duv, U * Dvv) const {
    //
    //  Non-linear irregular basis evaluation returns both the weights
    //  and the corresponding points of a sub-patch defined by a subset
    //  of the given patch points:
    //
    bool eval1stDerivs = (Du && Dv);
    bool eval2ndDerivs = eval1stDerivs && (Duu && Duv && Dvv);

    REAL wP[20], wDu[20], wDv[20], wDuu[20], wDuv[20], wDvv[20];
    ConstIndexArray subPatchPoints;

    if (!eval1stDerivs) {
        subPatchPoints = evalIrregularPatchBasis<REAL>(u, v,
                wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        subPatchPoints = evalIrregularPatchBasis<REAL>(u, v,
                wP, wDu, wDv, 0, 0, 0);
    } else {
        subPatchPoints = evalIrregularPatchBasis<REAL>(u, v,
                wP, wDu, wDv, wDuu, wDuv, wDvv);
    }

    P->Clear();
    if (eval1stDerivs) {
        Du->Clear();
        Dv->Clear();
        if (eval2ndDerivs) {
            Duu->Clear();
            Duv->Clear();
            Dvv->Clear();
        }
    }

    for (int i = 0; i < subPatchPoints.size(); ++i) {
        P->AddWithWeight( patchPoints[subPatchPoints[i]], wP[i]);
        if (eval1stDerivs) {
            Du->AddWithWeight(patchPoints[subPatchPoints[i]], wDu[i]);
            Dv->AddWithWeight(patchPoints[subPatchPoints[i]], wDv[i]);
            if (eval2ndDerivs) {
                Duu->AddWithWeight(patchPoints[subPatchPoints[i]], wDuu[i]);
                Duv->AddWithWeight(patchPoints[subPatchPoints[i]], wDuv[i]);
                Dvv->AddWithWeight(patchPoints[subPatchPoints[i]], wDvv[i]);
            }
        }
    }
}

template <typename REAL, class T, class U>
void
Surface::evalMultiLinearPatch(REAL u, REAL v, T const & patchPoints,
                              U * P,   U * Du,  U * Dv,
                              U * Duu, U * Duv, U * Dvv) const {
    //
    //  Linear evaluation of irregular N-sided faces (usually for varying
    //  or linear face-varying cases) quadrangulates the face implicitly
    //  as part of evaluation.  The control point that is the origin of
    //  the sub-face is returned along with the weights adjusted to apply
    //  to the full set of control points:
    //
    bool eval1stDerivs = (Du && Dv);
    bool eval2ndDerivs = eval1stDerivs && (Duu && Duv && Dvv);

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

    P->Clear();
    if (eval1stDerivs) {
        Du->Clear();
        Dv->Clear();
        if (eval2ndDerivs) {
            Duu->Clear();
            Duv->Clear();
            Dvv->Clear();
        }
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
        P->AddWithWeight( patchPoints[i], wP[wIndex]);
        if (eval1stDerivs) {
            Du->AddWithWeight(patchPoints[i], wDu[wIndex]);
            Dv->AddWithWeight(patchPoints[i], wDv[wIndex]);
            if (eval2ndDerivs) {
                Duu->AddWithWeight(patchPoints[i], wDuu[wIndex]);
                Duv->AddWithWeight(patchPoints[i], wDuv[wIndex]);
                Dvv->AddWithWeight(patchPoints[i], wDvv[wIndex]);
            }
        }
    }
}

template <typename REAL, class T, class U>
inline void
Surface::evaluate(double u, double v, T const & patchPoints,
                  U * P, U * Du, U * Dv, U * Duu, U * Duv, U * Dvv) const {

    if (_isRegular) {
        evalRegularPatch<REAL,T,U>((REAL)u, (REAL)v, patchPoints,
                                   P, Du, Dv, Duu, Duv, Dvv);
    } else if (_isLinear) {
        evalMultiLinearPatch<REAL,T,U>((REAL)u, (REAL)v, patchPoints,
                                       P, Du, Dv, Duu, Duv, Dvv);
    } else {
        evalIrregularPatch<REAL,T,U>((REAL)u, (REAL)v, patchPoints,
                                     P, Du, Dv, Duu, Duv, Dvv);
    }
}

template <class T, class U>
inline void
Surface::Evaluate(double u, double v, T const & patchPoints,
                  U * P, U * Du, U * Dv, U * Duu, U * Duv, U * Dvv) const {

    if (_useDouble) {
        evaluate<double,T,U>(u, v, patchPoints, P, Du, Dv, Duu, Duv, Dvv);
    } else {
        evaluate<float,T,U>(u, v, patchPoints, P, Du, Dv, Duu, Duv, Dvv);
    }
}

template <class T, class U>
inline void
Surface::Evaluate(double u, double v, T const & patchPoints,
                  U * P, U * Du, U * Dv) const {

    Evaluate<T,U>(u, v, patchPoints, P, Du, Dv, 0, 0, 0);
}

template <class T, class U>
inline void
Surface::Evaluate(double u, double v, T const & patchPoints, U * P) const {

    Evaluate<T,U>(u, v, patchPoints, P, 0, 0, 0, 0, 0);
}

template <typename REAL>
inline int
Surface::EvaluateStencils(double u, double v,
                          REAL sP[], REAL sDu[], REAL sDv[]) const {

    return EvaluateStencils<REAL>(u, v, sP, sDu, sDv, 0, 0, 0);
}

template <typename REAL>
inline int
Surface::EvaluateStencils(double u, double v, REAL sP[]) const {

    return EvaluateStencils<REAL>(u, v, sP, 0, 0, 0, 0, 0);
}

template <typename REAL, class T, class U>
void
Surface::ApplyStencil(REAL const sD[], T const & meshVertices, U * D) const {

    D->Clear();
    for (int i = 0; i < _numControlPoints; ++i) {
        D->AddWithWeight(meshVertices[_controlPoints[i]], sD[i]);
    }
}
template <typename REAL, class T, class U>
void
Surface::ApplyStencilGathered(REAL const sD[], T const & cvs, U * D) const {

    D->Clear();
    for (int i = 0; i < _numControlPoints; ++i) {
        D->AddWithWeight(cvs[i], sD[i]);
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_SURFACE */
