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

#ifndef OPENSUBDIV3_BFR_LIMIT_SURFACE_H
#define OPENSUBDIV3_BFR_LIMIT_SURFACE_H

#include "../version.h"

#include "../bfr/types.h"
#include "../bfr/parameterization.h"
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
//  LimitSurface is the main client-facing class that provides a potentially
//  piecewise piece of limit surface associated with the base face of a mesh.
//
//  At this public level, the details of the implementation are intentionally
//  hidden -- with the possibility of changing them in future without change
//  to the public interface.  When dealing with irregular topology, the
//  LimitSurface may manage a Far::PatchTree (or more when face-varying
//  topology is requested and differs).  Conversely, for the case of a purely
//  regular or varying patch, the overhead of a PatchTree is avoided in favor
//  of a more direct and simpler single patch evaluation.
//
//  Since both regular and irregular patch representations are shared
//  topological structures -- defined by a local set of control vertices --
//  a LimitSurface needs to identify the subset of vertices in the base mesh
//  that correspond to the control points of the patch.  The vertices of the
//  mesh are index "globally" while those of the LimitSurface are indexed
//  "locally".  Methods are provided to manage the corresponding "global" and
//  "local" data buffers.
//
//  A nested Evaluator class encapsulates all methods used for evaluation so
//  that they can be applied to vertex, face-varying or varying data.  Which
//  Evaluators are available is determined on construction by the Factory.
//
class LimitSurface {
public:
    //  Default constructor may be used when repopulating the same instance:
    LimitSurface();
    ~LimitSurface();

    bool IsValid() const { return _param.IsValid(); }

    int   GetFaceSize() const  { return _param.GetFaceSize(); }
    Index GetFaceIndex() const { return _faceIndex; }

    //  WIP - Consider making the face-vertex indices available.

    Parameterization const & GetParameterization() const   { return _param; }

    //  Methods for retrieving Evaluators for varying, face-varying, etc.:
    bool HasVertexEvaluator() const;
    bool HasVaryingEvaluator() const;
    bool HasFaceVaryingEvaluator() const;

    class Evaluator;
    Evaluator const * GetVertexEvaluator() const;
    Evaluator const * GetVaryingEvaluator() const;
    Evaluator const * GetFaceVaryingEvaluator() const;

    //
    //  The local Evaluator class contains the evaluation interface used for
    //  all data interpolation types (vertex, varying and face-varying):
    //
    class Evaluator {
    public:
        //
        //  A LimitSurface is evaluated by preparing a set of "patch points"
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

        //  WIP - overload with references or optional pointer arguments?
        template <class T, class U>
        void Evaluate(float u, float v, T const & patchPoints, U & P) const;

        template <class T, class U>
        void Evaluate(float u, float v, T const & patchPoints, U & P,
                      U & Du, U & Dv) const;

        //
        //  The "control vertices" identify the subset of vertices of the
        //  mesh that contribute to the limit surface of the face (use of
        //  "vertex" here is intended to emphasize their presence in the
        //  mesh, unlike patch "points" which may be derived from them).
        //  They will be a subset of the patch points and are intended for
        //  combination with "limit stencils" that can (in future) be
        //  evaluated below.
        //
        int GetNumControlVertices() const { return _numControlPoints; }

        ConstIndexArray GetControlVertexIndices() const;

        template <class T, class U>
        void GatherControlVertexValues(T const & meshVerts,
                                       U       & controlVerts) const;
/*
        int EvaluateStencils(float u, float v, float wP[],
                                               float wDu[] = 0,
                                               float wDv[] = 0) const;
*/

    private:
        friend class LimitSurface;
        friend class LimitSurfaceFactory;

        Evaluator(LimitSurface const & s) :
                _limitSurface(s), _numControlPoints(0) { }
        ~Evaluator() { }

        void initialize();
        void clear();

        bool isValid() const { return _numControlPoints > 0; }

        //  WIP - internal support of overloaded eval methods (this may yet
        //  replace them as the primary public eval method)
        template <class T, class U>
        void evaluate(float u, float v, T const & patchPoints,
                U * P, U * Du = 0, U * Dv = 0) const;

        //  Dealing directly with StencilTable may provide more flexibility
        //  than the public method that applies it (like the PatchTable):
        Far::StencilTableReal<float> const * getIrregPatchStencilTable() const;

        //  Evaluation of basis functions and contributing points of internal
        //  patch (implicit for a regular patch):
        void evalRegularPatchBasis(float u, float v,
                float wP[], float wDu[], float wDv[]) const;
        ConstIndexArray evalIrregularPatchBasis(float u, float v,
                float wP[], float wDu[], float wDv[]) const;
        int evalMultiLinearPatchBasis(float u, float v,
                float wP[4], float wDu[4], float wDv[4]) const;

        //  Evaluation to combine basis functions and contributing points:
        template <class T, class U>
        void evalRegularPatch(float u, float v, T const & patchPoints,
                U * P, U * Du = 0, U * Dv = 0) const;
        template <class T, class U>
        void evalIrregularPatch(float u, float v, T const & patchPoints,
                U * P, U * Du = 0, U * Dv = 0) const;
        template <class T, class U>
        void evalMultiLinearPatch(float u, float v, T const & patchPoints,
                U * P, U * Du = 0, U * Dv = 0) const;

    private:
        //  Private members (inc ref back to LimitSurface that contains it)
        LimitSurface const & _limitSurface;

        Vtr::internal::StackBuffer<Index,20,true> _controlPoints;

        int _numControlPoints;
        int _numPatchPoints;

        unsigned int _isRegular : 1;
        unsigned int _isLinear  : 1;
        unsigned int _isCached  : 1;

        Far::PatchDescriptor::Type _regPatchType;
        Far::PatchParam            _regPatchParam;

        Far::PatchTree const * _irregPatch;
    };

protected:
    friend class LimitSurfaceFactory;

    void clear();
    void initialize();
    void parameterize(Parameterization const & param);

private:
    //  Private members:
    int  _faceIndex;

    Parameterization _param;

    Evaluator _vtxEval;
    Evaluator _varEval;
    Evaluator _fvarEval;
};

//  WIP - unclear if Evaluator should be nested or not, revisit this later...
typedef LimitSurface::Evaluator Evaluator;

//
//  Inline methods for querying and retrieving Evaluators:
//
inline bool
LimitSurface::HasVertexEvaluator() const {
    return _vtxEval.isValid();
}
inline bool
LimitSurface::HasVaryingEvaluator() const {
    return _varEval.isValid();
}
inline bool
LimitSurface::HasFaceVaryingEvaluator() const {
    return _fvarEval.isValid();
}

inline Evaluator const *
LimitSurface::GetVertexEvaluator() const {
    return _vtxEval.isValid() ? &_vtxEval : 0;
}
inline Evaluator const *
LimitSurface::GetVaryingEvaluator() const {
    return _varEval.isValid() ? &_varEval : 0;
}
inline Evaluator const *
LimitSurface::GetFaceVaryingEvaluator() const {
    return _fvarEval.isValid() ? &_fvarEval : 0;
}

//
//  Inline methods and templates for gathering control points:
//
inline ConstIndexArray
Evaluator::GetControlVertexIndices() const {
    return ConstIndexArray(&_controlPoints[0], (int)_controlPoints.GetSize());
}

template <class T, class U>
void
Evaluator::GatherControlVertexValues(T const & meshPoints,
                                     U       & controlPoints) const {
    for (int i = 0; i < _numControlPoints; ++i) {
        //  WIP - Cannot guarantee that type T is copyable here, so must
        //  use Clear() and AddWithWeight():
        controlPoints[i].Clear();
        controlPoints[i].AddWithWeight(meshPoints[_controlPoints[i]], 1.0f);
    }
}

template <class T, class U>
void
Evaluator::PreparePatchPointValues(T const & meshPoints,
                                   U       & patchPoints) const {

    GatherControlVertexValues(meshPoints, patchPoints);

    if (_numPatchPoints > _numControlPoints) {
        getIrregPatchStencilTable()->UpdateValues(
            patchPoints, patchPoints, GetNumControlVertices());
    }
}

//
//  Evaluation method templates:
//
template <class T, class U>
void
Evaluator::evalRegularPatch(float u, float v, T const & patchPoints,
                            U * P, U * Du, U * Dv) const {
    //
    //  Regular basis evaluation simply returns weights for use with
    //  the entire set of patch control points:
    //
    if (Du && Dv) {
        float wP[20], wDu[20], wDv[20];
        evalRegularPatchBasis(u, v, wP, wDu, wDv);

        P->Clear();
        Du->Clear();
        Dv->Clear();
        for (int i = 0; i < _numControlPoints; ++i) {
            P->AddWithWeight( patchPoints[i], wP[i]);
            Du->AddWithWeight(patchPoints[i], wDu[i]);
            Dv->AddWithWeight(patchPoints[i], wDv[i]);
        }
    } else {
        float wP[20];
        evalRegularPatchBasis(u, v, wP, 0, 0);

        P->Clear();
        for (int i = 0; i < _numControlPoints; ++i) {
            P->AddWithWeight(patchPoints[i], wP[i]);
        }
    }
}

template <class T, class U>
void
Evaluator::evalIrregularPatch(float u, float v, T const & patchPoints,
                              U * P, U * Du, U * Dv) const {
    //
    //  Non-linear irregular basis evaluation returns both the weights
    //  and the corresponding points of a sub-patch defined by a subset
    //  of the given patch points:
    //
    if (Du && Dv) {
        float wP[20], wDu[20], wDv[20];
        ConstIndexArray subPatchPointIndices =
                evalIrregularPatchBasis(u, v, wP, wDu, wDv);

        P->Clear();
        Du->Clear();
        Dv->Clear();
        for (int i = 0; i < subPatchPointIndices.size(); ++i) {
            P->AddWithWeight( patchPoints[subPatchPointIndices[i]], wP[i]);
            Du->AddWithWeight(patchPoints[subPatchPointIndices[i]], wDu[i]);
            Dv->AddWithWeight(patchPoints[subPatchPointIndices[i]], wDv[i]);
        }
    } else {
        float wP[20];
        ConstIndexArray subPatchPointIndices =
                evalIrregularPatchBasis(u, v, wP, 0, 0);

        P->Clear();
        for (int i = 0; i < subPatchPointIndices.size(); ++i) {
            P->AddWithWeight( patchPoints[subPatchPointIndices[i]], wP[i]);
        }
    }
}

template <class T, class U>
void
Evaluator::evalMultiLinearPatch(float u, float v, T const & patchPoints,
                                U * P, U * Du, U * Dv) const {
    //
    //  Linear evaluation of irregular N-sided faces (usually for varying
    //  or linear face-varying cases) quadrangulates the face implicitly
    //  as part of evaluation.  The control point that is the origin of
    //  the sub-face is returned along with the weights adjusted to apply
    //  to the full set of control points:
    //
    bool eval1stDerivs = (Du && Dv);

    float wP[4], wDu[4], wDv[4];

    int iOrigin = eval1stDerivs ?
                  evalMultiLinearPatchBasis(u, v, wP, wDu, wDv) :
                  evalMultiLinearPatchBasis(u, v, wP, 0, 0);

    int iNext = (iOrigin + 1) % _numControlPoints;
    int iPrev = (iOrigin + _numControlPoints - 1) % _numControlPoints;

    P->Clear();
    if (eval1stDerivs) {
        Du->Clear();
        Dv->Clear();
    }
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
        }
    }
}

template <class T, class U>
inline void
Evaluator::evaluate(float u, float v, T const & patchPoints,
                    U * P, U * Du, U * Dv) const {

    if (_isRegular) {
        evalRegularPatch(u, v, patchPoints, P, Du, Dv);
    } else if (_isLinear) {
        evalMultiLinearPatch(u, v, patchPoints, P, Du, Dv);
    } else {
        evalIrregularPatch(u, v, patchPoints, P, Du, Dv);
    }
}

template <class T, class U>
inline void
Evaluator::Evaluate(float u, float v, T const & patchPoints,
                    U & P, U & Du, U & Dv) const {

    evaluate(u, v, patchPoints, &P, &Du, &Dv);
}

template <class T, class U>
inline void
Evaluator::Evaluate(float u, float v, T const & patchPoints, U & P) const {

    evaluate(u, v, patchPoints, &P);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_LIMIT_SURFACE */
