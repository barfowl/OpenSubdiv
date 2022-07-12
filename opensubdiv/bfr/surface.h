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

#include "../bfr/surfaceData.h"
#include "../bfr/parameterization.h"
#include "../vtr/array.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

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
template <typename REAL>
class Surface {
public:
    Surface();
    ~Surface() { }

private:  // non-copyable:
    Surface(Surface const &);
    Surface & operator=(Surface const &);

public:
    //
    //  Simple public queries:
    //
    bool IsValid() const { return _data.isValid(); }

    Parameterization GetParameterization() const { return _data.getParam(); }

    int GetFaceSize() const  { return GetParameterization().GetFaceSize(); }

    //
    //  Data from both the mesh and the Surface are managed in arrays of
    //  floating point values with variable size and stride. The simple
    //  PointDescriptor struct is used to encapsulate those parameters:
    //
    struct PointDescriptor {
        PointDescriptor() : size(0), stride(0) { }
        PointDescriptor(int n) : size(n), stride(n) { }
        PointDescriptor(int n, int m) : size(n), stride(m) { }

        int size, stride;
    };

    //
    //  A Surface is evaluated by preparing a set of "patch points"
    //  required for subsequent evaluation methods.  The patch points
    //  consist of a subset of the control vertices of the mesh in the
    //  neighborhood of the face plus any additional points derived from
    //  them that may be required to represent the limit surface as one
    //  or more parametric patches.
    //
    //  If the control points are already or must be separate gathered
    //  as part of a larger collection of patch points, the remaining
    //  patch points can be computed with a separate method.
    //
    int GetNumPatchPoints() const;

    template <typename REAL_MESH>
    void PreparePatchPoints(REAL_MESH       const   meshPoints[],
                            PointDescriptor const & meshPointDesc,
                            REAL                  * patchPoints,
                            PointDescriptor const & patchPointDesc =
                                                    PointDescriptor()) const;

    void ComputePatchPoints(REAL                  * patchPoints,
                            PointDescriptor const & patchPointDesc) const;

    void Evaluate(REAL const uv[2],
                  REAL const patchPoints[], PointDescriptor const & pointDesc,
                  REAL * P) const;

    void Evaluate(REAL const uv[2],
                  REAL const patchPoints[], PointDescriptor const & pointDesc,
                  REAL * P, REAL * Du, REAL * Dv) const;

    void Evaluate(REAL const uv[2],
                  REAL const patchPoints[], PointDescriptor const & pointDesc,
                  REAL * P, REAL * Du,  REAL * Dv,
                  REAL * Duu, REAL * Duv, REAL * Dvv) const;

    //
    //  The "control points" identify the subset of vertices of the
    //  mesh that contribute to the limit surface of the face.  They
    //  will be a subset of the patch points and are intended for
    //  combination with "limit stencils" that can be evaluated below.
    //
    //  Control points can be "gathered" into a local buffer for
    //  various purposes (e.g. repeated evaluation, computation of
    //  bounding box, etc.) and stencils can be optionally applied to
    //  control points in this form.
    //
    typedef int Index;

    int GetNumControlPoints() const { return _data.getNumCVs(); }
    int GetControlPointIndices(Index meshPointIndices[]) const;

    int EvaluateStencils(REAL const uv[2], REAL sP[]) const;

    int EvaluateStencils(REAL const uv[2], REAL sP[],
                         REAL sDu[], REAL sDv[]) const;

    int EvaluateStencils(REAL const uv[2], REAL sP[],
                         REAL sDu[],  REAL sDv[],
                         REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;

    //  WIP - with "mesh points" and "control points" more clearly defined,
    //        consider their use in naming of the two ApplyStencil methods
    void ApplyStencil(REAL const stencil[],
                      REAL const meshPoints[], PointDescriptor const &,
                      REAL result[]) const;

    //  Convenience methods to gather control points and apply stencil
    //  to the resulting local array of control points:
    template <typename REAL_MESH>
    void GatherControlPoints(REAL_MESH       const   meshPoints[],
                             PointDescriptor const & meshPointDesc,
                             REAL                  * controlPoints,
                             PointDescriptor const & controlPointDesc = 
                                                     PointDescriptor()) const;

    void ApplyStencilGathered(REAL const stencil[],
                            REAL const controlPoints[], PointDescriptor const &,
                            REAL result[]) const;

private:
    //  Internal methods for evaluating derivatives, basis weights and
    //  stencils for regular, irregular and irregular linear patches:
    typedef Vtr::ConstArray<int> IndexArray;

    void evaluateDerivs(REAL const uv[2], REAL const patchPoints[],
                        PointDescriptor const &, REAL * derivs[]) const;
    void evalRegularDerivs(REAL const uv[2], REAL const patchPoints[],
                           PointDescriptor const &, REAL * derivs[]) const;
    void evalIrregularDerivs(REAL const uv[2], REAL const patchPoints[],
                             PointDescriptor const &, REAL * derivs[]) const;
    void evalMultiLinearDerivs(REAL const uv[2], REAL const patchPoints[],
                               PointDescriptor const &, REAL * derivs[]) const;

    void       evalRegularBasis(REAL const uv[2], REAL * wDeriv[]) const;
    IndexArray evalIrregularBasis(REAL const uv[2], REAL * wDeriv[]) const;
    int        evalMultiLinearBasis(REAL const uv[2], REAL * wDeriv[]) const;

    int evaluateStencils(REAL const uv[2], REAL * sDeriv[]) const;
    int evalRegularStencils(REAL const uv[2], REAL * sDeriv[]) const;
    int evalIrregularStencils(REAL const uv[2], REAL * sDeriv[]) const;
    int evalMultiLinearStencils(REAL const uv[2], REAL * sDeriv[]) const;

    //  Internal methods to compute patch points:
    void computeLinearPatchPoints(REAL * p, PointDescriptor const &) const;
    void computeIrregularPatchPoints(REAL * p, PointDescriptor const &) const;

private:
    //  Simple member accessors for internal use:
    bool isValid() const   { return _data.isValid(); }
    bool isRegular() const { return _data.isRegular(); }
    bool isLinear() const  { return _data.isLinear(); }

    unsigned char getRegPatchType() const { return _data.getRegPatchType(); }
    unsigned char getRegPatchMask() const { return _data.getRegPatchMask(); }

    internal::IrregularPatchType const & getIrregPatch() const;

private:
    //  Access to the set of member variables - provided to the Factory:
    friend class SurfaceFactory;

    internal::SurfaceData       & getSurfaceData()       { return _data; }
    internal::SurfaceData const & getSurfaceData() const { return _data; }

private:
    //  All member variables encapsulated in a single class:
    internal::SurfaceData _data;
};


//
//  Simple inline methods composed of other methods:
//
template <typename REAL>
inline void
Surface<REAL>::ComputePatchPoints(REAL * points,
                                  PointDescriptor const & pointDesc) const {

    if (!isRegular()) {
        if (isLinear()) {
            computeLinearPatchPoints(points, pointDesc);
        } else {
            computeIrregularPatchPoints(points, pointDesc);
        }
    }
}

template <typename REAL>
template <typename REAL_MESH>
inline void
Surface<REAL>::PreparePatchPoints(
        REAL_MESH const meshPoints[], PointDescriptor const & meshPointDesc,
        REAL * patchPoints,  PointDescriptor const & optPointDesc) const {

    PointDescriptor const & patchPointDesc = optPointDesc.size
                                           ? optPointDesc : meshPointDesc;

    GatherControlPoints(meshPoints, meshPointDesc, patchPoints, patchPointDesc);
    ComputePatchPoints(patchPoints, patchPointDesc);
}

//
//  Inline invokations of more general methods for derivative overloads:
//
template <typename REAL>
inline void
Surface<REAL>::evaluateDerivs(REAL const uv[2],
                              REAL const patchPoints[],
                              PointDescriptor const & pointDesc,
                              REAL * derivatives[]) const {
    if (isRegular()) {
        evalRegularDerivs(uv, patchPoints, pointDesc, derivatives);
    } else if (isLinear()) {
        evalMultiLinearDerivs(uv, patchPoints, pointDesc, derivatives);
    } else {
        evalIrregularDerivs(uv, patchPoints, pointDesc, derivatives);
    }
}
template <typename REAL>
inline void
Surface<REAL>::Evaluate(REAL const uv[2],
                        REAL const patchPoints[],
                        PointDescriptor const & pointDesc,
                        REAL * P) const {

    REAL * derivatives[6] = { P, 0, 0, 0, 0, 0 };
    evaluateDerivs(uv, patchPoints, pointDesc, derivatives);
}
template <typename REAL>
inline void
Surface<REAL>::Evaluate(REAL const uv[2],
                        REAL const patchPoints[],
                        PointDescriptor const & pointDesc,
                        REAL * P, REAL * Du, REAL * Dv) const {

    REAL * derivatives[6] = { P, Du, Dv, 0, 0, 0 };
    evaluateDerivs(uv, patchPoints, pointDesc, derivatives);
}
template <typename REAL>
inline void
Surface<REAL>::Evaluate(REAL const uv[2],
                        REAL const patchPoints[],
                        PointDescriptor const & pointDesc,
                        REAL * P,   REAL * Du,  REAL * Dv,
                        REAL * Duu, REAL * Duv, REAL * Dvv) const {

    REAL * derivatives[6] = { P, Du, Dv, Duu, Duv, Dvv };
    evaluateDerivs(uv, patchPoints, pointDesc, derivatives);
}

template <typename REAL>
inline int
Surface<REAL>::evaluateStencils(REAL const uv[2], REAL * sDeriv[]) const {

    if (isRegular()) {
        return evalRegularStencils(uv, sDeriv);
    } else if (isLinear()) {
        return evalMultiLinearStencils(uv, sDeriv);
    } else {
        return evalIrregularStencils(uv, sDeriv);
    }
}
template <typename REAL>
inline int
Surface<REAL>::EvaluateStencils(REAL const uv[2], REAL sP[]) const {

    REAL * derivativeStencils[6] = { sP, 0, 0, 0, 0, 0 };
    return evaluateStencils(uv, derivativeStencils);
}
template <typename REAL>
inline int
Surface<REAL>::EvaluateStencils(REAL const uv[2],
                          REAL sP[], REAL sDu[], REAL sDv[]) const {

    REAL * derivativeStencils[6] = { sP, sDu, sDv, 0, 0, 0 };
    return evaluateStencils(uv, derivativeStencils);
}
template <typename REAL>
inline int
Surface<REAL>::EvaluateStencils(REAL const uv[2],
                          REAL sP[],   REAL sDu[],  REAL sDv[],
                          REAL sDuu[], REAL sDuv[], REAL sDvv[]) const {

    REAL * derivativeStencils[6] = { sP, sDu, sDv, sDuu, sDuv, sDvv };
    return evaluateStencils(uv, derivativeStencils);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_SURFACE */
