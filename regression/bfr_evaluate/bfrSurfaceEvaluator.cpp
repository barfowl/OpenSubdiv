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

#include "bfrSurfaceEvaluator.h"


template <typename REAL>
BfrSurfaceEvaluator<REAL>::BfrSurfaceEvaluator(
        Far::TopologyRefiner const & baseMesh,
        Vec3Vector           const & basePos,
        Vec3Vector           const & baseUVs,
        FactoryOptions               factoryOptions) :
            _baseMesh(baseMesh),
            _baseMeshPos(basePos),
            _baseMeshUVs(baseUVs),
            _factory(baseMesh, factoryOptions) {

}

template <typename REAL>
bool
BfrSurfaceEvaluator<REAL>::FaceHasLimit(IndexType baseFace) const {

    return _factory.FaceHasLimitSurface(baseFace);
}

template <typename REAL>
void
BfrSurfaceEvaluator<REAL>::Evaluate(IndexType                 baseFace,
                                    TessCoordVector   const & tessCoords,
                                    EvalResults<REAL>       & results) const {

    //  Allocate vectors for the properties to be evaluated:
    int numCoords = (int) tessCoords.size() / 2;

    results.Resize(numCoords);

    //  Create the Surfaces for position and UV (optional) and assert if
    //  not valid, since a limit surface is expected here. (Note we may
    //  create the position surface but not actually evaluate it.)
    SurfaceType pSurface;
    SurfaceType uvSurface;

    //  Figure out how to get a command line arg here to run both
    bool initSeparate = false;
    if (initSeparate || !results.evalUV) {
        _factory.InitVertexSurface(baseFace, &pSurface);
        if (results.evalUV) {
            _factory.InitFaceVaryingSurface(baseFace, &uvSurface);
        }
    } else {
        _factory.InitSurfaces(baseFace, &pSurface, &uvSurface);
    }

    assert(pSurface.IsValid());
    assert(uvSurface.IsValid() == results.evalUV);

    //  Evaluate directly or using stencils:
    if (results.useStencils) {
        evaluateByStencils(pSurface, uvSurface, tessCoords, results);
    } else {
        evaluateDirectly(pSurface, uvSurface, tessCoords, results);
    }
}

template <typename REAL>
void
BfrSurfaceEvaluator<REAL>::evaluateDirectly(
        SurfaceType const & pSurface, SurfaceType const & uvSurface,
        TessCoordVector const & tessCoords, EvalResults<REAL> & results) const {

    int numCoords = (int) tessCoords.size() / 2;

    if (results.evalPosition) {
        Vec3Vector baseFacePos(pSurface.GetNumPatchPoints());

        pSurface.PreparePatchPointValues(_baseMeshPos, baseFacePos);

        REAL const * st = &tessCoords[0];
        for (int i = 0; i < numCoords; ++i, st += 2) {
            if (!results.eval1stDeriv) {
                pSurface.Evaluate(st, baseFacePos,
                        &results.p[i]);
            } else if (!results.eval2ndDeriv) {
                pSurface.Evaluate(st, baseFacePos,
                        &results.p[i], &results.du[i], &results.dv[i]);
            } else {
                pSurface.Evaluate(st, baseFacePos,
                        &results.p[i], &results.du[i], &results.dv[i],
                        &results.duu[i], &results.duv[i], &results.dvv[i]);
            }
        }
    }
    if (results.evalUV) {
        Vec3Vector baseFaceUVs(uvSurface.GetNumPatchPoints());

        uvSurface.PreparePatchPointValues(_baseMeshUVs, baseFaceUVs);

        REAL const * st = &tessCoords[0];
        for (int i = 0; i < numCoords; ++i, st += 2) {
            uvSurface.Evaluate(st, baseFaceUVs, &results.uv[i]);
        }
    }
}

template <typename REAL>
void
BfrSurfaceEvaluator<REAL>::evaluateByStencils(
        SurfaceType const & pSurface, SurfaceType const & uvSurface,
        TessCoordVector const & tessCoords, EvalResults<REAL> & results) const {

    std::vector<REAL> stencilWeights;

    int numCoords = (int) tessCoords.size() / 2;

    if (results.evalPosition) {
        stencilWeights.resize(6 * pSurface.GetNumControlVertices());

        REAL * sP   = &stencilWeights[0];
        REAL * sDu  = sP   + pSurface.GetNumControlVertices();
        REAL * sDv  = sDu  + pSurface.GetNumControlVertices();
        REAL * sDuu = sDv  + pSurface.GetNumControlVertices();
        REAL * sDuv = sDuu + pSurface.GetNumControlVertices();
        REAL * sDvv = sDuv + pSurface.GetNumControlVertices();

        REAL const * st = &tessCoords[0];
        for (int i = 0; i < numCoords; ++i, st += 2) {
            if (!results.eval1stDeriv) {
                pSurface.EvaluateStencils(st, &sP[0]);
            } else if (!results.eval2ndDeriv) {
                pSurface.EvaluateStencils(st, &sP[0], &sDu[0], &sDv[0]);
            } else {
                pSurface.EvaluateStencils(st, &sP[0], &sDu[0], &sDv[0],
                                              &sDuu[0], &sDuv[0], &sDvv[0]);
            }

            if (results.evalPosition) {
                pSurface.ApplyStencil(&sP[0],  _baseMeshPos, &results.p[i]);
            }
            if (results.eval1stDeriv) {
                pSurface.ApplyStencil(&sDu[0], _baseMeshPos, &results.du[i]);
                pSurface.ApplyStencil(&sDv[0], _baseMeshPos, &results.dv[i]);
            }
            if (results.eval2ndDeriv) {
                pSurface.ApplyStencil(&sDuu[0], _baseMeshPos, &results.duu[i]);
                pSurface.ApplyStencil(&sDuv[0], _baseMeshPos, &results.duv[i]);
                pSurface.ApplyStencil(&sDvv[0], _baseMeshPos, &results.dvv[i]);
            }
        }
    }
    if (results.evalUV) {
        stencilWeights.resize(uvSurface.GetNumControlVertices());

        REAL * sUV = &stencilWeights[0];

        REAL const * st = &tessCoords[0];
        for (int i = 0; i < numCoords; ++i, st += 2) {
            uvSurface.EvaluateStencils(st, &sUV[0]);

            uvSurface.ApplyStencil(&sUV[0], _baseMeshUVs, &results.uv[i]);
        }
    }
}


//
//  Explicit instantiation for float and double:
//
template class BfrSurfaceEvaluator<float>;
template class BfrSurfaceEvaluator<double>;

