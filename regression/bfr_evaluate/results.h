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
#ifndef OPENSUBDIV3_REGRESSION_BFR_EVALUATE_RESULTS_H
#define OPENSUBDIV3_REGRESSION_BFR_EVALUATE_RESULTS_H

#include "vec3.h"

#include <vector>
#include <cassert>

//
//  Simple struct to hold the results of a face evaluation:
//
template <typename REAL>
struct EvalResults {
    EvalResults() : evalPosition(true),
                    eval1stDeriv(true),
                    eval2ndDeriv(false),
                    evalUV(false),
                    useStencils(false) { }

    bool evalPosition;
    bool eval1stDeriv;
    bool eval2ndDeriv;
    bool evalUV;
    bool useStencils;

    std::vector< Vec3<REAL> > p;
    std::vector< Vec3<REAL> > du;
    std::vector< Vec3<REAL> > dv;
    std::vector< Vec3<REAL> > duu;
    std::vector< Vec3<REAL> > duv;
    std::vector< Vec3<REAL> > dvv;

    std::vector< Vec3<REAL> > uv;

    void Resize(int size) {
       if (evalPosition) {
            p.resize(size);
            if (eval1stDeriv) {
                du.resize(size);
                dv.resize(size);
                if (eval2ndDeriv) {
                    duu.resize(size);
                    duv.resize(size);
                    dvv.resize(size);
                }
            }
        }
        if (evalUV) {
            uv.resize(size);
        }
    }
};

#endif /* OPENSUBDIV3_REGRESSION_BFR_EVALUATE_RESULTS_H */
