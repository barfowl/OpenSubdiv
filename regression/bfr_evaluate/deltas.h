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

#ifndef OPENSUBDIV3_REGRESSION_BFR_EVALUATE_DELTAS_H
#define OPENSUBDIV3_REGRESSION_BFR_EVALUATE_DELTAS_H

#include "vec3.h"

#include <cstring>


//
//  Simple struct to hold the differences between two vectors:
//
template <typename REAL>
class VectorDelta {
public:
    typedef std::vector< Vec3<REAL> >  VectorVec3;

public:
    //  Member variables:
    std::vector< Vec3<REAL> > const * vectorA;
    std::vector< Vec3<REAL> > const * vectorB;

    int  numDeltas;
    REAL maxDelta;
    REAL tolerance;

public:
    VectorDelta(REAL epsilon = 0.0f) :
            vectorA(0), vectorB(0),
            numDeltas(0), maxDelta(0.0f),
            tolerance(epsilon) { }

    void
    Compare(VectorVec3 const & a, VectorVec3 const & b) {

        assert(a.size() == b.size());

        vectorA = &a;
        vectorB = &b;

        numDeltas = 0;
        maxDelta = 0.0f;

        for (size_t i = 0; i < a.size(); ++i) {
            REAL const * ai = a[i].Coords();
            REAL const * bi = b[i].Coords();

            REAL dx = std::abs(ai[0] - bi[0]);
            REAL dy = std::abs(ai[1] - bi[1]);
            REAL dz = std::abs(ai[2] - bi[2]);
            if ((dx > tolerance) || (dy > tolerance) || (dz > tolerance)) {
                ++ numDeltas;

                if (maxDelta < dx) maxDelta = dx;
                if (maxDelta < dy) maxDelta = dy;
                if (maxDelta < dz) maxDelta = dz;
//printf("Diff:  Bfr(%12.6f %12.6f %12.6f)\n",            ai[0], ai[1], ai[2]);
//printf("    != Far(%12.6f %12.6f %12.6f) (%d of %d)\n", bi[0], bi[1], bi[2],
//                                                        i, size);
            }
        }
    }
};

template <typename REAL>
class FaceDelta {
public:
    //  Member variables:
    bool hasDeltas;
    bool hasGeomDeltas;
    bool hasUVDeltas;

    int numPDeltas;
    int numD1Deltas;
    int numD2Deltas;
    int numUVDeltas;

    REAL maxPDelta;
    REAL maxD1Delta;
    REAL maxD2Delta;
    REAL maxUVDelta;

public:
    FaceDelta() { Clear(); }

    void Clear() {
        std::memset(this, 0, sizeof(*this));
    }

    void AddPDelta(VectorDelta<REAL> const & pDelta) {
        if (pDelta.numDeltas) {
            numPDeltas = pDelta.numDeltas;
            maxPDelta  = pDelta.maxDelta;
            hasDeltas = hasGeomDeltas = true;
        }
    }
    void AddDuDelta(VectorDelta<REAL> const & duDelta) {
        if (duDelta.numDeltas) {
            numD1Deltas += duDelta.numDeltas;
            maxD1Delta   = std::max(maxD1Delta, duDelta.maxDelta);
            hasDeltas = hasGeomDeltas = true;
        }
    }
    void AddDvDelta(VectorDelta<REAL> const & dvDelta) {
        if (dvDelta.numDeltas) {
            numD1Deltas += dvDelta.numDeltas;
            maxD1Delta   = std::max(maxD1Delta, dvDelta.maxDelta);
            hasDeltas = hasGeomDeltas = true;
        }
    }
    void AddDuuDelta(VectorDelta<REAL> const & duuDelta) {
        if (duuDelta.numDeltas) {
            numD2Deltas += duuDelta.numDeltas;
            maxD2Delta   = std::max(maxD2Delta, duuDelta.maxDelta);
            hasDeltas = hasGeomDeltas = true;
        }
    }
    void AddDuvDelta(VectorDelta<REAL> const & duvDelta) {
        if (duvDelta.numDeltas) {
            numD2Deltas += duvDelta.numDeltas;
            maxD2Delta   = std::max(maxD2Delta, duvDelta.maxDelta);
            hasDeltas = hasGeomDeltas = true;
        }
    }
    void AddDvvDelta(VectorDelta<REAL> const & dvvDelta) {
        if (dvvDelta.numDeltas) {
            numD2Deltas += dvvDelta.numDeltas;
            maxD2Delta   = std::max(maxD2Delta, dvvDelta.maxDelta);
            hasDeltas = hasGeomDeltas = true;
        }
    }
    void AddUVDelta(VectorDelta<REAL> const & uvDelta) {
        if (uvDelta.numDeltas) {
            numUVDeltas = uvDelta.numDeltas;
            maxUVDelta  = uvDelta.maxDelta;
            hasDeltas = hasUVDeltas = true;
        }
    }
};

template <typename REAL>
class MeshDelta {
public:
    //  Member variables:
    int numFacesWithDeltas;
    int numFacesWithGeomDeltas;
    int numFacesWithUVDeltas;

    int numFacesWithPDeltas;
    int numFacesWithD1Deltas;
    int numFacesWithD2Deltas;

    REAL maxPDelta;
    REAL maxD1Delta;
    REAL maxD2Delta;
    REAL maxUVDelta;

public:
    MeshDelta() { Clear(); }

    void Clear() {
        std::memset(this, 0, sizeof(*this));
    }

    void AddFace(FaceDelta<REAL> const & faceDelta) {

        numFacesWithDeltas     += faceDelta.hasDeltas;
        numFacesWithGeomDeltas += faceDelta.hasGeomDeltas;
        numFacesWithUVDeltas   += faceDelta.hasUVDeltas;

        numFacesWithPDeltas  += (faceDelta.numPDeltas  > 0);
        numFacesWithD1Deltas += (faceDelta.numD1Deltas > 0);
        numFacesWithD2Deltas += (faceDelta.numD2Deltas > 0);

        maxPDelta  = std::max(maxPDelta,  faceDelta.maxPDelta);
        maxD1Delta = std::max(maxD1Delta, faceDelta.maxD1Delta);
        maxD2Delta = std::max(maxD2Delta, faceDelta.maxD2Delta);
        maxUVDelta = std::max(maxUVDelta, faceDelta.maxUVDelta);
    }
};

#endif /* OPENSUBDIV3_REGRESSION_BFR_EVALUATE_DELTAS_H */
