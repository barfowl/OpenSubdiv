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

#include <cstring>

#include "../bfr/parameterization.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

namespace {
    //
    //  Simple queries are bundled into structs for each of the three types
    //  of parameterizations.  These are also templated for precision -- as
    //  a reminder that the methods of Parameterization that involve float
    //  coords need to be similarly templated in future.
    //
    //  Simple coordinates and queries for a quad parameterization:
    //
    template <typename REAL>
    struct quad {
        static REAL cornerCoords[4][2];
        static REAL cornerDerivs[4][2];

        static void GetCornerCoord(int corner, REAL & u, REAL & v) {
            u = cornerCoords[corner][0];
            v = cornerCoords[corner][1];
        }

        static void GetCornerCoords(Coord coords[]) {
            std::memcpy(coords, cornerCoords, sizeof(cornerCoords));
        }

        static void GetBoundaryCoord(int edge, REAL t, REAL & u, REAL & v) {
            u = cornerCoords[edge][0] + t * cornerDerivs[edge][0];
            v = cornerCoords[edge][1] + t * cornerDerivs[edge][1];
        }

        static int GetBoundaryCoords(int edge, REAL t0, REAL dt,
                                     int nCoords, Coord coords[]) {
            REAL u0 = cornerCoords[edge][0];
            REAL v0 = cornerCoords[edge][1];
            REAL t = t0;
            for (int i = 0; i < nCoords; ++i, t += dt) {
                coords[i] = Coord(u0 + t * cornerDerivs[edge][0],
                                  v0 + t * cornerDerivs[edge][1]);
            }
            return nCoords;
        }
        static void GetCenterCoord(REAL & u, REAL & v) {
            u = 0.5f;
            v = 0.5f;
        }
    };
    template <typename REAL>
    REAL quad<REAL>::cornerCoords[4][2] = { { 0.0f, 0.0f },
                                            { 1.0f, 0.0f },
                                            { 1.0f, 1.0f },
                                            { 0.0f, 1.0f } };
    template <typename REAL>
    REAL quad<REAL>::cornerDerivs[4][2] =  { { 1.0f, 0.0f },
                                             { 0.0f, 1.0f },
                                             {-1.0f, 0.0f },
                                             { 0.0f,-1.0f } };

    //
    //  Simple coordinates and queries for a triangular parameterization:
    //
    template <typename REAL>
    struct tri {
        static REAL cornerCoords[3][2];
        static REAL cornerDerivs[3][2];

        static void GetCornerCoord(int corner, REAL & u, REAL & v) {
            u = cornerCoords[corner][0];
            v = cornerCoords[corner][1];
        }

        static void GetCornerCoords(Coord coords[]) {
            std::memcpy(coords, cornerCoords, sizeof(cornerCoords));
        }

        static void GetBoundaryCoord(int edge, REAL t, REAL & u, REAL & v) {
            u = cornerCoords[edge][0] + t * cornerDerivs[edge][0];
            v = cornerCoords[edge][1] + t * cornerDerivs[edge][1];
        }

        static int GetBoundaryCoords(int edge, REAL t0, REAL dt,
                                     int nCoords, Coord coords[]) {
            REAL u0 = cornerCoords[edge][0];
            REAL v0 = cornerCoords[edge][1];
            REAL t = t0;
            for (int i = 0; i < nCoords; ++i, t += dt) {
                coords[i] = Coord(u0 + t * cornerDerivs[edge][0],
                                  v0 + t * cornerDerivs[edge][1]);
            }
            return nCoords;
        }

        static void GetCenterCoord(REAL & u, REAL & v) {
            u = 1.0f / 3.0f;
            v = 1.0f / 3.0f;
        }
    };
    template <typename REAL>
    REAL tri<REAL>::cornerCoords[3][2] = { { 0.0f, 0.0f },
                                           { 1.0f, 0.0f },
                                           { 0.0f, 1.0f } };
    template <typename REAL>
    REAL tri<REAL>::cornerDerivs[3][2] = { { 1.0f, 0.0f },
                                           {-1.0f, 1.0f },
                                           { 0.0f,-1.0f } };

    //
    //  Simple queries for an N-sided quadrangulated parameterization:
    //
    //  Note that, unlike the quad and tri utilities above, since these are
    //  all static functions, we need to pass the face-size -- the "N" of
    //  our N-sided parameterization.  We may also need to pass the "udim"
    //  to some if we choose to make use of it in future.
    //
    template <typename REAL>
    struct qpoly {
        static void GetCornerCoord(int corner, REAL & u, REAL & v) {
            u = (REAL) corner;
            v = 0.0f;
        }

        static void GetCornerCoords(int N, Coord coords[]) {
            for (int i = 0; i < N; ++i) {
                coords[i] = Coord((REAL)i, 0.0f);
            }
        }

        static void GetBoundaryCoord(int N, int edge, REAL t, REAL & u, REAL & v) {
            if (t < 0.5f) {
                u = (REAL)edge + t;
                v = 0.0f;
            } else {
                u = (REAL)((edge < (N - 1)) ? (edge + 1) : 0);
                v = 1.0f - t;
            }
        }

        static int GetBoundaryCoords(int N, int edge, REAL t0, REAL dt,
                                     int nCoords, Coord coords[]) {
            REAL u0 = (REAL)edge;
            REAL u1 = (edge < (N - 1)) ? (edge + 1.0f) : 0.0f;
            REAL t = t0;
            for (int i = 0; i < nCoords; ++i, t += dt) {
                if (t < 0.5f) {
                    coords[i] = Coord(u0 + t, 0.0f);
                } else {
                    coords[i] = Coord(u1, 1.0f - t);
                }
            }
            return nCoords;
        }

        static void GetCenterCoord(REAL & u, REAL & v) {
            u = 0.5f;
            v = 0.5f;
        }
    };
}

//
//  Methods to return (u,v) coordinates for features of all parameterizations:
//
void
Parameterization::GetCornerCoord(int corner, float & u, float & v) const {

    switch (GetType()) {
    case QUAD:
        quad<float>::GetCornerCoord(corner, u, v);
        break;
    case TRI:
        tri<float>::GetCornerCoord(corner, u, v);
        break;
    case QPOLY:
        qpoly<float>::GetCornerCoord(corner, u, v);
        break;
    }
}

int
Parameterization::GetCornerCoords(Coord coords[]) const {

    switch (GetType()) {
    case QUAD:
        quad<float>::GetCornerCoords(coords);
        break;
    case TRI:
        tri<float>::GetCornerCoords(coords);
        break;
    case QPOLY:
        qpoly<float>::GetCornerCoords(_faceSize, coords);
        break;
    }
    return _faceSize;
}

void
Parameterization::GetBoundaryCoord(int edge,
                                   float t, float & u, float & v) const {

    switch (GetType()) {
    case QUAD:
        quad<float>::GetBoundaryCoord(edge, t, u, v);
        break;
    case TRI:
        tri<float>::GetBoundaryCoord(edge, t, u, v);
        break;
    case QPOLY:
        qpoly<float>::GetBoundaryCoord(_faceSize, edge, t, u, v);
        break;
    }
}

int
Parameterization::GetBoundaryCoords(int edge, float t0, float dt,
                                    int nCoords, Coord coords[]) const {

    switch (GetType()) {
    case QUAD:
        quad<float>::GetBoundaryCoords(edge, t0, dt, nCoords, coords);
        break;
    case TRI:
        tri<float>::GetBoundaryCoords(edge, t0, dt, nCoords, coords);
        break;
    case QPOLY:
        qpoly<float>::GetBoundaryCoords(_faceSize, edge, t0, dt, nCoords, coords);
        break;
    }
    return nCoords;
}

void
Parameterization::GetCenterCoord(float & u, float & v) const {

    switch (GetType()) {
    case QUAD:
        quad<float>::GetCenterCoord(u, v);
        break;
    case TRI:
        tri<float>::GetCenterCoord(u, v);
        break;
    case QPOLY:
        qpoly<float>::GetCenterCoord(u, v);
        break;
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
