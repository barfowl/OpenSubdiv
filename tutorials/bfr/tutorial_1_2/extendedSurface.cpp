//
//   Copyright 2026 barfowl
//
//   Licensed under the terms set forth in the LICENSE.txt file available at
//   https://opensubdiv.org/license.
//

#include <opensubdiv/bfr/surface.h>

#include <cassert>

#include "./extendedSurface.h"
#include "./patchNormal.h"


namespace tutorial {

//
//  Static functions to compute normal from existing derivatives:
//
bool
ExtendedSurface::ComputeNormal(float const [2],
              float const du[3], float const dv[3],
              float N[3], float tolerance) {

    return internal::computePatchNormal(du, dv, N, tolerance);
}

bool
ExtendedSurface::ComputeNormal(
              Parameterization const & param, float const uvOfSurface[2],
              float const du[3], float const dv[3],
              float const duu[3], float const duv[3], float const dvv[3],
              float N[3], float tolerance) {

    //
    //  Make sure to get the (u,v) of the underlying patch and identify
    //  its shape (quad or tri) -- may need to convert (u,v) to that of
    //  a sub-face (for non-quads of Catmark):
    //
    float uvOfPatch[2] = { uvOfSurface[0], uvOfSurface[1] };
    if (param.HasSubFaces()) {
        param.ConvertCoordToNormalizedSubFace(uvOfSurface, uvOfPatch);
    }
    bool isQuadPatch = (param.GetType() != Parameterization::TRI);

    return internal::computePatchNormal(uvOfPatch[0], uvOfPatch[1], isQuadPatch,
                                        du, dv, duu, duv, dvv, N, tolerance);
}

//
//  Member functions to evaluate the normal and associated derivatives:
//
bool
ExtendedSurface::EvaluateNormal(float const uv[2],
               float const points[], Surface::PointDescriptor const & pointDesc,
               float P[3], float N[3], float tolerance) {

    //  Will need arbitrary size if/when allowing size > 3
    assert(pointDesc.size == 3);
    float du[3], dv[3];

    _surface.Evaluate(uv, points, pointDesc, P, du, dv);

    if (!ComputeNormal(uv, du, dv, N, tolerance)) {
        assert(pointDesc.size == 3);
        float duu[3], duv[3], dvv[3];

        _surface.Evaluate(uv, points, pointDesc, P, du, dv, duu, duv, dvv);

        return ComputeNormal(_surface.GetParameterization(), uv,
                             du, dv, duu, duv, dvv, N, tolerance);
    }
    return true;
}

} // end namespace tutorial
