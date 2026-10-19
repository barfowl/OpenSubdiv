//
//   Copyright 2026 Pixar
//
//   Licensed under the terms set forth in the LICENSE.txt file available at
//   https://opensubdiv.org/license.
//

#ifndef OPENSUBDIV3_TUTORIAL_BFR_1_2_PATCH_NORMAL_H
#define OPENSUBDIV3_TUTORIAL_BFR_1_2_PATCH_NORMAL_H


//
//  Low-level functions supporting the computation of normals for patches
//  (independent of Bfr::Surface or anything else in Bfr, or OpenSubdiv).
//
//  These are placed in an "internal" namespace -- in part for additional
//  encapsulation here, but also because it is likely they will eventually
//  be located in OpenSubdiv::Far::internal with other low-level patch
//  utilities used by the public Far, Osd, and Bfr interfaces.
//
//  Consistent with similar internal utilities for patches, patches are
//  parameterized in terms of (s,t) not (u,v). The domain here is also
//  either the unit quad or unit triangle (lower half of the unit quad),
//  so parametric coordinates for non-quads must be converted into the
//  normalized coordinates for the appropriate quad sub-face.
//
namespace internal {

    //
    //  The simple (N = ds X dt) with the appropriate default tolerance
    //  applied to the normalization when unspecified (this function may
    //  not really be necessary...):
    //
    bool
    computePatchNormal(float const ds[3], float const dt[3],
                       float N[3], float tolerance = -1.0f);

    //
    //  For use when (N = ds X dt = 0), the degeneracy can often be
    //  resolved with second derivatives at the same (s,t). In addition
    //  to all 1st and 2nd derivatives, knowledge of the parametric
    //  location (s,t) and the shape of the domain (quad or tri) is also
    //  required:
    //
    bool
    computePatchNormal(float s, float t, bool isQuadPatch,
               float const ds[3],  float const dt[3],
               float const dss[3], float const dst[3], float const dtt[3],
               float N[3], float tolerance = -1.0f);

} // end namespace internal

#endif /* OPENSUBDIV3_TUTORIAL_BFR_1_2_PATCH_NORMAL_H */
