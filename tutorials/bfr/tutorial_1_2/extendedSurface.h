//
//   Copyright 2026 barfowl
//
//   Licensed under the terms set forth in the LICENSE.txt file available at
//   https://opensubdiv.org/license.
//

#ifndef OPENSUBDIV3_TUTORIAL_BFR_1_2_EXTENDED_SURFACE_H
#define OPENSUBDIV3_TUTORIAL_BFR_1_2_EXTENDED_SURFACE_H

#include <opensubdiv/bfr/surface.h>

//
//  Example code local to this tutorial:
//
namespace tutorial {
    //
    //  Simple class to wrap an instance of Bfr::Surface from which to
    //  expose additional functionality for computing surface normals.
    //
    //  This class is for illustration purposes only -- the focus being
    //  the public methods for computing/evaluating surface normals.
    //  The implementation of these methods also shows what is necessary
    //  to make use of the lower-level patch normal calculations with a
    //  Bfr::Surface.
    //
    class ExtendedSurface {
    public:
        typedef OpenSubdiv::Bfr::Surface<float>   Surface;
        typedef OpenSubdiv::Bfr::Parameterization Parameterization;

        ExtendedSurface(Surface const & s) : _surface(s) { }
        ExtendedSurface() = delete;
        ~ExtendedSurface() = default;

    public:
        //
        //  Methods using the verb "evaluate" will internally evaluate
        //  all partial derivates required to determine a surface normal,
        //  i.e. first derivatives and second (only if needed).
        //
        //  Just a single method for now -- additional overloads to
        //  return 1st and 2nd derivatives will likely be warranted here
        //  eventually (similar to Surface::Evaluate()).
        //
        bool EvaluateNormal(float const uv[2],
                            float const patchPoints[],
                            Surface::PointDescriptor const & pointDesc,
                            float P[3], float N[3], float tolerance = -1.0f);

    public:
        //
        //  Methods using the verb "compute" require pre-evaluated
        //  derivatives as input -- using them to compute the desired
        //  surface normal. No surface evaluation (i.e. use of Evaluate())
        //  is performed here. Making them static can be used to further
        //  distinguish them.
        //
        //  The first -- computation using 1st derivatives -- may be too
        //  trivial to include, but application of the default tolerance
        //  may make it worthwhile:
        //
        static bool ComputeNormal(float const uv[2],
                                  float const du[3], float const dv[3],
                                  float normal[3], float tolerance = -1.0f);

        //
        //  The second -- computation using 1st and 2nd derivatives --
        //  requires the Parameterization of the Surface with the given
        //  (u,v) coordinate where the derivatives were evaluated:
        //
        static bool ComputeNormal(Parameterization const & p, float const uv[2],
                    float const du[3], float const dv[3],
                    float const duu[3], float const duv[3], float const dvv[3],
                    float normal[3], float tolerance = -1.0f);

    private:
        Surface const & _surface;
    };

} // end namespace tutorial

#endif /* OPENSUBDIV3_TUTORIAL_BFR_1_2_EXTENDED_SURFACE_H */
