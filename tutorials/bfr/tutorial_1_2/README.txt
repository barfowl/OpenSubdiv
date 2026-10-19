
Overview:
    Changes to the tutorial here make use of some added external code to
    more robustly compute surface normals using second derivatives when
    the first derivatives alone are not sufficient.

    The tutorial itself is modified to make use of the new method by
    default, but has an additional command line option (-old) to generate
    the original normals for comparison.

    A new subdirectory with some test shapes (.obj files) was also added,
    with examples illustrating features where the simple cross product of
    first derivatives fails.


Computing patch normals (./patchNormal.*):
    This is where a given set of second derivatives is used to compute
    a non-zero normal for a parametric patch in cases when the first
    derivatives are not sufficient.

    Code here is completely independent of OpenSubdiv. It is written in
    terms of more primitive types as it will likely be integrated more
    deeply within OpenSubdiv (i.e. potentially within Far::internal and
    not just for use with Bfr). The different OpenSubdiv interfaces will
    need a thin layer to adapt to its use.

    In the meantime, without any significant dependencies, this code can
    be easy moved or ported for use elsewhere.


Computing normals for Bfr::Surface (./extendedSurface.*):
    These files contain the declaration and definition of a simple class
    that extends the functionality of Bfr::Surface to include methods to
    make use of the patch normal computation methods introduced above.

    This class is for illustration purposes only. It's public methods are
    an example of what might be added to Bfr::Surface in future.
    
    More importantly, the simple implementations of its methods show the
    small amount of code necessary to properly make use of the new patch
    normal computations with an instance of Bfr::Surface (particularly
    with non-quads).


A note on tolerances:
    An optional tolerance is provided to all functions that compute a
    normal vector. That tolerance is considered to be the length of the
    smallest non-zero vector that is considered accurate. Anything
    smaller is considered potentially unreliable due to floating point
    error and so may yield sporadic results when combined with others.
    This is first used to test if the cross product resulting from the
    two first derivatives is valid.

    A default tolerance is determined internally when not specified,
    and thus far has proved successful in the GPU shaders, but there
    may be cases where this default does not suffice.
    
    In general, a size proportional to the size of the data involved
    is most appropriate when making comparisons, e.g. the size of the
    bounding box of the patch (or some other spatial metric) multiplied
    by some small fraction. Computing such metrics on the fly during
    every evaluation is not practical (i.e. possible but inefficient
    and repeated for every evaluation), so computation and assignment
    of such tolerances is left to the caller.
    
