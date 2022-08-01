
This regression test is a work in progress, so this page gives some
insight into what still needs to be done and what has been done that
may need to be removed...


Overview of the process:

    - regression/bfr_evaluate compares evaluation of Bfr::Surfaces
      with evaluation of the same points using a Far::PatchTable

    - a high level "evaluator" class is defined for each of Bfr and
      Far to compute a common set of coordinates

    - both evaluators are given the same set of construction Options
      to ensure their internal evaluation is as similar as possible

    - both evaluators are given the same set of tessellation coords
      to ensure identical parametric coords and correspondence

    - differences between evaluated vectors are accumulated and
      printed according to a number of options (more or less detail)


Types of command line options available:

    - controlling which shapes are tested:
        - test a core subset of regression/shapes or all
        - specify a range or explicit subset of regression/shapes
        - override regression/shapes with any number of .obj files

    - controlling what types of data are compared:
        - typically vertex position and/or face-varying UVs
        - include first and/or second derivatives

    - controlling the density of points compared:
        - a uniform tessellation rate can be specified

    - varying the subdivision options applied to all shapes:
        - any boundary interpolation choice can be specified
        - any face-varying interpolation choice can be specified

    - varying tolerances used in comparisons:
        - assign an absolute or relative tolerance for position
        - assign an absolute tolerance for UVs

    - controlling the textual output, i.e including or suppressing:
        - summaries of test parameters and/or overall results
        - progress of each shape included
        - more detailed output for each shape (mesh only or faces)
        - differences for specific groups of data

    - controlling internal parameters of execution:
        - single versus double precision
        - maximum levels of adaptive refinement applied
        - whether stencils are used for Bfr evaluation
        - whether internal caching is used for Bfr evaluation


Areas for improvement:

    - stripping out options that are no longer useful:
       - some exist purely to avoid cases that were not properly
         supported at the time
       - some exist to support implementation options that have
         since been removed

    - better metrics for determining relative tolerances:
       - currently a proportion of the bounding box is used
       - a better approach is to test relative to the bounding box of
         face neighborhood (easily available with a Bfr::Surface)


Nagging complications:

One frustrating aspect of the comparison between Bfr and Far is that --
despite using the same patch evaluation at the lowest level -- the way
that Far couples the processing of vertex and face-varying data leads to
accumulated inaccuracies which trigger differences.

Far is forced to use the same level of adaptive refinement for both
vertex and face-varying data -- even if one or the other is purely
regular.  Bfr is not required to do this and identifies the desired patch
with as little refinement as possible.

So regular vertex patches can be refined to a high depth if there is a
face-varying irregularity present.  The accumulation of round-off thus
reaches a point where the Far result differs from Bfr enough to trigger
warnings.  (In such cases the Bfr results should be considered the more
accurate given the fewer combinations of patch points involved.)

To avoid this situation, the regression test can be run separately with
the focus on different data and with different comparison and reporting.

Running position only ensures that both Far and Bfr will be equally
refined.  Unfortunately, a separate execution for face-varying cannot
entirely avoid the problem since Far is still forced to refine face-varying
deeper than needed when more complex irregularities in the vertex topology
are present.  But command line arguments exist for suppressing differences
based on data type.

So we can run the face-varying execution while ignoring diffs in position
to suppress cases where vertex topology was forced to a greater depth in
Far.  We still can't avoid face-varying topology being forced to a greater
depth by Far, but the inaccuracies tend not to accumulate to the same
degree in UV space compared to XYZ (all UV coordinates being in the unit
square).

Another area where Bfr will not refine to the same depth as Far is with
certain non-manifold features.  In many cases with non-manifold edges,
the patches either side of a non-manifold edge are purely regular.  This
is detected by Bfr, but Far forces any non-manifold feature to be refined
to the deepest level specified.  So even just comparing positions, some
differences arise in these cases.
