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

#ifndef OPENSUBDIV3_BFR_LIMIT_SURFACE_FACTORY_H
#define OPENSUBDIV3_BFR_LIMIT_SURFACE_FACTORY_H

#include "../version.h"

#include "../bfr/types.h"
#include "../bfr/limitSurface.h"

#include "../sdc/options.h"
#include "../sdc/types.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Forward declarations of classes used by the factories:
//
class VertexTopology;
class FaceTopology;
class SurfaceDescriptor;
class TopologyCache;

//
//  LimitSurfaceFactory is an abstract class that provides the construction
//  of instances of LimitSurface from the faces of a mesh.
//
//  LimitSurfaceFactory provides both the public interface for construction
//  as well as the implementation for its subclasses -- each defined to
//  support a specific mesh class.  A subclass implements a small suite of
//  virtual methods to provide topological information about faces of that
//  mesh -- for which LimitSurface instances are created.
//
//  Unlike stateless factory classes with static methods in Far, we want to
//  create instances of this Factory class for specific instances of meshes
//  and construction options so that LimitSurface instances for all faces are
//  constructed consistently.  An instance of such a Factory may also manage
//  its own topology cache internally for all faces of the mesh.
//
class LimitSurfaceFactory {
public:
    //
    //  Options are primarily concerned with caching and approximation.
    //  Former options to create vertex, varying or face-varying evaluators
    //  for each LimitSurface have been moved to LimitSuface construction
    //  (but may eventually be duplicated here).
    //  
    //  The number of shape approximating options is minimized here (in
    //  contrast to the Far classes that are forced to maintain legacy
    //  option).  Using a max tessellation rate is under consideration to
    //  control local refinement depth, but may limit the effectiveness
    //  of caching across multiple meshes.
    //
    //  Given the regret elsewhere of exposing Option members directly,
    //  member variables not public and are accessed/assigned by methods.
    //
    class Options {
    public:
        Options() : maxLevelPrimary(6), maxLevelSecondary(2),
                    disableCache(0), extCachePtr(0) { }

        //  Alternatives to the default internal TopologyCache:
        void DisableTopologyCache(bool on) { disableCache = on; }
        bool DisableTopologyCache()  const { return disableCache; }

        void ExternalTopologyCache(TopologyCache * c) { extCachePtr = c; }
        TopologyCache * ExternalTopologyCache() const { return extCachePtr; }

        //  NOT MEANT FOR PUBLIC USE -- currently for development use:
        void MaxLevelPrimary( int n) { maxLevelPrimary = n; }
        int  MaxLevelPrimary() const { return maxLevelPrimary; }

        void MaxLevelSecondary( int n) { maxLevelSecondary = n; }
        int  MaxLevelSecondary() const { return maxLevelSecondary; }

    protected:
        //  Member variables:
        unsigned int maxLevelPrimary   : 4;
        unsigned int maxLevelSecondary : 4;
        unsigned int disableCache      : 1;

        TopologyCache * extCachePtr;
    };

public:
    //
    //  Simple queries reflecting the mesh associated with an instance:
    //
    Options GetOptions() const { return _limitOptions; }

    Sdc::SchemeType GetSchemeType() const    { return _schemeType; }
    Sdc::Options    GetSchemeOptions() const { return _schemeOptions; }

    int GetRegFaceSize() const { return _regFaceSize; }

    //
    //  Options to construct specific Evaluators for the LimitSurface:
    //
    //  Using these options per-LimitSurface provides added flexibility
    //  but added tedium for those cases that don't warrant it.  For
    //  that reason, duplicating these at the Factory level so that they
    //  do not have to be applied to every face, is being considered.
    //
    class EvaluatorOptions {
    public:
        EvaluatorOptions() : _vtxEvaluator(true),
                             _varEvaluator(false),
                             _fvarEvaluators(0),
                             _fvarIndices(0) { }

        //  Construct Evaluators for vertex and/or varying data:
        void CreateVertexEvaluator(bool on) { _vtxEvaluator = on; }
        bool CreateVertexEvaluator()  const { return _vtxEvaluator; }

        void CreateVaryingEvaluator(bool on) { _varEvaluator = on; }
        bool CreateVaryingEvaluator() const  { return _varEvaluator; }

        //  Specify construction of Evaluators for face-varying topologies:
        //      - specifying the count alone creates [0..count-1]
        //      - specify explicit indices for an unordered subset
        void CreateFVarEvaluators(int count) { _fvarEvaluators = count; }
        int  GetNumFVarEvaluators() const  { return _fvarEvaluators; }

        void SetFVarEvaluatorIndices(int const * iVec) { _fvarIndices = iVec; }
        int const * GetFVarEvaluatorIndices() const { return _fvarIndices; }

    protected:
        //  Member variables:
        unsigned int _vtxEvaluator :  1;
        unsigned int _varEvaluator :  1;

        int        _fvarEvaluators;
        int const *_fvarIndices;
    };

    //
    //  Methods to create or re-populate an existing LimitSurface:
    //
    //  The "has limit surface" query can be used to determine if a face
    //  has an associated limit surface -- usually the case except when the
    //  face is tagged as a hole, or due to boundary interpolation options
    //  when the face lies on a boundary (only for VTX_BOUNDARY_NONE).
    //
    //  But note that create/populate applies the same test and so also
    //  fails when no limit surface exists -- so there is little point
    //  using the test purely as a pre-condition to create/populate. The
    //  separate test exists to detemine existence of a limit surface for
    //  pre-processing needs when the surface is not actually needed.
    //
    //  Failure of create/populate is also possible if the subclass fails
    //  to provide a valid topological description of the face.
    //
    bool FaceHasLimitSurface(Index faceIndex) const;

    LimitSurface * Create(Index            faceIndex,
                          EvaluatorOptions opts = EvaluatorOptions()) const;

    bool Populate(LimitSurface &   instance,
                  Index            faceIndex,
                  EvaluatorOptions opts = EvaluatorOptions()) const;

protected:
    //  (REVIEW 1.1)
    //
    //  Virtual methods required to support LimitSurface construction:
    //
    //  These methods require a subclass to provide a complete description
    //  of the topology around a base face, as well as indices associated
    //  with it (both vertex and face-varying).  A goal here is to keep
    //  the number of methods required to a minimum, and also that these
    //  methods be invoked minimally by the base class as part of the
    //  construction process.
    //
    //  With the need to support both linear and non-linear cases (for
    //  which linear is trivial by comparison) and the limit surface for
    //  both vertex and face-varying topologies, the result is a small set
    //  of methods covering this matrix of functionality.
    //
    //  Trivial queries:
    virtual bool isFaceHole(Index faceIndex) const = 0;

    virtual int  getFaceSize(Index faceIndex) const = 0;

    //  Identifying indices for a single base face (for linear cases):
    //
    //  (Note use of "face vertex" vs "face fvar-value" for face-varying
    //  is consistent with Far topology queries and used elsewhere here.)
    virtual int getFaceVertexIndices(Index faceIndex,
                     Index vertexIndices[]) const = 0;

    virtual int getFaceFVarValueIndices(Index faceIndex,
                     Index fvarValueIndices[], int   fvarID) const = 0;

    //  Identifying topology and associated indices for the complete set
    //  of incident faces surrounding a face-vertex (corner) of a face
    //  (to support non-linear cases):
    //
    //  Populating the VertexTopology describes a set of incident faces
    //  and any related sharpness at our around the specific face-vertex.
    //  The associated methods to identify indices for the incident faces
    //  expect values for those faces to be ordered consistent with the
    //  specification in VertexTopology.
    //
    //  Notes:
    //      - naming here is a bit of a challenge -- terser names being
    //        too vague but more specific names becoming too verbose
    //      - all are prefaced with "face vertex" to indicate a specific
    //        corner vertex of a face
    //      - the index queries can be interpreted by parsing the names
    //        backward as "get the indices for the incident faces around
    //        the given face vertex"
    //      - introducing a term such as "neighborhood" may help here.
    //
    virtual int populateFaceVertexTopology(
                    Index faceIndex, int faceVertex,
                    VertexTopology & vt) const = 0;

    virtual int getFaceVertexIncidentFaceVertexIndices(
                    Index faceIndex, int faceVertex,
                    Index vertexIndices[]) const = 0;

    virtual int getFaceVertexIncidentFaceFVarValueIndices(
                    Index faceIndex, int faceVertex,
                    Index fvarValueIndices[], int fvarID) const = 0;

protected:
    //  (REVIEW 1.2)
    //
    //  Protected constructor/destructor for use by subclasses:
    //
    //  Initialization of base class members is no longer deferred to the
    //  subclass via initialize/finalize methods -- the old constructor for
    //  the base class has been restored.
    // 
    //  Construction requires specification of the subdivision scheme and
    //  options associated with the mesh (as is the case with other classes
    //  in Far). These will typically reflect the settings in the mesh but
    //  can also be used to override them -- as determined by the subclass.
    //  Common uses of overrides are to assign a subdivision scheme to a
    //  simple polygonal mesh, or to change the face-varying interpolation
    //  for the faster linear interpolation of UVs.
    //
    //  The subclass is responsible for determining the type and providing
    //  an instance for the optional internal TopologyCache. For now, this
    //  is explicitly assigned with a specific initialization method, but
    //  other ways to deal with this are under consideration (e.g. via the
    //  Options, an additional virtual method, etc.).
    //
    LimitSurfaceFactory(Sdc::SchemeType schemeType,
                        Sdc::Options    schemeOptions,
                        Options         limitOptions);
    virtual ~LimitSurfaceFactory();

    //  WIP - alternatives to this explicit initializer to be discussed...
    void assignInternalTopologyCache(TopologyCache * cache);

private:
    //  Supporting internal methods:
    //
    bool faceHasLimitLocal(       Index faceIndex, int faceSize) const;
    bool faceHasLimitNeighborhood(Index faceIndex, FaceTopology const *) const;

    bool populateLinearEvaluators(LimitSurface &   limitSurface,
                                  Index            faceIndex,
                                  EvaluatorOptions options) const;
    bool populateNonLinearEvaluators(LimitSurface &   limitSurface,
                                     Index            faceIndex,
                                     EvaluatorOptions options) const;

    //  Methods to assemble topology and corresponding indices for entire face:
    bool gatherFaceNeighborhoodTopology(Index faceIndex,
                                        FaceTopology & topology) const;

    int gatherFaceNeighborhoodIndices(Index faceIndex,
                                      FaceTopology const & topology,
                                      Index indices[], int fvarIndex) const;

    //  Methods to assemble Evaluators for the different categories of patch:
    void assignLinearEvaluator(LimitSurface::Evaluator & evaluator,
                               Index faceIndex, int fvarIndex) const;

    void assignRegularEvaluator(LimitSurface::Evaluator & evaluator,
                                SurfaceDescriptor const & surface) const;

    void assignIrregularEvaluator(LimitSurface::Evaluator & evaluator,
                                  SurfaceDescriptor const & surface) const;

    void copyNonLinearEvaluator(LimitSurface::Evaluator       & dstEvaluator,
                                LimitSurface::Evaluator const & srcEvaluator,
                                SurfaceDescriptor const       & surface) const;

private:
    //  Members describing options, subdivision properties and reference to
    //  an optional cache (very little memory and low initialization cost)
    Sdc::SchemeType _schemeType;
    Sdc::Options    _schemeOptions;
    Options         _limitOptions;

    TopologyCache mutable * _topologyCache;

    //  Members related to subdivision topology, options and limit tests:
    unsigned int _linearScheme      : 1;
    unsigned int _linearFVarInterp  : 1;

    unsigned int _testNeighborhoodForLimit       : 1;
    unsigned int _rejectSmoothBoundariesForLimit : 1;
    unsigned int _rejectIrregularFacesForLimit   : 1;

    int  _regFaceSize;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_LIMIT_SURFACE_FACTORY_H */
