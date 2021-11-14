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
    //  The Evaluators class is a simple optional container used to instruct
    //  the Factory to create instances of LimitSurface with Evaluators
    //  enabled for specific types of interpolation data, i.e. vertex, varying
    //  or face-varying.
    //
    //  The default is to enable only the vertex Evaluator, and if there is
    //  no interest in evaluating varying or face-varying data, explicit
    //  assignment of Evaluators can be completely ignored.
    //
    //  When necessary, a set of Evaluators can be specified either to the
    //  Factory on construction or specified individually for construction
    //  of a LimitSurface for each face.  Methods creating a LimitSurface
    //  are overloaded to allow the Factory's default set of Evaluators to
    //  be applied or to be explicitly overridden.
    //
    class Evaluators {
    public:
        Evaluators() : _vtxEval(true), _varEval(false),
                       _fvarEvalCount(0), _fvarEvalIDs(0) { }

        //  Construct Evaluators for vertex and/or varying data:
        void CreateVertexEvaluator(bool on) { _vtxEval = on; }
        bool CreateVertexEvaluator()  const { return _vtxEval; }

        void CreateVaryingEvaluator(bool on) { _varEval = on; }
        bool CreateVaryingEvaluator() const  { return _varEval; }

        //  Construct Evaluators for one or more face-varying topologies:
        //
        //  Each face-varying topology is identified by an ID (int) that
        //  the Factory implementation uses to access the appropriate
        //  mesh topology.  If not explicitly provided here, the IDs for
        //  N face-varying Evaluators are assumed to be [0..N-1].
        void CreateFVarEvaluator(int fvarID = 0);
        void CreateFVarEvaluators(int count, int const fvarIDs[] = 0);

        int GetNumFVarEvaluators() const { return _fvarEvalCount; }

        void SetFVarEvaluatorID(int i, int fvarID) { _fvarEvalIDs[i] = fvarID; }
        int  GetFVarEvaluatorID(int i) const       { return _fvarEvalIDs[i]; }

        //  WIP - for near-term backward compatibility (to be deprecated)
        void SetFVarEvaluatorIndices(int const * iVec) {
            std::memcpy(_fvarEvalIDs, iVec, _fvarEvalCount * sizeof(int));
        }
        int const * GetFVarEvaluatorIndices() const { return _fvarEvalIDs; }

    private:
        //  Member variables:
        unsigned int _vtxEval : 1;
        unsigned int _varEval : 1;

        int              _fvarEvalCount;
        int *            _fvarEvalIDs;
        int              _fvarEvalIDsStatic[4];
        std::vector<int> _fvarEvalIDsDynamic;
    };

    //  WIP - for near-term backward compatibility (to be deprecated)
    typedef Evaluators EvaluatorOptions;

    //
    //  The Options class is a simple container specfying options for the
    //  construction of the Factory.  These options cover three areas:
    //
    //      - specification of default Evaluators (see Evaluators)
    //      - caching of intermediate topological results for efficiency
    //      - approximation of the limit surface
    //  
    //  Choices for caching behavior include disabling all caching, the
    //  use of an internal cache for each Factory (the default), or the
    //  specification of an external cache shared between Factories (for
    //  advanced use only).
    //  
    //  The number of shape approximating options is minimized here (in
    //  contrast to the Far classes that are forced to maintain legacy
    //  option).  Using a max tessellation rate is under consideration to
    //  control local refinement depth, but may limit the effectiveness
    //  of caching across multiple meshes.
    //
    class Options {
    public:
        Options() : _evaluators(),
                    _maxLevelPrimary(6), _maxLevelSecondary(2),
                    _disableCache(false), _extCachePtr(0) { }

        //  Access Evaluators by reference to assign or query:
        Evaluators const & GetEvaluators() const { return _evaluators; }
        Evaluators       & GetEvaluators()       { return _evaluators; }

        //  Alternatives to the default internal TopologyCache:
        void DisableTopologyCache(bool on) { _disableCache = on; }
        bool DisableTopologyCache()  const { return _disableCache; }

        void ExternalTopologyCache(TopologyCache * c) { _extCachePtr = c; }
        TopologyCache * ExternalTopologyCache() const { return _extCachePtr; }

        //  NOT MEANT FOR PUBLIC USE -- currently for development use:
        void MaxLevelPrimary( int n) { _maxLevelPrimary = n; }
        int  MaxLevelPrimary() const { return _maxLevelPrimary; }

        void MaxLevelSecondary( int n) { _maxLevelSecondary = n; }
        int  MaxLevelSecondary() const { return _maxLevelSecondary; }

    private:
        //  Member variables:
        Evaluators      _evaluators;
        short           _maxLevelPrimary;
        short           _maxLevelSecondary;
        bool            _disableCache;
        TopologyCache * _extCachePtr;
    };

public:
    //
    //  Simple queries:
    //
    Options GetOptions() const { return _limitOptions; }

    Sdc::SchemeType GetSchemeType() const    { return _schemeType; }
    Sdc::Options    GetSchemeOptions() const { return _schemeOptions; }

    //  WIP - nice convenience but probably not necessary if we want to
    //        keep the number of public methods to a minimum
    int GetRegFaceSize() const { return _regFaceSize; }

    //
    //  Methods to create or re-populate an existing LimitSurface -- both
    //  exist with a variant taking a set of Evaluators to override those
    //  use by the Factory as defaults.
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

    LimitSurface * Create(Index faceIndex) const;
    LimitSurface * Create(Index faceIndex,
                          Evaluators const & evaluators) const;

    bool Populate(LimitSurface & instance, Index faceIndex) const;
    bool Populate(LimitSurface & instance, Index faceIndex,
                  Evaluators const & evaluators) const;

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

    bool populateLinearEvaluators(LimitSurface &     limitSurface,
                                  Index              faceIndex,
                                  Evaluators const & evaluators) const;
    bool populateNonLinearEvaluators(LimitSurface &     limitSurface,
                                     Index              faceIndex,
                                     Evaluators const & evaluators) const;

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

//
//  Inline methods for LimitSurfaceFactory:
//
inline LimitSurface *
LimitSurfaceFactory::Create(Index faceIndex) const {

    return Create(faceIndex, _limitOptions.GetEvaluators());
}

inline bool
LimitSurfaceFactory::Populate(LimitSurface & instance, Index faceIndex) const {

    return Populate(instance, faceIndex, _limitOptions.GetEvaluators());
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_LIMIT_SURFACE_FACTORY_H */
