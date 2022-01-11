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

#ifndef OPENSUBDIV3_BFR_SURFACE_FACTORY_H
#define OPENSUBDIV3_BFR_SURFACE_FACTORY_H

#include "../version.h"

#include "../bfr/surface.h"
#include "../bfr/types.h"
#include "../sdc/options.h"
#include "../sdc/types.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Forward declarations of internal classes used by the factories:
//
class VertexTopology;
class FaceTopology;
class FaceSurface;
class TopologyCache;

//
//  SurfaceFactory is an abstract class that provides both the interface
//  and the majority of the implementation for a factory that constructs
//  instances of Surfaces for the faces of a mesh.
//
//  A subclasses of SurfaceFactory is written to support a specific type
//  of connected mesh. The public interface of SurfaceFactory is both
//  inherited by and extended by the subclasses. Expected extensions to
//  the interface include one or more constructors (i.e. given a specific
//  instance of the subclass' mesh type) as well as other methods that
//  may involve the mesh's data types in their native form (potentially
//  identifying face-varying topologies).
//
//  A subclass of SurfaceFactory is also required to implement the small
//  suite of pure virtual methods to complete the factory's implementation
//  for the subclass' mesh type. Most of these methods provide the base
//  factory with topological information about faces of that mesh -- from
//  which it creates instances of Surface defining their limit surface.
//
//  It should be emphasized that a subclass is written to support a
//  specific type of "connected" mesh -- not simply a container of data
//  defining a mesh. The abstract methods required by the SurfaceFactory
//  describe the complete topological neighborhood around a specific face,
//  and without any connectivity between mesh components (e.g. given a
//  vertex, what are its incident faces?), satisfying these methods will
//  be impossible, or, at best, extremely inefficient.
//
//  Ultimately a subclass of SurfaceFactory is expected to be a lightweight
//  interface to a connected mesh -- lightweight in terms of both time and
//  memory usage. It's construction is expected to be trivial, after which
//  it can quickly and efficiently provide a Surface for one or more faces
//  of a mesh for immediate evaluation. So construction of an instance of
//  a subclass should involve no heavy pre-processing -- the greater the
//  overhead of a subclass constructor, the more it violates the intention
//  of the base class as a lightweight interface.
//
//  Instances of SurfaceFactory are initialized with a set of Options that
//  form part of the state of the factory and remain fixed for its lifetime.
//  Such options are intended to ensure that the instances of Surface that
//  it creates are consistent, as well as to enable/disable or otherwise
//  manage caching for construction efficiency -- either internally or
//  between itself and other factories (advanced).
//
class SurfaceFactory {
protected: // non-copyable:
    SurfaceFactory(SurfaceFactory const &);
    SurfaceFactory & operator=(SurfaceFactory const &);

public:
    //
    //  The Options class is a simple container specfying options for the
    //  construction of the Factory that will apply to it for its lifetime.
    //  These options cover two areas:
    //
    //      - caching of intermediate topological results (efficiency)
    //      - approximation of the limit surface (accuracy)
    //  
    //  Choices for caching behavior include disabling all caching, the
    //  use of an internal cache for each Factory (the default), or the
    //  specification of an external cache shared between Factories (for
    //  advanced use only).
    //  
    //  For surface approximation, the number of options is intentionally
    //  minimized here for simplicity (in contrast to the Far classes that
    //  are forced to maintain legacy options).  Using a max tessellation
    //  rate is under consideration to control local refinement depth, but
    //  may limit the effectiveness of caching across multiple meshes.
    //
    //  WIP - nesting this inhibits forward-declaration, reconsider...
    //
    class Options {
    public:
        Options() : _maxLevelPrimary(6), _maxLevelSecondary(2),
                    _useDoublePrecision(false), _useStencilTables(false),
                    _disableCache(false), _extCachePtr(0) { }

        //  Alternatives to the default internal TopologyCache:
        void DisableTopologyCache(bool on) { _disableCache = on; }
        bool DisableTopologyCache()  const { return _disableCache; }

        void ExternalTopologyCache(TopologyCache * c) { _extCachePtr = c; }
        TopologyCache * ExternalTopologyCache() const { return _extCachePtr; }

        //  Other configuration options:
        void UseDoublePrecision(bool on) { _useDoublePrecision = on; }
        bool UseDoublePrecision()  const { return _useDoublePrecision; }

        void UseStencilTables(bool on) { _useStencilTables = on; }
        bool UseStencilTables()  const { return _useStencilTables; }

        //  WIP - approximation options are currently in development
        //      - these are not yet recommended for public use
        void MaxLevelPrimary( int n) { _maxLevelPrimary = n; }
        int  MaxLevelPrimary() const { return _maxLevelPrimary; }

        void MaxLevelSecondary( int n) { _maxLevelSecondary = n; }
        int  MaxLevelSecondary() const { return _maxLevelSecondary; }

    private:
        //  Member variables:
        short           _maxLevelPrimary;
        short           _maxLevelSecondary;
        unsigned int    _useDoublePrecision;
        unsigned int    _useStencilTables;
        unsigned int    _disableCache;
        TopologyCache * _extCachePtr;
    };

public:
    //
    //  Simple public queries of the Factory:
    //
    Options GetOptions() const { return _limitOptions; }

    Sdc::SchemeType GetSchemeType() const    { return _schemeType; }
    Sdc::Options    GetSchemeOptions() const { return _schemeOptions; }

public:
    //
    //  Simple public queries of faces prior to Surface construction:
    //
    //  The "has limit surface" query can be used to determine if a face
    //  has an associated limit surface -- usually the case except when the
    //  face is tagged as a hole, or due to boundary interpolation options
    //  when the face lies on a boundary (only for VTX_BOUNDARY_NONE).
    //
    //  Note that the Surface creation methods apply the same test and also
    //  fail when no limit surface exists -- so there is little point
    //  using the test purely as a pre-condition to create/populate. This
    //  separate test exists to detemine existence of a limit surface for
    //  pre-processing needs when the surface is not actually needed.
    //
    //  Similarly, the Parameterization of a face may also be useful for
    //  processing prior to surface construction -- it assumes the face
    //  has been tested for a limit surface and so is trivial:
    //
    bool FaceHasLimitSurface(Index faceIndex) const;

    Parameterization GetFaceParameterization(Index faceIndex) const;

public:
    //
    //  Public methods to construct the limit Surface for a specific face:
    //
    //  Methods are available to both construct new instances of Surface
    //  or to initialize given instances. The latter is typically used to
    //  repeatedly re-initialize the same instance for multiple faces. In
    //  addition to avoiding the heap by declaring it on the stack, such
    //  an instance may also re-use dynamic memory that the Surface may
    //  internally allocate.
    //
    //  Failure of these construction methods is expected (and so to be
    //  tested) when a face has no limit surface -- either due to it being
    //  a hole or through the use of less common boundary interpolation
    //  options. Failure is also possible if the subclass fails to provide
    //  a valid topological description of the face. (WIP - consider more
    //  extreme failure for these cases, e.g. possible assertions.)
    //
    //  First, methods to create or initialize instances of a Surface for
    //  eac of the three different data interpolation types:
    //
    Surface * CreateVertexSurface(     Index faceIndex) const;
    Surface * CreateVaryingSurface(    Index faceIndex) const;
    Surface * CreateFaceVaryingSurface(Index faceIndex, int fvarID = 0) const;

    bool InitVertexSurface(     Index faceIndex, Surface * vtxSurface) const;
    bool InitVaryingSurface(    Index faceIndex, Surface * varSurface) const;
    bool InitFaceVaryingSurface(Index faceIndex, Surface * fvarSurface,
                                                 int       fvarID = 0) const;

    //
    //  Second, a single general method to initialize several Surfaces at
    //  once -- avoiding the duplicate effort spent when constructing each
    //  separately:
    //
    //  Use of this method is highly recommended when requiring a Surface
    //  for both the vertex data (position) and one or more sets of
    //  face-varying data (e.g. texture coordinates).
    //
    //  WIP - overloads and alternative interfaces are under consideration
    //
    bool InitSurfaces(Index faceIndex, Surface * vtxSurface,
                                       Surface * varSurface,
                                       Surface * fvarSurfaces,
                                       int       fvarCount,
                                       int const fvarIDs[] = 0) const;

protected:
    //
    //  Virtual methods required to support construction of Surfaces:
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
    //  Since face-varying data may differ in topology from the vertex
    //  data -- with each set of face-varying data potentially having its
    //  own unique topology -- sets of face-varying data are uniquely
    //  distinguished by an associated integer (a face-varying ID).
    //
    //  Trivial queries:
    virtual bool isFaceHole(Index faceIndex) const = 0;

    virtual int  getFaceSize(Index faceIndex) const = 0;

    //  Identifying indices for a single base face (for linear cases):
    //
    //  (Note use of "face vertex" vs "face fvar-value" for face-varying
    //  is consistent with Far topology queries and used elsewhere.)
    virtual int getFaceVertexIndices(Index faceIndex,
                    Index vertexIndices[]) const = 0;

    virtual int getFaceFVarValueIndices(Index faceIndex,
                    int fvarID, Index fvarValueIndices[]) const = 0;

    //  Identifying topology and associated indices for the complete set
    //  of incident faces surrounding a face-vertex (corner) of a face --
    //  necessary for supporting non-linear surfaces.
    //
    //  Methods here use "FaceVertex" in the name to emphasize that they
    //  require information for a particular corner vertex of the face.
    //
    //  The topology around the face-vertex is described by populating a
    //  given instance of a simple VertexTopology class -- which fully
    //  describes the face-vertex, it incident faces and any sharpness
    //  assigned at or around the face.vertex.  (See the comments with
    //  the VertexTopology definition for more details.)
    //
    //  Two associated methods are required to identify indices for the
    //  incident faces around a face-vertex (getFaceVertexIncidentFace...).
    //  One method gathers the indices for control vertices of the mesh
    //  assigned to the incident faces (their VertexIndices), while the
    //  other gathers indices for a particular set of face-varying values
    //  assigned to them (their FVarValueIndices).  Both methods expect
    //  the incident faces to be ordered consistent with the specification
    //  in VertexTopology, and all indices for all incident faces are
    //  required.
    //
    //  The order of indices assigned to each face for these methods must
    //  also be specified relative to the face-vertex, rather than the
    //  way the face is defined.  For example, if a quad Q is defined by
    //  the four vertices {A, B, C, D}, when gathering the indices for Q
    //  as part of face-vertex C, the indices should be specified starting
    //  with C, i.e. as {C, D, A, B}.  Ordering indices this way makes it
    //  much easier for the factory to identify when face-varying topology
    //  differs from the vertex topology, and both the face-varying and
    //  vertex indices are ordered this way for consistency.
    //
    virtual int populateFaceVertexTopology(
                    Index faceIndex, int faceVertex,
                    VertexTopology * vertexTopology) const = 0;

    virtual int getFaceVertexIncidentFaceVertexIndices(
                    Index faceIndex, int faceVertex,
                    Index vertexIndices[]) const = 0;

    virtual int getFaceVertexIncidentFaceFVarValueIndices(
                    Index faceIndex, int faceVertex,
                    int fvarID, Index fvarValueIndices[]) const = 0;

protected:
    //
    //  Optional virtual topology methods for advanced use:
    //
    virtual bool isFaceTopologyRegular(Index faceIndex,
                                       Index vertexIndices[]) const;

    virtual bool isFaceTopologyRegular(Index faceIndex,
                                       int   fvarID,
                                       Index fvarValueIndices[]) const;

protected:
    //
    //  Additional protected methods used to define a subclasses:
    //
    //  Construction requires specification of the subdivision scheme and
    //  options associated with the mesh (as is the case with other classes
    //  in Far). These will typically reflect the settings in the mesh but
    //  can also be used to override them -- as determined by the subclass.
    //  Common uses of overrides are to assign a subdivision scheme to a
    //  simple polygonal mesh, or to change the face-varying interpolation
    //  for the faster linear interpolation of UVs.
    //
    //  The subclass is also responsible for providing a reference to a
    //  mutable instance of a TopologyCache for use by the base class. The
    //  subclass is free to use any type of TopologyCache that it requires
    //  (e.g. one it has defined/declared for thread-safety) and manages
    //  the lifetime of that instance.  (WIP - currently this is provided
    //  by an additional virtual method, but other means are still under
    //  consideration, e.g. a separate initializer, via Options, etc.)
    //
    SurfaceFactory(Sdc::SchemeType schemeType,
                   Sdc::Options    schemeOptions,
                   Options         limitOptions);
    virtual ~SurfaceFactory();

    virtual TopologyCache * getInternalTopologyCache() const = 0;

private:
    //  Supporting internal methods:
    //
    bool faceHasLimitSimple(Index faceIndex, int faceSize) const;

    bool faceHasLimitNeighborhood(Index faceIndex) const;
    bool faceHasLimitNeighborhood(FaceTopology const & faceTopology) const;

    struct SurfaceSet;

    bool populateAllSurfaces(      Index faceIndex, SurfaceSet * sSetPtr) const;
    bool populateLinearSurfaces(   Index faceIndex, SurfaceSet * sSetPtr) const;
    bool populateNonLinearSurfaces(Index faceIndex, SurfaceSet * sSetPtr) const;

    //  Methods to assemble topology and corresponding indices for entire face:
    bool initFaceNeighborhoodTopology(Index          faceIndex,
                                      FaceTopology * topology) const;

    bool gatherFaceNeighborhoodTopology(Index          faceIndex,
                                        FaceTopology * topology) const;

    int gatherFaceNeighborhoodIndices(Index                faceIndex,
                                      FaceTopology const & topology,
                                      int                  fvarID,
                                      Index                indices[]) const;

    //  Methods to assemble Surfaces for the different categories of patch:
    void assignLinearSurface(Surface * surfacePtr,
                             Index     faceIndex,
                             int       fvarID) const;

    void assignRegularSurface(Surface     * surfacePtr,
                              Index const   surfacePatchPoints[]) const;

    void assignRegularSurface(Surface           * surfacePtr,
                              FaceSurface const & surfaceDescription) const;

    void assignIrregularSurface(Surface           * surfacePtr,
                                FaceSurface const & surfaceDescription) const;

    void copyNonLinearSurface(Surface           * surfacePtr,
                              Surface const     & surfaceSource,
                              FaceSurface const & surfaceDescription) const;

    //  Methods for dealing with optional cache:
    TopologyCache * getTopologyCache() const;

private:
    //  Members describing options and subdivision properties (very little
    //  memory and low initialization cost)
    Sdc::SchemeType _schemeType;
    Sdc::Options    _schemeOptions;
    Options         _limitOptions;

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

#endif /* OPENSUBDIV3_BFR_SURFACE_FACTORY_H */
