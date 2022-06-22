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
#include "../sdc/options.h"
#include "../sdc/types.h"

#include <cstdint>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Forward declarations of public and internal classes used by factories:
//
class VertexDescriptor;
class FaceTopology;
class FaceSurface;
class SurfaceFactoryCache;

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
    typedef int Index;

    //
    //  A face-varying ID is used to specifiy face-varying primvars for
    //  evaluation so that they can be identified by the subclass for
    //  the mesh.  It can be assigned as an integer ID or pointer -- as
    //  dictated by the use of the subclass.
    //
    //  Often only one face-varying primvar is of interest, so a default
    //  can be assigned to the factory to avoid repeated specification.
    //
    typedef std::intptr_t FVarID;

    //
    //  The Options class is a simple container specfying options for the
    //  construction of the Factory that will apply to it for its lifetime.
    //  These options currently include:
    //
    //      - a default identifier to use for face-varying surfaces
    //      - caching of intermediate topological results
    //      - parameters for approximating the limit surface
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
    class Options {
    public:
        Options() : _dfltFVarID(-1), _externCache(0), _enableCache(true),
                    _approxLevelSmooth(2), _approxLevelSharp(6) { }

        //  Assign the default face-varying ID (no valid default):
        Options & SetDefaultFVarID(FVarID id);
        FVarID    GetDefaultFVarID() const { return _dfltFVarID; }

        //  Enable caching via internal or external cache (default is true):
        Options & EnableCaching(bool on);
        bool      IsCachingEnabled() const { return _enableCache; }

        //  Assign an external cache, potentially shared between Factories:
        Options &             SetExternalCache(SurfaceFactoryCache * c);
        SurfaceFactoryCache * GetExternalCache() const { return _externCache; }

        //  Set refinement levels used to approximate the limit surface
        //  for smooth and sharp features (reasonable defaults assigned):
        Options & SetApproxLevelSmooth(int level);
        int       GetApproxLevelSmooth() const { return _approxLevelSmooth; }

        Options & SetApproxLevelSharp(int level);
        int       GetApproxLevelSharp() const { return _approxLevelSharp; }

    private:
        //  Member variables:
        FVarID _dfltFVarID;

        SurfaceFactoryCache * _externCache;

        unsigned char _enableCache : 1;
        unsigned char _approxLevelSmooth;
        unsigned char _approxLevelSharp;
    };

public:
    //
    //  Simple public queries of the Factory:
    //
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
    //  Public methods to initialize instances of the limit Surface for
    //  a specific face:
    //
    //  Failure of these initialization methods is expected (and so to be
    //  tested) when a face has no limit surface -- either due to it being
    //  a hole or through the use of less common boundary interpolation
    //  options. Failure is also possible if the subclass fails to provide
    //  a valid topological description of the face. (WIP - consider more
    //  extreme failure for these cases, e.g. possible assertions.)
    //
    //  Given the different interpolation types for data associated with
    //  mesh vertices (i.e. vertex, varying and face-varying data), the
    //  topology of the limit surface potentially (likely) differs between
    //  them. So it is necessary to identify the type of data associated
    //  with the Surface. Methods exist to initialize a single surface for
    //  each of the three data types, and to initialize multiple surfaces
    //  for different data types at once (which will avoid repeated effort
    //  in a single-threaded context).
    //
    //  Methods to initialize a Surface for a specific interpolation type:
    //
    template <typename REAL>
    bool InitVertexSurface(Index faceIndex, Surface<REAL> * surface) const;

    template <typename REAL>
    bool InitVaryingSurface(Index faceIndex, Surface<REAL> * surface) const;

    template <typename REAL>
    bool InitFaceVaryingSurface(Index faceIndex, Surface<REAL> * surface) const;
    template <typename REAL>
    bool InitFaceVaryingSurface(Index faceIndex, Surface<REAL> * surface,
                                                 FVarID          fvarID) const;

    //
    //  General method to initialize multiple Surfaces for any combination
    //  of the three different data interpolation types:
    //
    template <typename REAL>
    bool InitSurfaces(Index faceIndex, Surface<REAL> * vtxSurface,
                                       Surface<REAL> * fvarSurfaces,
                                       FVarID const    fvarIDs[] = 0,
                                       int             fvarCount = 0,
                                       Surface<REAL> * varSurface = 0) const;

    //
    //  Convenience methods to construct/allocate Surfaces:
    //      WIP - considering removing these since non-essential
    //
    template <typename REAL=float>
    Surface<REAL> * CreateVertexSurface(Index faceIndex) const;

    template <typename REAL=float>
    Surface<REAL> * CreateVaryingSurface(Index faceIndex) const;

    template <typename REAL=float>
    Surface<REAL> * CreateFaceVaryingSurface(Index faceIndex) const;
    template <typename REAL=float>
    Surface<REAL> * CreateFaceVaryingSurface(Index faceIndex, FVarID id) const;

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
                    FVarID fvarID, Index fvarValueIndices[]) const = 0;

    //  Identifying topology and associated indices for the complete set
    //  of incident faces surrounding a face-vertex (corner) of a face --
    //  necessary for supporting non-linear surfaces.
    //
    //  Methods here use "FaceVertex" in the name to emphasize that they
    //  require information for a particular corner vertex of the face.
    //
    //  The topology around the face-vertex is described by populating a
    //  given instance of a simple VertexDescriptor class -- which fully
    //  describes the face-vertex, it incident faces and any sharpness
    //  assigned at or around the face.vertex.  (See the comments with
    //  the VertexDescriptor definition for more details.)
    //
    //  Two associated methods are required to identify indices for the
    //  incident faces around a face-vertex (getFaceVertexIncidentFace...).
    //  One method gathers the indices for control vertices of the mesh
    //  assigned to the incident faces (their VertexIndices), while the
    //  other gathers indices for a particular set of face-varying values
    //  assigned to them (their FVarValueIndices).  Both methods expect
    //  the incident faces to be ordered consistent with the specification
    //  in VertexDescriptor, and all indices for all incident faces are
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
    virtual int populateFaceVertexDescriptor(
                    Index faceIndex, int faceVertex,
                    VertexDescriptor * vertexDescriptor) const = 0;

    virtual int getFaceVertexIncidentFaceVertexIndices(
                    Index faceIndex, int faceVertex,
                    Index vertexIndices[]) const = 0;

    virtual int getFaceVertexIncidentFaceFVarValueIndices(
                    Index faceIndex, int faceVertex,
                    FVarID fvarID, Index fvarValueIndices[]) const = 0;

protected:
    //
    //  Optional virtual topology methods for advanced use:
    //
    //  For cases when a mesh can quickly determine if the neighborhood
    //  around a faces is purely regular, these methods can be used to
    //  quickly identify the control point indices for the corresponding
    //  regular patch. In doing so, the more tedious topological assembly
    //  requiring information about each face-vertex can be avoided.
    //
    //  The indices returned must be ordered according to the regular
    //  patch type corresponding to the subdivision scheme of the mesh.
    //  Boundary vertices are allowed and indicated by an Index of -1.
    //
    //  The face-varying version will only be called if the vertex version
    //  is purely regular, in which case, the face-varying topology is
    //  expected to be similar.
    //
    //  Note that these methods may pass 0 for the index array[] in some
    //  cases -- in which case only the return value should be provided.
    //
    virtual bool getFaceNeighborhoodVertexIndicesIfRegular(
                    Index faceIndex,
                    Index vertexIndices[]) const;

    virtual bool getFaceNeighborhoodFVarValueIndicesIfRegular(
                    Index faceIndex,
                    FVarID fvarID, Index fvarValueIndices[]) const;

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
    //  mutable instance of a SurfaceFactoryCache for use by the base class.
    //  The subclass is free to use any type of SurfaceFactoryCache that it
    //  requires (e.g. one it has defined/declared for thread-safety) and
    //  manages the lifetime of that instance. (WIP - currently this is
    //  provided by an additional virtual method, but other means are under
    //  consideration, e.g. a separate initializer, via Options, etc.)
    //
    SurfaceFactory(Sdc::SchemeType      schemeType,
                   Sdc::Options const & schemeOptions,
                   Options      const & limitOptions);
    virtual ~SurfaceFactory();

    virtual SurfaceFactoryCache * getInternalCache() const = 0;

private:
    //  Supporting internal methods:
    //
    bool faceHasLimitSimple(Index faceIndex, int faceSize) const;

    bool faceHasLimitNeighborhood(Index faceIndex) const;
    bool faceHasLimitNeighborhood(FaceTopology const & faceTopology) const;

    class SurfaceSet;

    bool populateAllSurfaces(      Index faceIndex, SurfaceSet * sSetPtr) const;
    bool populateLinearSurfaces(   Index faceIndex, SurfaceSet * sSetPtr) const;
    bool populateNonLinearSurfaces(Index faceIndex, SurfaceSet * sSetPtr) const;

    bool initSurfaces(Index faceIndex, internal::SurfaceData * vtxSurface,
                                       internal::SurfaceData * varSurface,
                                       internal::SurfaceData * fvarSurfaces,
                                       int           fvarCount,
                                       FVarID const  fvarIDs[]) const;

    //  Methods to assemble topology and corresponding indices for entire face:
    bool isFaceNeighborhoodRegular(Index          faceIndex,
                                   FVarID const * fvarPtrOrVtx,
                                   Index          indices[]) const;

    bool initFaceNeighborhoodTopology(Index          faceIndex,
                                      FaceTopology * topology) const;

    bool gatherFaceNeighborhoodTopology(Index          faceIndex,
                                        FaceTopology * topology) const;

    int gatherFaceNeighborhoodIndices(Index                faceIndex,
                                      FaceTopology const & topology,
                                      FVarID       const * fvarPtrOrVtx,
                                      Index                indices[]) const;

    //  Methods to assemble Surfaces for the different categories of patch:
    typedef internal::SurfaceData SurfaceType;

    void assignLinearSurface(SurfaceType  * surfacePtr,
                             Index          faceIndex,
                             FVarID const * fvarPtrOrVtx) const;

    void assignRegularSurface(SurfaceType * surfacePtr,
                              Index const   surfacePatchPoints[]) const;

    void assignRegularSurface(SurfaceType       * surfacePtr,
                              FaceSurface const & surfaceDescription) const;

    void assignIrregularSurface(SurfaceType       * surfacePtr,
                                FaceSurface const & surfaceDescription) const;

    void copyNonLinearSurface(SurfaceType       * surfacePtr,
                              SurfaceType const & surfaceSource,
                              FaceSurface const & surfaceDescription) const;

    //  Methods for dealing with optional cache:
    SurfaceFactoryCache * getAssignedCache() const;

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

//
//  Inline Options:
//
inline SurfaceFactory::Options &
SurfaceFactory::Options::SetDefaultFVarID(FVarID id) {
    _dfltFVarID = id;
    return *this;
}
inline SurfaceFactory::Options &
SurfaceFactory::Options::EnableCaching(bool on) {
    _enableCache = on;
    return *this;
}
inline SurfaceFactory::Options & 
SurfaceFactory::Options::SetExternalCache(SurfaceFactoryCache * c) {
    _externCache = c;
    return *this;
}
inline SurfaceFactory::Options &
SurfaceFactory::Options::SetApproxLevelSmooth(int level) {
    _approxLevelSmooth = (unsigned char) level;
    return *this;
}
inline SurfaceFactory::Options &
SurfaceFactory::Options::SetApproxLevelSharp(int level) {
    _approxLevelSharp = (unsigned char) level;
    return *this;
}

//
//  Inline methods to initializes Surfaces:
//
template <typename REAL>
inline bool
SurfaceFactory::InitVertexSurface(Index face, Surface<REAL> * s) const {

    return initSurfaces(face, &s->getSurfaceData(), 0, 0, 0, 0);
}
template <typename REAL>
inline bool
SurfaceFactory::InitVaryingSurface(Index face, Surface<REAL> * s) const {

    return initSurfaces(face, 0, &s->getSurfaceData(), 0, 0, 0);
}
template <typename REAL>
inline bool
SurfaceFactory::InitFaceVaryingSurface(Index face, Surface<REAL> * s,
                                                   FVarID fvarID) const {
    return initSurfaces(face, 0, 0, &s->getSurfaceData(), 1, &fvarID);
}
template <typename REAL>
inline bool
SurfaceFactory::InitFaceVaryingSurface(Index face, Surface<REAL> * s) const {
    FVarID dfltID = _limitOptions.GetDefaultFVarID();
    return initSurfaces(face, 0, 0, &s->getSurfaceData(), 1, &dfltID);
}

template <typename REAL>
inline bool
SurfaceFactory::InitSurfaces(Index faceIndex, Surface<REAL> * vtxSurface,
        Surface<REAL> * fvarSurfaces, FVarID const fvarIDs[], int fvarCount,
        Surface<REAL> * varSurface) const {

    bool   useDfltFVarID = fvarSurfaces && (fvarIDs == 0) && (fvarCount == 0);
    FVarID dfltFVarID    = useDfltFVarID ? _limitOptions.GetDefaultFVarID() : 0;

    return initSurfaces(faceIndex,
                        vtxSurface    ? &vtxSurface->getSurfaceData()   : 0,
                        varSurface    ? &varSurface->getSurfaceData()   : 0,
                        fvarSurfaces  ? &fvarSurfaces->getSurfaceData() : 0,
                        fvarCount     ? fvarCount : (fvarSurfaces != 0),
                        useDfltFVarID ? &dfltFVarID : fvarIDs);
}

//
//  Inline methods to allocate and initialize Surfaces:
//
template <typename REAL>
inline Surface<REAL> *
SurfaceFactory::CreateVertexSurface(Index faceIndex) const {
    Surface<REAL> * s = new Surface<REAL>();
    if (InitVertexSurface<REAL>(faceIndex, s)) return s;
    delete s;
    return 0;
}
template <typename REAL>
inline Surface<REAL> *
SurfaceFactory::CreateVaryingSurface(Index faceIndex) const {
    Surface<REAL> * s = new Surface<REAL>();
    if (InitVaryingSurface<REAL>(faceIndex, s)) return s;
    delete s;
    return 0;
}
template <typename REAL>
inline Surface<REAL> *
SurfaceFactory::CreateFaceVaryingSurface(Index faceIndex, FVarID fvarID) const {
    Surface<REAL> * s = new Surface<REAL>();
    if (InitFaceVaryingSurface<REAL>(faceIndex, s, fvarID)) return s;
    delete s;
    return 0;
}
template <typename REAL>
inline Surface<REAL> *
SurfaceFactory::CreateFaceVaryingSurface(Index face) const {
    FVarID dfltID = _limitOptions.GetDefaultFVarID();
    return CreateFaceVaryingSurface<REAL>(face, dfltID);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_SURFACE_FACTORY_H */
