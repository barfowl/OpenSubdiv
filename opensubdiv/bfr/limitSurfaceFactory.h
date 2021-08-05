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

#include "../sdc/options.h"
#include "../sdc/types.h"
#include "../bfr/types.h"
#include "../bfr/limitSurface.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Forward declarations of classes used by the factories:
//
class FaceTopology;
class CornerSubset;
class VertexTopology;
class TopologyCache;

//
//  LimitSurfaceFactory is an abstract class that provides the construction
//  of instances of LimitSurface from the faces of a mesh -- whose type is
//  determined by a subclass.
//
//  Unlike stateless factory classes with static methods in Far, we want to
//  create instances of this Factory class for specific instances of meshes
//  and construction options so that LimitSurface instances for all faces are
//  constructed consistently.  An instance of such a Factory may also manage
//  its own topology cache internally for all faces of the mesh.
//
//  WIP - The nature of the virtual methods required by subclasses warrants
//  close inspection and review.
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

    int GetNumFaces() const { return _numFaces; }
    int GetNumFVarChannels() const { return _numFVarTopologies; }

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
    bool FaceHasLimitSurface(Index baseFace) const;

    LimitSurface * Create(Index            baseFace,
                          EvaluatorOptions opts = EvaluatorOptions()) const;

    bool Populate(LimitSurface &   instance,
                  Index            baseFace,
                  EvaluatorOptions opts = EvaluatorOptions()) const;

protected:
    //
    //  Virtual methods required to support LimitSurface construction:
    //
    virtual bool isFaceHole( Index baseFace) const = 0;
    virtual int  getFaceSize(Index baseFace) const = 0;

    virtual int getFaceVertexIndices(Index baseFace,
                                     Index indices[]) const = 0;
    virtual int getFaceFVarValueIndices(Index baseFace,
                                        Index indices[],
                                        int   fvarIndex) const = 0;

    //  WIP - naming here, i.e. use of "FaceCorner", is questionable
    //      - see notes in header for VertexTopology for details/examples
    virtual int populateFaceCornerTopology(Index baseFace, int cornerVertex,
                                           VertexTopology & vt) const = 0;

    virtual int getFaceCornerVertexIndices(Index baseFace, int cornerVertex,
                                           Index indices[]) const = 0;
    virtual int getFaceCornerFVarValueIndices(Index baseFace, int cornerVertex,
                                              Index indices[],
                                              int fvarIndex) const = 0;

protected:
    //
    //  Fully qualified constructor -- to be used by subclass constructors:
    //
    LimitSurfaceFactory(
        Sdc::SchemeType schemeType,
        Sdc::Options    schemeOptions,
        Options         limitOptions,
        //  WIP - these may not be necessary in the base class
        int             numFaces,
        int             numFVarTopologies);
    virtual ~LimitSurfaceFactory();

    int getRegularFaceSize() const { return _regFaceSize; }

private:
    //  Supporting internal methods:
    //
    //  WIP - hide some of these from public header if possible
    //
    //  Methods to assemble topology and corresponding indices for a face:
    bool populateFaceTopology(Index          baseFace,
                              FaceTopology & faceTopology) const;

    int gatherFaceTopologyIndices(Index                baseFace,
                                  FaceTopology const & faceTopology,
                                  Index                faceTopologyIndices[],
                                  int fvarIndex = -1) const;

    //  Methods to assemble Evaluators for the different categories of patch:
    void assignLinearEvaluator(LimitSurface::Evaluator & evaluator,
                               Index baseFace, int fvarIndex = -1) const;

    void assignRegularEvaluator(LimitSurface::Evaluator & evaluator,
                                FaceTopology const & faceTopology,
                                Index        const   faceIndices[],
                                CornerSubset const   faceSubsets[] = 0) const;

    void assignIrregularEvaluator(LimitSurface::Evaluator & evaluator,
                                  FaceTopology const & faceTopology,
                                  Index        const   faceIndices[],
                                  CornerSubset const   faceSubsets[] = 0) const;

    void copyNonLinearEvaluator(LimitSurface::Evaluator       & dstEvaluator,
                                LimitSurface::Evaluator const & srcEvaluator,
                                FaceTopology const & faceTopology,
                                Index        const   fvarIndices[],
                                CornerSubset const   fvarSubsets[]) const;

    //  Methods to deal with construction and caching of irregular patches:
    IrregPatchPtr findIrregularPatch(FaceTopology const & faceTopology,
                                     CornerSubset const   faceSubsets[],
                                     bool               & patchIsNew,
                                     bool               & patchIsCached) const;

    IrregPatchPtr buildIrregularPatch(FaceTopology const & faceTopology,
                                      CornerSubset const   faceSubsets[]) const;

private:
    Sdc::SchemeType _schemeType;
    Sdc::Options    _schemeOptions;
    Options         _limitOptions;

    unsigned int _linearScheme      : 1;
    unsigned int _linearFVarInterp  : 1;
    unsigned int _testBoundaryLimit : 1;

    int  _regFaceSize;

    TopologyCache mutable *  _topologyCache;

    //  WIP - can easily move these to a subclass, so may not be necessary
    int _numFaces;
    int _numFVarTopologies;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_LIMIT_SURFACE_FACTORY_H */
