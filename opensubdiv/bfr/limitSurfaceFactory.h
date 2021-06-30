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
class RegularFaceDescriptor;
class ManifoldFaceDescriptor;
class NonManifoldFaceDescriptor;
class TopologyCache;
namespace internal {
    class RegularFaceBuilder;
    class ManifoldFaceBuilder;
    class NonManifoldFaceBuilder;
}

//
//  WORK IN PROGRESS...
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
//  WIP - The nature of the virtual methods required by subclasses is going
//  to change significantly here.
//
class LimitSurfaceFactory {
public:
    //
    //  Options here include whether or not to create Evaluators for the
    //  different types of primvar data (vertex, face-varying or varying),
    //  and to control caching.
    //
    //  We prefer to avoid (or at least minimize) the number of shape
    //  approximating options here compared to the Far classes.  Using a
    //  max tessellation rate is under consideration, but may limit the
    //  effectiveness of caching across multiple meshes...
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
    //  WIP - The number of different FaceDescriptors available will be
    //  reduced, and the nature of those remaining is going to change.
    //
    virtual bool isFaceHole( Index baseFace) const = 0;
    virtual int  getFaceSize(Index baseFace) const = 0;

    virtual int getFaceVertexIndices(   Index baseFace,
                                        Index indices[]) const = 0;
    virtual int getFaceFVarValueIndices(Index baseFace,
                                        Index indices[], int fvIndex) const = 0;

    virtual bool populateDescriptor(Index baseFace,
                                    RegularFaceDescriptor &) const = 0;
    virtual bool populateDescriptor(Index baseFace,
                                    ManifoldFaceDescriptor &) const = 0;
    virtual bool populateDescriptor(Index baseFace,
                                    NonManifoldFaceDescriptor &) const = 0;

protected:
    //
    //  Fully qualified constructor -- to be used by subclass constructors:
    //
    LimitSurfaceFactory(
        Sdc::SchemeType schemeType,
        Sdc::Options    schemeOptions,
        Options         limitOptions,
        int             numFaces,
        int             numFVarTopologies);
    virtual ~LimitSurfaceFactory();

private:
    //  Supporting internal methods:
    //      - WIP - hide these from public header if possible
    bool assignLinearPatch(LimitSurface::Evaluator &, Parameterization p,
                           int faceIndex, int fvarIndex) const;

    bool assignRegularPatch(LimitSurface::Evaluator &, Parameterization p,
                            internal::RegularFaceBuilder const &) const;

    template <class INTERNAL_BUILDER_TYPE>
    bool assignIrregularPatch(LimitSurface::Evaluator &, Parameterization p,
                              INTERNAL_BUILDER_TYPE const &) const;

private:
    Sdc::SchemeType _schemeType;
    Sdc::Options    _schemeOptions;
    Options         _limitOptions;

    int _numFaces;
    int _numFVarTopologies;

    int  _regFaceSize;
    bool _linearScheme;
    bool _linearFVar;

    TopologyCache mutable *  _topologyCache;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_LIMIT_SURFACE_FACTORY_H */
