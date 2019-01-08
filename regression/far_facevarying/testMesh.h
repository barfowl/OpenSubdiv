//
//   Copyright 2016 Dreamworks
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

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/topologyRefiner.h>
#include <opensubdiv/far/topologyRefinerFactory.h>
#include <opensubdiv/far/topologyLevel.h>
#include <opensubdiv/far/patchTable.h>
#include <opensubdiv/far/patchTableFactory.h>

using namespace OpenSubdiv;
using namespace OpenSubdiv::OPENSUBDIV_VERSION;


//
//  Interpolatable classes for vertex positions and UV coordinates:
//
struct Coord3 {
    Coord3() { }
    Coord3(float a, float b, float c) : x(a), y(b), z(c) { }

    static float MaxCoordDelta(Coord3 const & a, Coord3 const & b) {
        return std::max(fabsf(a.x - b.x), std::max(fabsf(a.y - b.y), fabsf(a.z - b.z)));
    }

    void Clear()                                  { x  = 0.0f,  y  = 0.0f,  z  = 0.0f; }
    void AddWithWeight(Coord3 const & p, float s) { x += s*p.x, y += s*p.y, z += s*p.z; }

    float x, y, z;
};
typedef std::vector<Coord3> Coord3Vector;


//
//  The TopologyContainer class is a companion to the Far::TopologyDescriptor
//  class.  While TopologyDescriptor just contains raw references to arrays of
//  topology data, the TopologyContainer owns the actual data arrays and serves
//  as the source of the TopologyDescriptor's references.
//
class TopologyContainer {
public:
    typedef Far::Index Index;
public:
    TopologyContainer() : vertexCount(0), faceCount(0), fvarValueCount(0) { }
    TopologyContainer(TopologyContainer const & source);
    ~TopologyContainer() { }

    void GetTopologyDescriptor(Far::TopologyDescriptor &              descriptor,
                               Far::TopologyDescriptor::FVarChannel & descFVarChannel) const;

public:
    int vertexCount;
    int faceCount;
    int fvarValueCount;

    std::vector<int>   faceVertexCounts;
    std::vector<int>   faceVertexOffsets;
    std::vector<Index> faceVertexIndices;
    std::vector<Index> creaseVertexPairs;
    std::vector<float> creaseWeights;
    std::vector<Index> cornerVertexIndices;
    std::vector<float> cornerWeights;
    std::vector<Index> holeFaceIndices;
    std::vector<Index> fvarValueIndices;
    std::vector<int>   fvarPerVertex;
};


//
//  The TestMesh class holds a variety of mesh related data including the base
//  level topology, vertex positions and face-varying UVs, Far::TopologyRefiner,
//  Far::PatchTable, etc.
//
//  The Shape class is used to initialize a TestMesh, but is otherwise unsuitable
//  for our purposes here as its data is not in a form that is either accessible
//  or amenable to being modified and Refiners, PatchTables, etc. reconstructed.
//
//  The intent is that the base topology in the TopologyContainer and other options
//  be modified and the refined data and patch tables regenerated.
//
class TestMesh {
public:
    typedef Sdc::Options::FVarLinearInterpolation FVarOption;

    typedef Far::TopologyDescriptor                         TopologyDescriptor;
    typedef Far::TopologyRefinerFactory<TopologyDescriptor> TopologyRefinerFactory;
    typedef TopologyRefinerFactory::Options                 TopologyRefinerOptions;
    typedef Far::TopologyRefiner                            TopologyRefiner;
    typedef Far::TopologyLevel                              TopologyLevel;

    typedef Far::PatchTableFactory          PatchTableFactory;
    typedef Far::PatchTableFactory::Options PatchTableOptions;
    typedef Far::PatchTable                 PatchTable;

public:
    TestMesh();
    TestMesh(TestMesh const & sourceMesh);
    ~TestMesh();

    //  Alternate construction methods:
    bool BuildFromShape(Shape const & shape);
    bool BuildFromUvChannel(TestMesh const & sourceMesh);

    //  Inspection of properties that influence testing:
    bool HasUvs() const { return _uvwCoords.size() > 0; }
    bool IsCatmark() const { return _subdivType == Sdc::SCHEME_CATMARK; }
    int  GetRegFaceSize() const { return Sdc::SchemeTypeTraits::GetRegularFaceSize(_subdivType); }

    //  Access to primary Far components and primvar data vectors:
    TopologyRefiner const & GetTopologyRefiner() const { return *_topRefiner; }
    TopologyLevel const &   GetBaseLevel() const { return _topRefiner->GetLevel(0); }
    PatchTable const &      GetPatchTable() const { return *_patchTable; }
    Coord3Vector const &    GetXyzVector() const { return _xyzCoords; }
    Coord3Vector const &    GetUvwVector() const { return _uvwCoords; }
    int                     GetUvChannel() const { assert(HasUvs()); return _preUvChannels; }

    //  Methods to vary UV representations:
    void AssignUvChannelPerVertex();
    void AssignUvChannelInterpolation(FVarOption interpolationOption);
    void ExpandUvChannel();
    void ClearUvChannel();

    void AssignEmptyChannels(int numPreUvChannels, int numPostUvChannels);
    void ClearEmptyChannels();

    void SharpenFromUvChannel(TestMesh const & sourceMesh, FVarOption interpolationOption);

    void Rebuild(TopologyRefiner::AdaptiveOptions const * adaptiveOptions = 0,
                 TopologyRefiner::UniformOptions const *  uniformOptions = 0,
                 PatchTableOptions const *                patchOptions = 0);


    //  for debugging
    bool WriteToObjFile(char const * objFilename) const;

private:
    void initTopologyContainerFromRefiner();
    void interpolateVertexData();
    void interpolateFaceVaryingData();

private:
    Sdc::SchemeType _subdivType;
    Sdc::Options    _subdivOptions;
    int             _preUvChannels;
    int             _postUvChannels;

    TopologyContainer _topContainer;
    TopologyRefiner * _topRefiner;
    PatchTable *      _patchTable;

    Coord3Vector _xyzCoords;
    Coord3Vector _uvwCoords;
};
