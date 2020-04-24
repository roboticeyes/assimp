/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2018, Robotic Eyes GmbH

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/

#ifndef ASSIMP_BUILD_NO_EXPORT
#ifndef ASSIMP_BUILD_NO_REX_EXPORTER

#include "RexExporter.h"
#include <assimp/Exceptional.h>
#include <assimp/StringComparison.h>
#include <assimp/version.h>
#include <assimp/IOSystem.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/material.h>
#include <assimp/scene.h>
#include <memory>
#include <string>

using namespace Assimp;
using namespace rex;

static const uint16_t VERSION (1);
static const uint8_t MAGIC_SIZE (4);
static const uint16_t START_DATA_BLOCK (86);
static const uint16_t AUTH_NAME_SIZE (4);
static const uint32_t SRID (3876);
static const uint32_t RESERVED_SIZE (42);
static const unsigned char AUTH_NAME[5] = "EPSG";

static const uint16_t DATA_TYPE_MESH = 3;

static const uint8_t VERTEX_SIZE (12);
static const uint8_t NORMAL_SIZE (12);
static const uint8_t COLOR_SIZE (12);
static const uint8_t TRIANGLE_SIZE (12);
static const uint8_t MESH_HEADER_SIZE (68);
static const uint8_t MESH_HEADER_SIZE_V1 (60);
static const uint8_t DATA_BLOCK_HEADER_SIZE (16);
static const uint8_t HEADER_POS_NR_OF_DATA_BLOCKS (10);
static const uint8_t HEADER_POS_SIZE_OF_DATA_BLOCKS (14);
static const uint8_t LINE_SET_HEADER_SIZE (32);

unsigned char DATA_BLOCK_HEADER[DATA_BLOCK_HEADER_SIZE];
unsigned char MESH_HEADER_V1[MESH_HEADER_SIZE_V1];
unsigned char MESH_HEADER[MESH_HEADER_SIZE];


namespace Assimp
{

// ------------------------------------------------------------------------------------------------
// Worker function for exporting a scene to Robotic Eyes REX format. Prototyped and registered in Exporter.cpp
    void ExportSceneRex (const char *pFile, IOSystem *pIOSystem, const aiScene *pScene, const ExportProperties * /*pProperties*/)
    {
        RexExporter exporter (pFile, pScene);
        exporter.Start();
    }
}

// ------------------------------------------------------------------------------------------------
RexExporter::RexExporter (const char *fileName, const aiScene *pScene)
    : m_Scene (pScene)
{
    m_File = std::make_shared<FileWrapper> (fileName, "wb");
    if (m_File == nullptr)
    {
        throw std::runtime_error ("Cannot open file for writing.");
    }
}

// ------------------------------------------------------------------------------------------------
RexExporter::~RexExporter()
{

}

void RexExporter::Start()
{
    printf ("Starting REX exporter ...\n");

    WriteGeometryFile();
}

void RexExporter::WriteGeometryFile() {
    printf("Write geometry file\n");

    std::vector<MeshPtr> meshPtrs;
    std::vector<MaterialPtr> materialPtrs;

    rex_header *header = rex_header_create();

    // get materials
    WriteMaterials(header, materialPtrs);

    // get meshes
    WriteMeshes(header, meshPtrs);

    long header_sz;
    uint8_t *header_ptr = rex_header_write (header, &header_sz);

    ::fseek (m_File->ptr(), 0, SEEK_SET);
     m_File->write (header_ptr, header_sz, 1, "writeHeader");

     for (MaterialPtr m : materialPtrs) {
         m_File->write (m.material, m.size, 1, "writeMaterial");
     }

     for (MeshPtr m : meshPtrs) {
         m_File->write (m.mesh, m.size, 1, "writeMesh");
     }
}

void RexExporter::WriteMeshes(rex_header *header, std::vector<MeshPtr> &meshPtrs) {
    // collect mesh geometry
    aiMatrix4x4 mBase;
    AddNode(m_Scene->mRootNode, mBase);

    printf("Found %d meshes\n", (int)mMeshes.size());

    meshPtrs.resize(mMeshes.size());

    // now write all mesh instances
    int i = 0;
    for(MeshInstance& m : mMeshes) {
        rex_mesh rexMesh;
        rex_mesh_init(&rexMesh);

        rexMesh.lod = 0; //??
        rexMesh.max_lod = 0; //??
        sprintf(rexMesh.name, "%s", m.name.c_str());
        rexMesh.nr_triangles = m.triangles.size();
        rexMesh.nr_vertices = m.verticesWithColors.size();


        // vertices with colors
        std::vector<vertexData> verticesWithColors;
        m.verticesWithColors.getKeys(verticesWithColors);
        std::vector<aiVector3D> vertices;
        std::vector<aiColor3D> colors;
        for ( const vertexData& v : verticesWithColors ) {
            vertices.push_back(v.vp);
            colors.push_back(v.vc);
        }
        std::vector<float> vertexArray;
        std::vector<float> colorArray;

        getVertexArray(vertices, vertexArray);
        getColorArray(colors, colorArray);

        rexMesh.positions = &vertexArray[0];
        rexMesh.colors = &colorArray[0];

        // triangles
        std::vector<uint32_t> triangles;
        getTriangleArray(m.triangles, triangles);
        rexMesh.triangles = &triangles[0];

        // material
        rexMesh.material_id = m.materialId;//0x7fffffffffffffffL;

        meshPtrs[i].mesh = rex_block_write_mesh (i /*id*/, header, &rexMesh, &meshPtrs[i].size);
        i++;
    }
}

void RexExporter::getVertexArray( const std::vector<aiVector3D> vector, std::vector<float>& vectorArray ) {
    vectorArray.resize(vector.size() * 3);

    for (auto i = 0; i < vector.size(); i++) {
        int index = i * 3;
        vectorArray[index] = vector[i].x;
        vectorArray[index + 1] = vector[i].z;
        vectorArray[index + 2] = -vector[i].y;
    }
}

void RexExporter::getColorArray( const std::vector<aiColor3D> vector, std::vector<float>& colorArray ) {
    colorArray.resize(vector.size() * 3);
    for (auto i = 0; i < vector.size(); i++) {
        int index = i * 3;
        colorArray[index] = vector[i].r;
        colorArray[index + 1] = vector[i].g;
        colorArray[index + 2] = vector[i].b;
    }
}

void RexExporter::getTriangleArray( const std::vector<Triangle>& triangles, std::vector<uint32_t>& triangleArray ) {
    triangleArray.resize(triangles.size() * 3);
    for(auto i = 0; i < triangles.size(); i++){
        Triangle t = triangles.at(i);
        triangleArray[3 * i] = t.indices[0].vp;
        triangleArray[3 * i + 1] = t.indices[1].vp;
        triangleArray[3 * i + 2] = t.indices[2].vp;
    }
}

// ------------------------------------------------------------------------------------------------
std::string RexExporter::GetMaterialName(unsigned int index) {
    const aiMaterial* const mat = m_Scene->mMaterials[index];
    if ( nullptr == mat ) {
        static const std::string EmptyStr;
        return EmptyStr;
    }

    aiString s;
    if(AI_SUCCESS == mat->Get(AI_MATKEY_NAME,s)) {
        return std::string(s.data,s.length);
    }

    char number[ sizeof(unsigned int) * 3 + 1 ];
    ASSIMP_itoa10(number,index);
    return "$Material_" + std::string(number);
}

void RexExporter::WriteMaterials(rex_header *header, std::vector<MaterialPtr> &materialPtrs) {
    materialPtrs.resize(m_Scene->mNumMaterials);

    printf("Found %d materials\n", m_Scene->mNumMaterials);
    for(unsigned int i = 0; i < m_Scene->mNumMaterials; ++i) {
        const aiMaterial* const mat = m_Scene->mMaterials[i];

        // write material
        rex_material_standard rexMat;
        rexMat.alpha = 1;
        rexMat.ns = 0;

        aiColor4D c;
        if(AI_SUCCESS == mat->Get(AI_MATKEY_COLOR_DIFFUSE,c)) {
            rexMat.kd_red = c.r;
            rexMat.kd_green = c.g;
            rexMat.kd_blue = c.b;
//            mat.kd_textureId = ?
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_COLOR_AMBIENT,c)) {
            rexMat.ka_red = c.r;
            rexMat.ka_green = c.g;
            rexMat.ka_blue = c.b;
//            mat.ka_textureId = ?
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_COLOR_SPECULAR,c)) {
            rexMat.ks_red = c.r;
            rexMat.ks_green = c.g;
            rexMat.ks_blue = c.b;
//            mat.ks_textureId = ?
        }

//        aiString s;
//        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_DIFFUSE(0),s)) {
//            mOutputMat << "map_Kd " << s.data << endl;
//        }
//        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_AMBIENT(0),s)) {
//            mOutputMat << "map_Ka " << s.data << endl;
//        }
//        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_SPECULAR(0),s)) {
//            mOutputMat << "map_Ks " << s.data << endl;
//        }
//        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_SHININESS(0),s)) {
//            mOutputMat << "map_Ns " << s.data << endl;
//        }
//        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_OPACITY(0),s)) {
//            mOutputMat << "map_d " << s.data << endl;
//        }
//        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_HEIGHT(0),s) || AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_NORMALS(0),s)) {
//            // implementations seem to vary here, so write both variants
//            mOutputMat << "bump " << s.data << endl;
//            mOutputMat << "map_bump " << s.data << endl;
//        }

        materialPtrs[i].material = rex_block_write_material (i /*id*/, header, &rexMat, &materialPtrs[i].size);
    }
}


// ------------------------------------------------------------------------------------------------
void RexExporter::AddMesh(const aiString& name, const aiMesh* m, const aiMatrix4x4& mat) {
    mMeshes.push_back(MeshInstance() );
    MeshInstance& mesh = mMeshes.back();

    mesh.name = std::string( name.data, name.length );
    mesh.matname = GetMaterialName(m->mMaterialIndex);  //??? needed?
    mesh.materialId = m->mMaterialIndex;
    mesh.triangles.resize(m->mNumFaces);

    for(unsigned int i = 0; i < m->mNumFaces; ++i) {
        const aiFace& f = m->mFaces[i];

        Triangle& triangle = mesh.triangles[i];
        triangle.indices.resize(3);

        for(unsigned int a = 0; a < 3; ++a) {
            const unsigned int idx = f.mIndices[a];

            aiVector3D vert = mat * m->mVertices[idx];

            if ( nullptr != m->mColors[ 0 ] ) {
                aiColor4D col4 = m->mColors[ 0 ][ idx ];
                triangle.indices[a].vp = mesh.verticesWithColors.getIndex({vert, aiColor3D(col4.r, col4.g, col4.b)});
            } else {
                triangle.indices[a].vp = mesh.verticesWithColors.getIndex({vert, aiColor3D(0,0,0)});
            }

            if ( m->mTextureCoords[ 0 ] ) {
                triangle.indices[a].vt = mesh.textureCoords.getIndex(m->mTextureCoords[0][idx]);
            } else {
                triangle.indices[a].vt = 0;
            }
        }
    }

}

// ------------------------------------------------------------------------------------------------
void RexExporter::AddNode(const aiNode* nd, const aiMatrix4x4& mParent) {
    const aiMatrix4x4& mAbs = mParent * nd->mTransformation;

    aiMesh *cm( nullptr );
    for(unsigned int i = 0; i < nd->mNumMeshes; ++i) {
        cm = m_Scene->mMeshes[nd->mMeshes[i]];
        if (nullptr != cm) {
            AddMesh(cm->mName, m_Scene->mMeshes[nd->mMeshes[i]], mAbs);
        } else {
            AddMesh(nd->mName, m_Scene->mMeshes[nd->mMeshes[i]], mAbs);
        }
    }

    for(unsigned int i = 0; i < nd->mNumChildren; ++i) {
        AddNode(nd->mChildren[i], mAbs);
    }
}

// ------------------------------------------------------------------------------------------------

#endif // ASSIMP_BUILD_NO_REX_EXPORTER
#endif // ASSIMP_BUILD_NO_EXPORT
