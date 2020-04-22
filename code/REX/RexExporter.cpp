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

void RexExporter::WriteHeader(std::ostringstream& out)
{
    auto debugInfo = "FileHeader";
    char reserved[RESERVED_SIZE];
    uint32_t crc32 = 0;              //TODO
    uint32_t numberOfDataBlocks = 0; //TODO
    uint64_t sizeOfDataBlocks = 0;   //TODO

    ::fseek (m_File->ptr(), 0, SEEK_SET);

    m_File->write ("REX1", sizeof (char), MAGIC_SIZE, debugInfo);
    m_File->write (&VERSION, sizeof (uint16_t), 1, debugInfo);
    m_File->write (&crc32, sizeof (uint32_t), 1, debugInfo);
    m_File->write (&numberOfDataBlocks, sizeof (uint16_t), 1, debugInfo);
    m_File->write (&START_DATA_BLOCK, sizeof (uint16_t), 1, debugInfo);
    m_File->write (&sizeOfDataBlocks, sizeof (uint64_t), 1, debugInfo);
    m_File->write (reserved, sizeof (char), RESERVED_SIZE, debugInfo);
}

void RexExporter::WriteCoordinateSystemBlock()
{
    auto debugInfo = "CoordinateSystem";
    float offset = 0.0f;

    uint16_t sz = narrow_cast<uint16_t> (AUTH_NAME_SIZE);
    m_File->write (&SRID, sizeof (uint32_t), 1, debugInfo);
    m_File->write (&sz, sizeof (uint16_t), 1, debugInfo);
    m_File->write (&AUTH_NAME, sz, 1, debugInfo);
    m_File->write (&offset, sizeof (float), 1, debugInfo);
    m_File->write (&offset, sizeof (float), 1, debugInfo);
    m_File->write (&offset, sizeof (float), 1, debugInfo);
}

void RexExporter::WriteAllDataBlocks()
{

}

void RexExporter::WriteDataBlock()
{
//    WriteDataHeaderBlock();
}

void RexExporter::WriteDataHeaderBlock(uint16_t type, uint16_t version, uint32_t dataBlockSize, uint64_t dataId)
{
    auto debugInfo = "DataHeaderBlock";

    m_File->write (&type, sizeof (uint16_t), 1, debugInfo);
    m_File->write (&version, sizeof (uint16_t), 1, debugInfo);
    m_File->write (&dataBlockSize, sizeof (uint32_t), 1, debugInfo);
    m_File->write (&dataId, sizeof (uint64_t), 1, debugInfo);
}


void RexExporter::WriteGeometryFile() {
    printf("Write geometry file\n");

    struct MeshPtr{
        uint8_t* mesh;
        long size;
    };

    std::vector<MeshPtr> meshPtrs;

    // collect mesh geometry
    aiMatrix4x4 mBase;
    AddNode(m_Scene->mRootNode, mBase);


    printf("Found %d meshes\n", (int)mMeshes.size());

    struct rex_header *header = rex_header_create();

    // write material block

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
        rexMesh.nr_vertices = m.vertices.size();

        // vertices
        std::vector<float> vertices;
        vertices.resize(rexMesh.nr_vertices * 3);
        m.vertices.getKeysAsFloatRex(vertices);
        rexMesh.positions = &vertices[0];

        // normals werden selber gerechnet

        // texture coords
        std::vector<float> textureCoords;
        textureCoords.resize(rexMesh.nr_vertices * 2);
        m.textureCoords.getTextureCoordsAsFloat(textureCoords);
        rexMesh.tex_coords = &textureCoords[0];

        // colors
        std::vector<float> colors;
        colors.resize(rexMesh.nr_vertices * 3);
        m.colors.getColorsAsFloat(colors);
        rexMesh.colors = &colors[0];

        // triangles
        std::vector<uint32_t> triangles;
        getTriangleArray(m.triangles, triangles);
        rexMesh.triangles = &triangles[0];

        // material TODO
        rexMesh.material_id = 0x7fffffffffffffffL;

        meshPtrs[i].mesh = rex_block_write_mesh (i /*id*/, header, &rexMesh, &meshPtrs[i].size);
        i++;
    }

    long header_sz;
    uint8_t *header_ptr = rex_header_write (header, &header_sz);

    ::fseek (m_File->ptr(), 0, SEEK_SET);
     m_File->write (header_ptr, header_sz, 1, "writeHeader");

     for (MeshPtr m : meshPtrs) {
         m_File->write (m.mesh, m.size, 1, "writeMesh");
     }
}

void RexExporter::getTriangleArray( const std::vector<Triangle>& triangles, std::vector<uint32_t>& triangleArray ) {
    triangleArray.resize(triangles.size() * 3);
    for(auto i = 0; i < triangles.size(); i++){
        Triangle t = triangles.at(i);
        triangleArray[3 * i] = t.indices[0].vp;
        triangleArray[3 * i + 1] = t.indices[1].vp;
        triangleArray[3 * i + 2] = t.indices[2].vp;
        printf("Triangle %d, A: %d, B: %d, C: %d\n", i, triangleArray[3 * i], triangleArray[3 * i + 1], triangleArray[3 * i + 2]);
    }
}

void RexExporter::WriteMaterialBlock() {
    for(unsigned int i = 0; i < pScene->mNumMaterials; ++i) {
        const aiMaterial* const mat = pScene->mMaterials[i];

        int illum = 1;
        mOutputMat << "newmtl " << GetMaterialName(i)  << endl;

        aiColor4D c;
        if(AI_SUCCESS == mat->Get(AI_MATKEY_COLOR_DIFFUSE,c)) {
            mOutputMat << "Kd " << c.r << " " << c.g << " " << c.b << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_COLOR_AMBIENT,c)) {
            mOutputMat << "Ka " << c.r << " " << c.g << " " << c.b << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_COLOR_SPECULAR,c)) {
            mOutputMat << "Ks " << c.r << " " << c.g << " " << c.b << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_COLOR_EMISSIVE,c)) {
            mOutputMat << "Ke " << c.r << " " << c.g << " " << c.b << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_COLOR_TRANSPARENT,c)) {
            mOutputMat << "Tf " << c.r << " " << c.g << " " << c.b << endl;
        }

        ai_real o;
        if(AI_SUCCESS == mat->Get(AI_MATKEY_OPACITY,o)) {
            mOutputMat << "d " << o << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_REFRACTI,o)) {
            mOutputMat << "Ni " << o << endl;
        }

        if(AI_SUCCESS == mat->Get(AI_MATKEY_SHININESS,o) && o) {
            mOutputMat << "Ns " << o << endl;
            illum = 2;
        }

        mOutputMat << "illum " << illum << endl;

        aiString s;
        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_DIFFUSE(0),s)) {
            mOutputMat << "map_Kd " << s.data << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_AMBIENT(0),s)) {
            mOutputMat << "map_Ka " << s.data << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_SPECULAR(0),s)) {
            mOutputMat << "map_Ks " << s.data << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_SHININESS(0),s)) {
            mOutputMat << "map_Ns " << s.data << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_OPACITY(0),s)) {
            mOutputMat << "map_d " << s.data << endl;
        }
        if(AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_HEIGHT(0),s) || AI_SUCCESS == mat->Get(AI_MATKEY_TEXTURE_NORMALS(0),s)) {
            // implementations seem to vary here, so write both variants
            mOutputMat << "bump " << s.data << endl;
            mOutputMat << "map_bump " << s.data << endl;
        }

        mOutputMat << endl;
    }




    // write material
    struct rex_material_standard mat =
    {
        .ka_red = 0,
        .ka_green = 0,
        .ka_blue = 0,
        .ka_textureId = 0,
        .kd_red = 1,
        .kd_green = 0,
        .kd_blue = 0,
        .kd_textureId = 0,
        .ks_red = 0,
        .ks_green = 0,
        .ks_blue = 0,
        .ks_textureId = 0,
        .ns = 0,
        .alpha = 1
    };
    long mat_sz;
    uint8_t *mat_ptr = rex_block_write_material (1 /*id*/, header, &mat, &mat_sz);
}


// ------------------------------------------------------------------------------------------------
void RexExporter::AddMesh(const aiString& name, const aiMesh* m, const aiMatrix4x4& mat) {
    mMeshes.push_back(MeshInstance() );
    MeshInstance& mesh = mMeshes.back();

    mesh.name = std::string( name.data, name.length );
    printf("add mesh %s\n", mesh.name.c_str());

    mesh.triangles.resize(m->mNumFaces);
     printf("add mesh num faces %d\n", m->mNumFaces);

    for(unsigned int i = 0; i < m->mNumFaces; ++i) {
        printf("add mesh i %d\n", i);
        const aiFace& f = m->mFaces[i];

        Triangle& triangle = mesh.triangles[i];
        triangle.indices.resize(3);

        for(unsigned int a = 0; a < 3; ++a) {
            const unsigned int idx = f.mIndices[a];

            aiVector3D vert = mat * m->mVertices[idx];
            if ( nullptr != m->mColors[ 0 ] ) {
                printf("Colors found\n");
                aiColor4D col4 = m->mColors[ 0 ][ idx ];
                triangle.indices[a].vp = mesh.vertices.getIndex(vert);//{vert, aiColor3D(col4.r, col4.g, col4.b)});
                triangle.indices[a].vc = mesh.colors.getIndex(aiColor3D(col4.r, col4.g, col4.b));
            } else {
                triangle.indices[a].vp = mesh.vertices.getIndex(vert);//{vert, aiColor3D(0,0,0)});
                triangle.indices[a].vc = mesh.colors.getIndex(aiColor3D(0,0,0));
            }

            if ( m->mTextureCoords[ 0 ] ) {
                triangle.indices[a].vt = mesh.textureCoords.getIndex(m->mTextureCoords[0][idx]);
            } else {
                triangle.indices[a].vt = 0;
            }
        }
    }
    printf("VERTICES\n");
    std::vector<aiVector3D> vertices;
    mesh.vertices.getKeys(vertices);
    for (int i = 0; i < vertices.size(); i++) {
        auto vertex = vertices[i];
        printf(" X: %f, Y: %f, Z: %f\n", vertex.x, vertex.y, vertex.z);
    }

    printf("COLORS\n");
//    std::vector<aiColor3D> colors;
//    mesh.colors.getKeys(colors);
//    for (int i = 0; i < colors.size(); i++) {
//        auto color = colors[i];
//        printf(" X: %f, Y: %f, Z: %f\n", color.r, color.g, color.b);
//    }
}

// ------------------------------------------------------------------------------------------------
void RexExporter::AddNode(const aiNode* nd, const aiMatrix4x4& mParent) {
    const aiMatrix4x4& mAbs = mParent * nd->mTransformation;

    printf("add node\n");

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
