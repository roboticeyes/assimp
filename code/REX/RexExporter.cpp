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



//    WriteHeader();
//    WriteCoordinateSystemBlock();

//    WriteAllDataBlocks();
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

    std::vector<rex_mesh> meshes;
//    WriteHeader(mOutput);
//    if (!noMtl)
//        mOutput << "mtllib "  << GetMaterialLibName() << endl << endl;

    // collect mesh geometry
    aiMatrix4x4 mBase;
    AddNode(m_Scene->mRootNode, mBase);


    printf("Found %d meshes\n", (int)mMeshes.size());
    // now write all mesh instances
    for(MeshInstance& m : mMeshes) {
        rex_mesh rexMesh;
        rexMesh.lod = 0; //??
        rexMesh.max_lod = 0; //??
        sprintf(rexMesh.name, "%s", m.name.c_str()); //0-terminated?
        rexMesh.nr_triangles = m.triangles.size();
        rexMesh.nr_vertices = m.vertices.size();
        rexMesh.material_id = 0;//

        // vertices
        std::vector<float> vertices;
        m.vertices.getKeysAsFloat(vertices);
        rexMesh.positions = &vertices[0];

        // normals
        std::vector<float> normals;
        m.normals.getKeysAsFloat(normals);
        rexMesh.normals = &normals[0];

        // texture coords
        std::vector<float> textureCoords;
        m.textureCoords.getKeysAsFloat(textureCoords);
        rexMesh.tex_coords = &textureCoords[0];

        // material TODO
        rexMesh.material_id = 0x7fffffffffffffffL;
        meshes.push_back(rexMesh);
    }

    struct rex_header *header = rex_header_create();

    //TEST write only first mesh !!!!
    long mesh_sz;
    uint8_t *mesh_ptr = rex_block_write_mesh (0 /*id*/, header, &meshes[0], &mesh_sz);

    long header_sz;
    uint8_t *header_ptr = rex_header_write (header, &header_sz);

    printf("header mesh data blocks after header write %d\n", header->nr_datablocks);

    ::fseek (m_File->ptr(), 0, SEEK_SET);
     m_File->write (&header_ptr, header_sz, 1, "writeHeader");

     m_File->write (&mesh_ptr, mesh_sz, 1, "writeMesh");

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

            printf("colors\n");
            if ( nullptr != m->mColors[ 0 ] ) {
                aiColor4D col4 = m->mColors[ 0 ][ idx ];
                triangle.indices[a].vp = mesh.vertices.getIndex(vert);//{vert, aiColor3D(col4.r, col4.g, col4.b)});
                triangle.indices[a].vc = mesh.colors.getIndex(aiColor3D(col4.r, col4.g, col4.b));
            } else {
                printf("colors no color %d\n",(int)triangle.indices.size());
                triangle.indices[a].vp = mesh.vertices.getIndex(vert);//{vert, aiColor3D(0,0,0)});
                triangle.indices[a].vc = mesh.colors.getIndex(aiColor3D(0,0,0));
            }

            printf("normals\n");
            if (m->mNormals) {
                aiVector3D norm = aiMatrix3x3(mat) * m->mNormals[idx];
                triangle.indices[a].vn = mesh.normals.getIndex(norm);
            } else {
                triangle.indices[a].vn = 0;
            }

            printf("texture coords\n");
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
