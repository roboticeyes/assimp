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

    WriteHeader();
    WriteCoordinateSystemBlock();
}

void RexExporter::WriteHeader()
{
    auto debugInfo = "FileHeader";
    char reserved[RESERVED_SIZE];
    uint32_t crc32 = 0;
    uint32_t numberOfDataBlocks = 0;
    uint64_t sizeOfDataBlocks = 0;

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


// ------------------------------------------------------------------------------------------------

#endif // ASSIMP_BUILD_NO_REX_EXPORTER
#endif // ASSIMP_BUILD_NO_EXPORT
