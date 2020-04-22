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

/** @file RexExporter.h
 * Declares the exporter class to write a scene to a the REX file
 */
#ifndef AI_REXEXPORTER_H_INC
#define AI_REXEXPORTER_H_INC

#include <assimp/types.h>
#include <sstream>
#include <vector>
#include <memory>
#include <map>
#include <sstream>
#include <libopenrex/rex.h>

#ifdef _WIN32
#define fopen_rex( file, filename, mode ) \
auto fopenerror = fopen_s( &file, filename, mode ); \
if (fopenerror) file = 0;
#else
#define fopen_rex( file, filename, mode ) \
file = fopen ( filename, mode )
#endif


struct aiScene;
struct aiNode;
struct aiMesh;

namespace rex
{
    class FileWrapper;
    struct RexData;

    class RexExporter
    {
    public:
        /// Constructor for a specific scene to export
        RexExporter (const char *fileName, const aiScene *pScene);
        ~RexExporter();

        void Start();
        std::ostringstream mOutput;

    private:
        struct TriangleVertex {
            TriangleVertex()
            : vp()
            , vn()
            , vt()
            , vc() {
                // empty
            }

            // one-based, 0 means: 'does not exist'
            unsigned int vp, vn, vt, vc;
        };

        struct Triangle {
            std::vector<TriangleVertex> indices;
        };


        void WriteGeometryFile();
        void WriteHeader(std::ostringstream &out);
        void WriteCoordinateSystemBlock();
        void WriteAllDataBlocks();
        void WriteDataBlock();
        void WriteDataHeaderBlock(uint16_t type, uint16_t version, uint32_t dataBlockSize, uint64_t dataId);
        void WriteDataBlockMesh();

        void AddMesh(const aiString& name, const aiMesh* m, const aiMatrix4x4& mat);
        void AddNode(const aiNode* nd, const aiMatrix4x4& mParent);

        void getTriangleArray(const std::vector<Triangle>& triangles, std::vector<uint32_t> &triangleArray );

    private:
        const aiScene *const m_Scene;
        std::shared_ptr<FileWrapper> m_File;

    public:
        struct aiColorCompare {
            bool operator() ( const aiColor3D& a, const aiColor3D& b ) const {
                // color
                if (a.r < b.r) return true;
                if (a.r > b.r) return false;
                if (a.g < b.g) return true;
                if (a.g > b.g) return false;
                if (a.b < b.b) return true;
                if (a.b > b.b) return false;
                return false;
            }
        };

        struct aiVectorCompare {
            bool operator() (const aiVector3D& a, const aiVector3D& b) const {
                if(a.x < b.x) return true;
                if(a.x > b.x) return false;
                if(a.y < b.y) return true;
                if(a.y > b.y) return false;
                if(a.z < b.z) return true;
                return false;
            }
        };

        template <class T, class Compare = std::less<T>>
        class indexMap {
            int mNextIndex;
            typedef std::map<T, int, Compare> dataType;
            dataType vecMap;

        public:
            indexMap()
            : mNextIndex(1) {
                // empty
            }

            int getIndex(const T& key) {
                typename dataType::iterator vertIt = vecMap.find(key);
                // vertex already exists, so reference it
                if(vertIt != vecMap.end()){
                    return vertIt->second;
                }
                return vecMap[key] = mNextIndex++;
            };

            void getKeys( std::vector<T>& keys ) {
                keys.resize(vecMap.size());
                for(typename dataType::iterator it = vecMap.begin(); it != vecMap.end(); ++it){
                    keys[it->second-1] = it->first;
                }
            };

            void getKeysAsFloat( std::vector<float>& keys ) {
//                keys.resize(vecMap.size() * 3);
                for(typename dataType::iterator it = vecMap.begin(); it != vecMap.end(); ++it){
                    int index = (it->second-1) * 3;
                    keys[index] = it->first.x;
                    keys[index + 1] = it->first.y;
                    keys[index + 2] = it->first.z;
                }
            };

            size_t size() const {
                return vecMap.size();
            }
        };

//        indexMap<aiVector3D, aiVectorCompare> mVnMap, mVtMap, mVpMap;
//        indexMap<aiColor3D, aiColorCompare> mVcMap;
//        indexMap<vertexData, vertexDataCompare> mVpMap;
        struct MeshInstance {
            std::string name, matname;
            std::vector<Triangle> triangles;
            indexMap<aiVector3D, aiVectorCompare> vertices, normals, textureCoords;
            indexMap<aiColor3D, aiColorCompare> colors;
        };

        std::vector<MeshInstance> mMeshes;

        // this endl() doesn't flush() the stream
        const std::string endl;
    };

    /***
     * This is the main datastructure for the REX file format
     */
    struct RexData
    {
        // todo
    };

    /**
     * This file wrapper offers RAII (resource acquisition is initialization) for the
     * raw C FILE pointer. Whenever the FILE* is used, this wrapper ensures that the resource
     * is closed upon destruction. See Stroustrup 4th edition, page 356.
     */
    class FileWrapper
    {
    public:
        FileWrapper (const char *n, const char *a)
        {
            fopen_rex (m_file, n, a);
            if (m_file == nullptr)
            {
                throw std::runtime_error ("FileWrapper::FileWrapper: cannot open file");
            }
        }

        FileWrapper (const std::string &n, const char *a) :
            FileWrapper (n.c_str(), a)
        {
        }

        /**
         * This ctor assumes ownership of the file pointer
         */
        explicit FileWrapper (FILE *f) :
            m_file (f)
        {
            if (m_file == nullptr)
            {
                throw std::runtime_error ("FileWrapper::FileWrapper: file pointer is null");
            }
        }

        ~FileWrapper()
        {
            ::fclose (m_file);
        }

        operator FILE *()
        {
            return m_file;
        }

        FILE *ptr()
        {
            return m_file;
        }

        size_t write (const void *ptr, size_t size, size_t nitems, const std::string &debugInfo = "")
        {
            size_t ret = ::fwrite (ptr, size, nitems, m_file);
            if (ret != nitems)
            {
                throw std::runtime_error ("FileWrapper::fwrite: error writing file");
            }
            return  ret;
        }

        size_t read (void *ptr, size_t size, size_t nitems, const std::string &debugInfo = "")
        {
            size_t ret = ::fread (ptr, size, nitems, m_file);
            if (ret != nitems)
            {
                throw std::runtime_error ("FileWrapper::fread: error reading file");
            }
            return  ret;
        }

    private:
        FILE *m_file;
    };

    /*!
     * Narrow cast proposed by Stroustrup's C++ bible (pg. 299, 11.5)
     */
    template<typename Target, typename Source>
    Target narrow_cast (Source v)
    {
        auto r = static_cast<Target> (v);
        if (static_cast<Source> (r) != v)
        {
            throw std::runtime_error ("narrow_cast<>() failed");
        }
        return r;
    };

}

#endif
