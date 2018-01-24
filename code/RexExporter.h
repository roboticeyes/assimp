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
#include <map>

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

    private:
        void WriteHeader();
        void WriteCoordinateSystemBlock();

    private:
        const aiScene *const m_Scene;
        std::shared_ptr<FileWrapper> m_File;
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
