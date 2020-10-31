/*
 *             Copyright 2020 by Morgan
 *
 * This software BSD-new. See the included COPYING file for details.
 *
 * License: BSD-new
 * ==============================================================================
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the \<organization\> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ZRING_H
#define ZRING_H

#include "zglobal.h"
#include <assert.h>
#include <algorithm>

template<class T>
class ZRing {
public:
    ZRing(unsigned int _size)
    : size(_size + 1), ring(new T[size]), read_pos(0), write_pos(0)
    {
        assert(size >= 1);
    }

    ~ZRing() {
        delete[] ring;
    }

    T* writePtr() {
        return &ring[write_pos];
    }

    void write() {
        write_pos = (write_pos + 1) % size;
        if ( write_pos == read_pos ) {
            /* Remove first element */
            read_pos = (read_pos + 1) % size;
        }
    }

    void write(T o) {
        ring[write_pos] = o;
        write_pos = (write_pos + 1) % size;
        if ( write_pos == read_pos ) {
            /* Remove first element */
            read_pos = (read_pos + 1) % size;
        }
    }

    void write(const T* data, unsigned int _size) {
        unsigned int offset = 0;
        unsigned int _available = available();
        while ( offset < _size ) {
            uint chunk = size - write_pos;
            chunk = std::min(chunk, _size - offset);
            memcpy(ring + write_pos, data + offset, sizeof(T) * chunk);
            write_pos = (write_pos + chunk) % size;
            offset += chunk;
        }
        if ( _size >= _available ) {
            read_pos = (write_pos + 1) % size;
        }
    }

    T first() {
        if ( read_pos == write_pos ) return T();
        return ring[read_pos];
    }

    T peek(int position) {
        return ring[(read_pos + position) % size];
    }

    T read() {
        if ( read_pos == write_pos ) return T();
        int i = read_pos;
        read_pos = (read_pos + 1) % size;
        T r = ring[i];
        ring[i] = T();
        return r;
    }

    unsigned int read(T* data, unsigned int _size) {
        unsigned int offset = 0;
        unsigned int available = count();
        _size = std::min(_size, available);
        while ( offset < _size ) {
            uint chunk = size - read_pos;
            chunk = std::min(chunk, _size - offset);
            memcpy(data + offset, ring + read_pos, sizeof(T) * chunk);
            read_pos = (read_pos + chunk) % size;
            offset += chunk;
        }
        return _size;
    }

    bool isEmpty() const {
        return (read_pos == write_pos);
    }

    uint count() const {
        if ( write_pos >= read_pos ) {
            return write_pos - read_pos;
        }
        else {
            return size - (read_pos - write_pos);
        }
    }

    uint available() const {
        return size - count() - 1;
    }

    uint bufferSize() const {
        return size;
    }

    void setNewBufferSize(int new_buffer_size) {
        delete[] ring;
        size = new_buffer_size + 1;
        ring = new T[size];
        read_pos = write_pos = 0;
    }

    void clear()
    {
        for ( uint i = 0 ; i < size; ++i ) {
            ring[i] = T();
        }
        read_pos = write_pos = 0;
    }
private:
    uint size;
    T* ring;
    int read_pos;
    int write_pos;
};

#endif /* ZRING_H */
