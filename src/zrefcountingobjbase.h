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

#ifndef ZREFCOUNTINGOBJBASE_H_
#define ZREFCOUNTINGOBJBASE_H_

#include "zglobal.h"
#include <atomic>

class ZRefCountingObjBase
{
public:
    explicit ZRefCountingObjBase()
    : ref_count(1), floating(1)
    {
        /* no op */
    }

    void ref()
    {
        sink();
        ref_count.fetch_add();
    }

    void sinkRef()
    {
        if ( isFloating() ) {
            sink();
        }
        else {
            ref();
        }
    }

    void unref()
    {
        int non_zero = ref_count.fetch_sub();
        if ( !non_zero ){
            delete this;
        }
    }

    void sink()
    {
        int old_value;
        do {
            old_value = floating.load();
        } while ( !floating.compare_exchange_weak(old_value, 0,
                                                  std::memory_order_release,
                                                  std::memory_order_relaxed));
    }

    bool isFloating()
    {
        return (floating.load() == 1);
    }

    int refCount()
    {
        return ref_count.load();
    }

protected:
    virtual ~ZRefCountingObjBase();

private:
    std::atomic<int> ref_count;
    std::atomic<int> floating;
};

template<class T>
class ZRef {
private:
    T* object_ref;

public:
    ZRef(): object_ref(0)
    {
        /* no op */
    }

    ZRef(const ZRef<T>& _copy) : object_ref(0)
    {
        if ( 0 != _copy.object_ref )
            _copy.object_ref->ref();

        object_ref = _copy.object_ref;
    }

    ZRef(T* _object_ptr) : object_ref(_object_ptr)
    {
        if ( 0 != object_ref )
            object_ref->sinkRef();
    }

    ~ZRef()
    {
        if ( object_ref != 0 ) object_ref->unref();
    }

    void take(T* _object)
    {
        if ( object_ref != 0 ) object_ref->unref();
        object_ref = _object;
    }

    T* get() const
    {
        return object_ref;
    }

    T* operator->()
    {
        return get();
    }

    const T* operator->() const
    {
        return get();
    }


    operator T*() const {
        return get();
    }

    T* operator&() const {
        return get();
    }

    ZRef<T>& operator=(const ZRef<T>& _copy)
    {
        if ( object_ref != 0 ) object_ref->unref();

        this->object_ref = _copy.object_ref;

        if ( object_ref != 0 ) object_ref->ref();

        return *this;
    }

    ZRef<T>& operator=(T* object_ptr)
    {
        if ( object_ref != 0 ) object_ref->unref();

        this->object_ref = object_ptr;

        if ( object_ref != 0 ) object_ref->sinkRef();

        return *this;
    }

    template<class C>
    ZRef<C> cast() {
        return ZRef<C>(static_cast<C*>(object_ref));
    }
};

// template<class T>
// inline uint qHash(VxReference<T> object) {
//     return reinterpret_cast<qintptr>(object.get()) & 0xffffffffu;
// }

#endif /* ZREFCOUNTINGOBJBASE_H_ */
