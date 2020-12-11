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

#ifndef ZGLOBAL_H
#define ZGLOBAL_H

/**
  * Detect compiler
  */
#if defined(__GNUC__)
#define ZFUNC_INFO __PRETTY_FUNCTION__
#elif defined(__clang__)
#define ZFUNC_INFO __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
#define ZFUNC_INFO __FUNCDNAME__
#else
#endif


#if defined(__cplusplus) && (__cplusplus < 201103L) && !defined(_MSC_VER)
#    error C++11 compiler or newer is required fro this project.
#endif

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
   #define Z_OS_WINDOWS
   #define ZATTR_FORMAT_PRINTF(A, B)
   #ifdef _WIN64
      #define Z_OS_WINDOWS_64
   #else
      #define Z_OS_WINDOWS_32
   #endif
#elif __APPLE__
    #define Z_OS_APPLE
    #include <TargetConditionals.h>
    #if TARGET_IPHONE_SIMULATOR
         #define Z_OS_IPHONE_SIM
    #elif TARGET_OS_IPHONE
        #define Z_OS_IPHONE
    #elif TARGET_OS_MAC
        #define Z_OS_MAC
    #else
    #   error "Unknown Apple platform"
    #endif
#elif __linux__
    #define Z_OS_LINUX

    #define ZATTR_FORMAT_PRINTF(A, B) \
         __attribute__((format(printf, (A), (B))))

#elif __unix__ // all unices not caught above
    #define Z_OS_UNIX
    #define ZATTR_FORMAT_PRINTF(A, B)
#elif defined(_POSIX_VERSION)
    #define Z_OS_POSIX
    #define ZATTR_FORMAT_PRINTF(A, B)
#else
#   error "Unknown compiler"
#endif

#define ZUNUSED(x) (void)x;

#ifdef Z_OS_WINDOWS
#    define ZDECL_EXPORT     __declspec(dllexport)
#    define ZDECL_IMPORT     __declspec(dllimport)
#elif defined(Z_OS_LINUX)
#    define ZDECL_EXPORT     __attribute__((visibility("default")))
#    define ZDECL_IMPORT     __attribute__((visibility("default")))
#    define ZDECL_HIDDEN     __attribute__((visibility("hidden")))
#endif

#endif /* ZGLOBAL_H */
