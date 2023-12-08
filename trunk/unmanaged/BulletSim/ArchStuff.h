/*
 * Copyright (c) Contributors, http://opensimulator.org/
 * See CONTRIBUTORS.TXT for a full list of copyright holders.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyrightD
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the OpenSimulator Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE DEVELOPERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#ifndef ARCH_STUFF_H
#define ARCH_STUFF_H

// 20230107 RA: This file exists to create definitions for 32 vs 64 bit
//    architectures. Since we're moving into the future, 32bit is old
//    and is no longer a supported machine architecture.
//    That said, C# floats are 32bits and, for reporting collisions and
//    other interactions, item IDs are presumed to be the same size
//    as a float. Therefore, IDTYPE must be 32bits.

// Define types that are always 32bits (don't change on 64 bit systems)
// 20230107 RA: the symbol _MSC_VER is only defined for Win32 and thus will
//    never be true. Test and code kept for historical reference.
#ifdef _MSC_VER
	typedef signed __int32		int32_t;
	typedef unsigned __int32	uint32_t;
#else
	#include <inttypes.h>
#endif

// See above discussion about IDTYPE needing to be same size as a float.
#define IDTYPE uint32_t

#ifdef __x86_32__
	#define PACKLOCALID(xx) ((void*)(xx))
	#define CONVLOCALID(xx) ((IDTYPE)(xx))
#else
    // CONVLOCALID is usually called operating on a void* which will
    //    be 64bit on most modern systems. 
    //    64bit systems don't allow you to cast directly from a void* to an
    //    unsigned int thus we cast it to uint64_t before chopping it to a
    //    IDTYPE (uint32_t).
	#define PACKLOCALID(xx) ((void*)((uint64_t)(xx)))
	#define CONVLOCALID(xx) ((IDTYPE)((uint64_t)(xx)))
#endif

// key used for identifying meshes and hulls
#define MESHKEYTYPE unsigned long long
// key used to identify collisions based on the IDs of the colliding objects
#define COLLIDERKEYTYPE unsigned long long

#endif    // ARCH_STUFF_H
