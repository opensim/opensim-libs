/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

// TriMesh code by Erwin de Vries.
// Modified for FreeSOLID Compatibility by Rodrigo Hernandez
// Trimesh caches separation by Oleh Derevenko


#ifndef _ODE_COLLISION_TRIMESH_INTERNAL_H_
#define _ODE_COLLISION_TRIMESH_INTERNAL_H_

//****************************************************************************
// dxTriMesh class


#include "collision_kernel.h"
#include "collision_trimesh_colliders.h"
#include <ode/collision_trimesh.h>

#define BAN_OPCODE_AUTOLINK
#include "Opcode.h"
using namespace Opcode;

#if dTLS_ENABLED
#include "odetls.h"
#endif

// New trimesh collider hash table types
enum
{
    MAXCONTACT_X_NODE = 4,
    CONTACTS_HASHSIZE = 256
};

struct CONTACT_KEY
{
    dContactGeom * m_contact;
    unsigned int m_key;
};

struct CONTACT_KEY_HASH_NODE
{
    CONTACT_KEY m_keyarray[MAXCONTACT_X_NODE];
    int m_keycount;
};

struct CONTACT_KEY_HASH_TABLE
{
public:
    CONTACT_KEY_HASH_NODE &operator[](unsigned int index) { return m_storage[index]; }

private:
    CONTACT_KEY_HASH_NODE m_storage[CONTACTS_HASHSIZE];
};

struct VertexUseCache
{
public:
    VertexUseCache(): m_VertexUseBits(NULL), m_VertexUseElements(0) {}
    ~VertexUseCache() { FreeVertexUSEDFlags();  }

    bool ResizeAndResetVertexUSEDFlags(unsigned VertexCount)
    {
        bool Result = false;
        size_t VertexNewElements = (VertexCount + 7) / 8;
        if (VertexNewElements <= m_VertexUseElements || ReallocVertexUSEDFlags(VertexNewElements)) {
            memset(m_VertexUseBits, 0, VertexNewElements);
            Result = true;
        }
        return Result;
    }

    bool GetVertexUSEDFlag(unsigned VertexIndex) const { return (m_VertexUseBits[VertexIndex / 8] & (1 << (VertexIndex % 8))) != 0; }
    void SetVertexUSEDFlag(unsigned VertexIndex) { m_VertexUseBits[VertexIndex / 8] |= (1 << (VertexIndex % 8)); }

private:
    bool ReallocVertexUSEDFlags(size_t VertexNewElements)
    {
        bool Result = false;
        uint8 *VertexNewBits = (uint8 *)dRealloc(m_VertexUseBits, m_VertexUseElements * sizeof(m_VertexUseBits[0]), VertexNewElements * sizeof(m_VertexUseBits[0]));
        if (VertexNewBits) {
            m_VertexUseBits = VertexNewBits;
            m_VertexUseElements = VertexNewElements;
            Result = true;
        }
        return Result;
    }

    void FreeVertexUSEDFlags()
    {
        dFree(m_VertexUseBits, m_VertexUseElements * sizeof(m_VertexUseBits[0]));
        m_VertexUseBits = NULL;
        m_VertexUseElements = 0;
    }

private:
    uint8 *m_VertexUseBits;
    size_t m_VertexUseElements;
};


struct TrimeshCollidersCache
{
    TrimeshCollidersCache()
    {
        InitOPCODECaches();
    }

    void InitOPCODECaches();


    // Collider caches
    BVTCache ColCache;

    CONTACT_KEY_HASH_TABLE _hashcontactset;

    // Colliders
    /* -- not used -- also uncomment in InitOPCODECaches()
    PlanesCollider _PlanesCollider; -- not used 
    */
    SphereCollider _SphereCollider;
    OBBCollider _OBBCollider;
    RayCollider _RayCollider;
    AABBTreeCollider _AABBTreeCollider;
    /* -- not used -- also uncomment in InitOPCODECaches()
    LSSCollider _LSSCollider;
    */
    // Trimesh caches
    CollisionFaces Faces;
    SphereCache defaultSphereCache;
    OBBCache defaultBoxCache;
    LSSCache defaultCapsuleCache;

    // Trimesh-plane collision vertex use cache
    VertexUseCache VertexUses;
};

#if dTLS_ENABLED

inline TrimeshCollidersCache *GetTrimeshCollidersCache(unsigned uiTLSKind)
{
    return COdeTls::GetTrimeshCollidersCache((EODETLSKIND)uiTLSKind);
}


#else // dTLS_ENABLED

inline TrimeshCollidersCache *GetTrimeshCollidersCache(unsigned uiTLSKind)
{
    (void)uiTLSKind; // unused

    extern TrimeshCollidersCache g_ccTrimeshCollidersCache;

    return &g_ccTrimeshCollidersCache;
}

#endif // dTLS_ENABLED

struct dxTriMeshData  : public dBase 
{
    /* Array of flags for which edges and verts should be used on each triangle */
    enum UseFlags
    {
        kEdge0 = 0x1,
        kEdge1 = 0x2,
        kEdge2 = 0x4,
        kVert0 = 0x8,
        kVert1 = 0x10,
        kVert2 = 0x20,
        kUseAll = 0x3F,
        kvert0or1 = kVert0 | kVert1,
        kvert1or2 = kVert1 | kVert2,
        kvert0or2 = kVert0 | kVert2
    };

    enum MeshFlags
    {
        convex = 0x01,
        closedSurface = 0x02
    };

    /* Setup the UseFlags array */
    void Preprocess();
    /* For when app changes the vertices */
    void UpdateData();

    Model BVTree;
    MeshInterface Mesh;

    uint8 meshFlags;

    dxTriMeshData();
    ~dxTriMeshData();

    void Build(const void* Vertices, int VertexCount,
        const void* Indices, int IndexCount);

    /* aabb in model space */
    dVector3 AABBCenter;
    dVector3 AABBExtents;

    // data for use in collision resolution
    //const void* Normals;
    uint8* UseFlags;
};

struct dxTriMesh : public dxGeom{
    // Callbacks
    dTriCallback* Callback;
    dTriArrayCallback* ArrayCallback;
    dTriRayCallback* RayCallback;
    dTriTriMergeCallback* TriMergeCallback;

    // Data types
    dxTriMeshData* Data;

    bool doSphereTC;
    bool doBoxTC;
    bool doCapsuleTC;

    // Functions
    dxTriMesh(dSpaceID Space, dTriMeshDataID Data);
    ~dxTriMesh();

    void ClearTCCache();

    bool controlGeometry(int controlClass, int controlCode, void *dataValue, int *dataSize);

    void computeAABB();

    enum {
        MERGE_NORMALS__SPHERE_DEFAULT = DONT_MERGE_CONTACTS
    };
    bool controlGeometry_SetMergeSphereContacts(int dataValue);
    bool controlGeometry_GetMergeSphereContacts(int &returnValue);

    // Contact merging option
    dxContactMergeOptions SphereContactsMergeOption;
    // Instance data for last transform.
    dMatrix4 last_trans;

    // Some constants
    // Temporal coherence
    struct SphereTC : public SphereCache{
        dxGeom* Geom;
    };
    dArray<SphereTC> SphereTCCache;

    struct BoxTC : public OBBCache{
        dxGeom* Geom;
    };
    dArray<BoxTC> BoxTCCache;

    struct CapsuleTC : public LSSCache{
        dxGeom* Geom;
    };
    dArray<CapsuleTC> CapsuleTCCache;

};

#if 0
#include "collision_kernel.h"
// Fetches a contact
inline dContactGeom* SAFECONTACT(int Flags, dContactGeom* Contacts, int Index, int Stride){
    dIASSERT(Index >= 0 && Index < (Flags & NUMC_MASK));
    return ((dContactGeom*)(((char*)Contacts) + (Index * Stride)));
}
#endif

inline unsigned FetchTriangleCount(dxTriMesh* TriMesh)
{
    return TriMesh->Data->Mesh.GetNbTriangles();
}

inline void FetchTriangle(dxTriMesh* TriMesh, int Index, const dVector3 Position, const dMatrix3 Rotation, dVector3 Out[3])
{
    VertexPointers VP;
    TriMesh->Data->Mesh.GetTriangle(VP, Index);

    dVector3 invecs[3];
    for (int i = 0; i < 3; i++)
    {
        invecs[i][0] = VP.Vertex[i]->x;
        invecs[i][1] = VP.Vertex[i]->y;
        invecs[i][2] = VP.Vertex[i]->z;
    }
    dtriangleRotateTrans_r4(Out, invecs, Rotation, Position);
}

inline void FetchTransformedTriangle(dxTriMesh* TriMesh, int Index, dVector3 Out[3])
{
    dxPosR* dpr = TriMesh->GetRecomputePosR();
    const dVector3& Position = *(const dVector3*)dpr->pos;
    const dMatrix3& Rotation = *(const dMatrix3*)dpr->R;
    FetchTriangle(TriMesh, Index, Position, Rotation, Out);
}

inline Matrix4x4& MakeMatrix(const dVector3 Position, const dMatrix3 Rotation, Matrix4x4& Out){
    Out.m[0][0] = (float) Rotation[0];
    Out.m[1][0] = (float) Rotation[1];
    Out.m[2][0] = (float) Rotation[2];

    Out.m[0][1] = (float) Rotation[4];
    Out.m[1][1] = (float) Rotation[5];
    Out.m[2][1] = (float) Rotation[6];

    Out.m[0][2] = (float) Rotation[8];
    Out.m[1][2] = (float) Rotation[9];
    Out.m[2][2] = (float) Rotation[10];

    Out.m[3][0] = (float) Position[0];
    Out.m[3][1] = (float) Position[1];
    Out.m[3][2] = (float) Position[2];

    Out.m[0][3] = 0.0f;
    Out.m[1][3] = 0.0f;
    Out.m[2][3] = 0.0f;
    Out.m[3][3] = 1.0f;

    return Out;
}

inline Matrix4x4& MakeMatrix(dxGeom* g, Matrix4x4& Out){
    dxPosR* dpr = g->GetRecomputePosR();
    const dVector3& Position = *(const dVector3*)dpr->pos;
    const dMatrix3& Rotation = *(const dMatrix3*)dpr->R;
    return MakeMatrix(Position, Rotation, Out);
}

// Outputs a matrix to 3 vectors
inline void Decompose(const dMatrix3 Matrix, dVector3 Right, dVector3 Up, dVector3 Direction){
    Right[0] = Matrix[0 * 4 + 0];
    Right[1] = Matrix[1 * 4 + 0];
    Right[2] = Matrix[2 * 4 + 0];
    Right[3] = REAL(0.0);
    Up[0] = Matrix[0 * 4 + 1];
    Up[1] = Matrix[1 * 4 + 1];
    Up[2] = Matrix[2 * 4 + 1];
    Up[3] = REAL(0.0);
    Direction[0] = Matrix[0 * 4 + 2];
    Direction[1] = Matrix[1 * 4 + 2];
    Direction[2] = Matrix[2 * 4 + 2];
    Direction[3] = REAL(0.0);
}

// Outputs a matrix to 3 vectors
inline void Decompose(const dMatrix3 Matrix, dVector3 Vectors[3]){
    Decompose(Matrix, Vectors[0], Vectors[1], Vectors[2]);
}

// Finds barycentric
inline void GetPointFromBarycentric(const dVector3 dv[3], dReal u, dReal v, dVector3 Out){
    dReal w = REAL(1.0) - u - v;

    Out[0] = (dv[0][0] * w) + (dv[1][0] * u) + (dv[2][0] * v);
    Out[1] = (dv[0][1] * w) + (dv[1][1] * u) + (dv[2][1] * v);
    Out[2] = (dv[0][2] * w) + (dv[1][2] * u) + (dv[2][2] * v);
    Out[3] = (dv[0][3] * w) + (dv[1][3] * u) + (dv[2][3] * v);
}

// Performs a callback
inline bool Callback(dxTriMesh* TriMesh, dxGeom* Object, int TriIndex){
    if (TriMesh->Callback != NULL){
        return (TriMesh->Callback(TriMesh, Object, TriIndex)!=0);
    }
    else return true;
}

// Some utilities
template<class T> const T& dcMAX(const T& x, const T& y){
    return x > y ? x : y;
}

template<class T> const T& dcMIN(const T& x, const T& y){
    return x < y ? x : y;
}

#endif	//_ODE_COLLISION_TRIMESH_INTERNAL_H_
