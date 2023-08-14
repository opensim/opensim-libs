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


 /*************************************************************************
  *                                                                       *
  * Triangle-box collider by Alen Ladavac and Vedran Klanac.              *
  * Ported to ODE by Oskari Nyman.                                        *
  *                                                                       *
  *************************************************************************/

#include <ode/collision.h>
#include <ode/rotation.h>
#include "config.h"
#include "matrix.h"
#include "odemath.h"
#include "collision_util.h"
#include "collision_std.h"
#include "collision_trimesh_internal.h"

static void GenerateContact(int in_Flags, dContactGeom* in_Contacts, int in_Stride,
    dxGeom* in_g1, dxGeom* in_g2, int TriIndex,
    const dVector3 in_ContactPos, const dVector3 in_Normal, dReal in_Depth,
    int& OutTriCount);


// largest number, double or float
#if defined(dSINGLE)
#define MAXVALUE FLT_MAX
#else
#define MAXVALUE DBL_MAX
#endif

struct sTrimeshBoxColliderData
{
    sTrimeshBoxColliderData() : m_iBestAxis(0), m_ctContacts(0) {}

    void SetupInitialContext(dxTriMesh *TriMesh, dxGeom *BoxGeom,
        int Flags, dContactGeom* Contacts, int Stride);
    int TestCollisionForSingleTriangle(int ctContacts0, int Triint,
        dVector3 dv[3], bool &bOutFinishSearching);

    bool _cldTestNormal(const dReal depth, dVector3 vNormal);
    bool _cldTestFace(const dReal fp0, const dReal fp1, const dReal fp2, const dReal fR,
        const dVector3 vNormal, const int iAxis);
    bool _cldTestEdge(const dReal fp0, const dReal fp1, const dReal fR, dVector3 vNormal, const int iAxis);
    bool _cldTestSeparatingAxes(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2);
    void _cldClipping(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, int TriIndex);
    bool _cldTestOneTriangle(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, int TriIndex);

    // box data
    dVector3 m_vHullBoxPos;
    dVector3 m_vBoxHalfSize;
    dMatrix3 m_BoxRotTransposed;

    // global collider data
    dVector3 m_vBestNormal;
    dReal    m_fBestDepth;
    int    m_iBestAxis;
    dVector3 m_vE0, m_vE1, m_vE2, m_vNnorm;

    // global info for contact creation
    int m_iFlags;
    dContactGeom *m_ContactGeoms;
    int m_iStride;
    dxGeom *m_Geom1;
    dxGeom *m_Geom2;
    int m_ctContacts;
};

// Test normal of mesh face as separating axis for intersection
ODE_INLINE bool sTrimeshBoxColliderData::_cldTestNormal(const dReal depth, dVector3 vNormal)
{
    if (depth < dEpsilon)
    {
        return false;
    }
    // get minimum depth
    if (depth < m_fBestDepth)
    {
        dCopyNegatedVector3r4(m_vBestNormal, vNormal);
        m_iBestAxis = 1;
        //dAASSERT(fDepth>=0);
        m_fBestDepth = depth;
    }
    return true;
}

// Test box axis as separating axis
bool sTrimeshBoxColliderData::_cldTestFace(const dReal fp0, const dReal fp1, const dReal fp2, const dReal fR,
    const dVector3 vNormal, const int iAxis)
{
    dReal fDepth, fDepthMax;

    if (fp0 < fp1)
    {
        fDepth = fR - (fp0 < fp2 ? fp0 : fp2);
        if (fDepth < 0)
        {
            return false;
        }
        fDepthMax = fR + (fp1 > fp2 ? fp1 : fp2);
        if (fDepthMax < 0)
        {
            return false;
        }
    }
    else
    {
        fDepth = fR - (fp1 < fp2 ? fp1 : fp2);
        if (fDepth < 0)
        {
            return false;
        }
        fDepthMax = fR + (fp0 > fp2 ? fp0 : fp2);
        if (fDepthMax < 0)
        {
            return false;
        }
    }

    // if greater depth is on negative side
    if (fDepth > fDepthMax)
    {
        if (fDepthMax < m_fBestDepth)
        {
            dCopyNegatedVector3r4(m_vBestNormal, vNormal);
            m_iBestAxis = iAxis;
            //dAASSERT(fDepth>=0);
            m_fBestDepth = fDepthMax;
        }
    }
    else if (fDepth < m_fBestDepth)
    {
        dCopyVector3r4(m_vBestNormal, vNormal);
        m_iBestAxis = iAxis;
        //dAASSERT(fDepth>=0);
        m_fBestDepth = fDepth;
    }

    return true;
}

// Test cross products of box axis and triangle edges as separating axis
bool sTrimeshBoxColliderData::_cldTestEdge(const dReal fp0, const dReal fp1, const dReal fR, dVector3 vNormal, const int iAxis)
{
    dReal fDepthMin, fDepthMax;
    // calculate min and max interval values
    if (fp0 < fp1)
    {
        fDepthMin = fR - fp0;
        if (fDepthMin < 0)
        {
            return false;
        }
        fDepthMax = fR + fp1;
        if (fDepthMax < 0)
        {
            return false;
        }
    }
    else
    {
        fDepthMin = fR - fp1;
        if (fDepthMin < 0)
        {
            return false;
        }
        fDepthMax = fR + fp0;
        if (fDepthMax < 0)
        {
            return false;
        }
    }

    dReal fLength = dCalcVectorLengthSquare3(vNormal);
    if (fLength <= dEpsilon) /// THIS NORMAL WOULD BE DANGEROUS
        return true;

    dReal fDepth;
    dReal fOneOverLength = dRecipSqrt(fLength);

    // if greater depth is on negative side
    if (fDepthMin > fDepthMax)
    {
        // use smaller depth (one from positive side)
        fDepth = fDepthMax * fOneOverLength;
        if (fDepth * 1.5f < m_fBestDepth)
        {
            dCopyScaledVector3r4(m_vBestNormal, vNormal, -fOneOverLength);
            m_iBestAxis = iAxis;
            m_fBestDepth = fDepth;
        }
    }
    else
    {
        // normalize depth
        fDepth = fDepthMin * fOneOverLength;
        // if lower depth than best found so far (favor face over edges)
        if (fDepth * 1.5f < m_fBestDepth)
        {
            // remember current axis as best axis
            dCopyScaledVector3r4(m_vBestNormal, vNormal, fOneOverLength);
            m_iBestAxis = iAxis;
            m_fBestDepth = fDepth;
        }
    }
    return true;
}

// clip polygon with plane and generate new polygon points
static void _cldClipPolyToPlane(dVector3 avArrayIn[], const int ctIn, dVector3 avArrayOut[], int &ctOut,
    const dVector3 plPlaneNorm, const dReal plPlaneOffset)
{
    dVector3 *v0;
    dVector3 *v1;
    dReal fDistance0;
    dReal fDistance1;
    // start with no output points
    ctOut = 0;

    v0 = &avArrayIn[ctIn - 1];
    fDistance0 = dCalcVectorDot3(*v0, plPlaneNorm) + plPlaneOffset;

    // for each edge in input polygon
    for (int i1 = 0; i1 < ctIn; i1++)
    {
        v1 = &avArrayIn[i1];
        // calculate distance of edge points to plane
        fDistance1 = dCalcVectorDot3(*v1, plPlaneNorm) + plPlaneOffset;;

        // if first point is in front of plane
        if (fDistance0 == 0)
        {
            // emit point
            dCopyVector3r4(avArrayOut[ctOut], *v0);
            ctOut++;
        }
        else if (fDistance0 > 0)
        {
            // emit point
            dCopyVector3r4(avArrayOut[ctOut], *v0);
            ctOut++;

            if (fDistance1 < 0)
            {
                dReal fd = fDistance0 / (fDistance0 - fDistance1);
                dCalcLerpVectors3r4(avArrayOut[ctOut], *v0, *v1, fd);
                ctOut++;
            }
        }
        else if (fDistance1 > 0)
        {
            // find intersection point of edge and plane
            dReal fd = fDistance0 / (fDistance0 - fDistance1);
            dCalcLerpVectors3r4(avArrayOut[ctOut], *v0, *v1, fd);
            ctOut++;
        }
        v0 = v1;
        fDistance0 = fDistance1;
    }
}

// clip polygon with plane and generate new polygon points
static void _cldClipPolyToNegativePlane(dVector3 avArrayIn[], const int ctIn, dVector3 avArrayOut[], int &ctOut,
    const dVector3 plPlaneNorm, const dReal plPlaneOffset)
{
    dVector3 *v0;
    dVector3 *v1;
    dReal fDistance0;
    dReal fDistance1;
    // start with no output points
    ctOut = 0;

    v0 = &avArrayIn[ctIn - 1];
    fDistance0 = -dCalcVectorDot3(*v0, plPlaneNorm) + plPlaneOffset;

    // for each edge in input polygon
    for (int i1 = 0; i1 < ctIn; i1++)
    {
        v1 = &avArrayIn[i1];
        // calculate distance of edge points to plane
        fDistance1 = -dCalcVectorDot3(*v1, plPlaneNorm) + plPlaneOffset;;

        // if first point is in front of plane
        if (fDistance0 == 0)
        {
            // emit point
            dCopyVector3r4(avArrayOut[ctOut], *v0);
            ctOut++;
        }
        else if (fDistance0 > 0)
        {
            // emit point
            dCopyVector3r4(avArrayOut[ctOut], *v0);
            ctOut++;

            if (fDistance1 < 0)
            {
                dReal fd = fDistance0 / (fDistance0 - fDistance1);
                dCalcLerpVectors3r4(avArrayOut[ctOut], *v0, *v1, fd);
                ctOut++;
            }
        }
        else if (fDistance1 > 0)
        {
            // find intersection point of edge and plane
            dReal fd = fDistance0 / (fDistance0 - fDistance1);
            dCalcLerpVectors3r4(avArrayOut[ctOut], *v0, *v1, fd);
            ctOut++;
        }
        v0 = v1;
        fDistance0 = fDistance1;
    }
}
// clip polygon with plane and generate new polygon points
static void _cldClipPolyToPlaneAtOrigin(dVector3 avArrayIn[], const int ctIn, dVector3 avArrayOut[], int &ctOut,
    const dVector3 plPlaneNorm)
{
    dVector3 *v0;
    dVector3 *v1;
    dReal fDistance0;
    dReal fDistance1;
    // start with no output points
    ctOut = 0;

    v0 = &avArrayIn[ctIn - 1];
    fDistance0 = dCalcVectorDot3(*v0, plPlaneNorm);

    // for each edge in input polygon
    for (int i1 = 0; i1 < ctIn; i1++)
    {
        v1 = &avArrayIn[i1];
        // calculate distance of edge points to plane
        fDistance1 = dCalcVectorDot3(*v1, plPlaneNorm);

        // if first point is in front of plane
        if (fDistance0 == 0)
        {
            // emit point
            dCopyVector3r4(avArrayOut[ctOut], *v0);
            ctOut++;
        }
        else if (fDistance0 > 0)
        {
            // emit point
            dCopyVector3r4(avArrayOut[ctOut], *v0);
            ctOut++;

            if (fDistance1 < 0)
            {
                dReal fd = fDistance0 / (fDistance0 - fDistance1);
                dCalcLerpVectors3r4(avArrayOut[ctOut], *v0, *v1, fd);
                ctOut++;
            }
        }
        else if (fDistance1 > 0)
        {
            // find intersection point of edge and plane
            dReal fd = fDistance0 / (fDistance0 - fDistance1);
            dCalcLerpVectors3r4(avArrayOut[ctOut], *v0, *v1, fd);
            ctOut++;
        }
        v0 = v1;
        fDistance0 = fDistance1;
    }
}


ODE_INLINE bool sTrimeshBoxColliderData::_cldTestSeparatingAxes(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2) {
    // reset best axis
    m_iBestAxis = 0;
    m_fBestDepth = MAXVALUE;

    // calculate edges
    dSubtractVectors3r4(m_vE0, v1, v0);
    dSubtractVectors3r4(m_vE1, v2, v0);

    dVector3 vN;
    // calculate poly normal
    dCalcVectorCross3r4(vN, m_vE0, m_vE1);

    // calculate length of face normal
    dReal fNLen = dCalcVectorLengthSquare3(vN);

    // Even though all triangles might be initially valid, 
    // a triangle may degenerate into a segment after applying 
    // space transformation.
    if (fNLen < dEpsilon)
    {
        return false;
    }

    dReal invfNLen = dRecipSqrt(fNLen);
    dCopyScaledVector3r4(m_vNnorm, vN, invfNLen);

    // extract box axes as vectors
    dVector3& vA0 = *(dVector3*)m_BoxRotTransposed;
    dVector3& vA1 = *(dVector3*)(m_BoxRotTransposed + 4);
    dVector3& vA2 = *(dVector3*)(m_BoxRotTransposed + 8);

    // box halfsizes
    dReal fa0, fa1, fa2;
    fa0 = m_vBoxHalfSize[0];
    fa1 = m_vBoxHalfSize[1];
    fa2 = m_vBoxHalfSize[2];

    // calculate relative position between box and triangle
    dVector3 v0D;
    dSubtractVectors3r4(v0D, v0, m_vHullBoxPos);

    dVector3 vL;
    dReal fp0, fp1, fp2, fR;

    // Test separating axes for intersection
    // ************************************************
    fp0 = dCalcVectorDot3(m_vNnorm, v0D);
    fR = fa0 * dFabs(dCalcVectorDot3(m_vNnorm, vA0)) +
         fa1 * dFabs(dCalcVectorDot3(m_vNnorm, vA1)) +
         fa2 * dFabs(dCalcVectorDot3(m_vNnorm, vA2));

    if (!_cldTestNormal(fp0 + fR, m_vNnorm))
    {
        m_iBestAxis = -1;
        return false;
    }

    // Test Faces
    // ************************************************
    // Axis 2 - Box X-Axis
    const dReal vA0DotvE0 = dCalcVectorDot3(vA0, m_vE0);
    const dReal vA0DotvE1 = dCalcVectorDot3(vA0, m_vE1);

    fp0 = dCalcVectorDot3(vA0, v0D);
    fp1 = fp0 + vA0DotvE0;
    fp2 = fp0 + vA0DotvE1;
    fR = fa0;

    if (!_cldTestFace(fp0, fp1, fp2, fR, vA0, 2))
    {
        m_iBestAxis = -2;
        return false;
    }

    const dReal vA1DotvE0 = dCalcVectorDot3(vA1, m_vE0);
    const dReal vA1DotvE1 = dCalcVectorDot3(vA1, m_vE1);

    // Axis 3 - Box Y-Axis
    fp0 = dCalcVectorDot3(vA1, v0D);
    fp1 = fp0 + vA1DotvE0;
    fp2 = fp0 + vA1DotvE1;
    fR = fa1;

    if (!_cldTestFace(fp0, fp1, fp2, fR, vA1, 3))
    {
        m_iBestAxis = -3;
        return false;
    }

    const dReal vA2DotvE0 = dCalcVectorDot3(vA2, m_vE0);
    const dReal vA2DotvE1 = dCalcVectorDot3(vA2, m_vE1);

    // Axis 4 - Box Z-Axis
    fp0 = dCalcVectorDot3(vA2, v0D);
    fp1 = fp0 + vA2DotvE0;
    fp2 = fp0 + vA2DotvE1;
    fR = fa2;

    if (!_cldTestFace(fp0, fp1, fp2, fR, vA2, 4))
    {
        m_iBestAxis = -4;
        return false;
    }

    // Test Edges
    // ************************************************
    // Axis 5 - Box X-Axis cross Edge0
    const dReal vA0DotvN = dCalcVectorDot3(vA0, vN);

    dCalcVectorCross3r4(vL, vA0, m_vE0);
    fp0 = dCalcVectorDot3(vL, v0D);
    fp2 = fp0 + vA0DotvN;
    fR = fa1 * dFabs(vA2DotvE0) + fa2 * dFabs(vA1DotvE0);

    if (!_cldTestEdge(fp0, fp2, fR, vL, 5))
    {
        m_iBestAxis = -5;
            return false;
    }

    // ************************************************
    // Axis 6 - Box X-Axis cross Edge1
    dCalcVectorCross3r4(vL, vA0, m_vE1);
    fp0 = dCalcVectorDot3(vL, v0D);
    fp1 = fp0 - vA0DotvN;
    fR = fa1 * dFabs(vA2DotvE1) + fa2 * dFabs(vA1DotvE1);

    if (!_cldTestEdge(fp0, fp1, fR, vL, 6))
    {
        m_iBestAxis = -6;
        return false;
    }

    dSubtractVectors3r4(m_vE2, m_vE1, m_vE0);
    const dReal fAbsvA1DotvE2 = dFabs(dCalcVectorDot3(vA1, m_vE2));
    const dReal fAbsvA2DotvE2 = dFabs(dCalcVectorDot3(vA2, m_vE2));

    // Axis 7 - Box X-Axis cross Edge2
    dCalcVectorCross3r4(vL, vA0, m_vE2);
    fp0 = dCalcVectorDot3(vL, v0D);
    fp1 = fp0 - vA0DotvN;
    fR = fa1 * fAbsvA2DotvE2 + fa2 * fAbsvA1DotvE2;

    if (!_cldTestEdge(fp0, fp1, fR, vL, 7))
    {
        m_iBestAxis = -7;
        return false;
    }

    // ************************************************
    // Axis 8 - Box Y-Axis cross Edge0

    const dReal vA1DotvN = dCalcVectorDot3(vA1, vN);

    dCalcVectorCross3r4(vL, vA1, m_vE0);
    fp0 = dCalcVectorDot3(vL, v0D);
    fp2 = fp0 + vA1DotvN;
    fR = fa0 * dFabs(vA2DotvE0) + fa2 * dFabs(vA0DotvE0);

    if (!_cldTestEdge(fp0, fp2, fR, vL, 8))
    {
        m_iBestAxis = -8;
        return false;
    }

    // ************************************************

    // ************************************************
    // Axis 9 - Box Y-Axis cross Edge1
    dCalcVectorCross3r4(vL, vA1, m_vE1);
    fp0 = dCalcVectorDot3(vL, v0D);
    fp1 = fp0 - vA1DotvN;
    fR = fa0 * dFabs(vA2DotvE1) + fa2 * dFabs(vA0DotvE1);

    if (!_cldTestEdge(fp0, fp1, fR, vL, 9))
    {
        m_iBestAxis = -9;
        return false;
    }

    const dReal fAbsvA0Dotm_vE2 = dFabs(dCalcVectorDot3(vA0, m_vE2));

    // ************************************************
    // Axis 10 - Box Y-Axis cross Edge2
    dCalcVectorCross3r4(vL, vA1, m_vE2);
    fp0 = dCalcVectorDot3(vL, v0D);
    fp1 = fp0 - vA1DotvN;
    fR = fa0 * fAbsvA2DotvE2 + fa2 * fAbsvA0Dotm_vE2;

    if (!_cldTestEdge(fp0, fp1, fR, vL, 10))
    {
        m_iBestAxis = -10;
        return false;
    }

    // ************************************************
    // Axis 11 - Box Z-Axis cross Edge0
    const dReal vA2DotvN = dCalcVectorDot3(vA2, vN);

    dCalcVectorCross3r4(vL, vA2, m_vE0);
    fp0 = dCalcVectorDot3(vL, v0D);
    fp2 = fp0 + vA2DotvN;
    fR = fa0 * dFabs(vA1DotvE0) + fa1 * dFabs(vA0DotvE0);

    if (!_cldTestEdge(fp0, fp2, fR, vL, 11))
    {
        m_iBestAxis = -11;
        return false;
    }
    // ************************************************

    // ************************************************
    // Axis 12 - Box Z-Axis cross Edge1
    dCalcVectorCross3r4(vL, vA2, m_vE1);
    fp0 = dCalcVectorDot3(vL, v0D);
    fp1 = fp0 - vA2DotvN;
    fR = fa0 * dFabs(vA1DotvE1) + fa1 * dFabs(vA0DotvE1);

    if (!_cldTestEdge(fp0, fp1, fR, vL, 12))
    {
        m_iBestAxis = -12;
        return false;
    }
    // ************************************************

    // ************************************************
    // Axis 13 - Box Z-Axis cross Edge2
    dCalcVectorCross3r4(vL, vA2, m_vE2);
    fp0 = dCalcVectorDot3(vL, v0D);
    fp1 = fp0 - vA2DotvN;
    fR = fa0 * fAbsvA1DotvE2 + fa1 * fAbsvA0Dotm_vE2;

    if (!_cldTestEdge(fp0, fp1, fR, vL, 13))
    {
        m_iBestAxis = -13;
        return false;
    }


    // ************************************************
    return true;
}

// find two closest points on two lines
static bool _cldClosestPointOnTwoLines(
    const dVector3 vPoint1, const dVector3 vLenVec1, const dVector3 vPoint2, const dVector3 vLenVec2,
    dReal &fvalue1, dReal &fvalue2)
{
    // calculate denominator
    dReal fuaub = dCalcVectorDot3(vLenVec1, vLenVec2);
    dReal fd = 1.0f - fuaub * fuaub;

    // if denominator is positive
    if (fd > dEpsilon)
    {
        // calculate points of closest approach
        fd = 1.0f / fd;
        dVector3 vp;
        dSubtractVectors3r4(vp, vPoint2, vPoint1);

        dReal fq1 = dCalcVectorDot3(vLenVec1, vp);
        dReal fq2 = -dCalcVectorDot3(vLenVec2, vp);
        fvalue1 = (fq1 + fuaub * fq2) * fd;
        fvalue2 = (fuaub * fq1 + fq2) * fd;
        return true;
    }
    else
    {
        // lines are parallel
        fvalue1 = 0.0f;
        fvalue2 = 0.0f;
        return false;
    }
}

// clip and generate contacts
void sTrimeshBoxColliderData::_cldClipping(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, int TriIndex) {
    dIASSERT(!(m_iFlags & CONTACTS_UNIMPORTANT) || m_ctContacts < (m_iFlags & NUMC_MASK)); // Do not call the function if there is no room to store results

    // if we have edge/edge intersection
    if (m_iBestAxis > 4)
    {
        dVector3 vub, vPb, vPa;
        dCopyVector3r4(vPa, m_vHullBoxPos);

        // calculate point on box edge
        if (dCalcVectorDot3(m_vBestNormal, m_BoxRotTransposed) > 0)
            dAddScaledVector3r4(vPa, m_BoxRotTransposed, m_vBoxHalfSize[0]);
        else
            dAddScaledVector3r4(vPa, m_BoxRotTransposed, -m_vBoxHalfSize[0]);

        if (dCalcVectorDot3(m_vBestNormal, m_BoxRotTransposed + 4) > 0)
            dAddScaledVector3r4(vPa, m_BoxRotTransposed + 4, m_vBoxHalfSize[1]);
        else
            dAddScaledVector3r4(vPa, m_BoxRotTransposed + 4, -m_vBoxHalfSize[1]);

        if (dCalcVectorDot3(m_vBestNormal, m_BoxRotTransposed + 8))
            dAddScaledVector3r4(vPa, m_BoxRotTransposed + 8, m_vBoxHalfSize[2]);
        else
            dAddScaledVector3r4(vPa, m_BoxRotTransposed + 8, -m_vBoxHalfSize[2]);

        int iEdge = (m_iBestAxis - 5) % 3;

        // decide which edge is on triangle
        switch(iEdge)
        {
            case 0:
                dCopyVector3r4(vPb, v0);
                dCopyVector3r4(vub, m_vE0);
                break;
            case 1:
                dCopyVector3r4(vPb, v2);
                dCopyVector3r4(vub, m_vE1);
                break;
            default:
                dCopyVector3r4(vPb, v1);
                dCopyVector3r4(vub, m_vE2);
                break;
        }
        // setup direction parameter for face edge
        dNormalize3(vub);

        dReal fParam1, fParam2;

        // setup direction parameter for box edge
        int col = (m_iBestAxis - 5) / 3;
        dVector3& vua = *(dVector3*)(m_BoxRotTransposed + 4 * col);

        // find two closest points on both edges
        _cldClosestPointOnTwoLines(vPa, vua, vPb, vub, fParam1, fParam2);
        dAddScaledVector3r4(vPa, vua, fParam1);
        dAddScaledVector3r4(vPb, vub, fParam2);

        // calculate collision point
        dVector3 vPntTmp;
        dAddVectors3r4(vPntTmp, vPa, vPb);

        dScaleVector3r4(vPntTmp, 0.5f);

        GenerateContact(m_iFlags, m_ContactGeoms, m_iStride, m_Geom1, m_Geom2, TriIndex,
            vPntTmp, m_vBestNormal, m_fBestDepth, m_ctContacts);

        // if triangle is the referent face then clip box to triangle face
    }
    else if (m_iBestAxis == 1)
    {
        // vNr is normal in box frame, pointing from triangle to box
        dVector3 vNr;
        vNr[0] = -dCalcVectorDot3(m_BoxRotTransposed, m_vBestNormal);
        vNr[1] = -dCalcVectorDot3(m_BoxRotTransposed + 4, m_vBestNormal);
        vNr[2] = -dCalcVectorDot3(m_BoxRotTransposed + 8, m_vBestNormal);

        dVector3 vAbsNormal;
        dFabsVector3r4(vAbsNormal, vNr);

        // get closest face from box
        int iB0, iB1, iB2;
        if (vAbsNormal[1] > vAbsNormal[0])
        {
            if (vAbsNormal[1] > vAbsNormal[2])
            {
                iB0 = 1;
                iB1 = 0;
                iB2 = 2;
            }
            else
            {
                iB0 = 2;
                iB1 = 0;
                iB2 = 1;
            }
        }
        else
        {
            if (vAbsNormal[0] > vAbsNormal[2])
            {
                iB0 = 0;
                iB1 = 1;
                iB2 = 2;
            }
            else
            {
                iB0 = 2;
                iB1 = 0;
                iB2 = 1;
            }
        }

        // Here find center of box face we are going to project
        dVector3 vCenter;
        dSubtractVectors3r4(vCenter, m_vHullBoxPos, v0);

        dVector3& vRotCol0 = *(dVector3*)(m_BoxRotTransposed + 4 * iB0);
        if (vNr[iB0] > 0)
            dAddScaledVector3r4(vCenter, vRotCol0, -m_vBoxHalfSize[iB0]);
        else
            dAddScaledVector3r4(vCenter, vRotCol0, m_vBoxHalfSize[iB0]);

        // Here find 4 corner points of box
        dVector3 avPoints[4];

        dVector3& vRotCol1 = *(dVector3*)(m_BoxRotTransposed + 4 * iB1);
        dVector3& vRotCol2 = *(dVector3*)(m_BoxRotTransposed + 4 * iB2);

        dReal m_vBoxHalfSizeiB1 = m_vBoxHalfSize[iB1];
        dReal m_vBoxHalfSizeiB2 = m_vBoxHalfSize[iB2];

        dAddScaledVectors3r4(avPoints[0], vRotCol1, vRotCol2, m_vBoxHalfSizeiB1, -m_vBoxHalfSizeiB2);
        dAddVector3r4(avPoints[0], vCenter);

        dAddScaledVectors3r4(avPoints[1], vRotCol1, vRotCol2, -m_vBoxHalfSizeiB1, -m_vBoxHalfSizeiB2);
        dAddVector3r4(avPoints[1], vCenter);

        dAddScaledVectors3r4(avPoints[2], vRotCol1, vRotCol2, -m_vBoxHalfSizeiB1, m_vBoxHalfSizeiB2);
        dAddVector3r4(avPoints[2], vCenter);

        dAddScaledVectors3r4(avPoints[3], vRotCol1, vRotCol2, m_vBoxHalfSizeiB1, m_vBoxHalfSizeiB2);
        dAddVector3r4(avPoints[3], vCenter);

        // clip Box face with 4 planes of triangle (1 face plane, 3 egde planes)
        dVector3 avTempArray1[9];
        dVector3 avTempArray2[9];


        int iTempCnt1 = 0;
        int iTempCnt2 = 0;

        // Normal plane
        dVector3 vTemp;
        dCopyNegatedVector3r4(vTemp, m_vNnorm);
        _cldClipPolyToPlaneAtOrigin(avPoints, 4, avTempArray1, iTempCnt1, vTemp);

        // Plane p0
        dVector3 vTemp2;
        dSubtractVectors3r4(vTemp2, v1, v0);
        dCalcVectorCross3r4(vTemp, m_vNnorm, vTemp2);
        dNormalize3(vTemp);
        _cldClipPolyToPlaneAtOrigin(avTempArray1, iTempCnt1, avTempArray2, iTempCnt2, vTemp);

        // Plane p1
        dSubtractVectors3r4(vTemp2, v2, v1);
        dCalcVectorCross3r4(vTemp, m_vNnorm, vTemp2);
        dNormalize3(vTemp);
        dSubtractVectors3r4(vTemp2, v0, v2);
        _cldClipPolyToPlane(avTempArray2, iTempCnt2, avTempArray1, iTempCnt1, 
            vTemp, dCalcVectorDot3(vTemp2, vTemp));

        // Plane p2
        dCalcVectorCross3r4(vTemp, m_vNnorm, vTemp2);
        dNormalize3(vTemp);
        _cldClipPolyToPlaneAtOrigin(avTempArray1, iTempCnt1, avTempArray2, iTempCnt2, vTemp);

        // END of clipping polygons

        // for each generated contact point
        for (int i = 0; i < iTempCnt2; i++)
        {
            // calculate depth
            dReal fTempDepth = dCalcVectorDot3(m_vBestNormal, avTempArray2[i]);

            // clamp depth to zero
            if (fTempDepth < 0)
            {
                fTempDepth = 0;
            }

            dVector3 vPntTmp;
            dAddVectors3r4(vPntTmp, avTempArray2[i], v0);

            GenerateContact(m_iFlags, m_ContactGeoms, m_iStride, m_Geom1, m_Geom2, TriIndex,
                vPntTmp, m_vBestNormal, fTempDepth, m_ctContacts);

            if ((m_ctContacts | CONTACTS_UNIMPORTANT) == (m_iFlags & (NUMC_MASK | CONTACTS_UNIMPORTANT))) {
                break;
            }
        }

        //dAASSERT(m_ctContacts>0);

        // if box face is the referent face, then clip triangle on box face
    }
    else
    { // 2 <= if iBestAxis <= 4

        // get normal of box face
        dVector3 vNormal2;
        dCopyVector3r4(vNormal2, m_vBestNormal);

        // get indices of box axes in correct order
        int iA0, iA1, iA2;
        iA0 = m_iBestAxis - 2;
        if (iA0 == 0)
        {
            iA1 = 1;
            iA2 = 2;
        }
        else if (iA0 == 1)
        {
            iA1 = 0;
            iA2 = 2;
        }
        else
        {
            iA1 = 0;
            iA2 = 1;
        }

        dVector3 avPoints[3];
        // calculate triangle vertices in box frame
        dSubtractVectors3r4(avPoints[0], v0, m_vHullBoxPos);
        dSubtractVectors3r4(avPoints[1], v1, m_vHullBoxPos);
        dSubtractVectors3r4(avPoints[2], v2, m_vHullBoxPos);

        // CLIP Polygons
        // define temp data for clipping
        dVector3 avTempArray1[9];
        dVector3 avTempArray2[9];

        int iTempCnt1, iTempCnt2;

        // clip triangle with 5 box planes (1 face plane, 4 edge planes)

        // Normal plane
        dVector3 vTemp;
        dCopyNegatedVector3r4(vTemp, vNormal2);
        _cldClipPolyToPlane(avPoints, 3, avTempArray1, iTempCnt1, vTemp, m_vBoxHalfSize[iA0]);

        const dVector3& rotA1 = *(dVector3*)(m_BoxRotTransposed + 4 * iA1);
        // Plane p0
        _cldClipPolyToPlane(avTempArray1, iTempCnt1, avTempArray2, iTempCnt2, 
            rotA1, m_vBoxHalfSize[iA1]);

        // Plane p1
        _cldClipPolyToNegativePlane(avTempArray2, iTempCnt2, avTempArray1, iTempCnt1, 
            rotA1, m_vBoxHalfSize[iA1]);


        const dVector3& rotA2 = *(dVector3*)(m_BoxRotTransposed + 4 * iA2);
        // Plane p2
        _cldClipPolyToPlane(avTempArray1, iTempCnt1, avTempArray2, iTempCnt2, 
            rotA2, m_vBoxHalfSize[iA2]);

        // Plane p3
        _cldClipPolyToNegativePlane(avTempArray2, iTempCnt2, avTempArray1, iTempCnt1, 
            rotA2, m_vBoxHalfSize[iA2]);

        // for each generated contact point
        for (int i = 0; i < iTempCnt1; i++)
        {
            // calculate depth
            dReal fTempDepth = dCalcVectorDot3(vNormal2, avTempArray1[i]) - m_vBoxHalfSize[iA0];

            // clamp depth to zero
            if (fTempDepth > 0)
            {
                fTempDepth = 0;
            }

            // generate contact data
            dVector3 vPntTmp;
            dAddVectors3r4(vPntTmp, avTempArray1[i], m_vHullBoxPos);

            GenerateContact(m_iFlags, m_ContactGeoms, m_iStride, m_Geom1, m_Geom2, TriIndex,
                vPntTmp, m_vBestNormal, -fTempDepth, m_ctContacts);

            if ((m_ctContacts | CONTACTS_UNIMPORTANT) == (m_iFlags & (NUMC_MASK | CONTACTS_UNIMPORTANT))) {
                break;
            }
        }

        //dAASSERT(m_ctContacts>0);
    }
}

// test one mesh triangle on intersection with given box
ODE_INLINE bool sTrimeshBoxColliderData::_cldTestOneTriangle(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2, int TriIndex)//, void *pvUser)
{
    // do intersection test and find best separating axis
    if (_cldTestSeparatingAxes(v0, v1, v2))
    {
        // if best separation axis is found
        if (m_iBestAxis > 0)
        {
            _cldClipping(v0, v1, v2, TriIndex);
            return true;
        }
    }
    return false;
}

ODE_INLINE void sTrimeshBoxColliderData::SetupInitialContext(dxTriMesh *TriMesh, dxGeom *BoxGeom,
    int Flags, dContactGeom* Contacts, int Stride)
{
    // get source hull position, orientation and half size
    dxPosR* posr = BoxGeom->GetRecomputePosR();
    const dMatrix3& mRotBox = *(const dMatrix3*)posr->R;
    const dVector3& vPosBox = *(const dVector3*)posr->pos;

    dTransposetMatrix34(m_BoxRotTransposed, mRotBox);

    dCopyVector3r4(m_vHullBoxPos, vPosBox);
    dCopyVector3r4(m_vBoxHalfSize, ((dxBox*)BoxGeom)->halfside);

    // global info for contact creation
    m_ctContacts = 0;
    m_iStride = Stride;
    m_iFlags = Flags;
    m_ContactGeoms = Contacts;
    m_Geom1 = TriMesh;
    m_Geom2 = BoxGeom;

    // reset stuff
    m_fBestDepth = MAXVALUE;
    dZeroVector3r4(m_vBestNormal);
}

int sTrimeshBoxColliderData::TestCollisionForSingleTriangle(int ctContacts0, int Triint,
    dVector3 dv[3], bool &bOutFinishSearching)
{
    // test this triangle
    if (_cldTestOneTriangle(dv[0], dv[1], dv[2], Triint))
    {
        // fill-in tri index for generated contacts
        for (; ctContacts0 < m_ctContacts; ctContacts0++)
        {
            dContactGeom* pContact = SAFECONTACT(m_iFlags, m_ContactGeoms, ctContacts0, m_iStride);
            pContact->side1 = Triint;
            pContact->side2 = -1;
        }

        /*
        NOTE by Oleh_Derevenko:
        The function continues checking triangles after maximal number
        of contacts is reached because it selects maximal penetration depths.
        See also comments in GenerateContact()
        */
        bOutFinishSearching = ((m_ctContacts | CONTACTS_UNIMPORTANT) == (m_iFlags & (NUMC_MASK | CONTACTS_UNIMPORTANT)));
    }
    return ctContacts0;
}

// OPCODE version of box to mesh collider
static void dQueryBTLPotentialCollisionTriangles(OBBCollider &Collider,
    const sTrimeshBoxColliderData &cData, dxTriMesh *TriMesh, dxGeom *BoxGeom,
    OBBCache &BoxCache)
{
    // get source hull position, orientation and half size
    const dMatrix3& mRotTransBox = *(const dMatrix3*)cData.m_BoxRotTransposed;
    const dVector3& vPosBox = *(const dVector3*)dGeomGetPosition(BoxGeom);

    // Make OBB
    OBB Box;
    Box.mCenter.x = vPosBox[0];
    Box.mCenter.y = vPosBox[1];
    Box.mCenter.z = vPosBox[2];
    
    Box.mExtents.x = cData.m_vBoxHalfSize[0];
    Box.mExtents.y = cData.m_vBoxHalfSize[1];
    Box.mExtents.z = cData.m_vBoxHalfSize[2];

    Box.mRot.m[0][0] = mRotTransBox[0];
    Box.mRot.m[0][1] = mRotTransBox[1];
    Box.mRot.m[0][2] = mRotTransBox[2];

    Box.mRot.m[1][0] = mRotTransBox[4];
    Box.mRot.m[1][1] = mRotTransBox[5];
    Box.mRot.m[1][2] = mRotTransBox[6];

    Box.mRot.m[2][0] = mRotTransBox[8];
    Box.mRot.m[2][1] = mRotTransBox[9];
    Box.mRot.m[2][2] = mRotTransBox[10];

    // get destination hull position and orientation
    const dMatrix3& mRotMesh = *(const dMatrix3*)dGeomGetRotation(TriMesh);
    const dVector3& vPosMesh = *(const dVector3*)dGeomGetPosition(TriMesh);

    Matrix4x4 amatrix;

    // TC results
    if (TriMesh->doBoxTC)
    {
        dxTriMesh::BoxTC* BoxTC = 0;
        for (int i = 0; i < TriMesh->BoxTCCache.size(); i++)
        {
            if (TriMesh->BoxTCCache[i].Geom == BoxGeom)
            {
                BoxTC = &TriMesh->BoxTCCache[i];
                break;
            }
        }
        if (!BoxTC)
        {
            TriMesh->BoxTCCache.push(dxTriMesh::BoxTC());

            BoxTC = &TriMesh->BoxTCCache[TriMesh->BoxTCCache.size() - 1];
            BoxTC->Geom = BoxGeom;
            BoxTC->FatCoeff = 1.1f; // Pierre recommends this, instead of 1.0
        }

        // Intersect
        Collider.SetTemporalCoherence(true);
        Collider.Collide(*BoxTC, Box, TriMesh->Data->BVTree, null, &MakeMatrix(vPosMesh, mRotMesh, amatrix));
    }
    else
    {
        Collider.SetTemporalCoherence(false);
        Collider.Collide(BoxCache, Box, TriMesh->Data->BVTree, null,
            &MakeMatrix(vPosMesh, mRotMesh, amatrix));
    }
}

int dCollideBTL(dxGeom* g1, dxGeom* BoxGeom, int Flags, dContactGeom* Contacts, int Stride) {
    dIASSERT(Stride >= (int)sizeof(dContactGeom));
    dIASSERT(g1->type == dTriMeshClass);
    dIASSERT(BoxGeom->type == dBoxClass);
    dIASSERT((Flags & NUMC_MASK) >= 1);

    dxTriMesh* TriMesh = (dxTriMesh*)g1;

    sTrimeshBoxColliderData cData;
    cData.SetupInitialContext(TriMesh, BoxGeom, Flags, Contacts, Stride);

    const unsigned uiTLSKind = TriMesh->getParentSpaceTLSKind();
    dIASSERT(uiTLSKind == BoxGeom->getParentSpaceTLSKind()); // The colliding spaces must use matching cleanup method
    TrimeshCollidersCache *pccColliderCache = GetTrimeshCollidersCache(uiTLSKind);
    OBBCollider& Collider = pccColliderCache->_OBBCollider;

    dQueryBTLPotentialCollisionTriangles(Collider, cData, TriMesh, BoxGeom,
        pccColliderCache->defaultBoxCache);

    if (!Collider.GetContactStatus()) {
        // no collision occurred
        return 0;
    }

    // Retrieve data
    int TriCount = Collider.GetNbTouchedPrimitives();
    const int* Triangles = (const int*)Collider.GetTouchedPrimitives();

    if (TriCount != 0)
    {
        if (TriMesh->ArrayCallback != null)
        {
            TriMesh->ArrayCallback(TriMesh, BoxGeom, Triangles, TriCount);
        }

        // get destination hull position and orientation
        dxPosR* TriMeshPosr = TriMesh->GetRecomputePosR();
        const dMatrix3& mRotMesh = *(const dMatrix3*)TriMeshPosr->R;
        const dVector3& vPosMesh = *(const dVector3*)TriMeshPosr->pos;

        int ctContacts0 = 0;

        // loop through all intersecting triangles
        for (int i = 0; i < TriCount; i++)
        {
            const int Triint = Triangles[i];
            if (!Callback(TriMesh, BoxGeom, Triint))
                continue;

            dVector3 dv[3];
            FetchTriangle(TriMesh, Triint, vPosMesh, mRotMesh, dv);

            bool bFinishSearching;
            ctContacts0 = cData.TestCollisionForSingleTriangle(ctContacts0, Triint, dv, bFinishSearching);

            if (bFinishSearching)
            {
                break;
            }
        }
    }

    return cData.m_ctContacts;
}

// GenerateContact - Written by Jeff Smith (jeff@burri.to)
//   Generate a "unique" contact.  A unique contact has a unique
//   position or normal.  If the potential contact has the same
//   position and normal as an existing contact, but a larger
//   penetration depth, this new depth is used instead
//
static void
GenerateContact(int in_Flags, dContactGeom* in_Contacts, int in_Stride,
    dxGeom* in_g1, dxGeom* in_g2, int TriIndex,
    const dVector3 in_ContactPos, const dVector3 in_Normal, dReal in_Depth,
    int& OutTriCount)
{
    /*
    NOTE by Oleh_Derevenko:
    This function is called after maximal number of contacts has already been
    collected because it has a side effect of replacing penetration depth of
    existing contact with larger penetration depth of another matching normal contact.
    If this logic is not necessary any more, you can bail out on reach of contact
    number maximum immediately in dCollideBTL(). You will also need to correct
    conditional statements after invocations of GenerateContact() in _cldClipping().
    */

    dContactGeom* Contact;
    dVector3 diff;

    if (!(in_Flags & CONTACTS_UNIMPORTANT))
    {
        bool duplicate = false;
        for (int i = 0; i < OutTriCount; i++)
        {
            Contact = SAFECONTACT(in_Flags, in_Contacts, i, in_Stride);

            // same position?
            dSubtractVectors3r4(diff, in_ContactPos, Contact->pos);
            if (dCalcVectorLengthSquare3(diff) < dEpsilon)
            {
                // same normal?
                if (REAL(1.0) - dFabs(dCalcVectorDot3(in_Normal, Contact->normal)) < dEpsilon)
                {
                    if (in_Depth > Contact->depth)
                        Contact->depth = in_Depth;
                    duplicate = true;
                    /*
                    NOTE by Oleh_Derevenko:
                    There may be a case when two normals are close to each other but not duplicate
                    while third normal is detected to be duplicate for both of them.
                    This is the only reason I can think of, there is no "break" statement.
                    Perhaps author considered it to be logical that the third normal would
                    replace the depth in both of initial contacts.
                    However, I consider it a questionable practice which should not
                    be applied without deep understanding of underlaying physics.
                    Even more, is this situation with close normal triplet acceptable at all?
                    Should not be two initial contacts reduced to one (replaced with the latter)?
                    If you know the answers for these questions, you may want to change this code.
                    See the same statement in GenerateContact() of collision_trimesh_trimesh.cpp
                    */
                }
            }
        }
        if (duplicate || OutTriCount == (in_Flags & NUMC_MASK))
            return;
    }
    else
    {
        dIASSERT(OutTriCount < (in_Flags & NUMC_MASK));
    }

    // Add a new contact
    Contact = SAFECONTACT(in_Flags, in_Contacts, OutTriCount, in_Stride);

    dCopyVector3r4(Contact->pos, in_ContactPos);
    dCopyVector3r4(Contact->normal, in_Normal);

    Contact->depth = in_Depth;

    Contact->g1 = in_g1;
    Contact->g2 = in_g2;

    Contact->side1 = TriIndex;
    Contact->side2 = -1;

    OutTriCount++;
}
