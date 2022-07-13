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

/*

  standard ODE geometry primitives: public API and pairwise collision functions.

  the rule is that only the low level primitive collision functions should set
  dContactGeom::g1 and dContactGeom::g2.

*/

#include <ode/common.h>
#include <ode/collision.h>
#include <ode/rotation.h>
#include "config.h"
#include "matrix.h"
#include "odemath.h"
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_util.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

//****************************************************************************
// box public API

dxBox::dxBox (dSpaceID space, dReal lx, dReal ly, dReal lz) : dxGeom (space,1)
{
    dAASSERT (lx >= 0 && ly >= 0 && lz >= 0);
    type = dBoxClass;
    halfside[0] = REAL(0.5) * lx;
    halfside[1] = REAL(0.5) * ly;
    halfside[2] = REAL(0.5) * lz;
    updateZeroSizedFlag(!lx || !ly || !lz);
}

void dxBox::computeAABB()
{
    dVector3 fabsR;
    const dMatrix3& R = final_posr->R;
    const dVector3& pos = final_posr->pos;

    dFabsVector3r4(fabsR, R);
    dReal range = dCalcVectorDot3(fabsR, halfside);
    aabb[0] = pos[0] - range;
    aabb[1] = pos[0] + range;

    dFabsVector3r4(fabsR, &R[4]);
    range = dCalcVectorDot3(fabsR, halfside);
    aabb[2] = pos[1] - range;
    aabb[3] = pos[1] + range;

    dFabsVector3r4(fabsR, &R[8]);
    range = dCalcVectorDot3(fabsR, halfside);
    aabb[4] = pos[2] - range;
    aabb[5] = pos[2] + range;
}

dGeomID dCreateBox (dSpaceID space, dReal lx, dReal ly, dReal lz)
{
    return new dxBox (space,lx,ly,lz);
}


void dGeomBoxSetLengths (dGeomID g, dReal lx, dReal ly, dReal lz)
{
    dUASSERT (g && g->type == dBoxClass,"argument not a box");
    dAASSERT (lx >= 0 && ly >= 0 && lz >= 0);
    dxBox *b = (dxBox*) g;
    b->halfside[0] = REAL(0.5) * lx;
    b->halfside[1] = REAL(0.5) * ly;
    b->halfside[2] = REAL(0.5) * lz;
    b->updateZeroSizedFlag(!lx || !ly || !lz);
    dGeomMoved (g);
}

void dGeomBoxGetLengths (dGeomID g, dVector3 result)
{
    dUASSERT (g && g->type == dBoxClass,"argument not a box");
    dxBox *b = (dxBox*) g;
    dCopyScaledVector3r4(result, b->halfside, 2.0f);
}

dReal dGeomBoxPointDepth (dGeomID g, dReal x, dReal y, dReal z)
{
    dUASSERT (g && g->type == dBoxClass,"argument not a box");

    // Set p = (x,y,z) relative to box center
    dVector3 p;
    p[0] = x;
    p[1] = y;
    p[2] = z;
    
    dxPosR *box_posr = g->GetRecomputePosR();
    dSubtractVectors3r4(p, box_posr->pos);

    // Rotate p into box's coordinate frame, so we can
    // treat the OBB as an AABB
    dVector3 q;
    dMultiply1_331(q, box_posr->R, p);
    dFabsVector3r4(p, q);
    dSubtractVectors3r4(q, ((dxBox*)g)->halfside, p);

    int   i;
    dReal tmp;
    dReal in_dist = dInfinity;
    dReal out_dist = -dInfinity;
    bool inside = true;

    for (i=0; i < 3; ++i)
    {
        tmp = q[i];
        if(tmp < 0)
        {
            inside = false;
            if(tmp > out_dist)
                out_dist = tmp;
        }
        else if (tmp < in_dist)
            in_dist = tmp;
    }

    // If point is inside the box, the depth is the smallest positive distance
    // to any side

    if (inside)
        return in_dist;

    // Otherwise, if point is outside the box, the depth is the largest
    // distance to any side.  This is an approximation to the 'proper'
    // solution (the proper solution may be larger in some cases).
    return out_dist;
}

//****************************************************************************
// box-box collision utility


// find all the intersection points between the 2D rectangle with vertices
// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//
// the intersection points are returned as x,y pairs in the 'ret' array.
// the number of intersection points is returned by the function (this will
// be in the range 0 to 8).

static int intersectRectQuad(dReal h[2], dReal p[8], dReal ret[16])
{
    // q (and r) contain nq (and nr) coordinate points for the current (and
    // chopped) polygons
    int nq = 4, nr;
    dReal buffer[16];
    dReal *q = p;
    dReal *r = ret;
    int otherdir;
    dReal hdir;
    for (int dir = 0; dir <= 1; dir++)
    {
        otherdir = 1 - dir;
        hdir = h[dir];
        // direction notation: xy[0] = x axis, xy[1] = y axis
        for (int sign=-1; sign <= 1; sign += 2)
        {
            // chop q along the line xy[dir] = sign*h[dir]
            dReal *pq = q;
            dReal *pr = r;
            nr = 0;
            dReal signhdir = sign * hdir;

            for (int i = nq; i > 0; i--)
            {
                // go through all points in q and all lines between adjacent points
                if (sign * pq[dir] < hdir)
                {
                    // this point is inside the chopping line
                    pr[0] = pq[0];
                    pr[1] = pq[1];
                    pr += 2;
                    ++nr;
                    if (nr & 8) {
                        q = r;
                        goto done;
                    }
                }
                dReal *nextq = (i > 1) ? pq + 2 : q;
                if ((sign * pq[dir] < hdir) ^ (sign * nextq[dir] < hdir))
                {
                    // this line crosses the chopping line
                    pr[otherdir] = pq[otherdir] + (nextq[otherdir] - pq[otherdir]) /
                        (nextq[dir] - pq[dir]) * (signhdir - pq[dir]);
                    pr[dir] = signhdir;
                    pr += 2;
                    ++nr;
                    if (nr & 8)
                    {
                        q = r;
                        goto done;
                    }
                }
                pq += 2;
            }
            q = r;
            r = (q == ret) ? buffer : ret;
            nq = nr;
        }
    }
done:
    if (q != ret)
        memcpy (ret, q, nr * 2 * sizeof(dReal));
    return nr;
}


// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].

void cullPoints (int n, dReal p[], int m, int i0, int iret[])
{
    // compute the centroid of the polygon in cx,cy
    int i,j;
    dReal a,cx,cy,q, pj, pj1, pj2, pj3, pn21, pn22;
    if (n==1)
    {
        cx = p[0];
        cy = p[1];
    }
    else if (n ==2)
    {
        cx = REAL(0.5) * (p[0] + p[2]);
        cy = REAL(0.5) * (p[1] + p[3]);
    }
    else {
        a = 0;
        cx = 0;
        cy = 0;
        for (i = 0; i < (n - 1); i++)
        {
            j = 2 * i;
            pj = p[j++];
            pj1 = p[j++];
            pj2 = p[j++];
            pj3 = p[j];
            q = pj * pj3 - pj2 * pj1;
            cx += q * (pj + pj2);
            cy += q * (pj1 + pj3);
            a += q;
        }
        i = 2 * n;
        pn22 = p[i - 2];
        pn21 = p[i - 1];
        q = pn22 * p[1] - p[0] * pn21 ;
        a = dRecip(REAL(3.0) * (a + q));
        cx = a * (cx + q * (pn22 + p[0]));
        cy = a * (cy + q * (pn21 + p[1]));
    }

    // compute the angle of each point w.r.t. the centroid
    int avail[8];
    dReal A[8];
    for (i = 0; i < n; i++)
    {
        j = 2 * i;
        pj = p[j++] - cx;
        pj1 = p[j] - cy;
        A[i] = dAtan2(pj1, pj);
    }

    // search for points that have angles closest to A[i0] + i*(2*pi/m).
    for (i = 0; i < n; i++)
        avail[i] = 1;

    avail[i0] = 0;
    iret[0] = i0;
    iret++;
    
    dReal astep = REAL(2.0) * M_PI / m;
    dReal base = A[i0];
    for (j = 1; j < m; j++)
    {
        a = dReal(j) * astep + base;
        if (a > M_PI)
            a -= (dReal)(2 * M_PI);
        dReal maxdiff=1e9,diff;
#ifndef dNODEBUG
        *iret = i0;			// iret is not allowed to keep this value
#endif
        for (i = 0; i < n; i++) 
        {
            if (avail[i])
            {
                diff = dFabs (A[i] - a);
                if (diff > M_PI)
                    diff = (dReal) (2 * M_PI - diff);
                if (diff < maxdiff)
                {
                    maxdiff = diff;
                    *iret = i;
                }
            }
        }
#ifndef dNODEBUG
        dIASSERT (*iret != i0);	// ensure iret got set
#endif
        avail[*iret] = 0;
        iret++;
    }
}


// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `return_code' returns a number indicating the type of contact that was
// detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.


int dBoxBox (const dVector3 p1, const dMatrix3 R1,
             const dVector3 A, const dVector3 p2,
             const dMatrix3 R2, const dVector3 B,
             dVector3 normal, dReal *depth,
             int flags, dContactGeom *contact, int skip)
{
    const dReal fudge_factor = REAL(1.2);
    dVector3 p, pp, normalC = {0, 0, 0};
    const dReal *normalR = 0;

    dVector3 R1x;
    dVector3 R2x;
    dVector3 R3x;

    dVector3 Q1x;
    dVector3 Q2x;
    dVector3 Q3x;

#define R11 R1x[0]
#define R12 R1x[1]
#define R13 R1x[2]
#define R21 R2x[0]
#define R22 R2x[1]
#define R23 R2x[2]
#define R31 R3x[0]
#define R32 R3x[1]
#define R33 R3x[2]

#define Q11 Q1x[0]
#define Q12 Q1x[1]
#define Q13 Q1x[2]
#define Q21 Q2x[0]
#define Q22 Q2x[1]
#define Q23 Q2x[2]
#define Q31 Q3x[0]
#define Q32 Q3x[1]
#define Q33 Q3x[2]

    dReal s, s2, l, expr1_val,expr2_val;
    int i, j, invert_normal, code;
    dReal tmp;
    // get vector from centers of box 1 to box 2, relative to box 1
    dSubtractVectors3r4(p, p2, p1);
    dMultiply1_331 (pp, R1, p);		// get pp = p relative to body 1

    // for all 15 possible separating axes:
    //   * see if the axis separates the boxes. if so, return 0.
    //   * find the depth of the penetration along the separating axis (s2)
    //   * if this is the largest depth so far, record it.
    // the normal vector will be set to the separating axis with the smallest
    // depth. note: normalR is set to point to a column of R1 or R2 if that is
    // the smallest depth normal so far. otherwise normalR is 0 and normalC is
    // set to a vector relative to body 1. invert_normal is 1 if the sign of
    // the normal should be flipped.

    do {
#define TST(expr1,expr2,norm,cc) \
    expr1_val = (expr1); /* Avoid duplicate evaluation of expr1 */ \
    expr2_val = (expr2); \
    s2 = dFabs(expr1_val) - (expr2_val); \
    if (s2 > 0) \
        return 0; \
    if (s2 > s) \
    { \
        s = s2; \
        normalR = norm; \
        invert_normal = ((expr1_val) < 0); \
        code = (cc); \
        if (flags & CONTACTS_UNIMPORTANT) break; \
    }

        s = -dInfinity;
        invert_normal = 0;
        code = 0;

        // Rij is R1'*R2, i.e. the relative rotation between R1 and R2

        const dReal *r1ptr = R1;
        const dReal *r2ptr = R2;
        R11 = dCalcVectorDot3_44(r1ptr, r2ptr++);
        R12 = dCalcVectorDot3_44(r1ptr, r2ptr++);
        R13 = dCalcVectorDot3_44(r1ptr, r2ptr);
        dFabsVector3r4(Q1x, R1x);

        tmp = A[0] + dCalcVectorDot3(B, Q1x);
        // separating axis = u1,u2,u3
        TST (pp[0], tmp, r1ptr, 1);

        r1ptr++;
        r2ptr = R2;
        R21 = dCalcVectorDot3_44(r1ptr, r2ptr++);
        R22 = dCalcVectorDot3_44(r1ptr, r2ptr++);
        R23 = dCalcVectorDot3_44(r1ptr, r2ptr);

        dFabsVector3r4(Q2x, R2x);
        tmp = A[1] + dCalcVectorDot3(B, Q2x);

        TST(pp[1], tmp, r1ptr, 2);

        r1ptr++;
        r2ptr = R2;
        R31 = dCalcVectorDot3_44(r1ptr, r2ptr++);
        R32 = dCalcVectorDot3_44(r1ptr, r2ptr++);
        R33 = dCalcVectorDot3_44(r1ptr, r2ptr);
        dFabsVector3r4(Q3x, R3x);

        tmp = A[2] + dCalcVectorDot3(B, Q3x);

        TST(pp[2], tmp, r1ptr, 3);

        // separating axis = v1,v2,v3
        r2ptr = R2;
        TST(dCalcVectorDot3_41(r2ptr, p),(A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]), r2ptr, 4);
        r2ptr++;
        TST(dCalcVectorDot3_41(r2ptr, p),(A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1]), r2ptr, 5);
        r2ptr++;
        TST(dCalcVectorDot3_41(r2ptr, p),(A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2]), r2ptr, 6);

        // note: cross product axes need to be scaled when s is computed.
        // normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1, expr2, n2, n3, cc) \
    expr1_val = (expr1); /* Avoid duplicate evaluation of expr1 */ \
    expr2_val = (expr2); /* Avoid duplicate evaluation of expr1 */ \
    s2 = dFabs(expr1_val) - (expr2_val); \
    if (s2 > 0) \
        return 0; \
    l = (n2)*(n2) + (n3)*(n3); \
    if (l > dEpsilon) \
    { \
        l = dRecipSqrt(l);\
        s2 *= l; \
        if (s2 * fudge_factor > s) \
            { \
            s = s2; \
            normalR = 0; \
            normalC[0] = 0; \
            normalC[1] = (n2) * l; \
            normalC[2] = (n3) * l; \
            invert_normal = ((expr1_val) < 0); \
            code = (cc); \
            if (flags & CONTACTS_UNIMPORTANT) break; \
            } \
    }

        // We only need to check 3 edges per box 
        // since parallel edges are equivalent.

        // separating axis = u1 x (v1,v2,v3)
        TST(pp[2] * R21 - pp[1] * R31, (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12), -R31, R21, 7);
        TST(pp[2] * R22 - pp[1] * R32, (A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11), -R32, R22, 8);
        TST(pp[2] * R23 - pp[1] * R33, (A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11), -R33, R23, 9);

#undef TST
#define TST(expr1, expr2, n1, n3, cc) \
    expr1_val = (expr1); /* Avoid duplicate evaluation of expr1 */ \
    expr2_val = (expr2); /* Avoid duplicate evaluation of expr1 */ \
    s2 = dFabs(expr1_val) - (expr2_val); \
    if (s2 > 0) \
        return 0; \
    l = (n1)*(n1) + (n3)*(n3); \
    if (l > dEpsilon) \
    { \
        l = dRecipSqrt(l); \
        s2 *= l; \
        if (s2 * fudge_factor > s) \
        { \
            s = s2; \
            normalR = 0; \
            normalC[0] = (n1) * l; \
            normalC[1] = 0;  \
            normalC[2] = (n3) * l; \
            invert_normal = ((expr1_val) < 0); \
            code = (cc); \
            if (flags & CONTACTS_UNIMPORTANT) break; \
        } \
    }

        // separating axis = u2 x (v1,v2,v3)
        TST(pp[0] * R31 - pp[2] * R11, (A[0] * Q31 + A[2] *Q11 + B[1] * Q23 + B[2] * Q22), R31, -R11, 10);
        TST(pp[0] * R32 - pp[2] * R12, (A[0] * Q32 + A[2] *Q12 + B[0] * Q23 + B[2] * Q21), R32, -R12, 11);
        TST(pp[0] * R33 - pp[2] * R13, (A[0] * Q33 + A[2] *Q13 + B[0] * Q22 + B[1] * Q21), R33, -R13, 12);

#undef TST
#define TST(expr1, expr2, n1, n2, cc) \
    expr1_val = (expr1); /* Avoid duplicate evaluation of expr1 */ \
    expr2_val = (expr2); /* Avoid duplicate evaluation of expr1 */ \
    s2 = dFabs(expr1_val) - (expr2_val); \
    if (s2 > 0) \
        return 0; \
    l = (n1)*(n1) + (n2)*(n2); \
    if (l > dEpsilon) \
    { \
        l = dRecipSqrt(l);\
        s2 *= l; \
        if (s2 * fudge_factor > s) \
        { \
            s = s2; \
            normalR = 0; \
            normalC[0] = (n1) * l; \
            normalC[1] = (n2) * l; \
            normalC[2] = 0; \
            invert_normal = ((expr1_val) < 0); \
            code = (cc); \
            if (flags & CONTACTS_UNIMPORTANT) break; \
        } \
    }

        // separating axis = u3 x (v1,v2,v3)
        TST(pp[1] * R11 - pp[0] * R21, (A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32), -R21, R11, 13);
        TST(pp[1] * R12 - pp[0] * R22, (A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31), -R22, R12, 14);
        TST(pp[1] * R13 - pp[0] * R23, (A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31), -R23, R13, 15);
#undef TST
    } while (0);

    if (!code)
        return 0;

    // if we get to this point, the boxes interpenetrate. compute the normal
    // in global coordinates.
    if (normalR) {
        normal[0] = normalR[0];
        normal[1] = normalR[4];
        normal[2] = normalR[8];
    }
    else
    {
        dMultiply0_331 (normal, R1, normalC);
    }

    if (invert_normal) {
        dNegateVector3r4(normal);
    }
    *depth = -s;

    // compute contact point(s)

    if (code > 6)
    {
        // An edge from box 1 touches an edge from box 2.
        // find a point pa on the intersecting edge of box 1
        dVector3 pa;
        // Copy p1 into pa
        dCopyVector3r4(pa, p1);
        // Get world position of p2 into pa
        for (j = 0; j < 3; j++)
        {
            if(dCalcVectorDot3_14(normal, R1 + j) > 0)
            {
                for (i=0; i < 3; i++)
                    pa[i] += A[j] * R1[i * 4 + j];
            }
            else
            {
                for (i=0; i < 3; i++)
                    pa[i] -= A[j] * R1[i * 4 + j];
            }
        }

        // find a point pb on the intersecting edge of box 2
        dVector3 pb;
        // Copy p2 into pb
        dCopyVector3r4(pb, p2);
        // Get world position of p2 into pb
        for (j = 0; j < 3; j++)
        {
            if(dCalcVectorDot3_14(normal, R2 + j) < 0) // oposite sign
            {
                for (i=0; i < 3; i++)
                    pb[i] += B[j] * R2[i * 4 + j];
            }
            else
            {
                for (i=0; i < 3; i++)
                    pb[i] -= B[j] * R2[i * 4 + j];
            }
        }

        dReal alpha, beta;
        dVector3 ua, ub;
        // Get direction of first edge
        int nc = code - 7;
        int ja = nc / 3;
        // Get direction of second edge
        int jb = nc - 3 * ja;
        for (i = 0; i < 3; i++)
        {
            j = 4 * i;
            ua[i] = R1[ja + j];
            ub[i] = R2[jb + j];
        }
        // Get closest points between edges (one at each)
        dLineClosestApproach(pa, ua, pb, ub, &alpha, &beta);    

        dAddScaledVector3r4(pb, ub, beta);
        dAddScaledVector3r4(pa, ua, alpha);
        dAddVector3r4(pa, pb);
        dCopyScaledVector3r4(contact[0].pos, pa, REAL(0.5));
        contact[0].depth = *depth;
    
        return 1;
    }

    // okay, we have a face-something intersection (because the separating
    // axis is perpendicular to a face). define face 'a' to be the reference
    // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
    // the incident face (the closest face of the other box).
    // Note: Unmodified parameter values are being used here
    // nr = normal vector of reference face dotted with axes of incident box.
    // anr = absolute values of nr.
    /*
    The normal is flipped if necessary so it always points outward from box 'a',
    box 'b' is thus always the incident box
    */
    const dReal *Ra, *Rb, *pa, *pb, *Sa, *Sb;
    dVector3 normal2, nr, anr;
    int codeN, code1, code2;

    if (code <= 3)
    { // One of the faces of box 1 is the reference face
        Ra = R1; // Rotation of 'a'
        Rb = R2; // Rotation of 'b'
        pa = p1; // Center (location) of 'a'
        pb = p2; // Center (location) of 'b'
        Sa = A;  // Side Lenght of 'a'
        Sb = B;  // Side Lenght of 'b'
        dCopyVector3r4(normal2, normal);
        codeN = code - 1;
    }
    else
    { // One of the faces of box 2 is the reference face
        Ra = R2; // Rotation of 'a'
        Rb = R1; // Rotation of 'b'
        pa = p2; // Center (location) of 'a'
        pb = p1; // Center (location) of 'b'
        Sa = B;  // Side Lenght of 'a'
        Sb = A;  // Side Lenght of 'b'
        dCopyNegatedVector3r4(normal2, normal);
        codeN = code - 4;
    }
    // Rotate normal2 in incident box opposite direction
    dMultiply1_331 (nr, Rb, normal2);
    dFabsVector3r4(anr, nr);

    // find the largest compontent of anr: this corresponds to the normal
    // for the incident face. the other axis numbers of the incident face
    // are stored in a1,a2.
    int lanr,a1,a2;
    if (anr[1] > anr[0])
    {
        if (anr[1] > anr[2])
        {
            a1 = 0;
            lanr = 1;
            a2 = 2;
        }
        else
        {
            a1 = 0;
            a2 = 1;
            lanr = 2;
        }
    }
    else
    {
        if (anr[0] > anr[2])
        {
            lanr = 0;
            a1 = 1;
            a2 = 2;
        }
        else
        {
            a1 = 0;
            a2 = 1;
            lanr = 2;
        }
    }

    // compute center point of incident face, in reference-face coordinates
    dVector3 center;
    if (nr[lanr] < 0)
    {
        for (i = 0; i < 3; i++)
            center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i * 4 + lanr];
    }
    else
    {
        for (i = 0; i < 3; i++)
            center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i * 4 + lanr];
    }

    // find the normal and non-normal axis numbers of the reference box
    if (codeN == 0)
    {
        code1 = 1;
        code2 = 2;
    }
    else if (codeN == 1)
    {
        code1 = 0;
        code2 = 2;
    }
    else
    {
        code1 = 0;
        code2 = 1;
    }

    // find the four corners of the incident face, in reference-face coordinates
    dReal quad[8];	// 2D coordinate of incident face (x,y pairs)
    dReal c1, c2, m11, m12, m21, m22;
    const dReal *racode1 = Ra + code1;
    const dReal *racode2 = Ra + code2;
    c1 = dCalcVectorDot3_14(center, racode1);
    c2 = dCalcVectorDot3_14(center, racode2);
    // optimize this? - we have already computed this data above, but it is not
    // stored in an easy-to-index format. for now it's quicker just to recompute
    // the four dot products.
    const dReal *rba1 = Rb + a1;
    const dReal *rba2 = Rb + a2;

    m11 = dCalcVectorDot3_44(racode1, rba1);
    m12 = dCalcVectorDot3_44(racode1, rba2);
    m21 = dCalcVectorDot3_44(racode2, rba1);
    m22 = dCalcVectorDot3_44(racode2, rba2);
    dReal k1 = m11 * Sb[a1];
    dReal k2 = m21 * Sb[a1];
    dReal k3 = m12 * Sb[a2];
    dReal k4 = m22 * Sb[a2];
    quad[0] = c1 - k1 - k3;
    quad[1] = c2 - k2 - k4;
    quad[2] = c1 - k1 + k3;
    quad[3] = c2 - k2 + k4;
    quad[4] = c1 + k1 + k3;
    quad[5] = c2 + k2 + k4;
    quad[6] = c1 + k1 - k3;
    quad[7] = c2 + k2 - k4;

    // find the size of the reference face
    dReal rect[2];
    rect[0] = Sa[code1];
    rect[1] = Sa[code2];

    // intersect the incident and reference faces
    dReal ret[16];
    int n = intersectRectQuad(rect, quad, ret);
    if (n < 1)
        return 0;		// this should never happen

    // convert the intersection points into reference-face coordinates,
    // and compute the contact position and depth for each point. only keep
    // those points that have a positive (penetrating) depth. delete points in
    // the 'ret' array as necessary so that 'point' and 'ret' correspond.
    dReal point[3*8];		// penetrating contact points
    dReal dep[8];			// depths for those points
    dReal det1 = dRecip(m11 * m22 - m12 * m21);
    m11 *= det1;
    m12 *= det1;
    m21 *= det1;
    m22 *= det1;
    int cnum = 0;			// number of penetrating contact points found
    int maxc = flags & NUMC_MASK;

    for (j=0; j < n; j++)
    {
        dReal k1 =  m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
        dReal k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
        for (i = 0; i < 3; i++)
            point[cnum * 3 + i] = center[i] + k1 * Rb[i * 4 + a1] + k2 * Rb[i * 4 + a2];
        dep[cnum] = Sa[codeN] - dCalcVectorDot3(normal2, point + cnum * 3);
        if (dep[cnum] >= 0)
        {
            ret[cnum * 2] = ret[j * 2];
            ret[cnum * 2 + 1] = ret[j * 2 + 1];
            cnum++;
            if (cnum  == maxc)
                break;
        }
    }
    if (cnum < 1)
    { 
        return 0;	// this should not happen, yet does at times (demo_plane2d single precision).
    }

    // we can't generate more contacts than we actually have
    if (maxc > cnum)
        maxc = cnum;
    if (maxc < 1)
        maxc = 1;	// Even though max count must not be zero this check is kept for backward compatibility as this is a public function

    if (cnum <= maxc)
    {
        // we have less contacts than we need, so we use them all
        for (j=0; j < cnum; j++)
        {
            dContactGeom *con = CONTACT(contact, skip * j);
            dAddVectors3r4(con->pos, &point[j * 3], pa);
            con->depth = dep[j];
        }
    }
    else
    {
        dIASSERT(!(flags & CONTACTS_UNIMPORTANT)); // cnum should be generated not greater than maxc so that "then" clause is executed
        // we have more contacts than are wanted, some of them must be culled.
        // find the deepest point, it is always the first contact.
        int i1 = 0;
        dReal maxdepth = dep[0];
        for (i=1; i<cnum; i++)
        {
            if (dep[i] > maxdepth)
            {
                maxdepth = dep[i];
                i1 = i;
            }
        }

        int iret[8];
        cullPoints (cnum, ret, maxc, i1, iret);

        for (j = 0; j < maxc; j++)
        {
            dContactGeom *con = CONTACT(contact,skip * j);
            dAddVectors3r4(con->pos, &point[iret[j] * 3], pa);
            con->depth = dep[iret[j]];
        }
        cnum = maxc;
    }

    return cnum;
}

int dCollideBoxBox (dxGeom *o1, dxGeom *o2, int flags,
                    dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dBoxClass);
    dIASSERT (o2->type == dBoxClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dVector3 normal;
    dReal depth;
    dxBox *b1 = (dxBox*) o1;
    dxBox *b2 = (dxBox*) o2;
    dxPosR *posr1 = o1->GetRecomputePosR();
    dxPosR *posr2 = o2->GetRecomputePosR();
    int num = dBoxBox (posr1->pos, posr1->R, b1->halfside,
                    posr2->pos, posr2->R, b2->halfside,
                    normal, &depth, flags, contact, skip);
    for (int i=0; i<num; i++)
    {
        dContactGeom *currContact = CONTACT(contact,i * skip);
        dCopyNegatedVector3r4(currContact->normal, normal);
        currContact->g1 = o1;
        currContact->g2 = o2;
        currContact->side1 = -1;
        currContact->side2 = -1;
    }
    return num;
}

int dCollideBoxPlane(dxGeom *o1, dxGeom *o2,
    int flags, dContactGeom *contact, int skip)
{
    dIASSERT(skip >= (int)sizeof(dContactGeom));
    dIASSERT(o1->type == dBoxClass);
    dIASSERT(o2->type == dPlaneClass);
    dIASSERT((flags & NUMC_MASK) >= 1);

    dVector3 Q, A, B;
    dxBox *box = (dxBox*)o1;
    dxPlane *plane = (dxPlane*)o2;

    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;

    int ret = 0;

    //@@@ problem: using 4-vector (plane->p) as 3-vector (normal).
    const dReal *R = o1->final_posr->R;		// rotation of box
    const dReal *n = plane->p;		// normal vector

    // project sides lengths along normal vector, get absolute values
    Q[0] = dCalcVectorDot3_14(n, R);
    Q[1] = dCalcVectorDot3_14(n, R + 1);
    Q[2] = dCalcVectorDot3_14(n, R + 2);
    dMultVectors3r4(A, box->halfside, Q);
    dFabsVector3(B, A);

    // early exit test
    dReal depth = plane->p[3] + (B[0] + B[1] + B[2]) - dCalcVectorDot3(n, o1->final_posr->pos);
    if (depth < 0)
        return 0;

    // find number of contacts requested
    int maxc = flags & NUMC_MASK;
    // if (maxc < 1) maxc = 1; // an assertion is made on entry
    if (maxc > 4) maxc = 4;	// not more than 4 contacts per box allowed

    // find deepest point
    dVector3 p;
    dCopyVector3r4(p, o1->final_posr->pos);

#define FOO(i,op) \
    p[0] op box->halfside[i] * R[0+i]; \
    p[1] op box->halfside[i] * R[4+i]; \
    p[2] op box->halfside[i] * R[8+i];

#define BAR(i) if (A[i] > 0) { FOO(i,-=) } else { FOO(i,+=) }
    BAR(0);
    BAR(1);
    BAR(2);
#undef FOO
#undef BAR

    // the deepest point is the first contact point
    dCopyVector3r4(contact->pos, p);
    contact->depth = depth;
    ret = 1;		// ret is number of contact points found so far
    if (maxc == 1) goto done;

    // get the second and third contact points by starting from `p' and going
    // along the two sides with the smallest projected length.

#define FOO(i,j,op) \
    CONTACT(contact,i*skip)->pos[0] = p[0] op box->halfside[j] * R[j]; \
    CONTACT(contact,i*skip)->pos[1] = p[1] op box->halfside[j] * R[4+j]; \
    CONTACT(contact,i*skip)->pos[2] = p[2] op box->halfside[j] * R[8+j];
#define BAR(ctact,side,sideinc) \
    if (depth - B[sideinc] < 0) goto done; \
    if (A[sideinc] > 0) { FOO(ctact,side,+); } else { FOO(ctact,side,-); } \
    CONTACT(contact,ctact*skip)->depth = depth - B[sideinc]; \
    ret++;

    if (B[0] < B[1])
    {
        if (B[2] < B[0])
            goto use_side_3;
        else
        {
            BAR(1,0,0);	// use side 1
            if (maxc == 2) goto done;
            if (B[1] < B[2])
                goto contact2_2;
            else goto contact2_3;
        }
    }
    else {
        if (B[2] < B[1]) {
use_side_3:	// use side 3
            BAR(1,2,2);
            if (maxc == 2)
                goto done;
            if (B[0] < B[1])
                goto contact2_1;
            else goto contact2_2;
        }
        else {
            BAR(1,1,1);	// use side 2
            if (maxc == 2) goto done;
            if (B[0] < B[2])
                goto contact2_1;
            else goto contact2_3;
        }
    }

contact2_1: BAR(2,0,0); goto done;
contact2_2: BAR(2,1,1); goto done;
contact2_3: BAR(2,2,2); goto done;
#undef FOO
#undef BAR

done:

    if (maxc == 4 && ret == 3)
    { // If user requested 4 contacts, and the first 3 were created...
        // Combine contacts 2 and 3 (vectorial sum) and get the fourth one
        // Result: if a box face is completely inside a plane, contacts are created for all the 4 vertices
        dReal d4 = CONTACT(contact, skip)->depth + CONTACT(contact,2 * skip)->depth - depth;  // depth is the depth for first contact
        if (d4 > 0)
        {
            dContactGeom *target = CONTACT(contact, 3 * skip);
            dAddVectors3r4(target->pos, CONTACT(contact, skip)->pos, CONTACT(contact, 2 * skip)->pos);
            dSubtractVectors3r4(target->pos, p);
            target->depth  = d4;
            ret++;
        }
    }

    for (int i=0; i<ret; i++)
    {
        dContactGeom *currContact = CONTACT(contact,i * skip);
        currContact->g1 = o1;
        currContact->g2 = o2;
        currContact->side1 = -1;
        currContact->side2 = -1;

        dCopyVector3r4(currContact->normal, n);
    }
    return ret;
}
