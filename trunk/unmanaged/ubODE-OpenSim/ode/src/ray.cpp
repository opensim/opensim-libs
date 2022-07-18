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
// ray public API

dxRay::dxRay (dSpaceID space, dReal _length) : dxGeom (space,1)
{
    type = dRayClass;
    length = _length;
}

void dxRay::computeAABB()
{
    dReal *pos = final_posr->pos;

    dReal e = final_posr->R[2] * length;
    if (e >= 0)
    {
        aabb[0] = pos[0];
        aabb[1] = pos[0] + e;
    }
    else
    {
        aabb[0] = pos[0] + e;
        aabb[1] = pos[0];
    }

    e = final_posr->R[6] * length;
    if (e >= 0)
    {
        aabb[2] = pos[1];
        aabb[3] = pos[1] + e;
    }
    else
    {
        aabb[2] = pos[1] + e;
        aabb[3] = pos[1];
    }

    e = final_posr->R[10] * length;
    if (e >= 0)
    {
        aabb[4] = pos[2];
        aabb[5] = pos[2] + e;
    }
    else
    {
        aabb[4] = pos[2] + e;
        aabb[5] = pos[2];
    }
}


dGeomID dCreateRay (dSpaceID space, dReal length)
{
    return new dxRay (space, length);
}


void dGeomRaySetLength (dGeomID g, dReal length)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    ((dxRay*)g)->length = length;
    dGeomMoved (g);
}


dReal dGeomRayGetLength (dGeomID g)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    return ((dxRay*)g)->length;
}


void dGeomRaySet (dGeomID g, dReal px, dReal py, dReal pz,
                  dReal dx, dReal dy, dReal dz)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");

    dxPosR *rayPosR = g->final_posr;

    dReal* pos = rayPosR->pos;
    pos[0] = px;
    pos[1] = py;
    pos[2] = pz;

    dVector3 n;
    n[0] = dx;
    n[1] = dy;
    n[2] = dz;
    dNormalize3(n);

    dReal* R = rayPosR->R;
    R[2] = n[0];
    R[6] = n[1];
    R[10] = n[2];

    dGeomMoved (g);
}

void dGeomRayGet (dGeomID g, dVector3 start, dVector3 dir)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    dxPosR *dpr = g->GetRecomputePosR();
    const dReal* R = dpr->R;
    dir[0] = R[2];
    dir[1] = R[6];
    dir[2] = R[10];
    const dReal* pos = dpr->pos;
    dCopyVector3r4(start, pos);
}

void dGeomRaySetParams (dxGeom *g, int FirstContact, int BackfaceCull)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");

    dGeomRaySetFirstContact(g, FirstContact);
    dGeomRaySetBackfaceCull(g, BackfaceCull);
}


void dGeomRayGetParams (dxGeom *g, int *FirstContact, int *BackfaceCull)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");

    (*FirstContact) = ((g->gflags & RAY_FIRSTCONTACT) != 0);
    (*BackfaceCull) = ((g->gflags & RAY_BACKFACECULL) != 0);
}


// set/get backface culling flag
void dGeomRaySetBackfaceCull (dxGeom *g, int backfaceCull) 
{
    
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    if (backfaceCull) {
        g->gflags |= RAY_BACKFACECULL;
    } else {
        g->gflags &= ~RAY_BACKFACECULL;
    }
}

int dGeomRayGetBackfaceCull (dxGeom *g)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    return ((g->gflags & RAY_BACKFACECULL) != 0);
}

// set/get first contact flag
void dGeomRaySetFirstContact (dxGeom *g, int firstContact)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    if (firstContact) {
        g->gflags |= RAY_FIRSTCONTACT;
    } else {
        g->gflags &= ~RAY_FIRSTCONTACT;
    }
}

int dGeomRayGetFirstContact (dxGeom *g)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    return ((g->gflags & RAY_FIRSTCONTACT) != 0);
}

void dGeomRaySetClosestHit (dxGeom *g, int closestHit)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    if (closestHit){
        g->gflags |= RAY_CLOSEST_HIT;
    }
    else g->gflags &= ~RAY_CLOSEST_HIT;
}

int dGeomRayGetClosestHit (dxGeom *g)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    return ((g->gflags & RAY_CLOSEST_HIT) != 0);
}

// if mode==1 then use the sphere exit contact, not the entry contact

static int ray_sphere_helper (const dxRay *ray, const dVector3 sphere_pos, const dReal radius,
        dContactGeom *contact, const int mode)
{
    dVector3 q;

    const dReal* pos = ray->final_posr->pos;
    const dReal* R = ray->final_posr->R;

    dSubtractVectors3r4(q, pos, sphere_pos);

    const dReal B = dCalcVectorDot3_14(q, R + 2);
    const dReal C = dCalcVectorLengthSquare3(q) - radius * radius;
    // note: if C <= 0 then the start of the ray is inside the sphere
    dReal k = B * B - C;
    if (k < 0)
        return 0;
    k = dSqrt(k);
    dReal alpha;
    if (mode && C >= 0)
    {
        alpha = -B + k;
        if (alpha < 0)
            return 0;
    }
    else
    {
        alpha = -B - k;
        if (alpha < 0)
        {
            alpha = -B + k;
            if (alpha < 0)
                return 0;
        }
    }
    if (alpha > ray->length)
        return 0;

    contact->pos[0] = pos[0] + alpha * R[2];
    contact->pos[1] = pos[1] + alpha * R[6];
    contact->pos[2] = pos[2] + alpha * R[10];

    if(C < 0 || mode)
    {
        dSubtractVectors3r4(contact->normal, sphere_pos, contact->pos);
    }
    else
    {
        dSubtractVectors3r4(contact->normal, contact->pos, sphere_pos);
    }
    dNormalize3 (contact->normal);
    contact->depth = alpha;
    return 1;
}


int dCollideRaySphere (dxGeom *o1, dxGeom *o2, const int flags,
        dContactGeom *contact, const int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dRayClass);
    dIASSERT (o2->type == dSphereClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dxPosR *dpr = o1->final_posr;
    const dReal* pos = dpr->pos;
    const dReal* R = dpr->R;

    dpr = o2->final_posr;
    const dReal* sphere_pos = dpr->pos;

    dVector3 q;
    dSubtractVectors3r4(q, pos, sphere_pos);

    const dReal radius = ((dxSphere*)o2)->radius;
    const dReal C = dCalcVectorLengthSquare3(q) - radius * radius;

    //if C <= 0 then the start of the ray is inside the sphere
    if (C <= 0 && (o1->gflags & RAY_BACKFACECULL) != 0)
        return 0;

    const dReal B = dCalcVectorDot3_14(q, R + 2);
    dReal k = B * B - C;
    if (k < 0)
        return 0;
    k = dSqrt(k);
    dReal alpha = -B - k;
    if (alpha < 0)
    {
        alpha = -B + k;
        if (alpha < 0)
            return 0;
    }
    if (alpha > ((dxRay*)o1)->length)
        return 0;

    contact->pos[0] = pos[0] + alpha * R[2];
    contact->pos[1] = pos[1] + alpha * R[6];
    contact->pos[2] = pos[2] + alpha * R[10];

    if (C < 0)
        dSubtractVectors3r4(contact->normal, sphere_pos, contact->pos);
    else
        dSubtractVectors3r4(contact->normal, contact->pos, sphere_pos);

    dNormalize3(contact->normal);
    contact->depth = alpha;
    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;

    return 1;
}


int dCollideRayBox (dxGeom *o1, dxGeom *o2, const int flags,
            dContactGeom *contact, const int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dRayClass);
    dIASSERT (o2->type == dBoxClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dxPosR *dpr = o1->final_posr;
    const dReal* pos = dpr->pos;
    const dReal* R = dpr->R;

    dpr = o2->final_posr;
    const dReal* boxpos = dpr->pos;
    const dReal* boxR = dpr->R;

    dVector3 tmp;
    tmp[0] = R[2];
    tmp[1] = R[6];
    tmp[2] = R[10];

    dVector3 v;
    dMultiply1_331(v, boxR, tmp);
    if (v[0] == 0 && v[1] == 0 && v[2] == 0)
        return 0;

    // compute the start and delta of the ray relative to the box.
    // we will do all subsequent computations in this box-relative coordinate
    // system. we have to do a translation and rotation for each point.
    dVector3 start;
    dSubtractVectors3r4(tmp, pos, boxpos);
    dMultiply1_331 (start, boxR, tmp);

    // mirror the line so that v has all components >= 0
    dVector3 sign;
    int i;
    for (i = 0; i < 3; ++i)
    {
        if (v[i] < 0)
        {
            start[i] = -start[i];
            v[i] = -v[i];
            sign[i] = 1;
        }
        else sign[i] = -1;
    }

     const dReal *h = ((dxBox*)o2)->halfside;

    // do a few early exit tests
    if (start[0] > h[0] || start[1] > h[1] || start[2] > h[2])
        return 0;

    if ((o1->gflags & RAY_BACKFACECULL) != 0)
    {
        if (start[0] > -h[0] && start[1] > -h[1] && start[2] > -h[2])
            return 0;
    }

    // compute the t=[lo..hi] range for where s+v*t intersects the box
    dReal lo = -dInfinity;
    dReal hi = dInfinity;
    int nlo = 0, nhi = 0;
    for (i = 0; i < 3; i++)
    {
        if (v[i] != 0)
        {
            dReal invV = REAL(1.0) / v[i];
            dReal k = -(h[i] + start[i]) * invV;
            if (k > lo)
            {
                lo = k;
                nlo = i;
            }
            k = (h[i] - start[i]) * invV;
            if (k < hi)
            {
                hi = k;
                nhi = i;
            }
        }
    }

    // check if the ray intersects
    if (lo > hi)
        return 0;
    dReal alpha;
    int n;
    if (lo >= 0)
    {
        alpha = lo;
        n = nlo;
    }
    else
    {
        alpha = hi;
        n = nhi;
    }

    if (alpha < 0 || alpha > ((dxRay*)o1)->length)
        return 0;


    contact->pos[0] = pos[0] + alpha * R[2];
    contact->pos[1] = pos[1] + alpha * R[6];
    contact->pos[2] = pos[2] + alpha * R[10];

    if(sign[n] > 0)
    {
        contact->normal[0] = boxR[n];
        contact->normal[1] = boxR[4 + n];
        contact->normal[2] = boxR[8 + n];
    }
    else
    {
        contact->normal[0] = -boxR[n];
        contact->normal[1] = -boxR[4 + n];
        contact->normal[2] = -boxR[8 + n];
    }

    contact->depth = alpha;
    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;
    return 1;
}


int dCollideRayCapsule (dxGeom *o1, dxGeom *o2, const int flags, dContactGeom *contact, const int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dRayClass);
    dIASSERT (o2->type == dCapsuleClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    const dxRay *ray = (dxRay*) o1;
    const dxCapsule *ccyl = (dxCapsule*) o2;

    const dReal radiusSQ = ccyl->radius * ccyl->radius;
    const dReal lz2 = ccyl->halfLenZ;

    dxPosR* dpr = ray->final_posr;
    const dReal* pos = dpr->pos;
    const dReal* R = dpr->R;
    dpr = ccyl->final_posr;
    const dReal* cpos = dpr->pos;
    const dReal* cR = dpr->R;

    // compute some useful info
    dVector3 cs;
    dSubtractVectors3r4(cs, pos, cpos);
    
    dReal k = dCalcVectorDot3_41(cR + 2, cs);	// position of ray start along ccyl axis
    dVector3 q;
    q[0] = k * cR[2] - cs[0];
    q[1] = k * cR[6] - cs[1];
    q[2] = k * cR[10] - cs[2];

    dReal C = dCalcVectorLengthSquare3(q) - radiusSQ;
    // if C < 0 then ray start position within infinite extension of cylinder
    // see if ray start position is inside the capped cylinder
    int inside_ccyl = 0;
    if (C < 0)
    {
        if (k < -lz2)
            k = -lz2;
        else if (k > lz2)
            k = lz2;

        if (dCalcVectorLengthSquare3(cs[0] - k * cR[2],
                                     cs[1] - k * cR[6],
                                     cs[2] - k * cR[10])
                                 < radiusSQ)
            inside_ccyl = 1;
    }

    if(inside_ccyl && ((o1->gflags & RAY_BACKFACECULL) != 0))
        return 0;

    // compute ray collision with infinite cylinder, except for the case where
    // the ray is outside the capped cylinder but within the infinite cylinder
    // (it that case the ray can only hit endcaps)
    if (!inside_ccyl && C < 0)
    {
        // set k to cap position to check
        if (k < 0)
            k = -lz2;
        else 
            k = lz2;
    }
    else
    {
        dReal uv = dCalcVectorDot3_44(cR + 2, R + 2);
        dVector3 r;
        r[0] = uv * cR[2] - R[2];
        r[1] = uv * cR[6] - R[6];
        r[2] = uv * cR[10] - R[10];
        dReal A = dCalcVectorLengthSquare3(r);
        // A == 0 means that the ray and ccylinder axes are parallel
        if (A == 0)
        { // There is a division by A below...
            // set k to cap position to check
            if (uv < 0)
                k = -lz2;
            else
                k = lz2;
        }
        else
        {
            dReal B = 2 * dCalcVectorDot3(q, r);
            k = B * B - 4 * A * C;
            if (k < 0)
            {
                // the ray does not intersect the infinite cylinder, but if the ray is
                // inside and parallel to the cylinder axis it may intersect the end
                // caps. set k to cap position to check.
                if (!inside_ccyl)
                    return 0;
                if (uv < 0)
                    k = -lz2;
                else
                    k = lz2;
            }
            else
            {
                k = dSqrt(k);
                A = dRecip (2 * A);
                dReal alpha = (-B - k) * A;
                if (alpha < 0)
                {
                    alpha = (-B + k) * A;
                    if (alpha < 0)
                        return 0;
                }
                if (alpha > ray->length)
                    return 0;

                // the ray intersects the infinite cylinder. check to see if the
                // intersection point is between the caps
                contact->pos[0] = pos[0] + alpha * R[2];
                contact->pos[1] = pos[1] + alpha * R[6];
                contact->pos[2] = pos[2] + alpha * R[10];
                dSubtractVectors3r4(q, contact->pos, cpos);
                k = dCalcVectorDot3_14(q, cR + 2);
                if (k >= -lz2 && k <= lz2)
                {
                    if(inside_ccyl)
                    {
                        contact->normal[0] = k * cR[2] - q[0];
                        contact->normal[1] = k * cR[6] - q[1];
                        contact->normal[2] = k * cR[10] - q[2];
                    }
                    else
                    {
                        contact->normal[0] = q[0] - k * cR[2];
                        contact->normal[1] = q[1] - k * cR[6];
                        contact->normal[2] = q[2] - k * cR[10];
                    }

                    dNormalize3 (contact->normal);
                    contact->depth = alpha;
                    contact->g1 = o1;
                    contact->g2 = o2;
                    contact->side1 = -1;
                    contact->side2 = -1;
                    return 1;
                }

                // the infinite cylinder intersection point is not between the caps.
                // set k to cap position to check.
                if (k < 0)
                    k = -lz2;
                else
                    k = lz2;
            }
        }
    }

    // check for ray intersection with the caps. k must indicate the cap
    // position to check
    q[0] = cpos[0] + k * cR[2];
    q[1] = cpos[1] + k * cR[6];
    q[2] = cpos[2] + k * cR[10];

    int ret = ray_sphere_helper(ray, q, ccyl->radius, contact, inside_ccyl);
    if (ret > 0)
    {
        contact->g1 = o1;
        contact->g2 = o2;
        contact->side1 = -1;
        contact->side2 = -1;
        return ret;
    }
    return 0;
}


int dCollideRayPlane (dxGeom *o1, dxGeom *o2, int flags,
                      dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dRayClass);
    dIASSERT (o2->type == dPlaneClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    const dxRay *ray = (dxRay*) o1;
    const dxPlane *plane = (dxPlane*) o2;

    const dReal* pos = ray->final_posr->pos;
    const dReal* R = ray->final_posr->R;

    const dReal k = dCalcVectorDot3_14(plane->p, R + 2);
    if (k==0)
        return 0;		// ray parallel to plane

    // note: if alpha > 0 the starting point is below the plane
    dReal alpha = plane->p[3] - dCalcVectorDot3(plane->p, pos);
    int nsign;
    if (alpha > 0)
    {
        if (k > 0 && ((o1->gflags & RAY_BACKFACECULL) != 0))
            return 0;
        nsign = -1;
    }
    else
        nsign = 1;

    alpha /= k;
    if (alpha < 0 || alpha > ray->length)
        return 0;

    contact->pos[0] = pos[0] + alpha * R[2];
    contact->pos[1] = pos[1] + alpha * R[6];
    contact->pos[2] = pos[2] + alpha * R[10];
    if(nsign > 0)
        dCopyVector3r4(contact->normal, plane->p);
    else
        dCopyNegatedVector3r4(contact->normal, plane->p);

    contact->depth = alpha;
    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;
    return 1;
}

// Ray-Cylinder collider by Joseph Cooper (2011)
int dCollideRayCylinder( dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip )
{
    dIASSERT( skip >= (int)sizeof( dContactGeom ) );
    dIASSERT( o1->type == dRayClass );
    dIASSERT( o2->type == dCylinderClass );
    dIASSERT( (flags & NUMC_MASK) >= 1 );

    dxRay* ray = (dxRay*)( o1 );
    dxCylinder* cyl = (dxCylinder*)( o2 );

    // Fill in contact information.
    contact->g1 = ray;
    contact->g2 = cyl;
    contact->side1 = -1;
    contact->side2 = -1;

    const dReal half_length = cyl->lz * REAL( 0.5 );


    /* Possible collision cases:
     *  Ray origin between/outside caps
     *  Ray origin within/outside radius
     *  Ray direction left/right/perpendicular
     *  Ray direction parallel/perpendicular/other
     * 
     *  Ray origin cases (ignoring origin on surface)
     *
     *  A          B
     *     /-\-----------\
     *  C (   )    D      )
     *     \_/___________/
     *
     *  Cases A and D can collide with caps or cylinder
     *  Case C can only collide with the caps
     *  Case B can only collide with the cylinder
     *  Case D will produce inverted normals
     *  If the ray is perpendicular, only check the cylinder
     *  If the ray is parallel to cylinder axis,
     *  we can only check caps
     *  If the ray points right,
     *    Case A,C Check left cap
     *    Case  D  Check right cap
     *  If the ray points left
     *    Case A,C Check right cap
     *    Case  D  Check left cap
     *  Case B, check only first possible cylinder collision
     *  Case D, check only second possible cylinder collision
     */
    // Find the ray in the cylinder coordinate frame:
    dVector3 tmp;
    dVector3 pos;  // Ray origin in cylinder frame
    dVector3 dir;  // Ray direction in cylinder frame
    // Translate ray start by inverse cyl
    dSubtractVectors3r4(tmp,ray->final_posr->pos,cyl->final_posr->pos);
    // Rotate ray start by inverse cyl
    dMultiply1_331(pos,cyl->final_posr->R,tmp);

    // Get the ray's direction
    tmp[0] = ray->final_posr->R[2];
    tmp[1] = ray->final_posr->R[6];
    tmp[2] = ray->final_posr->R[10];
    // Rotate the ray direction by inverse cyl
    dMultiply1_331(dir,cyl->final_posr->R,tmp); 

    // Is the ray origin inside of the (extended) cylinder?
    dReal r2 = cyl->radius*cyl->radius;
    dReal C = pos[0]*pos[0] + pos[1]*pos[1] - r2;

    // Find the different cases
    // Is ray parallel to the cylinder length?
    int parallel = (dir[0]==0 && dir[1]==0);
    // Is ray perpendicular to the cylinder length?
    int perpendicular = (dir[2]==0);
    // Is ray origin within the radius of the caps?
    int inRadius = (C<=0);
    // Is ray origin between the top and bottom caps?
    int inCaps   = (dFabs(pos[2])<=half_length);

    int checkCaps = (!perpendicular && (!inCaps || inRadius));
    int checkCyl  = (!parallel && (!inRadius || inCaps));
    int flipNormals = (inCaps&&inRadius);

    dReal tt=-dInfinity; // Depth to intersection
    dVector3 tmpNorm = {dNaN, dNaN, dNaN}; // ensure we don't leak garbage

    if (checkCaps) {
        // Make it so we only need to check one cap
        int flipDir = 0;
        // Wish c had logical xor...
        if ((dir[2]<0 && flipNormals) || (dir[2]>0 && !flipNormals)) {
            flipDir = 1;
            dir[2]=-dir[2];
            pos[2]=-pos[2];
        }
        // The cap is half the cylinder's length
        // from the cylinder's origin
        // We only checkCaps if dir[2]!=0
        tt = (half_length-pos[2])/dir[2];
        if (tt>=0 && tt<=ray->length) {
            tmp[0] = pos[0] + tt*dir[0];
            tmp[1] = pos[1] + tt*dir[1];
            // Ensure collision point is within cap circle
            if (tmp[0]*tmp[0] + tmp[1]*tmp[1] <= r2) {
                // Successful collision
                tmp[2] = (flipDir)?-half_length:half_length;
                tmpNorm[0]=0;
                tmpNorm[1]=0;
                tmpNorm[2]=(flipDir!=flipNormals)?REAL(-1.0):REAL(1.0);
                checkCyl = 0;  // Short circuit cylinder check
            } else {
                // Ray hits cap plane outside of cap circle
                tt=-dInfinity; // No collision yet
            }
        } else {
            // The cap plane is beyond (or behind) the ray length
            tt=-dInfinity; // No collision yet
        }
        if (flipDir) {
            // Flip back
            dir[2]=-dir[2];
            pos[2]=-pos[2];
        }
    }
    if (checkCyl) {
        // Compute quadratic formula for parametric ray equation
        dReal A =    dir[0]*dir[0] + dir[1]*dir[1];
        dReal B = 2*(pos[0]*dir[0] + pos[1]*dir[1]);
        // Already computed C

        dReal k = B*B - 4*A*C;
        // Check collision with infinite cylinder
        // k<0 means the ray passes outside the cylinder
        // k==0 means ray is tangent to cylinder (or parallel)
        //
        //  Our quadratic formula: tt = (-B +- sqrt(k))/(2*A)   
        // 
        // A must be positive (otherwise we wouldn't be checking
        // cylinder because ray is parallel)
        //    if (k<0) ray doesn't collide with sphere
        //    if (B > sqrt(k)) then both times are negative
        //         -- don't calculate
        //    if (B<-sqrt(k)) then both times are positive (Case A or B)
        //         -- only calculate first, if first isn't valid
        //         -- second can't be without first going through a cap
        //    otherwise (fabs(B)<=sqrt(k)) then C<=0 (ray-origin inside/on cylinder)
        //         -- only calculate second collision
        if (k>=0 && (B<0 || B*B<=k)) {
            k = dSqrt(k); 
            A = dRecip(2*A);
            if (dFabs(B)<=k) {
                tt = (-B + k)*A; // Second solution
                // If ray origin is on surface and pointed out, we
                // can get a tt=0 solution...
            } else {
                tt = (-B - k)*A; // First solution
            }
            if (tt<=ray->length) {
                tmp[2] = pos[2] + tt*dir[2];
                if (dFabs(tmp[2])<=half_length) {
                    // Valid solution
                    tmp[0] = pos[0] + tt*dir[0];
                    tmp[1] = pos[1] + tt*dir[1];
                    tmpNorm[0] = tmp[0]/cyl->radius;
                    tmpNorm[1] = tmp[1]/cyl->radius;
                    tmpNorm[2] = 0;
                    if (flipNormals) {
                        // Ray origin was inside cylinder
                        tmpNorm[0] = -tmpNorm[0];
                        tmpNorm[1] = -tmpNorm[1];
                    }
                } else {
                    // Ray hits cylinder outside of caps
                    tt=-dInfinity;
                }
            } else {
                // Ray doesn't reach the cylinder
                tt=-dInfinity;
            }
        }
    }

    if (tt>0) {
        contact->depth = tt;
        // Transform the point back to world coordinates
        tmpNorm[3]=0;
        tmp[3] = 0;
        dMultiply0_331(contact->normal,cyl->final_posr->R,tmpNorm);
        dMultiply0_331(contact->pos,cyl->final_posr->R,tmp);
        contact->pos[0]+=cyl->final_posr->pos[0];
        contact->pos[1]+=cyl->final_posr->pos[1];
        contact->pos[2]+=cyl->final_posr->pos[2];

        return 1;
    }
    // No contact with anything.
    return 0;
}
