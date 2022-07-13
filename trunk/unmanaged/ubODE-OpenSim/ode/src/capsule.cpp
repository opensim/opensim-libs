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
// capped cylinder public API

dxCapsule::dxCapsule (dSpaceID space, dReal _radius, dReal _length) :
dxGeom (space,1)
{
    dAASSERT (_radius >= 0 && _length >= 0);
    type = dCapsuleClass;
    radius = _radius;
    halfLenZ = _length * REAL(0.5);
    updateZeroSizedFlag(!_radius/* || !_length -- zero length capsule is not a zero sized capsule*/);
}

void dxCapsule::computeAABB()
{
    dxPosR *dpr = final_posr;
    dVector3 extend;
    dGetMatrixColumn3(extend, dpr->R, 2);
    dScaleVector3r4(extend, halfLenZ);
    dFabsVector3r4(extend);

    const dVector3& pos = dpr->pos;

    dReal range = extend[0] + radius;
    aabb[0] = pos[0] - range;
    aabb[1] = pos[0] + range;

    range = extend[1] + radius;
    aabb[2] = pos[1] - range;
    aabb[3] = pos[1] + range;

    range = extend[2] + radius;
    aabb[4] = pos[2] - range;
    aabb[5] = pos[2] + range;
}


dGeomID dCreateCapsule (dSpaceID space, dReal radius, dReal length)
{
    return new dxCapsule (space,radius,length);
}


void dGeomCapsuleSetParams (dGeomID g, dReal radius, dReal length)
{
    dUASSERT (g && g->type == dCapsuleClass,"argument not a ccylinder");
    dAASSERT (radius >= 0 && length >= 0);
    ((dxCapsule*)g)->radius = radius;
    ((dxCapsule*)g)->halfLenZ = REAL(0.5) * length;
    ((dxCapsule*)g)->updateZeroSizedFlag(!radius/* || !length -- zero length capsule is not a zero sized capsule*/);
    dGeomMoved (g);
}


void dGeomCapsuleGetParams (dGeomID g, dReal *radius, dReal *length)
{
    dUASSERT (g && g->type == dCapsuleClass,"argument not a ccylinder");
    *radius = ((dxCapsule*)g)->radius;
    *length = REAL(2.0) * ((dxCapsule*)g)->halfLenZ;
}


dReal dGeomCapsulePointDepth (dGeomID g, dReal x, dReal y, dReal z)
{
    dUASSERT (g && g->type == dCapsuleClass,"argument not a ccylinder");

    dxPosR *dpr = g->GetRecomputePosR();
    dReal* pos = dpr->pos;
 
    dVector3 a;
    a[0] = x;
    a[1] = y;
    a[2] = z;

    dSubtractVectors3r4(a, pos);

    dVector3 vrot;
    dGetMatrixColumn3(vrot, dpr->R, 2);

    dReal beta = dCalcVectorDot3(a, vrot);
    dReal lz2 = ((dxCapsule*)g)->halfLenZ;
    if (beta < -lz2)
        dAddScaledVector3r4(a, vrot, lz2);
    else if (beta > lz2)
        dAddScaledVector3r4(a, vrot, -lz2);
    else
       dAddScaledVector3r4(a, vrot, -beta);
    return ((dxCapsule*)g)->radius - dCalcVectorLength3(a);
}

int dCollideCapsuleSphere (dxGeom *o1, dxGeom *o2, int flags,
                           dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dCapsuleClass);
    dIASSERT (o2->type == dSphereClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;

    dxPosR *final_posr1 = o1->final_posr;
    dReal *pos = final_posr1->pos;

    dReal *spherepos = o2->final_posr->pos;

    dVector3 diff;
    dSubtractVectors3r4(diff, spherepos, pos);
    dVector3 vrot;
    dGetMatrixColumn3(vrot, final_posr1->R, 2);

    // find the point on the cylinder axis that is closest to the sphere
    dVector3 p;
    dReal lz2 = ((dxCapsule*)o1)->halfLenZ;
    dReal alpha = dCalcVectorDot3(diff, vrot);
    if (alpha > lz2)
        dAddScaledVector3(p, pos, vrot, lz2);
    if (alpha < -lz2)
        dAddScaledVector3(p, pos, vrot, -lz2);
    else
        dAddScaledVector3(p, pos, vrot, alpha);

    return dCollideSpheres (p, ((dxCapsule*)o1)->radius, spherepos, ((dxSphere*)o2)->radius, contact);
}


int dCollideCapsuleBox (dxGeom *o1, dxGeom *o2, int flags,
                        dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dCapsuleClass);
    dIASSERT (o2->type == dBoxClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dVector3 p1, p2, vaxis;

    // get p1,p2 = cylinder axis endpoints, get radius

    dxPosR *final_posr1 = o1->GetRecomputePosR();
    dReal *pos = final_posr1->pos;
    dReal *cR = final_posr1->R;
    dReal clen = ((dxCapsule*)o1)->halfLenZ;

    dGetMatrixColumn3(vaxis, cR, 2);
    dScaleVector3r4(vaxis, clen);

    dAddVectors3r4(p1, pos, vaxis);
    dSubtractVectors3r4(p2, pos, vaxis);

    // copy out box center, rotation matrix, and side array
    dxPosR *final_posr2 = o2->GetRecomputePosR();
    dReal *c = final_posr2->pos;
    dReal *R = final_posr2->R;
    const dReal *side = ((dxBox*)o2)->halfside;

    // get the closest point between the cylinder axis and the box
    dVector3 pl,pb;
    dClosestLineBoxPoints(p1, p2, c, R, side, pl, pb);

    // if the capsule is penetrated further than radius 
    //  then pl and pb are equal -> unknown normal
    // use normal vector of closest box surface

    dSubtractVectors3r4(p2, pl, pb);   
    if (dCalcVectorLengthSquare3(p2) < dEpsilon)
    {
        // consider capsule as box
        dVector3 normal;
        dReal depth;
        dReal radius = ((dxCapsule*)o1)->radius;
        const dVector3 capboxside = {radius, radius, clen + radius};
        int num = dBoxBox (c, R, side, 
            pos, cR, capboxside,
            normal, &depth, flags, contact, skip);

        for (int i=0; i<num; i++)
        {
            dContactGeom *currContact = CONTACT(contact,i*skip);
            dCopyVector3r4(currContact->normal, normal);
            currContact->g1 = o1;
            currContact->g2 = o2;
            currContact->side1 = -1;
            currContact->side2 = -1;
        }
        return num;
    }
    else
    {
        // generate contact point
        dReal radius = ((dxCapsule*)o1)->radius;
        if (dCollideSpheres(pl, radius, pb, 0, contact))
        {
            contact->g1 = o1;
            contact->g2 = o2;
            contact->side1 = -1;
            contact->side2 = -1;
            return 1;
        }
    }
    return 0;
}

int dCollideCapsuleCapsule (dxGeom *o1, dxGeom *o2,
                            int flags, dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dCapsuleClass);
    dIASSERT (o2->type == dCapsuleClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dVector3 axis1, axis2;
    dVector3 a1, a2, b1, b2;
    dVector3 tt;

    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;

    // copy out some variables, for convenience
    dReal lz1 = ((dxCapsule*)o1)->halfLenZ;
    dReal radius1 = ((dxCapsule*)o1)->radius;
    dReal lz2 = ((dxCapsule*)o2)->halfLenZ;
    dReal radius2 = ((dxCapsule*)o2)->radius;

    dxPosR *dpr1 = o1->GetRecomputePosR();
    dReal *pos1 = dpr1->pos;
    dGetMatrixColumn3(axis1, dpr1->R, 2);

    dxPosR *dpr2 = o2->GetRecomputePosR();
    dReal *pos2 = dpr2->pos;
    dGetMatrixColumn3(axis2, dpr2->R, 2);

    // if the cylinder axes are close to parallel, we'll try to detect up to
    // two contact points along the body of the cylinder. if we can't find any
    // points then we'll fall back to the closest-points algorithm. note that
    // we are not treating this special case for reasons of degeneracy, but
    // because we want two contact points in some situations. the closet-points
    // algorithm is robust in all casts, but it can return only one contact.

    dVector3 sphere1, sphere2;
    dReal a1a2 = dCalcVectorDot3(axis1, axis2);
    dReal det = REAL(1.0) - a1a2 * a1a2;
    if (det < dEpsilon)
    {
        // the cylinder axes (almost) parallel, so we will generate up to two
        // contacts. alpha1 and alpha2 (line position parameters) are related by:
        //       alpha2 =   alpha1 + (pos1-pos2)'*axis1   (if axis1==axis2)
        //    or alpha2 = -(alpha1 + (pos1-pos2)'*axis1)  (if axis1==-axis2)
        // first compute where the two cylinders overlap in alpha1 space:
        if (a1a2 < 0)
        {
            dNegateVector3r4(axis2);
        }

        dSubtractVectors3r4(tt, pos1, pos2);
        dReal k = dCalcVectorDot3(axis1, tt);
        dReal a1lo = -lz1;
        dReal a1hi = lz1;
        dReal a2lo = -lz2 - k;
        dReal a2hi = lz2 - k;
        dReal lo = (a1lo > a2lo) ? a1lo : a2lo;
        dReal hi = (a1hi < a2hi) ? a1hi : a2hi;
        if (lo <= hi)
        {
            int num_contacts = flags & NUMC_MASK;
            if (num_contacts >= 2 && lo < hi)
            {
                // generate up to two contacts. if one of those contacts is
                // not made, fall back on the one-contact strategy.
                dAddScaledVector3r4(sphere1, pos1, axis1, lo);
                dAddScaledVector3r4(sphere2, pos2, axis2, lo + k);
                int n1 = dCollideSpheres(sphere1, radius1, sphere2, radius2, contact);
                if (n1)
                {
                    dAddScaledVector3r4(sphere1, pos1, axis1, hi);
                    dAddScaledVector3r4(sphere2, pos2, axis2, hi + k);
                    dContactGeom *c2 = CONTACT(contact,skip);
                    int n2 = dCollideSpheres(sphere1, radius1, sphere2, radius2, c2);
                    if (n2)
                    {
                        c2->g1 = o1;
                        c2->g2 = o2;
                        c2->side1 = -1;
                        c2->side2 = -1;
                        return 2;
                    }
                }
            }

            // just one contact to generate, so put it in the middle of
            // the range
            dReal alpha1 = (lo + hi) * REAL(0.5);
            dAddScaledVector3r4(sphere1, pos1, axis1, alpha1);
            dAddScaledVector3r4(sphere2, pos2, axis2, alpha1 + k);
            return dCollideSpheres (sphere1, radius1, sphere2, radius2, contact);
        }
    }

    // use the closest point algorithm

    dScaleVector3r4(tt, axis1, lz1);
    dAddVectors3r4(a1, pos1, tt);
    dSubtractVectors3r4(a2, pos1, tt);

    dScaleVector3r4(tt, axis2, lz2);
    dAddVectors3r4(b1, pos2, tt);
    dSubtractVectors3r4(b2, pos2, tt);

    dClosestLineSegmentPoints(a1, a2, b1, b2, sphere1, sphere2);
    return dCollideSpheres(sphere1, radius1, sphere2, radius2, contact);
}

int dCollideCapsulePlane(dxGeom *o1, dxGeom *o2, int flags,
                          dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dCapsuleClass);
    dIASSERT (o2->type == dPlaneClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dVector3 p, vrot;

    dVector3& planeNorm = *(dVector3*)((dxPlane*)o2)->p;
    dReal planeOffset = ((dxPlane*)o2)->p[3];

    dxPosR *dpr = o1->GetRecomputePosR();
    dReal *pos = dpr->pos;

    dGetMatrixColumn3(vrot, dpr->R, 2);

    dReal capRadius = ((dxCapsule*)o1)->radius;

    // collide the deepest capping sphere with the plane
    dReal lzsign = (dCalcVectorDot3(planeNorm, vrot) > 0) ? -(((dxCapsule*)o1)->halfLenZ) : ((dxCapsule*)o1)->halfLenZ;

    dAddScaledVector3r4(p, pos, vrot, lzsign);
    dReal k = dCalcVectorDot3 (p, planeNorm);
    dReal depth = planeOffset - k + capRadius;
    if (depth < 0)
        return 0;

    dCopyVector3r4(contact->normal, planeNorm);
    dAddScaledVector3r4(contact->pos, p, planeNorm, -capRadius);
    contact->depth = depth;

    int ncontacts = 1;
    if ((flags & NUMC_MASK) >= 2)
    {
        // collide the other capping sphere with the plane
        dAddScaledVector3r4(p, pos, vrot, -lzsign);
        k = dCalcVectorDot3 (p, planeNorm);
        depth = planeOffset - k + capRadius;
        if (depth >= 0)
        {
            dContactGeom *c2 = CONTACT(contact,skip);
            dCopyVector3r4(c2->normal, planeNorm);
            dAddScaledVector3r4(contact->pos, p, planeNorm, -capRadius);
            c2->depth = depth;
            ncontacts = 2;
        }
    }

    for (int i=0; i < ncontacts; i++) {
        dContactGeom *currContact = CONTACT(contact,i*skip);
        currContact->g1 = o1;
        currContact->g2 = o2;
        currContact->side1 = -1;
        currContact->side2 = -1;
    }
    return ncontacts;
}

