/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
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

#include <ode/common.h>
#include "config.h"
#include "odemath.h"


#undef dSafeNormalize3
#undef dSafeNormalize4
#undef dNormalize3
#undef dNormalize4

/*
#if defined(__AVX__)
int _dSafeNormalize3(dVector3 a)
{
    __m128 ma,mb,mc;
    mb = _mm_loadu_ps(a);

    ma = _mm_dp_ps(mb, mb, 0x71);
    dReal l = (dReal)_mm_cvtss_f32(ma);
    if (l > dEpsilon)
    {
        mc = _mm_set_ss(1.0f);

        ma = _mm_sqrt_ss(ma);
        ma = _mm_div_ss(mc, ma);

        ma = _mm_insert_ps(ma,ma,0x0E);
        ma = _mm_shuffle_ps(ma,ma, 0x40);
        ma = _mm_mul_ps(ma, mb);
        _mm_storeu_ps(a, ma);
        return 1;
    }
    a[0] = 1;	// if all a's are zero, this is where we'll end up.
    a[1] = 0;	// return a default unit length vector.
    a[2] = 0;
    return 0;
}
#else
*/
int _dSafeNormalize3 (dVector3 a)
{
    dIASSERT(a);

    dReal l = dCalcVectorLengthSquare3(a);
    if (l > dEpsilon)
    {
        dScaleVector3r4(a, dRecipSqrt(l));
        return 1;
    }

    a[0] = 1;	// if all a's are zero, this is where we'll end up.
    a[1] = 0;	// return a default unit length vector.
    a[2] = 0;
    return 0;
}
//#endif

int _dSafeNormalize3fast(dVector3 a)
{
    dReal l = dCalcVectorLengthSquare3(a);
    if (l > dEpsilon)
    {
        dScaleVector3r4(a, dRecipSqrt(l));
        return 1;
    }
    return 0;
}

/* OLD VERSION */
/*
void dNormalize3 (dVector3 a)
{
    dIASSERT (a);
    dReal l = dCalcVectorlenghtSquare3(a,a);
    if (l > 0) {
        l = dRecipSqrt(l);
        a[0] *= l;
        a[1] *= l;
        a[2] *= l;
    }
    else {
        a[0] = 1;
        a[1] = 0;
        a[2] = 0;
    }
}
*/

int  dSafeNormalize3 (dVector3 a)
{
    return _dSafeNormalize3(a);
}

void dNormalize3(dVector3 a)
{
    _dNormalize3(a);
}


int _dSafeNormalize4 (dVector4 a)
{
    dAASSERT (a);
    dReal l = dCalcVectorLengthSquare4(a);
    if (l > dEpsilon) {
        l = dRecipSqrt(l);
        dScaleVector4(a, l);
        return 1;
    }
    else {
        a[0] = 1;
        a[1] = 0;
        a[2] = 0;
        a[3] = 0;
        return 0;
    }
}

int  dSafeNormalize4 (dVector4 a)
{
    return _dSafeNormalize4(a);
}

void dNormalize4(dVector4 a)
{
    _dNormalize4(a);
}


void dPlaneSpace (const dVector3 n, dVector3 p, dVector3 q)
{
    dAASSERT (n && p && q);
    if (dFabs(n[2]) > M_SQRT1_2) {
        // choose p in y-z plane
        dReal a = n[1]*n[1] + n[2]*n[2];
        dReal k = dRecipSqrt (a);
        p[0] = 0;
        p[1] = -n[2]*k;
        p[2] = n[1]*k;
        // set q = n x p
        q[0] = a*k;
        q[1] = -n[0]*p[2];
        q[2] = n[0]*p[1];
    }
    else {
        // choose p in x-y plane
        dReal a = n[0]*n[0] + n[1]*n[1];
        dReal k = dRecipSqrt (a);
        p[0] = -n[1]*k;
        p[1] = n[0]*k;
        p[2] = 0;
        // set q = n x p
        q[0] = -n[2]*p[1];
        q[1] = n[2]*p[0];
        q[2] = a*k;
    }
}


/*
* This takes what is supposed to be a rotation matrix,
* and make sure it is correct.
* Note: this operates on rows, not columns, because for rotations
* both ways give equivalent results.
*/
void dOrthogonalizeR(dMatrix3 m)
{
    dReal n0 = dCalcVectorLengthSquare3(m);
    if (n0 != 1)
        dSafeNormalize3(m);

    // project row[0] on row[1], should be zero
    dReal proj = dCalcVectorDot3(m, m+4);
    if (proj != 0) {
        // Gram-Schmidt step on row[1]
        m[4] -= proj * m[0];
        m[5] -= proj * m[1];
        m[6] -= proj * m[2];
    }
    dReal n1 = dCalcVectorLengthSquare3(m+4);
    if (n1 != 1)
        dSafeNormalize3(m+4);

    /* just overwrite row[2], this makes sure the matrix is not
    a reflection */
    dCalcVectorCross3(m+8, m, m+4);
    m[3] = m[4+3] = m[8+3] = 0;
}
