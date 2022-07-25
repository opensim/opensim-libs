///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for 3D vectors.
 *	\file		IcePoint.cpp
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	3D point.
 *
 *	The name is "Point" instead of "Vector" since a vector is N-dimensional, whereas a point is an implicit "vector of dimension 3".
 *	So the choice was between "Point" and "Vector3", the first one looked better (IMHO).
 *
 *	Some people, then, use a typedef to handle both points & vectors using the same class: typedef Point Vector3;
 *	This is bad since it opens the door to a lot of confusion while reading the code. I know it may sounds weird but check this out:
 *
 *	\code
 *		Point P0,P1 = some 3D points;
 *		Point Delta = P1 - P0;
 *	\endcode
 *
 *	This compiles fine, although you should have written:
 *
 *	\code
 *		Point P0,P1 = some 3D points;
 *		Vector3 Delta = P1 - P0;
 *	\endcode
 *
 *	Subtle things like this are not caught at compile-time, and when you find one in the code, you never know whether it's a mistake
 *	from the author or something you don't get.
 *
 *	One way to handle it at compile-time would be to use different classes for Point & Vector3, only overloading operator "-" for vectors.
 *	But then, you get a lot of redundant code in thoses classes, and basically it's really a lot of useless work.
 *
 *	Another way would be to use homogeneous points: w=1 for points, w=0 for vectors. That's why the HPoint class exists. Now, to store
 *	your model's vertices and in most cases, you really want to use Points to save ram.
 *
 *	\class		Point
 *	\author		Pierre Terdiman
 *	\version	1.0
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

using namespace IceMaths;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Creates a positive unit random vector.
 *	\return		Self-reference
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Point& Point::PositiveUnitRandomVector()
{
	x = UnitRandomFloat();
	y = UnitRandomFloat();
	z = UnitRandomFloat();
	Normalize();
	return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Creates a unit random vector.
 *	\return		Self-reference
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Point& Point::UnitRandomVector()
{
	x = UnitRandomFloat() - 0.5f;
	y = UnitRandomFloat() - 0.5f;
	z = UnitRandomFloat() - 0.5f;
	Normalize();
	return *this;
}

// Cast operator
// WARNING: not inlined
Point::operator HPoint() const	{ return HPoint(x, y, z, 0.0f); }

Point& Point::Refract(const Point& eye, const Point& n, float refractindex, Point& refracted)
{
	//	Point EyePt = eye position
	//	Point p = current vertex
	//	Point n = vertex normal
	//	Point rv = refracted vector
	//	Eye vector - doesn't need to be normalized
	Point Env;
	Env.x = eye.x - x;
	Env.y = eye.y - y;
	Env.z = eye.z - z;

	float NDotE = n|Env;
	float NDotN = n|n;
	NDotE /= refractindex;

	// Refracted vector
	refracted = n*NDotE - Env*NDotN;

	return *this;
}

Point& Point::ProjectToPlane(const Plane& p)
{
	*this-= (p.d + (*this|p.n))*p.n;
	return *this;
}

void Point::ProjectToScreen(float halfrenderwidth, float halfrenderheight, const Matrix4x4& mat, HPoint& projected) const
{
	projected = HPoint(x, y, z, 1.0f) * mat;
	projected.w = 1.0f / projected.w;

	projected.x*=projected.w;
	projected.y*=projected.w;
	projected.z*=projected.w;

	projected.x *= halfrenderwidth;		projected.x += halfrenderwidth;
	projected.y *= -halfrenderheight;	projected.y += halfrenderheight;
}

#if defined(__AVX__)
Point& Point::Mult(const Matrix3x3& mat, const Point& a)
{
    __m128 ma, mp, mt;
    float xx, yy, zz;

    ma = _mm_loadu_ps(a);

    mp = _mm_loadu_ps(mat.m[0]);
    mt = _mm_dp_ps(ma, mp, 0x71);
    xx = _mm_cvtss_f32(mt);

    mp = _mm_loadu_ps(mat.m[1]);
    mt = _mm_dp_ps(ma, mp, 0x71);
    yy = _mm_cvtss_f32(mt);

    mp = _mm_loadu_ps(mat.m[2]);
    mt = _mm_dp_ps(ma, mp, 0x71);
    zz = _mm_cvtss_f32(mt);
    x = xx; y = yy; z = zz;
    return *this;
}
#else
Point& Point::Mult(const Matrix3x3& mat, const Point& a)
{
	x = a.x * mat.m[0][0] + a.y * mat.m[0][1] + a.z * mat.m[0][2];
	y = a.x * mat.m[1][0] + a.y * mat.m[1][1] + a.z * mat.m[1][2];
	z = a.x * mat.m[2][0] + a.y * mat.m[2][1] + a.z * mat.m[2][2];
	return *this;
}
#endif

Point& Point::Mult2(const Matrix3x3& mat1, const Point& a1, const Matrix3x3& mat2, const Point& a2)
{
	x = a1.x * mat1.m[0][0] + a1.y * mat1.m[0][1] + a1.z * mat1.m[0][2] + a2.x * mat2.m[0][0] + a2.y * mat2.m[0][1] + a2.z * mat2.m[0][2];
	y = a1.x * mat1.m[1][0] + a1.y * mat1.m[1][1] + a1.z * mat1.m[1][2] + a2.x * mat2.m[1][0] + a2.y * mat2.m[1][1] + a2.z * mat2.m[1][2];
	z = a1.x * mat1.m[2][0] + a1.y * mat1.m[2][1] + a1.z * mat1.m[2][2] + a2.x * mat2.m[2][0] + a2.y * mat2.m[2][1] + a2.z * mat2.m[2][2];
	return *this;
}

#if defined(__AVX__)
Point& Point::Mac(const Matrix3x3& mat, const Point& a)
{
    __m128 ma, mp, mt;
    float xx, yy, zz;

    ma = _mm_loadu_ps(a);

    mp = _mm_loadu_ps(mat.m[0]);
    mt = _mm_dp_ps(ma, mp, 0x71);
    xx = _mm_cvtss_f32(mt);

    mp = _mm_loadu_ps(mat.m[1]);
    mt = _mm_dp_ps(ma, mp, 0x71);
    yy = _mm_cvtss_f32(mt);

    mp = _mm_loadu_ps(mat.m[2]);
    mt = _mm_dp_ps(ma, mp, 0x71);
    zz = _mm_cvtss_f32(mt);

    x += xx; y += yy; z += zz;
    return *this;
}
#else
Point& Point::Mac(const Matrix3x3& mat, const Point& a)
{
	x += a.x * mat.m[0][0] + a.y * mat.m[0][1] + a.z * mat.m[0][2];
	y += a.x * mat.m[1][0] + a.y * mat.m[1][1] + a.z * mat.m[1][2];
	z += a.x * mat.m[2][0] + a.y * mat.m[2][1] + a.z * mat.m[2][2];
	return *this;
}
#endif

#if defined(__AVX__)
Point& Point::TransMult(const Matrix3x3& mat, const Point& a)
{
    __m128 ma, t0, t1, t2, m0, m1, m2, m3;
    float xx, yy, zz;

    t0 = _mm_loadu_ps(mat.m[0]);
    t1 = _mm_loadu_ps(mat.m[1]);
    t2 = _mm_loadu_ps(mat.m[2]);

    m0 = _mm_shuffle_ps(t0, t1, _MM_SHUFFLE(1, 0, 1, 0)); // x0 y0 x1 y1
    m2 = _mm_shuffle_ps(t0, t1, _MM_SHUFFLE(3, 2, 3, 2)); // z0 w0 z1 w1
    m1 = _mm_shuffle_ps(t1, t2, _MM_SHUFFLE(1, 0, 1, 0)); // x1 y1 x2 y2
    m3 = _mm_shuffle_ps(t1, t2, _MM_SHUFFLE(3, 2, 3, 2)); // z1 w1 z2 w2

    t0 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(2, 2, 2, 0)); //x0 x1 x2 x2
    t1 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(3, 3, 3, 1)); //y0 y1 y2 y2
    t2 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(2, 2, 2, 0)); //z0 z1 z2 z2

    ma = _mm_loadu_ps(a);
    m0 = _mm_dp_ps(ma, t0, 0x71);
    xx = _mm_cvtss_f32(m0);
    m1 = _mm_dp_ps(ma, t1, 0x71);
    yy = _mm_cvtss_f32(m1);
    m2 = _mm_dp_ps(ma, t2, 0x71);
    zz = _mm_cvtss_f32(m2);
    x = xx; y = yy; z = zz;
    return *this;
}
#else
Point& Point::TransMult(const Matrix3x3& mat, const Point& a)
{
	x = a.x * mat.m[0][0] + a.y * mat.m[1][0] + a.z * mat.m[2][0];
	y = a.x * mat.m[0][1] + a.y * mat.m[1][1] + a.z * mat.m[2][1];
	z = a.x * mat.m[0][2] + a.y * mat.m[1][2] + a.z * mat.m[2][2];
	return *this;
}
#endif

Point& Point::Transform(const Point& r, const Matrix3x3& rotpos, const Point& linpos)
{
	x = r.x * rotpos.m[0][0] + r.y * rotpos.m[0][1] + r.z * rotpos.m[0][2] + linpos.x;
	y = r.x * rotpos.m[1][0] + r.y * rotpos.m[1][1] + r.z * rotpos.m[1][2] + linpos.y;
	z = r.x * rotpos.m[2][0] + r.y * rotpos.m[2][1] + r.z * rotpos.m[2][2] + linpos.z;
	return *this;
}

Point& Point::InvTransform(const Point& r, const Matrix3x3& rotpos, const Point& linpos)
{
	float sx = r.x - linpos.x;
	float sy = r.y - linpos.y;
	float sz = r.z - linpos.z;
	x = sx * rotpos.m[0][0] + sy * rotpos.m[1][0] + sz * rotpos.m[2][0];
	y = sx * rotpos.m[0][1] + sy * rotpos.m[1][1] + sz * rotpos.m[2][1];
	z = sx * rotpos.m[0][2] + sy * rotpos.m[1][2] + sz * rotpos.m[2][2];
	return *this;
}
