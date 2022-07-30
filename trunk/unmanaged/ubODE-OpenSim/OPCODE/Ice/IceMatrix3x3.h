///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *    Contains code for 3x3 matrices.
 *    \file        IceMatrix3x3.h
 *    \author        Pierre Terdiman
 *    \date        April, 4, 2000
 */
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // Include Guard
#ifndef __ICEMATRIX3X3_H__
#define __ICEMATRIX3X3_H__

    // Forward declarations
class Quat;

#define    MATRIX3X3_EPSILON (1.0e-7f)

class ICEMATHS_API Matrix3x3
{
public:
    //! Empty constructor
    inline_ Matrix3x3() {}
    //! Constructor from 9 values
    inline_ Matrix3x3(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
    {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }
    //! Copy constructor
    inline_ Matrix3x3(const Matrix3x3& mat) { CopyMemory(m, &mat.m, 9 * sizeof(float)); }
    //! Destructor
    inline_ ~Matrix3x3() {}

    //! Assign values
    inline_ void Set(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
    {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
    }

    //! Sets the scale from a Point. The point is put on the diagonal.
    inline_ void SetScale(const Point& p) { m[0][0] = p.x; m[1][1] = p.y; m[2][2] = p.z; }

    //! Sets the scale from floats. Values are put on the diagonal.
    inline_ void SetScale(float sx, float sy, float sz) { m[0][0] = sx; m[1][1] = sy; m[2][2] = sz; }

    //! Scales from a Point. Each row is multiplied by a component.
    inline_ void Scale(const Point& p)
    {
        m[0][0] *= p.x; m[0][1] *= p.x; m[0][2] *= p.x;
        m[1][0] *= p.y; m[1][1] *= p.y; m[1][2] *= p.y;
        m[2][0] *= p.z; m[2][1] *= p.z; m[2][2] *= p.z;
    }

    //! Scales from floats. Each row is multiplied by a value.
    inline_ void Scale(float sx, float sy, float sz)
    {
        m[0][0] *= sx; m[0][1] *= sx; m[0][2] *= sx;
        m[1][0] *= sy; m[1][1] *= sy; m[1][2] *= sy;
        m[2][0] *= sz; m[2][1] *= sz; m[2][2] *= sz;
    }

    //! Copy from a Matrix3x3
    inline_ void Copy(const Matrix3x3& source) { CopyMemory(m, source.m, 9 * sizeof(float)); }

    // Row-column access
    //! Returns a row.
    inline_ void GetRow(const udword r, Point& p) const { p.x = m[r][0]; p.y = m[r][1]; p.z = m[r][2]; }
    //! Returns a row.
    inline_ Point& GetRow(const udword r) { return *(Point*)&m[r][0]; }
    //! Sets a row.
    inline_ void SetRow(const udword r, const Point& p) { m[r][0] = p.x; m[r][1] = p.y; m[r][2] = p.z; }
    //! Returns a column.
    inline_ void GetCol(const udword c, Point& p) const { p.x = m[0][c]; p.y = m[1][c]; p.z = m[2][c]; }
    //! Sets a column.
    inline_ void SetCol(const udword c, const Point& p) { m[0][c] = p.x; m[1][c] = p.y; m[2][c] = p.z; }

    //! Computes the trace. The trace is the sum of the 3 diagonal components.
    inline_ float Trace() const { return m[0][0] + m[1][1] + m[2][2]; }
    //! Clears the matrix.
    inline_ void Zero() { ZeroMemory(&m, sizeof(m)); }
    //! Sets the identity matrix.
    inline_ void Identity() { Zero(); m[0][0] = m[1][1] = m[2][2] = 1.0f; }
    //! Checks for identity
    inline_ bool IsIdentity()  const
    {
        if (m[0][0] != 1.0f) return false;
        if (m[0][1] != 0)    return false;
        if (m[0][2] != 0)    return false;

        if (m[1][0] != 0)    return false;
        if (m[1][1] != 1.0f) return false;
        if (m[1][2] != 0)    return false;

        if (m[2][0] != 0)    return false;
        if (m[2][1] != 0)    return false;
        if (m[2][2] != 1.0f) return false;

        return true;
    }

    //! Checks matrix validity
    inline_ BOOL IsValid() const
    {
        for (int j = 0; j < 3; j++)
        {
            for (int i = 0; i < 3; i++)
            {
                if (!IsValidFloat(m[j][i])) return FALSE;
            }
        }
        return TRUE;
    }

    //! Makes a skew-symmetric matrix (a.k.a. Star(*) Matrix)
    //!    [  0.0  -a.z   a.y ]
    //!    [  a.z   0.0  -a.x ]
    //!    [ -a.y   a.x   0.0 ]
    //! This is also called a "cross matrix" since for any vectors A and B,
    //! A^B = Skew(A) * B = - B * Skew(A);
    inline_ void SkewSymmetric(const Point& a)
    {
        m[0][0] = 0.0f;
        m[0][1] = -a.z;
        m[0][2] = a.y;

        m[1][0] = a.z;
        m[1][1] = 0.0f;
        m[1][2] = -a.x;

        m[2][0] = -a.y;
        m[2][1] = a.x;
        m[2][2] = 0.0f;
    }

    //! Negates the matrix
    inline_ void Neg()
    {
        m[0][0] = -m[0][0]; m[0][1] = -m[0][1]; m[0][2] = -m[0][2];
        m[1][0] = -m[1][0]; m[1][1] = -m[1][1]; m[1][2] = -m[1][2];
        m[2][0] = -m[2][0]; m[2][1] = -m[2][1]; m[2][2] = -m[2][2];
    }

    //! Neg from another matrix
    inline_ void Neg(const Matrix3x3& mat)
    {
        m[0][0] = -mat.m[0][0]; m[0][1] = -mat.m[0][1]; m[0][2] = -mat.m[0][2];
        m[1][0] = -mat.m[1][0]; m[1][1] = -mat.m[1][1]; m[1][2] = -mat.m[1][2];
        m[2][0] = -mat.m[2][0]; m[2][1] = -mat.m[2][1]; m[2][2] = -mat.m[2][2];
    }

    //! Add another matrix
    inline_ void Add(const Matrix3x3& mat)
    {
        m[0][0] += mat.m[0][0]; m[0][1] += mat.m[0][1]; m[0][2] += mat.m[0][2];
        m[1][0] += mat.m[1][0]; m[1][1] += mat.m[1][1]; m[1][2] += mat.m[1][2];
        m[2][0] += mat.m[2][0]; m[2][1] += mat.m[2][1]; m[2][2] += mat.m[2][2];
    }

    //! Sub another matrix
    inline_ void Sub(const Matrix3x3& mat)
    {
        m[0][0] -= mat.m[0][0]; m[0][1] -= mat.m[0][1]; m[0][2] -= mat.m[0][2];
        m[1][0] -= mat.m[1][0]; m[1][1] -= mat.m[1][1]; m[1][2] -= mat.m[1][2];
        m[2][0] -= mat.m[2][0]; m[2][1] -= mat.m[2][1]; m[2][2] -= mat.m[2][2];
    }
    //! Mac
    inline_ void Mac(const Matrix3x3& a, const Matrix3x3& b, float s)
    {
        m[0][0] = a.m[0][0] + b.m[0][0] * s;
        m[0][1] = a.m[0][1] + b.m[0][1] * s;
        m[0][2] = a.m[0][2] + b.m[0][2] * s;

        m[1][0] = a.m[1][0] + b.m[1][0] * s;
        m[1][1] = a.m[1][1] + b.m[1][1] * s;
        m[1][2] = a.m[1][2] + b.m[1][2] * s;

        m[2][0] = a.m[2][0] + b.m[2][0] * s;
        m[2][1] = a.m[2][1] + b.m[2][1] * s;
        m[2][2] = a.m[2][2] + b.m[2][2] * s;
    }
    //! Mac
    inline_ void Mac(const Matrix3x3& a, float s)
    {
        m[0][0] += a.m[0][0] * s;
        m[0][1] += a.m[0][1] * s;
        m[0][2] += a.m[0][2] * s;
        m[1][0] += a.m[1][0] * s;
        m[1][1] += a.m[1][1] * s;
        m[1][2] += a.m[1][2] * s;
        m[2][0] += a.m[2][0] * s;
        m[2][1] += a.m[2][1] * s;
        m[2][2] += a.m[2][2] * s;
    }

    //! this = A * s
    inline_ void Mult(const Matrix3x3& a, float s)
    {
        m[0][0] = a.m[0][0] * s;
        m[0][1] = a.m[0][1] * s;
        m[0][2] = a.m[0][2] * s;
        m[1][0] = a.m[1][0] * s;
        m[1][1] = a.m[1][1] * s;
        m[1][2] = a.m[1][2] * s;
        m[2][0] = a.m[2][0] * s;
        m[2][1] = a.m[2][1] * s;
        m[2][2] = a.m[2][2] * s;
    }

    inline_ void Add(const Matrix3x3& a, const Matrix3x3& b)
    {
        m[0][0] = a.m[0][0] + b.m[0][0]; m[0][1] = a.m[0][1] + b.m[0][1]; m[0][2] = a.m[0][2] + b.m[0][2];
        m[1][0] = a.m[1][0] + b.m[1][0]; m[1][1] = a.m[1][1] + b.m[1][1]; m[1][2] = a.m[1][2] + b.m[1][2];
        m[2][0] = a.m[2][0] + b.m[2][0]; m[2][1] = a.m[2][1] + b.m[2][1]; m[2][2] = a.m[2][2] + b.m[2][2];
    }

    inline_ void Sub(const Matrix3x3& a, const Matrix3x3& b)
    {
        m[0][0] = a.m[0][0] - b.m[0][0]; m[0][1] = a.m[0][1] - b.m[0][1]; m[0][2] = a.m[0][2] - b.m[0][2];
        m[1][0] = a.m[1][0] - b.m[1][0]; m[1][1] = a.m[1][1] - b.m[1][1]; m[1][2] = a.m[1][2] - b.m[1][2];
        m[2][0] = a.m[2][0] - b.m[2][0]; m[2][1] = a.m[2][1] - b.m[2][1]; m[2][2] = a.m[2][2] - b.m[2][2];
    }

    //! this = a * b
    inline_ void Mult(const Matrix3x3& a, const Matrix3x3& b)
    {
        m[0][0] = a.m[0][0] * b.m[0][0] + a.m[0][1] * b.m[1][0] + a.m[0][2] * b.m[2][0];
        m[0][1] = a.m[0][0] * b.m[0][1] + a.m[0][1] * b.m[1][1] + a.m[0][2] * b.m[2][1];
        m[0][2] = a.m[0][0] * b.m[0][2] + a.m[0][1] * b.m[1][2] + a.m[0][2] * b.m[2][2];
        m[1][0] = a.m[1][0] * b.m[0][0] + a.m[1][1] * b.m[1][0] + a.m[1][2] * b.m[2][0];
        m[1][1] = a.m[1][0] * b.m[0][1] + a.m[1][1] * b.m[1][1] + a.m[1][2] * b.m[2][1];
        m[1][2] = a.m[1][0] * b.m[0][2] + a.m[1][1] * b.m[1][2] + a.m[1][2] * b.m[2][2];
        m[2][0] = a.m[2][0] * b.m[0][0] + a.m[2][1] * b.m[1][0] + a.m[2][2] * b.m[2][0];
        m[2][1] = a.m[2][0] * b.m[0][1] + a.m[2][1] * b.m[1][1] + a.m[2][2] * b.m[2][1];
        m[2][2] = a.m[2][0] * b.m[0][2] + a.m[2][1] * b.m[1][2] + a.m[2][2] * b.m[2][2];
    }

    //! this = transpose(a) * b
    inline_ void  MultAtB(const Matrix3x3& a, const Matrix3x3& b)
    {
        m[0][0] = a.m[0][0] * b.m[0][0] + a.m[1][0] * b.m[1][0] + a.m[2][0] * b.m[2][0];
        m[0][1] = a.m[0][0] * b.m[0][1] + a.m[1][0] * b.m[1][1] + a.m[2][0] * b.m[2][1];
        m[0][2] = a.m[0][0] * b.m[0][2] + a.m[1][0] * b.m[1][2] + a.m[2][0] * b.m[2][2];
        m[1][0] = a.m[0][1] * b.m[0][0] + a.m[1][1] * b.m[1][0] + a.m[2][1] * b.m[2][0];
        m[1][1] = a.m[0][1] * b.m[0][1] + a.m[1][1] * b.m[1][1] + a.m[2][1] * b.m[2][1];
        m[1][2] = a.m[0][1] * b.m[0][2] + a.m[1][1] * b.m[1][2] + a.m[2][1] * b.m[2][2];
        m[2][0] = a.m[0][2] * b.m[0][0] + a.m[1][2] * b.m[1][0] + a.m[2][2] * b.m[2][0];
        m[2][1] = a.m[0][2] * b.m[0][1] + a.m[1][2] * b.m[1][1] + a.m[2][2] * b.m[2][1];
        m[2][2] = a.m[0][2] * b.m[0][2] + a.m[1][2] * b.m[1][2] + a.m[2][2] * b.m[2][2];
    }

    //! this = a * transpose(b)
    inline_ void MultABt(const Matrix3x3& a, const Matrix3x3& b)
    {
        m[0][0] = a.m[0][0] * b.m[0][0] + a.m[0][1] * b.m[0][1] + a.m[0][2] * b.m[0][2];
        m[0][1] = a.m[0][0] * b.m[1][0] + a.m[0][1] * b.m[1][1] + a.m[0][2] * b.m[1][2];
        m[0][2] = a.m[0][0] * b.m[2][0] + a.m[0][1] * b.m[2][1] + a.m[0][2] * b.m[2][2];
        m[1][0] = a.m[1][0] * b.m[0][0] + a.m[1][1] * b.m[0][1] + a.m[1][2] * b.m[0][2];
        m[1][1] = a.m[1][0] * b.m[1][0] + a.m[1][1] * b.m[1][1] + a.m[1][2] * b.m[1][2];
        m[1][2] = a.m[1][0] * b.m[2][0] + a.m[1][1] * b.m[2][1] + a.m[1][2] * b.m[2][2];
        m[2][0] = a.m[2][0] * b.m[0][0] + a.m[2][1] * b.m[0][1] + a.m[2][2] * b.m[0][2];
        m[2][1] = a.m[2][0] * b.m[1][0] + a.m[2][1] * b.m[1][1] + a.m[2][2] * b.m[1][2];
        m[2][2] = a.m[2][0] * b.m[2][0] + a.m[2][1] * b.m[2][1] + a.m[2][2] * b.m[2][2];
    }

    //! Makes a rotation matrix mapping vector "from" to vector "to".
    Matrix3x3& FromTo(const Point& from, const Point& to);

    //! Set a rotation matrix around the X axis.
    //!         1        0        0
    //!    RX = 0        cx        sx
    //!         0        -sx        cx
    void RotX(float angle);
    //! Set a rotation matrix around the Y axis.
    //!         cy        0        -sy
    //!    RY = 0        1        0
    //!         sy        0        cy
    void            RotY(float angle);
    //! Set a rotation matrix around the Z axis.
    //!         cz        sz        0
    //!    RZ = -sz    cz        0
    //!         0        0        1
    void            RotZ(float angle);
    //!            cy        sx.sy        -sy.cx
    //!    RY.RX    0        cx            sx
    //!            sy        -sx.cy        cx.cy
    void RotYX(float y, float x);

    //! Make a rotation matrix about an arbitrary axis
    Matrix3x3& Rot(float angle, const Point& axis);

#if defined(__AVX__)
    void Transpose()
    {
        __m128 t0, t1, m0, m1, m2, m3;
        t0 = _mm_loadu_ps(m[0]);
        t1 = _mm_loadu_ps(m[1]);

        m0 = _mm_shuffle_ps(t0, t1, _MM_SHUFFLE(1, 0, 1, 0)); // x0 y0 x1 y1
        m2 = _mm_shuffle_ps(t0, t1, _MM_SHUFFLE(3, 2, 3, 2)); // z0 w0 z1 w1
        t0 = _mm_loadu_ps(m[2]);
        m1 = _mm_shuffle_ps(t1, t0, _MM_SHUFFLE(1, 0, 1, 0)); // x1 y1 x2 y2
        m3 = _mm_shuffle_ps(t1, t0, _MM_SHUFFLE(3, 2, 3, 2)); // z1 w1 z2 w2

        t0 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(2, 2, 2, 0)); //x0 x1 x2 x2
        _mm_storeu_ps(m[0], t0);
        t1 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(3, 3, 3, 1)); //y0 y1 y2 y2
        _mm_storeu_ps(m[1], t1);
        t0 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(2, 2, 2, 0)); //z0 z1 z2 z2
        _mm_storeu_ps(m[2], t0);
    }
#else
    //! Transpose the matrix.
    void Transpose()
    {
        TSwap(m[1][0], m[0][1]);
        TSwap(m[2][0], m[0][2]);
        TSwap(m[2][1], m[1][2]);
    }
#endif

    //! this = Transpose(a)
#if defined(__AVX__)
    void Transpose(const Matrix3x3& a)
    {
        __m128 t0, t1, m0, m1, m2, m3;
        t0 = _mm_loadu_ps(a.m[0]);
        t1 = _mm_loadu_ps(a.m[1]);

        m0 = _mm_shuffle_ps(t0, t1, _MM_SHUFFLE(1, 0, 1, 0)); // x0 y0 x1 y1
        m2 = _mm_shuffle_ps(t0, t1, _MM_SHUFFLE(3, 2, 3, 2)); // z0 w0 z1 w1
        t0 = _mm_loadu_ps(a.m[2]);
        m1 = _mm_shuffle_ps(t1, t0, _MM_SHUFFLE(1, 0, 1, 0)); // x1 y1 x2 y2
        m3 = _mm_shuffle_ps(t1, t0, _MM_SHUFFLE(3, 2, 3, 2)); // z1 w1 z2 w2

        t0 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(2, 2, 2, 0)); //x0 x1 x2 x2
        _mm_storeu_ps(m[0], t0);
        t1 = _mm_shuffle_ps(m0, m1, _MM_SHUFFLE(3, 3, 3, 1)); //y0 y1 y2 y2
        _mm_storeu_ps(m[1], t1);
        t0 = _mm_shuffle_ps(m2, m3, _MM_SHUFFLE(2, 2, 2, 0)); //z0 z1 z2 z2
        _mm_storeu_ps(m[2], t0);
    }
#else
    void Transpose(const Matrix3x3& a)
    {
        m[0][0] = a.m[0][0];    m[0][1] = a.m[1][0];    m[0][2] = a.m[2][0];
        m[1][0] = a.m[0][1];    m[1][1] = a.m[1][1];    m[1][2] = a.m[2][1];
        m[2][0] = a.m[0][2];    m[2][1] = a.m[1][2];    m[2][2] = a.m[2][2];
    }
#endif
    //! Compute the determinant of the matrix. We use the rule of Sarrus.
#if defined(__AVX__)
    float Determinant() const
    {
        __m128 ma, mb, t1, t2, t3, t4;
        ma = _mm_loadu_ps(m[1]);
        mb = _mm_loadu_ps(m[2]);

        t1 = _mm_shuffle_ps(ma, ma, _MM_SHUFFLE(3, 0, 2, 1)); // 5 6 4 7 
        t2 = _mm_shuffle_ps(mb, mb, _MM_SHUFFLE(3, 1, 0, 2)); // 10 8 9 11
        t3 = _mm_mul_ps(t1, t2); //5*10 6*8 4*9 7-11

        t1 = _mm_shuffle_ps(t1, t1, _MM_SHUFFLE(3, 0, 2, 1)); // 6 4 5 7
        t2 = _mm_shuffle_ps(t2, t2, _MM_SHUFFLE(3, 1, 0, 2)); // 9 10 8 7
        t4 = _mm_mul_ps(t1, t2); // 6*9 4*10 5*6 7*7;

        ma = _mm_sub_ps(t3, t4);

        mb = _mm_loadu_ps(m[0]); // 0 1 2 3
        t1 = _mm_dp_ps(ma, mb, 0x7f);

        return _mm_cvtss_f32(t1);
    }
#else
    float Determinant() const
    {
        return (m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1])
            - (m[2][0] * m[1][1] * m[0][2] + m[2][1] * m[1][2] * m[0][0] + m[2][2] * m[1][0] * m[0][1]);
    }
#endif
    /*
            //! Compute a cofactor. Used for matrix inversion.
                    float            CoFactor(ubyte row, ubyte column)    const
                    {
                        static const sdword gIndex[3+2] = { 0, 1, 2, 0, 1 };
                        return    (m[gIndex[row+1]][gIndex[column+1]]*m[gIndex[row+2]][gIndex[column+2]] - m[gIndex[row+2]][gIndex[column+1]]*m[gIndex[row+1]][gIndex[column+2]]);
                    }
    */
    //! Invert the matrix. Determinant must be different from zero, else matrix can't be inverted.
    Matrix3x3& Invert()
    {
        float Det = Determinant();    // Must be !=0
        float OneOverDet = 1.0f / Det;

        Matrix3x3 Temp;
        Temp.m[0][0] = +(m[1][1] * m[2][2] - m[2][1] * m[1][2]) * OneOverDet;
        Temp.m[1][0] = -(m[1][0] * m[2][2] - m[2][0] * m[1][2]) * OneOverDet;
        Temp.m[2][0] = +(m[1][0] * m[2][1] - m[2][0] * m[1][1]) * OneOverDet;
        Temp.m[0][1] = -(m[0][1] * m[2][2] - m[2][1] * m[0][2]) * OneOverDet;
        Temp.m[1][1] = +(m[0][0] * m[2][2] - m[2][0] * m[0][2]) * OneOverDet;
        Temp.m[2][1] = -(m[0][0] * m[2][1] - m[2][0] * m[0][1]) * OneOverDet;
        Temp.m[0][2] = +(m[0][1] * m[1][2] - m[1][1] * m[0][2]) * OneOverDet;
        Temp.m[1][2] = -(m[0][0] * m[1][2] - m[1][0] * m[0][2]) * OneOverDet;
        Temp.m[2][2] = +(m[0][0] * m[1][1] - m[1][0] * m[0][1]) * OneOverDet;

        *this = Temp;

        return    *this;
    }

    Matrix3x3& Normalize();

    //! this = exp(a)
    Matrix3x3& Exp(const Matrix3x3& a);

    void FromQuat(const Quat &q);
    void FromQuatL2(const Quat &q, float l2);

    // Arithmetic operators
    //! Operator for Matrix3x3 Plus = Matrix3x3 + Matrix3x3;
    inline_ Matrix3x3 operator+(const Matrix3x3& mat) const
    {
        return Matrix3x3(
            m[0][0] + mat.m[0][0], m[0][1] + mat.m[0][1], m[0][2] + mat.m[0][2],
            m[1][0] + mat.m[1][0], m[1][1] + mat.m[1][1], m[1][2] + mat.m[1][2],
            m[2][0] + mat.m[2][0], m[2][1] + mat.m[2][1], m[2][2] + mat.m[2][2]);
    }

    //! Operator for Matrix3x3 Minus = Matrix3x3 - Matrix3x3;
    inline_ Matrix3x3 operator-(const Matrix3x3& mat) const
    {
        return Matrix3x3(
            m[0][0] - mat.m[0][0], m[0][1] - mat.m[0][1], m[0][2] - mat.m[0][2],
            m[1][0] - mat.m[1][0], m[1][1] - mat.m[1][1], m[1][2] - mat.m[1][2],
            m[2][0] - mat.m[2][0], m[2][1] - mat.m[2][1], m[2][2] - mat.m[2][2]);
    }

    //! Operator for Matrix3x3 Mul = Matrix3x3 * Matrix3x3;
    inline_ Matrix3x3 operator*(const Matrix3x3& mat) const
    {
        return Matrix3x3(
            m[0][0] * mat.m[0][0] + m[0][1] * mat.m[1][0] + m[0][2] * mat.m[2][0],
            m[0][0] * mat.m[0][1] + m[0][1] * mat.m[1][1] + m[0][2] * mat.m[2][1],
            m[0][0] * mat.m[0][2] + m[0][1] * mat.m[1][2] + m[0][2] * mat.m[2][2],

            m[1][0] * mat.m[0][0] + m[1][1] * mat.m[1][0] + m[1][2] * mat.m[2][0],
            m[1][0] * mat.m[0][1] + m[1][1] * mat.m[1][1] + m[1][2] * mat.m[2][1],
            m[1][0] * mat.m[0][2] + m[1][1] * mat.m[1][2] + m[1][2] * mat.m[2][2],

            m[2][0] * mat.m[0][0] + m[2][1] * mat.m[1][0] + m[2][2] * mat.m[2][0],
            m[2][0] * mat.m[0][1] + m[2][1] * mat.m[1][1] + m[2][2] * mat.m[2][1],
            m[2][0] * mat.m[0][2] + m[2][1] * mat.m[1][2] + m[2][2] * mat.m[2][2]);
    }

    //! Operator for Point Mul = Matrix3x3 * Point;
    inline_ Point operator*(const Point& v) const
    {
        return Point(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                     m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                     m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
    }

    //! Operator for Matrix3x3 Mul = Matrix3x3 * float;
    inline_ Matrix3x3 operator*(float s) const
    {
        return Matrix3x3(
            m[0][0] * s, m[0][1] * s, m[0][2] * s,
            m[1][0] * s, m[1][1] * s, m[1][2] * s,
            m[2][0] * s, m[2][1] * s, m[2][2] * s);
    }

    //! Operator for Matrix3x3 Mul = float * Matrix3x3;
    inline_ friend Matrix3x3 operator*(float s, const Matrix3x3& mat)
    {
        return Matrix3x3(
            s*mat.m[0][0], s*mat.m[0][1], s*mat.m[0][2],
            s*mat.m[1][0], s*mat.m[1][1], s*mat.m[1][2],
            s*mat.m[2][0], s*mat.m[2][1], s*mat.m[2][2]);
    }

    //! Operator for Matrix3x3 Div = Matrix3x3 / float;
    inline_    Matrix3x3        operator/(float s)                const
    {
        if (s)    s = 1.0f / s;
        return Matrix3x3(
            m[0][0] * s, m[0][1] * s, m[0][2] * s,
            m[1][0] * s, m[1][1] * s, m[1][2] * s,
            m[2][0] * s, m[2][1] * s, m[2][2] * s);
    }

    //! Operator for Matrix3x3 Div = float / Matrix3x3;
    inline_ friend Matrix3x3 operator/(float s, const Matrix3x3& mat)
    {
        return Matrix3x3(
            s / mat.m[0][0], s / mat.m[0][1], s / mat.m[0][2],
            s / mat.m[1][0], s / mat.m[1][1], s / mat.m[1][2],
            s / mat.m[2][0], s / mat.m[2][1], s / mat.m[2][2]);
    }

    //! Operator for Matrix3x3 += Matrix3x3
    inline_ Matrix3x3& operator+=(const Matrix3x3& mat)
    {
        m[0][0] += mat.m[0][0];        m[0][1] += mat.m[0][1];        m[0][2] += mat.m[0][2];
        m[1][0] += mat.m[1][0];        m[1][1] += mat.m[1][1];        m[1][2] += mat.m[1][2];
        m[2][0] += mat.m[2][0];        m[2][1] += mat.m[2][1];        m[2][2] += mat.m[2][2];
        return    *this;
    }

    //! Operator for Matrix3x3 -= Matrix3x3
    inline_ Matrix3x3& operator-=(const Matrix3x3& mat)
    {
        m[0][0] -= mat.m[0][0]; m[0][1] -= mat.m[0][1]; m[0][2] -= mat.m[0][2];
        m[1][0] -= mat.m[1][0]; m[1][1] -= mat.m[1][1]; m[1][2] -= mat.m[1][2];
        m[2][0] -= mat.m[2][0]; m[2][1] -= mat.m[2][1]; m[2][2] -= mat.m[2][2];
        return *this;
    }

    //! Operator for Matrix3x3 *= Matrix3x3
    inline_ Matrix3x3& operator*=(const Matrix3x3& mat)
    {
        Point TempRow;

        GetRow(0, TempRow);
        m[0][0] = TempRow.x*mat.m[0][0] + TempRow.y*mat.m[1][0] + TempRow.z*mat.m[2][0];
        m[0][1] = TempRow.x*mat.m[0][1] + TempRow.y*mat.m[1][1] + TempRow.z*mat.m[2][1];
        m[0][2] = TempRow.x*mat.m[0][2] + TempRow.y*mat.m[1][2] + TempRow.z*mat.m[2][2];

        GetRow(1, TempRow);
        m[1][0] = TempRow.x*mat.m[0][0] + TempRow.y*mat.m[1][0] + TempRow.z*mat.m[2][0];
        m[1][1] = TempRow.x*mat.m[0][1] + TempRow.y*mat.m[1][1] + TempRow.z*mat.m[2][1];
        m[1][2] = TempRow.x*mat.m[0][2] + TempRow.y*mat.m[1][2] + TempRow.z*mat.m[2][2];

        GetRow(2, TempRow);
        m[2][0] = TempRow.x*mat.m[0][0] + TempRow.y*mat.m[1][0] + TempRow.z*mat.m[2][0];
        m[2][1] = TempRow.x*mat.m[0][1] + TempRow.y*mat.m[1][1] + TempRow.z*mat.m[2][1];
        m[2][2] = TempRow.x*mat.m[0][2] + TempRow.y*mat.m[1][2] + TempRow.z*mat.m[2][2];
        return    *this;
    }

    //! Operator for Matrix3x3 *= float
    inline_ Matrix3x3& operator*=(float s)
    {
        m[0][0] *= s; m[0][1] *= s; m[0][2] *= s;
        m[1][0] *= s; m[1][1] *= s; m[1][2] *= s;
        m[2][0] *= s; m[2][1] *= s; m[2][2] *= s;
        return *this;
    }

    //! Operator for Matrix3x3 /= float
    inline_ Matrix3x3& operator/=(float s)
    {
        if (s)
            s = 1.0f / s;
        m[0][0] *= s;
        m[0][1] *= s;
        m[0][2] *= s;
        m[1][0] *= s;
        m[1][1] *= s;
        m[1][2] *= s;
        m[2][0] *= s;
        m[2][1] *= s;
        m[2][2] *= s;
        return *this;
    }

    // Cast operators
    //! Cast a Matrix3x3 to a Matrix4x4.
    operator Matrix4x4() const;
    //! Cast a Matrix3x3 to a Quat.
    operator Quat() const;

    inline_ const Point& operator[](int row) const { return *(const Point*)&m[row][0]; }
    inline_ Point& operator[](int row) { return *(Point*)&m[row][0]; }

public:
    float m[3][3];
};

#endif // __ICEMATRIX3X3_H__

