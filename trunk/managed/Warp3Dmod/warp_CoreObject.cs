using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_CoreObject.
    /// </summary>
    /// 

    public class warp_CoreObject
    {
        public warp_Matrix matrix = new();
        public warp_Matrix normalmatrix = new();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void transform(warp_Matrix m)
        {
            matrix.transform(m);
            normalmatrix.transform(m);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void shift(float dx, float dy, float dz)
        {
            matrix.shift(dx, dy, dz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void shift(warp_Vector v)
        {
            matrix.shift(v.x, v.y, v.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void scale(float d)
        {
            matrix.scale(d);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void scale(float dx, float dy, float dz)
        {
            matrix.scale(dx, dy, dz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void scaleSelf(float d)
        {
            matrix.scaleSelf(d);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void scaleSelf(float dx, float dy, float dz)
        {
            matrix.scaleSelf(dx, dy, dz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotate(warp_Vector d)
        {
            rotateSelf(d.x, d.y, d.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotateSelf(warp_Vector d)
        {
            rotateSelf(d.x, d.y, d.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotate(float dx, float dy, float dz)
        {
            matrix.rotate(dx, dy, dz);
            normalmatrix.rotate(dx, dy, dz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotate(warp_Quaternion quat, float x, float y, float z)
        {
            matrix.rotate(quat, x, y, z);
            normalmatrix.rotate(quat, x, y, z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotate(warp_Matrix m)
        {
            matrix.rotate(m);
            normalmatrix.rotate(m);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotateSelf(float dx, float dy, float dz)
        {
            matrix.rotateSelf(dx, dy, dz);
            normalmatrix.rotateSelf(dx, dy, dz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotateSelf(warp_Quaternion quat)
        {
            matrix.rotateSelf(quat);
            normalmatrix.rotateSelf(quat);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotateSelf(warp_Matrix m)
        {
            matrix.rotateSelf(m);
            normalmatrix.rotateSelf(m);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void setPos(float x, float y, float z)
        {
            matrix.m03 = x;
            matrix.m13 = y;
            matrix.m23 = z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void setPos(warp_Vector v)
        {
            setPos(v.x, v.y, v.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Vector getPos()
        {
            return new warp_Vector(matrix.m03, matrix.m13, matrix.m23);
        }
    }
}
