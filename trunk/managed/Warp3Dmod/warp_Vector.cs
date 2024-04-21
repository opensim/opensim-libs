using System;
using System.Runtime.CompilerServices;

namespace Warp3D
{
    public struct warp_Vector
    {
        public float x = 0;
        public float y = 0;
        public float z = 0;

        public warp_Vector()
        {
        }

        public warp_Vector(float xpos, float ypos, float zpos)
        {
            x = xpos;
            y = ypos;
            z = zpos;
        }

        public static warp_Vector FromXYZ(float xpos, float zpos, float ypos)
        {
            return new warp_Vector
            {
                x = xpos,
                y = ypos,
                z = zpos
            };
        }
        public void normalize()
        // Normalizes the vector
        {
            float dist = lengthSquare();
            if (dist > 1e-6f)
            {

                float invdist = 1f / MathF.Sqrt(dist);
                x *= invdist;
                y *= invdist;
                z *= invdist;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Vector reverse()
        // Reverses the vector
        {
            x = -x;
            y = -y;
            z = -z;
            return this;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float length()
        {
            return MathF.Sqrt(x * x + y * y + z * z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float lengthSquare()
        {
            return (x * x + y * y + z * z);
        }

        public warp_Vector transform(warp_Matrix m)
        // Modifies the vector by matrix m
        {
            return new warp_Vector(
                x * m.m00 + y * m.m01 + z * m.m02 + m.m03,
                x * m.m10 + y * m.m11 + z * m.m12 + m.m13,
                x * m.m20 + y * m.m21 + z * m.m22 + m.m23
                );
        }

        public void transformToXY(warp_Matrix m, out float nx, out float ny)
        // Modifies the vector by matrix m
        {
                nx = x * m.m00 + y * m.m01 + z * m.m02 + m.m03;
                ny = x * m.m10 + y * m.m11 + z * m.m12 + m.m13;
        }

        public warp_Vector pointtransform(warp_Matrix m)
        // Modifies the point by matrix m
        {
            float newx = x * m.m00 + y * m.m01 + z * m.m02 + m.m03;
            float newy = x * m.m10 + y * m.m11 + z * m.m12 + m.m13;
            float newz = x * m.m20 + y * m.m21 + z * m.m22 + m.m23;
            float w = x * m.m30 + y * m.m31 + z * m.m32 + m.m33;
            if (w != 1.0f && w != 0f)
            {
                w = 1.0f / w;
                newx *= w;
                newy *= w;
                newy *= w;
            }

            return new warp_Vector(newx, newy, newz);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static warp_Vector vectorProduct(ref warp_Vector a, ref warp_Vector b)
        // returns a x b
        {
            return new warp_Vector(a.y * b.z - b.y * a.z, a.z * b.x - b.z * a.x, a.x * b.y - b.x * a.y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float vectorsCos(ref warp_Vector a, ref warp_Vector b)
        {
            a.normalize();
            b.normalize();
            return (a.x * b.x + a.y * b.y + a.z * b.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot(ref warp_Vector a, ref warp_Vector b)
        {
            return (a.x * b.x + a.y * b.y + a.z * b.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static warp_Vector add(ref warp_Vector a, ref warp_Vector b)
        {
            return new warp_Vector(a.x + b.x, a.y + b.y, a.z + b.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static warp_Vector sub(ref warp_Vector a, ref warp_Vector b)
        {
            return new warp_Vector(a.x - b.x, a.y - b.y, a.z - b.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static warp_Vector scale(float f, ref warp_Vector a)
        {
            return new warp_Vector(f * a.x, f * a.y, f * a.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float len(warp_Vector a)
        // length of vector
        {
            return MathF.Sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Vector getClone()
        {
            return new warp_Vector(x, y, z);
        }

        public static warp_Vector Zero = new();
    }
}
