using System;
using System.Runtime.CompilerServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Math.
    /// </summary>
    public class warp_Math
    {
        public warp_Math()
        {
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float interpolate(float a, float b, float d)
        {
            float f = (1f - MathF.Cos(d * MathF.PI)) * 0.5f;
            return a + f * (b - a);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float random()
        {
            return Random.Shared.NextSingle();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float random(float min, float max)
        {
            return Random.Shared.NextSingle() * (max - min) + min;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float randomWithDelta(float averidge, float delta)
        {
            return averidge + random() * delta;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float deg2rad(float deg)
        {
            return deg * 0.0174532925194f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float rad2deg(float rad)
        {
            return rad * 57.295779514719f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float sin(float angle)
        {
            return MathF.Sin(angle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float cos(float angle)
        {
            return MathF.Cos(angle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float pythagoras(float a, float b)
        {
            return MathF.Sqrt(a * a + b * b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int pythagoras(int a, int b)
        {
            return (int)MathF.Sqrt(a * a + b * b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int crop(int num, int min, int max)
        {
            return (num < min) ? min : (num > max) ? max : num;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float crop(float num, float min, float max)
        {
            return (num < min) ? min : (num > max) ? max : num;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool inrange(int num, int min, int max)
        {
            return ((num >= min) && (num < max));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void clearBuffer(int[] buffer, int c)
        {
            if (c == 0)
                Array.Clear(buffer);
            else
                Array.Fill(buffer, c);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void clearBuffer(byte[] buffer, byte c)
        {
            if (c == 0)
                Array.Clear(buffer);
            else
                Array.Fill(buffer, c);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void copyBuffer(int[] source, int[] target)
        {
            Array.Copy(source, target, crop(source.GetLength(0), 0, target.GetLength(0)));
        }
    }
}
