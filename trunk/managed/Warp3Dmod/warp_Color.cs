using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Color.
    /// </summary>
    public class warp_Color
    {
        public const int MASKALPHA = unchecked((int)0xFF000000); // alpha mask
        public const int MASKHALFALPHA = unchecked((int)0x7F000000); // alpha mask
        public const int MASKRED = unchecked((int)0xFF0000);  // red mask
        public const int MASKGREEN = unchecked((int)0xFF00);  // green mask
        public const int MASKBLUE = unchecked((int)0xFF);  // blue mask
        public const int MASKRB = unchecked((int)0x00FF00FF);  // mask for additive/subtractive shading
        public const int MASKAG = unchecked((int)0xFF00FF00);  // mask for additive/subtractive shading
        public const int RBHALF = unchecked((int)0x00800080);
        public const int MASKRBHALF = unchecked((int)0xFF80FF80);
        public const int MASKRBOVER = unchecked((int)0x01000100);  
        public const int MASK7Bit = unchecked((int)0x00FEFEFF);  // mask for additive/subtractive shading
        public const int MASK6Bit = unchecked((int)0x00FCFCFC);  // mask for additive/subtractive shading
        public const int MASK7BitA = unchecked((int)0x00FEFEFF);  // mask for additive/subtractive shading
        public const int MASK6BitA = unchecked((int)0x00FCFCFC);  // mask for additive/subtractive shading
        public const int MASKRGB = unchecked((int)0x00FFFFFF);  // rgb mask

        public const int White = unchecked((int)0xFFFFFFFF);
        public const int Grey = unchecked((int)0xFF7F7F7F);
        public const int Black = unchecked((int)0xFF000000);

        public static int random(int color, int delta)
        {
            int r = (color >> 16) & 0xff;
            int g = (color >> 8) & 0xff;
            int b = color & 0xff;

            float df = delta;
            r += (int)(Random.Shared.NextSingle() * df);
            g += (int)(Random.Shared.NextSingle() * df);
            b += (int)(Random.Shared.NextSingle() * df);

            return getCropColor(r, g, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int random()
        {
            return (int)(Random.Shared.NextSingle() * 16777216f);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getAlpha(int c)
        {
            return (c >> 24) & 0xff;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getRed(int c)
        {
            return (c >> 16) & 0xff;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getGreen(int c)
        {
            return (c >> 8) & 0xff;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getBlue(int c)
        {
            return c & 0xff;
        }

        public static int[] makeGradient(int[] colors, int size)
        {
            int[] pal = new int[size];
            int c1, c2, pos1, pos2, range;
            int r1, g1, b1, r2, g2, b2, dr, dg, db;
            int clen = colors.GetLength(0) - 1;
            if (clen == 0)
            {
                c1 = colors[0];
                for(int i = 0; i < pal.Length; i++)
                {
                    pal[i] = c1;
                }
                return pal;
            }
            ref int cref = ref MemoryMarshal.GetArrayDataReference(colors);
            ref int palref = ref MemoryMarshal.GetArrayDataReference(pal);
            int invclenm1 = 1 / clen;
            for (int c = 0; c < clen; c++)
            {
                c1 = Unsafe.As<int, int>(ref Unsafe.Add(ref cref, c));
                c2 = Unsafe.As<int, int>(ref Unsafe.Add(ref cref, c + 1));
                pos1 = size * c * invclenm1;
                pos2 = size * (c + 1) * invclenm1;
                range = pos2 - pos1;
                range = 1 / range;
                r1 = warp_Color.getRed(c1) << 16;
                g1 = warp_Color.getGreen(c1) << 16;
                b1 = warp_Color.getBlue(c1) << 16;
                r2 = warp_Color.getRed(c2) << 16;
                g2 = warp_Color.getGreen(c2) << 16;
                b2 = warp_Color.getBlue(c2) << 16;
                dr = (r2 - r1) * range;
                dg = (g2 - g1) * range;
                db = (b2 - b1) * range;
                for(int i = pos1; i < pos2; i++)
                {
                    Unsafe.As<int, int>(ref Unsafe.Add(ref palref, i)) = getColor(r1 >> 16, g1 >> 16, b1 >> 16);
                    r1 += dr;
                    g1 += dg;
                    b1 += db;
                }
            }

            return pal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int add(int color1, int color2)
        {
            int t1,t2,t3,t4;

            t1 = color1 & MASKRB;
            t2 = color2 & MASKRB;
            t3 = t1 + t2;
            t3 |= MASKRBOVER - ((t3 >> 8) & MASKRB);
            t3 &= MASKRB;

            t1 = (color1 >> 8) & MASKRB;
            t2 = (color2 >> 8) & MASKRB;
            t4 = t1 + t2;
            t4 |= MASKRBOVER - ((t4 >> 8) & MASKRB);
            t4 &= MASKRB;

            return (t4 << 8) | t3;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int avg2c(int color1, int color2)
        {
            int t1 = (color1 & MASKRB) + (color2 & MASKRB);
            t1 >>= 1;
            t1 &= MASKRB;

            long t2 = (color1 & MASKAG) + (color2 & MASKAG);
            t2 >>= 1;
            t2 &= MASKAG;

            return (int)t2 | t1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int sub(int color1, int color2)
        // Substracts color2 from color1
        {
            int t1,t2,t3,t4;

            t1 = color1 & MASKRB;
            t2 = (~color2) & MASKRB;
            t3 = t1 + t2;
            t3 |= MASKRBOVER - ((t3 >> 8) & MASKRB);
            t3 &= MASKRB;

            t1 = (color1 >> 8) & MASKRB;
            t2 = ((~color2) >> 8) & MASKRB;
            t4 = t1 + t2;
            t4 |= MASKRBOVER - ((t4 >> 8) & MASKRB);
            t4 &= MASKRB;

            return (t4 << 8) | t3;
        }
        /*
        public static int subneg(int color1, int color2)
        // Substracts the negative of color2 from color1
        {
            int pixel = (color1 & MASK7Bit) + (color2 & MASK7Bit);
            int overflow = ~pixel & 0x1010100;
            overflow = overflow - (overflow >> 8);
            return ALPHA | (~overflow & pixel);
        }

        public static int inv(int color)
        // returns the inverse of the given color
        {
            color = (~color) &  RGB;
            return ALPHA | color;
        }
        */
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int mix(int color1, int color2)
        // Returns the averidge color from 2 colors
        {
            return MASKALPHA | (((color1 & MASK7Bit) >> 1) + ((color2 & MASK7Bit) >> 1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int scale(int color, int factor)
        {
            if(factor == 255)
                return color;

            if(factor == 0)
                return 0;

            int a = color & MASKALPHA;

            if(factor == 127)
                return a | ((color >> 1) & 0x7F7F7F);

            int RB = (((color & MASKRB) * factor) >> 8) & MASKRB;
            int b = (((color & MASKBLUE) * factor) >> 8) & MASKBLUE;
            return a | RB | b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int multiply(int color1, int color2)
        {
            int t1, t2, t3, t4;

            t1 = color1;
            t2 = color2;

            t3 = (t1 & 0xff) * (t2 & 0xff); // b on g
            t3 |= (t1 & MASKRED) * ((t2 >> 16) & 0xff); // r on a
            t3 += RBHALF; // add halfbit
            t3 = (t3 + ((t3 >> 8) & MASKRB)) >> 8; // in rb place
            t3 &= MASKRB;

            t1 = (color1 >> 8) & MASKRB;
            t2 = (color2 >> 8) & MASKRB;

            t4 = (t1 & 0xff) * (t2 & 0xff); // g on g
            t4 |= (t1 & MASKRED) * ((t2 >> 16) & 0xff); // a on a
            t4 += RBHALF; // add halfbit
            t4 += (t4 >> 8) & MASKRB;
            t4 &= MASKAG;

            return t3 | t4;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int transparency(int bkgrd, int color, int alpha)
        {
            switch(alpha)
            {
                case 0:
                    return bkgrd;
                case 255:
                    return color;   
                case 127:
                    return mix(bkgrd, color);
                default:
                    break;
            }

            int colorRB = bkgrd & 0xff00ff;
            colorRB += (((color & 0xff00ff) - colorRB) * alpha) >> 8;
            colorRB &= 0xff00ff;

            int colorg = bkgrd & 0x00ff00;
            colorg += (((color & 0x00ff00) - colorg) * alpha) >> 8;
            colorg &= 0x00ff00;

            return MASKALPHA | colorRB | colorg;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // blends color alpha, but result image is solid
        public static int overSolid(int bkgrd, int color)
        {
            int alpha = color & MASKALPHA;
            switch(alpha)
            {
                case 0:
                    return bkgrd;
                case Black:
                    return color;
                case MASKHALFALPHA:
                    return mix(bkgrd, color);
                default:
                    break;
            }
            alpha = (alpha >> 24) & 0xff;
            int colorRB = bkgrd & MASKRB;
            colorRB += (((color & MASKRB) - colorRB) * alpha) >> 8;
            colorRB &= 0xff00ff;

            int colorg = bkgrd & MASKGREEN;
            colorg += (((color & MASKGREEN) - colorg) * alpha) >> 8;
            colorg &= MASKGREEN;

            return MASKALPHA | colorRB | colorg;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getCropColor(int r, int g, int b)
        {
            return MASKALPHA | (warp_Math.crop(r, 0, 255) << 16) | (warp_Math.crop(g, 0, 255) << 8) | warp_Math.crop(b, 0, 255);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getCropColor(int r, int g, int b, int a)
        {
            return (warp_Math.crop(a, 0, 255) << 24) | (warp_Math.crop(r, 0, 255) << 16) | (warp_Math.crop(g, 0, 255) << 8) | warp_Math.crop(b, 0, 255);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getColor(int r, int g, int b)
        {
            return MASKALPHA | (r << 16) | (g << 8) | b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getColor(int r, int g, int b, int a)
        {
            return (a << 24) | (r << 16) | (g << 8) | b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getGray(int color)
        {
            int r = ((color & MASKRED) >> 16);
            int g = ((color & MASKGREEN) >> 8);
            int b = (color & MASKBLUE);
            int Y = (r * 3 + g * 6 + b) / 10;
            return (color & MASKALPHA) | (Y << 16) | (Y << 8) | Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int getAverage(int color)
        {
            return (((color & MASKRED) >> 16) + ((color & MASKGREEN) >> 8) + (color & MASKBLUE)) / 3;
        }
    }
}
