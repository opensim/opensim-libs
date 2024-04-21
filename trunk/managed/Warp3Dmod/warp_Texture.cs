using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Globalization;
using System.Net;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Xml.Linq;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Texture.
    /// </summary>

    public class warp_Texture
    {
        public int width;
        public int height;
        public int bitWidth;
        public int bitHeight;
        public int[] pixel;
        public string path = null;
        public int averageColor;
        public bool hasAlpha;
        public bool opaque;
        public bool hasmips;
        public int[][] mips;
        public int[] mipsBitWidth;
        public int[] mipsBitHeight;
        public int[] mipstw;
        public int[] mipsth;
        public int maxmips;

        public static int ALPHA = unchecked((int)0xFF000000); // alpha mask

        public warp_Texture(int w, int h)
        {
            height = h;
            width = w;
            pixel = new int[w * h];
            cls();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Texture(int w, int h, int[] data)
        {
            height = h;
            width = w;
            pixel = new int[width * height];

            Array.Copy(data, pixel, pixel.Length);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Texture(Bitmap map, int maxBitSize = -1)
        {
            loadTexture(map, maxBitSize);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Texture(string path, int maxBitSize = -1)
        {
            using Bitmap map = new(path, false);
            loadTexture(map, maxBitSize);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void resize()
        {
            int w = (int)Math.Pow(2, bitWidth = (int)Math.Ceiling(MathF.Log2(width)));
            int h = (int)Math.Pow(2, bitHeight = (int)Math.Ceiling(MathF.Log2(height)));

            if (w != width || h != height)
                resize(w, h);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void resize(int w, int h)
        {
            setSize(w, h);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void cls()
        {
            warp_Math.clearBuffer(pixel, 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Texture toAverage()
        {
            ref int rpixel = ref MemoryMarshal.GetArrayDataReference(pixel);
            for (int i = 0; i < pixel.Length; i++)
                Unsafe.As<int, int>(ref Unsafe.Add(ref rpixel, i)) = warp_Color.getAverage(Unsafe.As<int, int>(ref Unsafe.Add(ref rpixel, i)));

            return this;
        }

        public warp_Texture toGray()
        {
            ref int rpixel = ref MemoryMarshal.GetArrayDataReference(pixel);
            for (int i = 0; i < pixel.Length; i++)
                Unsafe.As<int, int>(ref Unsafe.Add(ref rpixel, i)) = warp_Color.getGray(Unsafe.As<int, int>(ref Unsafe.Add(ref rpixel, i)));

            return this;
        }

        public warp_Texture valToGray()
        {
            int intensity;
            ref int rpixel = ref MemoryMarshal.GetArrayDataReference(pixel);
            for (int i = 0; i < pixel.Length; i++)
            {
                intensity = warp_Math.crop(Unsafe.As<int, int>(ref Unsafe.Add(ref rpixel, i)), 0, 255);
                Unsafe.As<int, int>(ref Unsafe.Add(ref rpixel, i)) = warp_Color.getColor(intensity, intensity, intensity);
            }

            return this;
        }

        public warp_Texture colorize(int[] pal)
        {
            int range = pal.GetLength(0) - 1;
            ref int rpixel = ref MemoryMarshal.GetArrayDataReference(pixel);
            for (int i = 0; i < pixel.Length; i++)
                Unsafe.As<int, int>(ref Unsafe.Add(ref rpixel, i)) =
                    pal[warp_Math.crop(Unsafe.As<int, int>(ref Unsafe.Add(ref rpixel, i)), 0, range)];

            return this;
        }

        private void loadTexture(Bitmap pmap, int maxBitSize = -1)
        {
            Bitmap map = pmap;
            bool disposemap = false;

            opaque = true;
            width = map.Width;
            height = map.Height;

            PixelFormat format = pmap.PixelFormat;
            PixelFormat targetformat = format switch
            {
                PixelFormat.Format16bppArgb1555 => PixelFormat.Format32bppArgb,
                PixelFormat.Format32bppArgb => PixelFormat.Format32bppArgb,
                PixelFormat.Format32bppPArgb => PixelFormat.Format32bppArgb,
                PixelFormat.Format64bppArgb => PixelFormat.Format32bppArgb,
                PixelFormat.Format64bppPArgb => PixelFormat.Format32bppArgb,
                _ => PixelFormat.Format24bppRgb
            };

            // make sure sizes are power of 2
            bitWidth = (int)MathF.Ceiling(MathF.Log2(width));
            bitHeight = (int)MathF.Ceiling(MathF.Log2(height));

            if (maxBitSize > 3)
            {
                if (bitWidth > bitHeight)
                {
                    if (bitWidth > maxBitSize)
                    {
                        int ex = bitWidth - maxBitSize;
                        bitWidth = maxBitSize;
                        bitHeight -= ex;
                        if (bitHeight < 0)
                            bitHeight = 0;
                    }
                }
                else
                {
                    if (bitHeight > maxBitSize)
                    {
                        int ex = bitHeight - maxBitSize;
                        bitHeight = maxBitSize;
                        bitWidth -= ex;
                        if (bitWidth < 0)
                            bitWidth = 0;
                    }
                }
            }

            int w = (int)MathF.Pow(2, bitWidth);
            int h = (int)MathF.Pow(2, bitHeight);

            if (width != w || height != h || format != targetformat)
            {
                map = ResizeBitmap(pmap, w, h, targetformat);
                width = w;
                height = h;
                disposemap = true;
            }

            ulong avgR = 0;
            ulong avgG = 0;
            ulong avgB = 0;
            ulong avgA = 0;

            byte blue;
            byte green;
            byte red;
            byte alpha;

            pixel = new int[width * height];
            hasAlpha = targetformat == PixelFormat.Format32bppArgb;

            BitmapData bmData = map.LockBits(new Rectangle(0, 0, map.Width, map.Height),
                            ImageLockMode.ReadOnly, targetformat);
            unsafe
            {
                byte* p = (byte*)bmData.Scan0;
                fixed (int* px = pixel)
                {
                    int nPixel = 0;
                    if (hasAlpha)
                    {
                        int nOffset = bmData.Stride - width * 4;
                        for (int i = 0; i < height; i++)
                        {
                            for (int j = 0; j < width; j++)
                            {
                                blue = *(p++);
                                avgB += blue;
                                green = *(p++);
                                avgG += green;
                                red = *(p++);
                                avgR += red;
                                alpha = *(p++);
                                avgA += alpha;

                                if (opaque && alpha != 0xff)
                                    opaque = false;
                                px[nPixel++] = alpha << 24 | red << 16 | green << 8 | blue;
                            }
                            p += nOffset;
                        }
                    }
                    else
                    {
                        int nOffset = bmData.Stride - width * 3;
                        for (int i = 0; i < height; i++)
                        {
                            for (int j = 0; j < width; j++)
                            {
                                blue = *(p++);
                                avgB += blue;
                                green = *(p++);
                                avgG += green;
                                red = *(p++);
                                avgR += red;

                                px[nPixel++] = ALPHA | red << 16 | green << 8 | blue;
                            }
                            p += nOffset;
                        }
                    }
                }
            }

            map.UnlockBits(bmData);
            if (disposemap)
                map.Dispose();

            ulong np = (ulong)(width * height);

            blue = (byte)(avgB / np);
            green = (byte)(avgG / np);
            red = (byte)(avgR / np);
            if (hasAlpha)
            {
                alpha = (byte)(avgA / np);
                averageColor = alpha << 24 | red << 16 | green << 8 | blue;
            }
            else
                averageColor = ALPHA | red << 16 | green << 8 | blue;

            maxmips = bitHeight;
            if (maxmips > bitWidth)
                maxmips = bitWidth;

            maxmips -= 3; // less 1x1 2x2 and max
            hasmips = false;
        }

        public void GenMips()
        {
            int nmips = maxmips + 1;
            mips = new int[nmips + 1][];
            mips[nmips] = pixel;

            mipsBitWidth = new int[maxmips + 1];
            mipsBitHeight = new int[maxmips + 1];
            mipstw = new int[maxmips + 1];
            mipsth = new int[maxmips + 1];

            int w = width;
            int h = height;
            int bw = bitWidth;
            int bh = bitHeight;

            int[] src;
            int[] dst = mips[maxmips + 1];
            for (int n = maxmips; n >= 0; --n)
            {
                src = dst;
                w >>= 1;
                h >>= 1;
                dst = dec2XY(src, w, h);
                mips[n] = dst;
                mipsBitWidth[n] = --bw;
                mipsBitHeight[n] = --bh;
                mipstw[n] = w;
                mipsth[n] = h;
            }
            hasmips = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe int[] dec2XY(int[] src, int w, int h)
        {
            /*
            int[] dst = new int[w * h];
            int sw = 2 * w;
            int stot = 2 * h * sw;
            int c1;
            int c2;
            int c3;
            int c4;
            int sxl2 = sw;
            int dx = 0;
            fixed (int* s = src, d = dst)
            {
                for (int sx = 0; sx < stot; sx += sw, sxl2 += sw)
                {
                    for (int i = 0; i < w; i++, sx += 2, sxl2 += 2)
                    {
                        c1 = s[sx];
                        c2 = s[sx + 1];
                        c3 = s[sxl2];
                        c4 = s[sxl2 + 1];
                        d[dx++] = warp_Color.avg4c(c1, c2, c3, c4);
                    }
                }
            }
            return dst;
            */
            int[] dst = new int[w * h];
            int sw = 8 * w; // 4 bytes * 2 * w

            fixed (int* si = src, di = dst)
            {
                byte* d = (byte*)di;
                byte* s = (byte*)si;
                byte* s2 = s + sw;
                for (int y = 0; y < h; y++)
                {
                    for (int x = 0; x < w; x++)
                    {
                        int t = s[0] + s[4] + s2[0] + s2[4] + 2;
                        d[0] = (byte)(t >> 2);

                        t = s[1] + s[5] + s2[1] + s2[5] + 2;
                        d[1] = (byte)(t >> 2);

                        t = s[2] + s[6] + s2[2] + s2[6] + 2;
                        d[2] = (byte)(t >> 2);

                        t = s[3] + s[7] + s2[3] + s2[7] + 2;
                        d[3] = (byte)(t >> 2);

                        d += 4;
                        s += 8;
                        s2 += 8;
                    }
                    s += sw;
                    s2 += sw;
                }
            }
            return dst;
        }

        private unsafe void setSize(int w, int h)
        {
            int offset = w * h;
            int offset2;
            if (w * h != 0)
            {
                int[] newpixels = new int[w * h];
                fixed (int* np = newpixels, p = pixel)
                {
                    for (int j = h - 1; j >= 0; j--)
                    {
                        offset -= w;
                        offset2 = (j * height / h) * width;
                        for (int i = w - 1; i >= 0; i--)
                            np[i + offset] = p[(i * width / w) + offset2];
                    }
                }
                width = w;
                height = h;
                pixel = newpixels;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool inrange(int a, int b, int c)
        {
            return (a >= b) & (a < c);
        }

        public warp_Texture getClone()
        {
            warp_Texture t = new(width, height);
            warp_Math.copyBuffer(pixel, t.pixel);
            return t;
        }

        public static Bitmap ResizeBitmap(Image image, int width, int height, PixelFormat format)
        {
            Bitmap result = new(width, height, format);

            using (ImageAttributes atrib = new())
            using (Graphics graphics = Graphics.FromImage(result))
            {
                atrib.SetWrapMode(System.Drawing.Drawing2D.WrapMode.TileFlipXY);
                atrib.SetWrapMode(System.Drawing.Drawing2D.WrapMode.TileFlipXY);
                graphics.CompositingQuality = System.Drawing.Drawing2D.CompositingQuality.HighQuality;
                graphics.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                graphics.PixelOffsetMode = System.Drawing.Drawing2D.PixelOffsetMode.None;

                graphics.DrawImage(image, new Rectangle(0, 0, result.Width, result.Height),
                    0, 0, image.Width, image.Height, GraphicsUnit.Pixel, atrib);
            }

            return result;
        }

    }
}
