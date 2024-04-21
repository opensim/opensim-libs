using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using static System.Net.Mime.MediaTypeNames;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Screen.
    /// </summary>
    unsafe public class warp_Screen
    {
        public int width;
        public int height;
        public int[] pixels;

        public warp_Screen(int w, int h)
        {
            width = w;
            height = h;

            pixels = new int[w * h];
        }

        public void clear(int c)
        {
            warp_Math.clearBuffer(pixels, c);
        }

        public void draw(warp_Texture texture, int posx, int posy, int xsize, int ysize)
        {
            draw(width, height, texture, posx, posy, xsize, ysize);
        }

        public void drawBackground(warp_Texture texture, int posx, int posy, int xsize, int ysize)
        {
            draw(width, height, texture, posx, posy, xsize, ysize);
        }

        public Bitmap getImage()
        {
            GCHandle handle = GCHandle.Alloc(pixels, GCHandleType.Pinned);
            IntPtr pointer = Marshal.UnsafeAddrOfPinnedArrayElement(pixels, 0);
            Bitmap image = new Bitmap(width, height, width * 4, PixelFormat.Format32bppPArgb, pointer);
            Bitmap ret = new Bitmap(image); //duhhhh
            image.Dispose();
            handle.Free();
            return ret;
        }

        private unsafe void draw(int width, int height, warp_Texture texture, int posx, int posy, int xsize, int ysize)
        {
            if (texture is null)
            {
                return;
            }

            int w = xsize;
            int h = ysize;
            int xBase = posx;
            int yBase = posy;
            int tx = texture.width * 255;
            int ty = texture.height * 255;
            int tw = texture.width;
            int dtx = tx / w;
            int dty = ty / h;
            int txBase = warp_Math.crop(-xBase * dtx, 0, 255 * tx);
            int tyBase = warp_Math.crop(-yBase * dty, 0, 255 * ty);
            int xend = warp_Math.crop(xBase + w, 0, width);
            int yend = warp_Math.crop(yBase + h, 0, height);
            int offset1, offset2;
            xBase = warp_Math.crop(xBase, 0, width);
            yBase = warp_Math.crop(yBase, 0, height);

            fixed (int* px = pixels, txp = texture.pixel)
            {
                ty = tyBase;
                for (int j = yBase; j < yend; j++)
                {
                    tx = txBase;
                    offset1 = j * width;
                    offset2 = (ty >> 8) * tw;
                    for (int i = xBase; i < xend; i++)
                    {
                        px[i + offset1] = unchecked((int)0xff000000) | txp[(tx >> 8) + offset2];
                        tx += dtx;
                    }
                    ty += dty;
                }
            }
        }

        public void add(warp_Texture texture, int posx, int posy, int xsize, int ysize)
        {
            add(width, height, texture, posx, posy, xsize, ysize);
        }

        private void add(int width, int height, warp_Texture texture, int posx, int posy, int xsize, int ysize)
        {
            if (texture is null)
            {
                return;
            }

            int w = xsize;
            int h = ysize;
            int xBase = posx;
            int yBase = posy;
            int tx = texture.width * 255;
            int ty = texture.height * 255;
            int tw = texture.width;
            int dtx = tx / w;
            int dty = ty / h;
            int txBase = warp_Math.crop(-xBase * dtx, 0, 255 * tx);
            int tyBase = warp_Math.crop(-yBase * dty, 0, 255 * ty);
            int xend = warp_Math.crop(xBase + w, 0, width);
            int yend = warp_Math.crop(yBase + h, 0, height);
            int offset1, offset2;
            xBase = warp_Math.crop(xBase, 0, width);
            yBase = warp_Math.crop(yBase, 0, height);

            ty = tyBase;
            fixed (int* px = pixels, txp = texture.pixel)
            {
                for (int j = yBase; j < yend; j++)
                {
                    tx = txBase;
                    offset1 = j * width;
                    offset2 = (ty >> 8) * tw;
                    for (int i = xBase; i < xend; i++)
                    {
                        px[i + offset1] = unchecked((int)0xff000000) | warp_Color.add(txp[(tx >> 8) + offset2], px[i + offset1]);
                        tx += dtx;
                    }
                    ty += dty;
                }
            }
        }
    }
}
