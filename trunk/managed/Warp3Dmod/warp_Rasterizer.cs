using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Rasterizer.
    /// </summary>
    public class warp_Rasterizer
    {
        private bool materialLoaded = false;
        private bool lightmapLoaded = false;
        public bool ready = false;

        // Current material settings
        private int color = 0;
        private int currentColor = 0;
        private int reflectivity = 0;
        private warp_Texture texture = null;
        private int[] tpixels = null;
        private int[] envmap = null;
        private int[] diffuse = null;
        private int[] specular = null;
        private int tw = 0;
        private int th = 0;
        private int tbitW = 0;
        private int tbitH = 0;

        // Rasterizer hints
        private int mode = 0;
        private const int F = 0;      // FLAT
        private const int W = 1;  // WIREFRAME
        private const int P = 2;      // PHONG
        private const int E = 4;      // ENVMAP
        private const int T = 8;  // TEXTURED
        private const int OPAQUE = 16;  // opace

        private int
            y, z, dx, dy, dz, offset,
            xL, xR, xBase, zBase, xMax, dxL, dxR, dzBase,
            nxBase, nyBase,
            dnx, dny, nx, ny,
            dnxBase, dnyBase;

        private float
            txBase, tyBase,
            tx, ty, sw,
            dtxBase, dtyBase,
            dtx, dty,
            swBase, dsw, dswBase;

        warp_Screen screen;
        int[] zBuffer;
        int width, height;

        // Constructor
        public warp_Rasterizer(warp_RenderPipeline pipeline)
        {
            rebuildReferences(pipeline);
            loadLightmap(pipeline.lightmap);
        }

        // References
        public void rebuildReferences(warp_RenderPipeline pipeline)
        {
            screen = pipeline.screen;
            zBuffer = pipeline.zBuffer;
            width = screen.width;
            height = screen.height;
        }

        public void clean()
        {
            screen = null;
            zBuffer = null;
            tpixels = null;
            envmap = null;
            diffuse = null;
            specular = null;
        }

        // Lightmap loader
        public void loadLightmap(warp_Lightmap lm)
        {
            if (lm == null)
                return;
            diffuse = lm.diffuse;
            specular = lm.specular;
            lightmapLoaded = true;
            ready = materialLoaded;
        }

        // Material loader
        public void loadMaterial(warp_Object obj)
        {
            warp_Material material = obj.material;
            color = material.color;
            reflectivity = material.reflectivity;
            texture = material.texture;
            envmap = material.envmap is null ? null : envmap = material.envmap.pixel;

            if (texture is not null)
            {
                if (obj.projectedmaxMips < 2)
                {
                    color = warp_Color.multiply(color, texture.averageColor);
                    texture = null;
                }
                else
                {
                    int mip = obj.projectedmaxMips - 2;
                    if (mip < texture.maxmips)
                    {
                        if (!texture.hasmips)
                            texture.GenMips();

                        tpixels = texture.mips[mip];
                        tbitW = texture.mipsBitWidth[mip];
                        tbitH = texture.mipsBitHeight[mip];
                        tw = texture.mipstw[mip] - 1;
                        th = texture.mipsth[mip] - 1;
                    }
                    else
                    {
                        tpixels = texture.pixel;
                        tw = texture.width - 1;
                        th = texture.height - 1;
                        tbitW = texture.bitWidth;
                        tbitH = texture.bitHeight;
                    }
                }
            }

            mode = 0;
            if (!material.flat)
                mode |= P;
            if (envmap is not null)
                mode |= E;
            if (texture is not null)
                mode |= T;
            if (material.wireframe)
                mode |= W;
            if (material.opaque)
                mode |= OPAQUE;

            materialLoaded = true;
            ready = lightmapLoaded;
        }

        public void render(warp_Triangle tri)
        {
            if (!ready || tri.parent is null)
                return;

            if ((mode & W) != 0)
            {
                drawWireframe(tri, color);
                if ((mode & ~W) == 0)
                {
                    return;
                }
            }

            warp_Vertex p1;
            warp_Vertex p2;

            if (tri.p1.y > tri.p2.y)
            {
                p1 = tri.p2;
                p2 = tri.p1;
            }
            else
            {
                p1 = tri.p1;
                p2 = tri.p2;
            }

            warp_Vertex p3;
            if (p2.y > tri.p3.y)
            {
                p3 = p2;
                if (p1.y > tri.p3.y)
                {
                    p2 = p1;
                    p1 = tri.p3;
                }
                else
                    p2 = tri.p3;
            }
            else
                p3 = tri.p3;

            if (p1.y >= height)
                return;

            if (p3.y < 0)
                return;

            if (p1.y == p3.y)
                return;

            if ((mode & 0x0f) == F)
            {
                int lutID = (int)(tri.n2.x * 127 + 127) + ((int)(tri.n2.y * 127 + 127) << 8);
                int c = warp_Color.multiply(color, diffuse[lutID]);
                int s = warp_Color.scale(specular[lutID], reflectivity);
                currentColor = warp_Color.add(c, s);
            }

            int x1 = p1.x << 8;
            int x2 = p2.x << 8;
            int x3 = p3.x << 8;
            int y1 = p1.y;
            int y2 = p2.y;
            int y3 = p3.y;

            dy = y2 - y1;
            int dy31 = y3 - y1;

            float tf = (float)dy / dy31;

            int x4 = x1 + (int)((x3 - x1) * tf);

            dx = (x4 - x2) >> 8;
            if (dx == 0)
                return;

            x1 <<= 8;
            x2 <<= 8;
            x3 <<= 8;
            x4 <<= 8;

            //temp = (dy21 << 8) / dy31;

            int z1 = p1.z;
            int z3 = p3.z;
            //z4 = z1 + ((z3 - z1) >> 8) * temp;
            int z4 = z1 + (int)((z3 - z1) * tf);
            int z2 = p2.z;
            dz = (z4 - z2) / dx;

            int nx1 = p1.nx;
            int nx3 = p3.nx;
            //nx4 = nx1 + ((nx3 - nx1) >> 8) * temp;
            int nx4 = nx1 + (int)((nx3 - nx1) * tf);
            int nx2 = p2.nx;
            dnx = (nx4 - nx2) / dx;

            int ny1 = p1.ny;
            int ny3 = p3.ny;
            //ny4 = ny1 + ((ny3 - ny1) >> 8) * temp;
            int ny4 = ny1 + (int)((ny3 - ny1) * tf);
            int ny2 = p2.ny;
            dny = (ny4 - ny2) / dx;

            float sw1 = p1.invZ;
            float sw3 = p3.invZ;
            float sw4 = sw1 + ((sw3 - sw1) * tf);
            float sw2 = p2.invZ;
            dsw = (sw4 - sw2) / dx;

            float tx1 = sw1 * p1.u * tw;
            float tx3 = sw3 * p3.u * tw;
            float tx4 = tx1 + ((tx3 - tx1) * tf);
            float tx2 = sw2 * p2.u * tw;
            dtx = (tx4 - tx2) / dx;

            float ty1 = sw1 * p1.v * th;
            float ty3 = sw3 * p3.v * th;
            float ty4 = ty1 + ((ty3 - ty1) * tf);
            float ty2 = sw2 * p2.v * th;
            dty = (ty4 - ty2) / dx;


            if (dx < 0)
            {
                (x4, x2) = (x2, x4);
                z2 = z4;
                tx2 = tx4;
                ty2 = ty4;
                sw2 = sw4;
                nx2 = nx4;
                ny2 = ny4;
            }

            if (y2 >= 0 && dy > 0)
            {
                dxL = (x2 - x1) / dy;
                dxR = (x4 - x1) / dy;
                dzBase = (z2 - z1) / dy;
                dnxBase = (nx2 - nx1) / dy;
                dnyBase = (ny2 - ny1) / dy;
                dtxBase = (tx2 - tx1) / dy;
                dtyBase = (ty2 - ty1) / dy;
                dswBase = (sw2 - sw1) / dy;

                xBase = x1;
                xMax = x1;
                zBase = z1;
                nxBase = nx1;
                nyBase = ny1;
                txBase = tx1;
                tyBase = ty1;
                swBase = sw1;

                if (y1 < 0)
                {
                    xBase -= y1 * dxL;
                    xMax -= y1 * dxR;
                    zBase -= y1 * dzBase;
                    nxBase -= y1 * dnxBase;
                    nyBase -= y1 * dnyBase;
                    txBase -= y1 * dtxBase;
                    tyBase -= y1 * dtyBase;
                    swBase -= y1 * dswBase;
                    y1 = 0;
                }

                if (y2 > height)
                    y2 = height;

                renderLines(y1 * width, y2 * width);
            }

            dy = y3 - y2;
            if (y2 < height && dy > 0)
            {
                dxL = (x3 - x2) / dy;
                dxR = (x3 - x4) / dy;
                dzBase = (z3 - z2) / dy;
                dnxBase = (nx3 - nx2) / dy;
                dnyBase = (ny3 - ny2) / dy;
                dtxBase = (tx3 - tx2) / dy;
                dtyBase = (ty3 - ty2) / dy;
                dswBase = (sw3 - sw2) / dy;

                xBase = x2;
                xMax = x4;
                zBase = z2;
                nxBase = nx2;
                nyBase = ny2;
                txBase = tx2;
                tyBase = ty2;
                swBase = sw2;

                if (y2 < 0)
                {
                    xBase -= y2 * dxL;
                    xMax -= y2 * dxR;
                    zBase -= y2 * dzBase;
                    nxBase -= y2 * dnxBase;
                    nyBase -= y2 * dnyBase;
                    txBase -= y2 * dtxBase;
                    tyBase -= y2 * dtyBase;
                    swBase -= y2 * dswBase;
                    y2 = 0;
                }

                if (y3 > height)
                    y3 = height;

                renderLines(y2 * width, y3 * width);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            void renderLines(int offsetStart, int offsetEnd)
            {
                ref int rpixels = ref MemoryMarshal.GetArrayDataReference(screen.pixels);
                ref int rzbuf = ref MemoryMarshal.GetArrayDataReference(zBuffer);
                ref int rdifuse = ref MemoryMarshal.GetArrayDataReference(diffuse);
                ref int rspecular = ref MemoryMarshal.GetArrayDataReference(specular);
                ref int rtpixels = ref MemoryMarshal.GetArrayDataReference(tpixels);

                for (offset = offsetStart; offset < offsetEnd; offset += width)
                {
                    xL = xBase >> 16;
                    xR = xMax >> 16;
                    z = zBase;
                    nx = nxBase;
                    ny = nyBase;
                    tx = txBase;
                    ty = tyBase;
                    sw = swBase;

                    if (xL < 0)
                    {
                        z -= xL * dz;
                        nx -= xL * dnx;
                        ny -= xL * dny;
                        tx -= xL * dtx;
                        ty -= xL * dty;
                        sw -= xL * dsw;
                        xL = 0;
                    }
                    if (xR > width)
                        xR = width;

                    int posEnd = xR + offset;
                    switch (mode)
                    {
                        case F:
                            {
                                //renderLineF;
                                for (int pos = xL + offset; pos < posEnd; pos++)
                                {
                                    if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                    {
                                        Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;
                                        int bkgrd = Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos));
                                        Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = warp_Color.overSolid(bkgrd, currentColor);
                                    }
                                    z += dz;
                                }
                                break;
                            }

                        case F | OPAQUE:
                            {
                                //renderLineFO
                                for (int pos = xL + offset; pos < posEnd; pos++)
                                {
                                    if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                    {
                                        Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;
                                        Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = currentColor;
                                    }
                                    z += dz;
                                }
                                break;
                            }
                        case P:
                            //renderLineP();
                            for (int pos = xL + offset; pos < posEnd; pos++)
                            {
                                if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                {
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;

                                    int lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);
                                    int c = warp_Color.multiply(color, Unsafe.As<int, int>(ref Unsafe.Add(ref rdifuse, lutID)));
                                    int s = warp_Color.scale(Unsafe.As<int, int>(ref Unsafe.Add(ref rspecular, lutID)), reflectivity);
                                    int bkgrd = Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos));
                                    c = warp_Color.overSolid(bkgrd, c);
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = warp_Color.add(c, s);
                                }
                                z += dz;
                                nx += dnx;
                                ny += dny;
                            }
                            break;
                        case P | OPAQUE:
                            //renderLinePO();
                            for (int pos = xL + offset; pos < posEnd; pos++)
                            {
                                if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                {
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;

                                    int lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);
                                    int c = warp_Color.multiply(color, Unsafe.As<int, int>(ref Unsafe.Add(ref rdifuse, lutID)));
                                    int s = warp_Color.scale(Unsafe.As<int, int>(ref Unsafe.Add(ref rspecular, lutID)), reflectivity);
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = warp_Color.add(c, s);
                                }
                                z += dz;
                                nx += dnx;
                                ny += dny;
                            }
                            break;
                        case E:
                            //renderLineE();
                            ref int renvmap = ref MemoryMarshal.GetArrayDataReference(envmap);
                            for (int pos = xL + offset; pos < posEnd; pos++)
                            {
                                if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                {
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;

                                    int lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);
                                    int s = Unsafe.As<int, int>(ref Unsafe.Add(ref rspecular, lutID));
                                    s = warp_Color.add(s, Unsafe.As<int, int>(ref Unsafe.Add(ref renvmap, lutID)));
                                    s = warp_Color.scale(s, reflectivity);

                                    int bkgrd = Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos));
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = warp_Color.overSolid(bkgrd, s);
                                }

                                z += dz;
                                nx += dnx;
                                ny += dny;
                            }
                            break;

                        case E | OPAQUE:
                            //renderLineEO();
                            renvmap = ref MemoryMarshal.GetArrayDataReference(envmap);
                            for (int pos = xL + offset; pos < posEnd; pos++)
                            {
                                if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                {
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;

                                    int lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);
                                    int s = Unsafe.As<int, int>(ref Unsafe.Add(ref rspecular, lutID));
                                    s = warp_Color.add(s, Unsafe.As<int, int>(ref Unsafe.Add(ref renvmap, lutID)));
                                    s = warp_Color.scale(s, reflectivity);
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = warp_Color.scale(s, reflectivity);
                                }

                                z += dz;
                                nx += dnx;
                                ny += dny;
                            }
                            break;
                        case T:
                            //renderLineT();
                            for (int pos = xL + offset; pos < posEnd; pos++)
                            {
                                if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                {
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;

                                    int tpindx = (((int)(tx / sw)) & tw) + ((((int)(ty / sw)) & th) << tbitW);
                                    int c = Unsafe.As<int, int>(ref Unsafe.Add(ref rtpixels, tpindx));
                                    c = warp_Color.multiply(color, c);

                                    int bkgrd = Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos));
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = warp_Color.overSolid(bkgrd, c);
                                }

                                z += dz;
                                tx += dtx;
                                ty += dty;
                                sw += dsw;
                            }
                            break;

                        case T | OPAQUE:
                            //renderLineTO();
                            for (int pos = xL + offset; pos < posEnd; pos++)
                            {
                                if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                {
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;

                                    int tpindx = (((int)(tx / sw)) & tw) + ((((int)(ty / sw)) & th) << tbitW);
                                    int c = Unsafe.As<int, int>(ref Unsafe.Add(ref rtpixels, tpindx));
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = warp_Color.multiply(color, c);
                                }

                                z += dz;
                                tx += dtx;
                                ty += dty;
                                sw += dsw;
                            }
                            break;
                        case P | E:
                            renderLinePE();
                            break;
                        case P | E | OPAQUE:
                            renderLinePEO();
                            break;

                        case P | T:
                            //renderLinePT();
                            for (int pos = xL + offset; pos < posEnd; pos++)
                            {
                                if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                {
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;

                                    int tpindx = (((int)(tx / sw)) & tw) + ((((int)(ty / sw)) & th) << tbitW);
                                    int c = Unsafe.As<int, int>(ref Unsafe.Add(ref rtpixels, tpindx));
                                    c = warp_Color.multiply(color, c);

                                    int lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);
                                    c = warp_Color.multiply(c, Unsafe.As<int, int>(ref Unsafe.Add(ref rdifuse, lutID)));
                                    int s = warp_Color.scale(Unsafe.As<int, int>(ref Unsafe.Add(ref rspecular, lutID)), reflectivity);

                                    int bkgrd = Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos));
                                    c = warp_Color.overSolid(bkgrd, c);
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = warp_Color.add(c, s);
                                }

                                z += dz;
                                nx += dnx;
                                ny += dny;
                                tx += dtx;
                                ty += dty;
                                sw += dsw;
                            }
                            break;

                        case P | T | OPAQUE:
                            //renderLinePTO();
                            for (int pos = xL + offset; pos < posEnd; pos++)
                            {
                                if (z < Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)))
                                {
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rzbuf, pos)) = z;

                                    int tpindx = (((int)(tx / sw)) & tw) + ((((int)(ty / sw)) & th) << tbitW);
                                    int c = Unsafe.As<int, int>(ref Unsafe.Add(ref rtpixels, tpindx));
                                    c = warp_Color.multiply(color, c);

                                    int lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);
                                    c = warp_Color.multiply(c, Unsafe.As<int, int>(ref Unsafe.Add(ref rdifuse, lutID)));
                                    int s = warp_Color.scale(Unsafe.As<int, int>(ref Unsafe.Add(ref rspecular, lutID)), reflectivity);
                                    Unsafe.As<int, int>(ref Unsafe.Add(ref rpixels, pos)) = warp_Color.add(c, s);
                                }

                                z += dz;
                                nx += dnx;
                                ny += dny;
                                tx += dtx;
                                ty += dty;
                                sw += dsw;
                            }
                            break;

                        case P | E | T:
                            renderLinePET();
                            break;
                        case P | E | T | OPAQUE:
                            renderLinePETO();
                            break;
                        default:
                            return;
                    };

                    xBase += dxL;
                    xMax += dxR;
                    zBase += dzBase;
                    nxBase += dnxBase;
                    nyBase += dnyBase;
                    txBase += dtxBase;
                    tyBase += dtyBase;
                    swBase += dswBase;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void renderLinePE()
        {
            int bkgrd;
            int c;
            int s;
            int lutID;
            int pos;

            fixed (int* sp = screen.pixels, zB = zBuffer, df = diffuse, spc = specular, env = envmap)
            {
                for (int x = xL; x < xR; x++)
                {
                    pos = x + offset;
                    if (z < zB[pos])
                    {
                        lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);

                        bkgrd = sp[pos];
                        s = warp_Color.add(spc[lutID], env[lutID]);
                        s = warp_Color.scale(s, reflectivity);
                        c = warp_Color.multiply(color, df[lutID]);
                        c = warp_Color.overSolid(bkgrd, c);
                        sp[pos] = warp_Color.add(c, s);
                        zB[pos] = z;
                    }
                    z += dz;
                    nx += dnx;
                    ny += dny;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void renderLinePEO()
        {
            int c;
            int s;
            int lutID;
            int pos;

            fixed (int* sp = screen.pixels, zB = zBuffer, df = diffuse, spc = specular, env = envmap)
            {
                for (int x = xL; x < xR; x++)
                {
                    pos = x + offset;
                    if (z < zB[pos])
                    {
                        lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);

                        s = warp_Color.add(spc[lutID], env[lutID]);
                        s = warp_Color.scale(s, reflectivity);
                        c = warp_Color.multiply(color, df[lutID]);
                        sp[pos] = warp_Color.add(c, s);
                        zB[pos] = z;
                    }
                    z += dz;
                    nx += dnx;
                    ny += dny;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void renderLinePET()
        {
            fixed (int* sp = screen.pixels, zB = zBuffer, df = diffuse, spc = specular, tp = tpixels, env = envmap)
            {
                for (int x = xL; x < xR; x++)
                {
                    int pos = x + offset;
                    if (z < zB[pos])
                    {
                        int lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);
                        int bkgrd = sp[pos];
                        int s = warp_Color.add(spc[lutID], env[lutID]);
                        s = warp_Color.scale(s, reflectivity);
                        int c = tp[(((int)(tx / sw)) & tw) + ((((int)(ty / sw)) & th) << tbitW)];
                        c = warp_Color.multiply(color, c);
                        c = warp_Color.multiply(c, df[lutID]);
                        c = warp_Color.overSolid(bkgrd, c);
                        sp[pos] = warp_Color.add(c, s);
                        zB[pos] = z;
                    }
                    z += dz;
                    nx += dnx;
                    ny += dny;
                    tx += dtx;
                    ty += dty;
                    sw += dsw;
                }
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void renderLinePETO()
        {
            fixed (int* sp = screen.pixels, zB = zBuffer, df = diffuse, spc = specular, tp = tpixels, env = envmap)
            {
                for (int x = xL; x < xR; x++)
                {
                    int pos = x + offset;
                    if (z < zB[pos])
                    {
                        int lutID = ((nx >> 16) & 255) + (((ny >> 16) & 255) << 8);
                        int s = warp_Color.add(spc[lutID], env[lutID]);
                        s = warp_Color.scale(s, reflectivity);
                        int c = tp[(((int)(tx / sw)) & tw) + ((((int)(ty / sw)) & th) << tbitW)];
                        c = warp_Color.multiply(color, c);
                        c = warp_Color.multiply(c, df[lutID]);
                        sp[pos] = warp_Color.add(c, s);
                        zB[pos] = z;
                    }
                    z += dz;
                    nx += dnx;
                    ny += dny;
                    tx += dtx;
                    ty += dty;
                    sw += dsw;
                }
            }
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private void drawWireframe(warp_Triangle tri, int defaultcolor)
        {
            drawLine(tri.p1, tri.p2, defaultcolor);
            drawLine(tri.p2, tri.p3, defaultcolor);
            drawLine(tri.p3, tri.p1, defaultcolor);
        }

        private unsafe void drawLine(warp_Vertex a, warp_Vertex b, int color)
        {
            warp_Vertex temp;
            if ((a.clipcode & b.clipcode) != 0)
                return;

            dx = (int)MathF.Abs(a.x - b.x);
            dy = (int)MathF.Abs(a.y - b.y);
            dz = 0;

            fixed (int* sp = screen.pixels, zB = zBuffer)
            {
                if (dx > dy)
                {
                    if (a.x > b.x)
                    {
                        temp = a;
                        a = b;
                        b = temp;
                    }
                    if (dx > 0)
                    {
                        dz = (b.z - a.z) / dx;
                        dy = ((b.y - a.y) << 16) / dx;
                    }
                    z = a.z;
                    y = a.y << 16;
                    for (int x = a.x; x <= b.x; x++)
                    {
                        int y2 = y >> 16;
                        if (warp_Math.inrange(x, 0, width - 1) && warp_Math.inrange(y2, 0, height - 1))
                        {
                            offset = y2 * width + x;
                            if (z < zB[offset])
                            {
                                {
                                    sp[offset] = color;
                                    zB[offset] = z;
                                }
                            }
                        }
                        z += dz;
                        y += dy;
                    }
                }
                else
                {
                    if (a.y > b.y)
                    {
                        temp = a;
                        a = b;
                        b = temp;
                    }
                    if (dy > 0)
                    {
                        dz = (b.z - a.z) / dy;
                        dx = ((b.x - a.x) << 16) / dy;
                    }
                    z = a.z;
                    int x = a.x << 16;
                    for (y = a.y; y <= b.y; y++)
                    {
                        int x2 = x >> 16;
                        if (warp_Math.inrange(x2, 0, width - 1) && warp_Math.inrange(y, 0, height - 1))
                        {
                            offset = y * width + x2;
                            if (z < zB[offset])
                            {
                                {
                                    sp[offset] = color;
                                    zB[offset] = z;
                                }
                            }
                        }
                        z += dz;
                        x += dx;
                    }
                }
            }
        }
    }
}
