using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_RenderPipeline.
    /// </summary>
    public class warp_RenderPipeline
    {
        public warp_Screen screen;
        readonly warp_Scene scene;
        public warp_Lightmap lightmap;

        public bool useId = false;

        warp_Rasterizer rasterizer;
        int zFar = int.MaxValue;

        public int[] zBuffer;

        public warp_RenderPipeline(warp_Scene scene, int w, int h)
        {
            this.scene = scene;

            screen = new warp_Screen(w, h);
            zBuffer = new int[screen.width * screen.height];
            rasterizer = new warp_Rasterizer(this);
        }

        public void buildLightMap()
        {
            if (lightmap == null)
            {
                lightmap = new warp_Lightmap(scene);
            }
            else
            {
                lightmap.rebuildLightmap();
            }

            rasterizer.loadLightmap(lightmap);
        }

        public void render(warp_Camera cam)
        {
            rasterizer.rebuildReferences(this);

            warp_Math.clearBuffer(zBuffer, zFar);

            if (scene.environment.background is not null)
            {
                screen.drawBackground(scene.environment.background, 0, 0, screen.width, screen.height);
            }
            else
            {
                screen.clear(scene.environment.bgcolor);
            }

            cam.setScreensize(screen.width, screen.height);
            scene.prepareForRendering();

            // Project
            warp_Matrix m = warp_Matrix.multiply(cam.getMatrix(), scene.matrix);
            warp_Matrix nm = warp_Matrix.multiply(cam.getNormalMatrix(), scene.normalmatrix);
            warp_Matrix vertexProjection = new warp_Matrix();
            warp_Matrix normalProjection = new warp_Matrix();
            warp_Object obj;
            warp_Triangle t;
            warp_Vertex v;
            warp_Material objmaterial;

            int w = screen.width;
            int h = screen.height;
            int minx;
            int miny;
            int maxx;
            int maxy;

            List<warp_Triangle> transparentQueue = new List<warp_Triangle>(4 * scene.wobject.Length);

            for (int id = 0; id < scene.wobject.Length; ++id)
            {
                obj = scene.wobject[id];
                if (!obj.visible)
                    continue;
                objmaterial = obj.material;
                if (objmaterial is null)
                    continue;
                if (objmaterial.opaque && objmaterial.reflectivity == 0)
                    continue;

                vertexProjection.SetFromtransform(obj.matrix, m);
                normalProjection.SetFromtransform(obj.normalmatrix, nm);

                minx = int.MaxValue;
                miny = int.MaxValue;
                maxx = int.MinValue;
                maxy = int.MinValue;

                for (int i = 0; i < obj.vertexData.Count; ++i)
                {
                    v = obj.vertexData[i];
                    v.project(vertexProjection, normalProjection, cam);
                    v.clipFrustrum(w, h);
                    if (minx > v.x)
                        minx = v.x;
                    if (maxx < v.x)
                        maxx = v.x;
                    if (miny > v.y)
                        miny = v.y;
                    if (maxy < v.y)
                        maxy = v.y;
                }
                maxx = Math.Abs(maxx - minx);
                maxy = Math.Abs(maxy - miny);
                if (maxy > maxx)
                    maxx = maxy;

                obj.projectedmaxMips = System.Numerics.BitOperations.Log2((uint)maxx + 1);
                if (objmaterial.opaque)
                {
                    rasterizer.loadMaterial(obj);
                    for (int i = 0; i < obj.triangleData.Count; ++i)
                    {
                        t = obj.triangleData[i];
                        t.project(normalProjection);
                        if (t.clipFrustrum(w, h))
                            rasterizer.render(t);
                    }
                }
                else
                {
                    for (int i = 0; i < obj.triangleData.Count; ++i)
                    {
                        t = obj.triangleData[i];
                        t.project(normalProjection);
                        if (t.clipFrustrum(w, h))
                            transparentQueue.Add(t);
                    }
                }
            }

            //screen.lockImage();
            obj = null;
            if (transparentQueue.Count > 0)
            {
                transparentQueue.Sort(CompareTrisDistance);
                for (int i = 0; i < transparentQueue.Count; i++)
                {
                    if (obj != transparentQueue[i].parent)
                    {
                        obj = transparentQueue[i].parent;
                        rasterizer.loadMaterial(obj);
                    }
                    rasterizer.render(transparentQueue[i]);
                }
            }
            transparentQueue = null;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            int CompareTrisDistance(warp_Triangle x, warp_Triangle y)
            {
                return y.minDistZ.CompareTo(x.minDistZ);
            }
        }
    }
}
