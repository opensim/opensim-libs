using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Lightmap.
    /// </summary>
    public class warp_Lightmap
    {
        public int[] diffuse = new int[65536];
        public int[] specular = new int[65536];
        private readonly float[] sphere = new float[65536];
        private readonly warp_Light[] light;
        private readonly int ambient;
        private int pos;

        const float inv128 = 1.0f / 128f;
        const float inv255 = 1.0f / 255f;
        const float inv4096 = 1.0f / 4096f;

        public warp_Lightmap(warp_Scene scene)
        {
            scene.rebuild();

            light = scene.light;
            ambient = scene.environment.ambient;

            buildSphereMap();
            rebuildLightmap();
        }

        private void buildSphereMap()
        {
            float fnx, fny, fnz;
            float fnySQ;
            int pos;
            ref float sphereref = ref MemoryMarshal.GetArrayDataReference(sphere);

            for (int ny = -128; ny < 128; ny++)
            {
                fny = ny * inv128;
                fnySQ = fny * fny;
                pos = (ny + 128) << 8;
                for (int nx = -128; nx < 128; nx++, pos++)
                {
                    fnx = nx * inv128;
                    fnz = 1f - MathF.Sqrt(fnx * fnx + fnySQ);
                    Unsafe.As<float, float>(ref Unsafe.Add(ref sphereref, pos)) = (fnz > 0) ? fnz : 0;
                }
            }
        }

        public void rebuildLightmap()
        {
            float fnx, fny, phongfact, sheen, spread;
            int diffuse, specular, cos, dr, dg, db, sr, sg, sb;

            ref int difuseref = ref MemoryMarshal.GetArrayDataReference(this.diffuse);
            ref int specularref = ref MemoryMarshal.GetArrayDataReference(this.specular);
            ref float sphereref = ref MemoryMarshal.GetArrayDataReference(sphere);

            for (int ny = -128; ny < 128; ny++)
            {
                fny = ny * inv128;
                pos = (ny + 128) << 8;
                for (int nx = -128; nx < 128; nx++, pos++)
                {
                    fnx = nx * inv128;
                    sr = sg = sb = 0;

                    dr = warp_Color.getRed(ambient);
                    dg = warp_Color.getGreen(ambient);
                    db = warp_Color.getBlue(ambient);

                    for (int i = 0; i < light.Length; i++)
                    {
                        diffuse = light[i].diffuse;
                        specular = light[i].specular;
                        sheen = light[i].highlightSheen * inv255;
                        spread = light[i].highlightSpread * inv4096;
                        spread = (spread < 0.01f) ? 0.01f : spread;
                        warp_Vector tmp = new warp_Vector(fnx, fny, Unsafe.As<float, float>(ref Unsafe.Add(ref sphereref, pos)));
                        cos = (int)(255 * warp_Vector.vectorsCos(ref light[i].v, ref tmp));
                        cos = (cos > 0) ? cos : 0;
                        dr += (warp_Color.getRed(diffuse) * cos) >> 8;
                        dg += (warp_Color.getGreen(diffuse) * cos) >> 8;
                        db += (warp_Color.getBlue(diffuse) * cos) >> 8;
                        phongfact = sheen * (float)MathF.Pow((float)cos * inv255, 1 / spread);
                        sr += (int)((float)warp_Color.getRed(specular) * phongfact);
                        sg += (int)((float)warp_Color.getGreen(specular) * phongfact);
                        sb += (int)((float)warp_Color.getBlue(specular) * phongfact);
                    }
                    Unsafe.As<int, int>(ref Unsafe.Add(ref difuseref, pos)) = warp_Color.getCropColor(dr, dg, db);
                    Unsafe.As<int, int>(ref Unsafe.Add(ref specularref, pos)) = warp_Color.getCropColor(sr, sg, sb);
                }
            }
        }
    }
}
