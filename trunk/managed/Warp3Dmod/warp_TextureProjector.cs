using System;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_TextureProjector.
    /// </summary>
    public class warp_TextureProjector
    {
        public static void projectFrontal(warp_Object obj)
        {
            obj.rebuild();
            warp_Vector min = obj.fastMin;
            warp_Vector max = obj.fastMax;
            float du = 1 / (max.x - min.x);
            float dv = 1 / (max.y - min.y);

            for (int i = 0; i < obj.vertexData.Count; i++)
            {
                obj.vertexData[i].u = (obj.vertexData[i].pos.x - min.x) * du;
                obj.vertexData[i].v = 1 - (obj.vertexData[i].pos.y - min.y) * dv;
            }
        }

        public static void projectTop(warp_Object obj)
        {
            obj.rebuild();
            warp_Vector min = obj.fastMin;
            warp_Vector max = obj.fastMax;
            float du = 1 / (max.x - min.x);
            float dv = 1 / (max.z - min.z);
            for (int i = 0; i < obj.vertexData.Count; i++)
            {
                obj.vertexData[i].u = (obj.vertexData[i].pos.x - min.x) * du;
                obj.vertexData[i].v = (obj.vertexData[i].pos.z - min.z) * dv;
            }
        }
        /*
		public static void projectCylindric(warp_Object obj)
		{
			obj.rebuild();
			warp_Vector min = obj.minimum();
			warp_Vector max = obj.maximum();
			float dz = 1 / (max.z - min.z);
			for (int i = 0; i < obj.vertices; i++)
			{
				obj.fastvertex[i].pos.buildCylindric();
				obj.fastvertex[i].u = obj.fastvertex[i].pos.theta / (2 * 3.14159265f);
				obj.fastvertex[i].v = (obj.fastvertex[i].pos.z - min.z) * dz;
			}
		}
		*/

        public static void projectCylindric(warp_Object obj)
        {
            obj.rebuild();
            const float inv2p = 0.5f / MathF.PI;
            warp_Vector min = obj.fastMin;
            warp_Vector max = obj.fastMax;
            float dz = 1 / (max.z - min.z);
            for (int i = 0; i < obj.vertexData.Count; i++)
            {
                float theta = MathF.Atan2(obj.vertexData[i].pos.x, obj.vertexData[i].pos.y);
                obj.vertexData[i].u = theta * inv2p;
                obj.vertexData[i].v = (obj.vertexData[i].pos.z - min.z) * dz;
            }
        }
    }
}
