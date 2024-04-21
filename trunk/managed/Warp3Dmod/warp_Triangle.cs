using System;
using System.Runtime.CompilerServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Triangle.
    /// </summary>
    public class warp_Triangle
    {
        public const float oneThird = 1.0f / 3f;
        public warp_Object parent;  // the object which obtains this triangle
        public warp_Vertex p1;  // first  vertex
        public warp_Vertex p2;  // second vertex
        public warp_Vertex p3;  // third  vertex

        public warp_Vector n;  // Normal vector of flat triangle
        public warp_Vector n2; // Projected Normal vector

        public int minDistZ = 0;

        public int id = 0;

        public warp_Triangle(warp_Vertex a, warp_Vertex b, warp_Vertex c)
        {
            p1 = a;
            p2 = b;
            p3 = c;
        }

        public bool clipFrustrum(int w, int h)
        {
            if (n2.z < -0.6f)
                return false;

            return (p1.clipcode & p2.clipcode & p3.clipcode) == 0;
        }

        public void project(warp_Matrix normalProjection)
        {
            n2 = n.transform(normalProjection);
            minDistZ = getMinDistZ();
        }

        public void regenerateNormal()
        {
            warp_Vector tmpa = warp_Vector.sub(ref p2.pos, ref p1.pos);
            warp_Vector tmpb = warp_Vector.sub(ref p3.pos, ref p1.pos);
            n = warp_Vector.vectorProduct(ref tmpa, ref tmpb);
            n.normalize();
        }

        public warp_Vertex getMedium()
        {
            float cx = (p1.pos.x + p2.pos.x + p3.pos.x) * oneThird;
            float cy = (p1.pos.y + p2.pos.y + p3.pos.y) * oneThird;
            float cz = (p1.pos.z + p2.pos.z + p3.pos.z) * oneThird;
            float cu = (p1.u + p2.u + p3.u) * oneThird;
            float cv = (p1.v + p2.v + p3.v) * oneThird;
            return new warp_Vertex(cx, cy, cz, cu, cv);
        }

        public warp_Vector getCenter()
        {
            float cx = (p1.pos.x + p2.pos.x + p3.pos.x) * oneThird;
            float cy = (p1.pos.y + p2.pos.y + p3.pos.y) * oneThird;
            float cz = (p1.pos.z + p2.pos.z + p3.pos.z) * oneThird;
            return new warp_Vector(cx, cy, cz);
        }

        public int getMinDistZ()
        {
            int m = p1.z;
            if (m > p2.z)
                m = p2.z;
            return m > p3.z ? p3.z : m;
        }

        public bool degenerated()
        {
            return p1.equals(p2) || p2.equals(p3) || p3.equals(p1);
        }

        public warp_Triangle getClone()
        {
            return new warp_Triangle(p1, p2, p3);
        }
    }
}
