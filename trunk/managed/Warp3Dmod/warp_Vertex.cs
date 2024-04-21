using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Vertex.
    /// </summary>
    public class warp_Vertex
    {
        public warp_Object parent = null;

        public warp_Vector pos;   //(x,y,z) Coordinate of vertex
        public warp_Vector n;   //Normal at vertex
        public float u; // Texture x-coordinate (relative)
        public float v; // Texture y-coordinate (relative)
        public int id; // Vertex index

        // projected data cache
        public int x;  //x coordinate
        public int y;  //y coordinate
        public int z;  //z coordinate for z-Buffer

        public int nx; // Normal x-coordinate for envmapping
        public int ny; // Normal y-coordinate for envmapping

        public float invZ;

        public int clipcode;

        private readonly List<warp_Triangle> neighbor = new();

        public warp_Vertex()
        {
            pos = new(0f, 0f, 0f);
        }

        public warp_Vertex(float xpos, float ypos, float zpos)
        {
            pos = new(xpos, ypos, zpos);
        }

        public warp_Vertex(float xpos, float ypos, float zpos, float u, float v)
        {
            pos = new(xpos, ypos, zpos);
            this.u = u;
            this.v = v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Vertex(warp_Vector ppos)
        {
            pos = ppos;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Vertex(warp_Vector ppos, float u, float v)
        {
            pos = ppos;
            this.u = u;
            this.v = v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Vertex(warp_Vector ppos, warp_Vector norm, float u, float v)
        {
            pos = ppos;
            n = norm;
            this.u = u;
            this.v = v;
        }

        public void project(warp_Matrix vertexProjection, warp_Matrix normalProjection, warp_Camera camera)
        // Projects this vertex into camera space
        {
            warp_Vector pos2 = pos.transform(vertexProjection);
            if (pos2.z < 0.001f && pos2.z > -0.0001f)
                pos2.z = 0.001f;

            if (camera.isOrthographic)
            {
                x = (int)pos2.x;
                y = (int)pos2.y;
                invZ = -1.0f;
            }
            else
            {
                invZ = 1.0f / pos2.z;
                x = (int)(pos2.x * invZ + camera.halfscreenwidth);
                y = (int)(pos2.y * invZ + camera.halfscreenheight);
                invZ = -invZ;
            }
            z = (int)(65536f * pos2.z);

            n.transformToXY(normalProjection, out float nxf, out float nyf);
            nx = ((int)(nxf * 127 + 127)) << 16;
            ny = ((int)(nyf * 127 + 127)) << 16;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void setUV(float u, float v)
        {
            this.u = u;
            this.v = v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void clipFrustrum(int w, int h)
        {
            // View plane clipping
            clipcode = 0;
            if (x < 0)
                clipcode |= 1;
            else if (x >= w)
                clipcode |= 2;
            if (y < 0)
                clipcode |= 4;
            else if (y >= h)
                clipcode |= 8;
            if (z < 0)
                clipcode |= 16;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void registerNeighbor(warp_Triangle triangle)
        {
            if (!neighbor.Contains(triangle))
                neighbor.Add(triangle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void resetNeighbors()
        {
            neighbor.Clear();
        }

        public void regenerateNormal()
        {
            float nx = 0;
            float ny = 0;
            float nz = 0;
            for (int i = 0; i < neighbor.Count; ++i)
            {
                nx += neighbor[i].n.x;
                ny += neighbor[i].n.y;
                nz += neighbor[i].n.z;
            }

            n = new warp_Vector(nx, ny, nz);
            n.normalize();
        }

        /*
                public void regenerateNormal()
                {
                    float nx=0;
                    float ny=0;
                    float nz=0;
                    Enumeration enum=neighbor.elements();
                    warp_Triangle tri;
                    warp_Vector wn;
                    while (enum.hasMoreElements())
                    {	
                        tri=(warp_Triangle)enum.nextElement();
                        wn=tri.getWeightedNormal();
                        nx+=wn.x;
                        ny+=wn.y;
                        nz+=wn.z;
                    }

                    n=new warp_Vector(nx,ny,nz).normalize();
                }
        */
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void scaleTextureCoordinates(float fx, float fy)
        {
            u *= fx;
            v *= fy;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void moveTextureCoordinates(float fx, float fy)
        {
            u += fx;
            v += fy;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Vertex getClone()
        {
            warp_Vertex newVertex = new()
            {
                pos = pos,
                n = n,
                u = u,
                v = v
            };

            return newVertex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool equals(warp_Vertex v)
        {
            return ((pos.x == v.pos.x) && (pos.y == v.pos.y) && (pos.z == v.pos.z));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool equals(warp_Vertex v, float tolerance)
        {
            return warp_Vector.sub(ref pos, ref v.pos).lengthSquare() < tolerance * tolerance;
        }
    }
}

