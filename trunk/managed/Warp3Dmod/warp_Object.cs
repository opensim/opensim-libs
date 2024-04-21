using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Object.
    /// </summary>
    public class warp_Object : warp_CoreObject
    {
        public List<warp_Vertex> vertexData;
        public List<warp_Triangle> triangleData;

        public int id;  // This object's index
        public string name = "";  // This object's name
        public bool visible = true; // Visibility tag
        public warp_Scene parent = null;
        private bool dirty = true;  // Flag for dirty handling

        public warp_Vector fastMin;
        public warp_Vector fastMax;

        public warp_Material material = null;
        public int projectedmaxMips = 0;

        public warp_Object()
        {
            vertexData = new List<warp_Vertex>(256);
            triangleData = new(256);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Vertex vertex(int id)
        {
            return vertexData[id];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Triangle triangle(int id)
        {
            return triangleData[id];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void addVertex(warp_Vertex newVertex)
        {
            newVertex.parent = this;
            vertexData.Add(newVertex);
            dirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void addTriangle(warp_Triangle newTriangle)
        {
            newTriangle.parent = this;
            triangleData.Add(newTriangle);
            dirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void addTriangle(int v1, int v2, int v3)
        {
            addTriangle(vertex(v1), vertex(v2), vertex(v3));
            dirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void removeVertex(warp_Vertex v)
        {
            vertexData.Remove(v);
            dirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void removeTriangle(warp_Triangle t)
        {
            triangleData.Remove(t);
            dirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void removeVertexAt(int pos)
        {
            vertexData.RemoveAt(pos);
            dirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void removeTriangleAt(int pos)
        {
            triangleData.RemoveAt(pos);
            dirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void setMaterial(warp_Material m)
        {
            material = m;
        }

        public void rebuild()
        {
            if (!dirty)
                return;

            float minX = 0;
            float maxX = minX;
            float minY = 0;
            float maxY = minY;
            float minZ = 0;
            float maxZ = minZ;
            float t;

            for (int i = 0; i < vertexData.Count; i++)
            {
                vertexData[i].id = i;
                vertexData[i].resetNeighbors();

                t = vertexData[i].pos.x;
                if (t < minX)
                    minX = t;
                if (t > maxX)
                    maxX = t;

                t = vertexData[i].pos.y;
                if (t < minY)
                    minY = t;
                if (t > maxY)
                    maxY = t;

                t = vertexData[i].pos.z;
                if (t < minZ)
                    minZ = t;
                if (t > maxZ)
                    maxZ = t;
            }
            fastMin = new warp_Vector(minX, minY, minZ);
            fastMax = new warp_Vector(maxX, maxY, maxZ);

            for (int i = 0; i < triangleData.Count; i++)
            {
                warp_Triangle tri = triangleData[i];
                tri.id = i;
                tri.p1.registerNeighbor(tri);
                tri.p2.registerNeighbor(tri);
                tri.p3.registerNeighbor(tri);
            }

            regenerate();
            dirty = false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void addVertex(float x, float y, float z)
        {
            addVertex(new(x, y, z));
            dirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void addVertex(float x, float y, float z, float u, float v)
        {
            addVertex(new warp_Vertex(x, y, z, u, v));
            dirty = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void addTriangle(warp_Vertex a, warp_Vertex b, warp_Vertex c)
        {
            addTriangle(new warp_Triangle(a, b, c));
            dirty = true;
        }

        public void regenerate()
        // Regenerates the vertex normals
        {
            for (int i = 0; i < triangleData.Count; i++)
                triangleData[i].regenerateNormal();
            for (int i = 0; i < vertexData.Count; i++)
                vertexData[i].regenerateNormal();
        }

        /*
        public void remapUV(int w, int h, float sx, float sy)
        {
            rebuild();
            for (int j = 0, p = 0; j < h; j++)
            {
                float v = ((float)j / (float)(h - 1)) * sy;
                for (int i = 0; i < w; i++)
                {
                    float u = ((float)i / (float)(w - 1)) * sx;
                    vertexData[p++].setUV(u, v);
                }
            }
        }
        */
        /*
		public void tilt(float fact)
		{
			rebuild();
			for (int i=0;i<vertices;i++)
				fastvertex[i].pos=warp_Vector.add(fastvertex[i].pos,warp_Vector.random(fact));

			regenerate();
		}
		*/

        public void detach()
        // Centers the object in its coordinate system
        // The offset from origin to object center will be transfered to the matrix,
        // so your object actually does not move.
        // Usefull if you want prepare objects for self rotation.
        {
            warp_Vector center = getCenter();

            for (int i = 0; i < vertexData.Count; i++)
            {
                vertexData[i].pos.x -= center.x;
                vertexData[i].pos.y -= center.y;
                vertexData[i].pos.z -= center.z;
            }

            shift(center);
        }

        public warp_Vector getCenter()
        // Returns the center of this object
        {
            return new warp_Vector((fastMax.x + fastMin.x) / 2, (fastMax.y + fastMin.y) / 2, (fastMax.z + fastMin.z) / 2);
        }

        public warp_Vector getDimension()
        // Returns the x,y,z - Dimension of this object
        {
            return new warp_Vector(fastMax.x - fastMin.x, fastMax.y - fastMin.y, fastMax.z - fastMin.z);
        }

        public void matrixMeltdown()
        // Applies the transformations in the matrix to all vertices
        // and resets the matrix to untransformed.
        {
            rebuild();
            for (int i = 0; i < vertexData.Count; i++)
                vertexData[i].pos = vertexData[i].pos.transform(matrix);

            regenerate();
            matrix.reset();
            normalmatrix.reset();
        }
        public void destroy()
        {
            name = null;
            material = null;
            matrix = null;
            normalmatrix = null;
            parent = null;

            vertexData = null;
            triangleData = null;
        }
        public warp_Object getClone()
        {
            warp_Object obj = new();
            rebuild();
            for (int i = 0; i < vertexData.Count; i++)
                obj.addVertex(vertexData[i].getClone());
            for (int i = 0; i < triangleData.Count; i++)
                obj.addTriangle(triangleData[i].getClone());
            obj.name = name + " [cloned]";
            obj.material = material;
            obj.matrix = matrix.getClone();
            obj.normalmatrix = normalmatrix.getClone();
            obj.rebuild();
            return obj;
        }


        /*
				public void removeDuplicateVertices()
				{
					rebuild();
					Vector edgesToCollapse=new Vector();
					for (int i=0;i<vertices;i++)
						for (int j=i+1;j<vertices;j++)
							if (vertex[i].equals(vertex[j],0.0001f))
								edgesToCollapse.addElement(new warp_Edge(vertex[i],vertex[j]));


					Enumeration enum=edgesToCollapse.elements();
					while(enum.hasMoreElements()) 
					{
						edgeCollapse((warp_Edge)enum.nextElement());
					}

					removeDegeneratedTriangles();
				}

				public void removeDegeneratedTriangles()
				{
					rebuild();
					for (int i=0;i<triangles;i++)
						if (triangle[i].degenerated()) removeTriangleAt(i);

					dirty=true;
					rebuild();
				}

				private void edgeCollapse(warp_Edge edge)
				// Collapses the edge [u,v] by replacing v by u
				{
					warp_Vertex u=edge.start();
					warp_Vertex v=edge.end();
					if (!vertexData.contains(u)) return;
					if (!vertexData.contains(v)) return;
					rebuild();
					warp_Triangle tri;
					for (int i=0; i<triangles; i++)
					{
						tri=triangle(i);
						if (tri.p1==v) tri.p1=u;
						if (tri.p2==v) tri.p2=u;
						if (tri.p3==v) tri.p3=u;
					}
					vertexData.removeElement(v);
				}
				*/

    }
}
