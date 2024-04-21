using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Timers;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Scene.
    /// </summary>
    public class warp_Scene : warp_CoreObject
    {
        public static string version = "1.0.1";
        public static string release = "0010";

        public warp_RenderPipeline renderPipeline;
        public int width, height;

        public warp_Environment environment = new();
        public warp_Camera defaultCamera = warp_Camera.FRONT();

        public warp_Object[] wobject;
        public warp_Light[] light;

        private bool objectsNeedRebuild = true;
        private bool lightsNeedRebuild = true;

        protected bool preparedForRendering = false;

        public warp_Vector normalizedOffset = new(0f, 0f, 0f);
        public float normalizedScale = 1f;

        public Dictionary<string, warp_Object> objectData = new();
        public Dictionary<string, warp_Light> lightData = new();
        public Dictionary<string, warp_Material> materialData = new();
        public Dictionary<string, warp_Camera> cameraData = new();

        public warp_Scene()
        {
        }

        public warp_Scene(int w, int h)
        {
            width = w;
            height = h;
            renderPipeline = new warp_RenderPipeline(this, w, h);
            defaultCamera.setScreensize(w, h);
        }

        public void destroy()
        {
            foreach (warp_Object o in objectData.Values)
                o.destroy();

            objectData = null;
            lightData = null;
            materialData = null;
            cameraData = null;
            renderPipeline = null;
            environment = null;
            defaultCamera = null;
            wobject = null;
        }

        public void removeAllObjects()
        {
            objectData = new Dictionary<string, warp_Object>();
            objectsNeedRebuild = true;
            rebuild();
        }

        public void rebuild()
        {
            if (objectsNeedRebuild)
            {
                objectsNeedRebuild = false;

                wobject = new warp_Object[objectData.Count];

                int i = 0;
                foreach (warp_Object o in objectData.Values)
                {
                    wobject[i] = o;
                    wobject[i].id = i;
                    wobject[i].rebuild();
                    i++;
                }
            }

            if (lightsNeedRebuild)
            {
                lightsNeedRebuild = false;
                light = new warp_Light[lightData.Count];
                lightData.Values.CopyTo(light, 0);
            }
        }

        public warp_Object sceneobject(string key)
        {
            if (objectData.TryGetValue(key, out warp_Object obj))
                return obj;
            return null;
        }
        public bool TryGetSceneObject(string key, out warp_Object obj)
        {
            return objectData.TryGetValue(key, out obj);
        }

        public bool TryGetMaterial(string key, out warp_Material material)
        {
            return materialData.TryGetValue(key, out material);
        }

        public warp_Camera camera(string key)
        {
            if (cameraData.TryGetValue(key, out warp_Camera camera))
                return camera;
            return null;
        }

        public void addObject(string key, warp_Object obj)
        {
            obj.name = key;
            objectData[key] = obj;
            obj.parent = this;
            objectsNeedRebuild = true;
            preparedForRendering = false;
        }

        public void removeObject(String key)
        {
            objectData.Remove(key);
            objectsNeedRebuild = true;
            preparedForRendering = false;
        }

        public void addMaterial(string key, warp_Material m)
        {
            materialData[key] = m;
        }

        public void removeMaterial(string key)
        {
            materialData.Remove(key);
        }

        public void addCamera(String key, warp_Camera c)
        {
            cameraData[key] = c;
        }

        public void removeCamera(String key)
        {
            cameraData.Remove(key);
        }

        public void addLight(string key, warp_Light l)
        {
            lightData[key] = l;
            lightsNeedRebuild = true;
        }

        public void removeLight(String key)
        {
            lightData.Remove(key);
            lightsNeedRebuild = true;
            preparedForRendering = false;
        }

        public void prepareForRendering()
        {
            if (preparedForRendering)
                return;
            preparedForRendering = true;

            rebuild();
            renderPipeline.buildLightMap();
        }

        public void render()
        {
            renderPipeline.render(this.defaultCamera);
        }

        public Bitmap getImage()
        {
            return renderPipeline.screen.getImage();
        }

        public void setBackgroundColor(int bgcolor)
        {
            environment.bgcolor = bgcolor | warp_Color.MASKALPHA; // needs to be solid for now
        }

        public void setBackground(warp_Texture t)
        {
            environment.setBackground(t);
        }

        public void setAmbient(int ambientcolor)
        {
            environment.ambient = ambientcolor;
        }

        public int countVertices()
        {
            int counter = 0;
            for (int i = 0; i < wobject.Length; i++)
                counter += wobject[i].vertexData.Count;
            return counter;
        }

        public int countTriangles()
        {
            int counter = 0;
            for (int i = 0; i < wobject.Length; i++)
                counter += wobject[i].triangleData.Count;
            return counter;
        }

        public void normalize()
        {
            objectsNeedRebuild = true;
            rebuild();

            if (wobject.Length == 0)
                return;

            matrix = new warp_Matrix();
            normalmatrix = new warp_Matrix();

            warp_Vector min = wobject[0].fastMin;
            warp_Vector max = wobject[0].fastMax;
            for (int i = 0; i < wobject.Length; i++)
            {
                warp_Vector tempmin = wobject[i].fastMin;
                warp_Vector tempmax = wobject[i].fastMax;
                if (tempmax.x > max.x)
                    max.x = tempmax.x;
                if (tempmax.y > max.y)
                    max.y = tempmax.y;
                if (tempmax.z > max.z)
                    max.z = tempmax.z;
                if (tempmin.x < min.x)
                    min.x = tempmin.x;
                if (tempmin.y < min.y)
                    min.y = tempmin.y;
                if (tempmin.z < min.z)
                    min.z = tempmin.z;
            }
            float xdist = max.x - min.x;
            float ydist = max.y - min.y;
            float zdist = max.z - min.z;
            float xmed = (max.x + min.x) / 2;
            float ymed = (max.y + min.y) / 2;
            float zmed = (max.z + min.z) / 2;

            float diameter = (xdist > ydist) ? xdist : ydist;
            diameter = (zdist > diameter) ? zdist : diameter;

            normalizedOffset = new warp_Vector(xmed, ymed, zmed);
            normalizedScale = 2 / diameter;

            shift(normalizedOffset.reverse());
            scale(normalizedScale);
        }

        public float EstimateBoxProjectedArea(warp_Vector pos, warp_Vector size, warp_Matrix rotation)
        {
            warp_Matrix om = new();
            om.scale(size.x, size.y, size.z);
            om.transform(rotation);

            float xmax;
            float ymax;
            /*
            if (defaultCamera.isOrthographic)
            {
                xmax = Math.Abs(om.m00);
                ymax = Math.Abs(om.m22);
                if (xmax < 1f || ymax < 1f)
                    return -1;
                return xmax * ymax / (width * height);
            }
            */
            om.m03 = pos.x;
            om.m13 = pos.y;
            om.m23 = pos.z;
            warp_Vector side;
            warp_Vector v;
            float xmin;
            float ymin;

            warp_Matrix m = warp_Matrix.multiply(defaultCamera.getMatrix(), matrix);
            om.transform(m);

            side = new warp_Vector(-1f, -1f, -1f);
            v = side.transform(om);
            xmin = v.x;
            xmax = xmin;
            ymin = v.y;
            ymax = ymin;

            side.x = 1f;
            v = side.transform(om);
            if (xmin > v.x)
                xmin = v.x;
            else if (xmax < v.x)
                xmax = v.x;
            if (ymin > v.y)
                ymin = v.y;
            else if (ymax < v.y)
                ymax = v.y;

            side.x = -1f;
            side.y = 1f;
            v = side.transform(om);
            if (xmin > v.x)
                xmin = v.x;
            else if (xmax < v.x)
                xmax = v.x;
            if (ymin > v.y)
                ymin = v.y;
            else if (ymax < v.y)
                ymax = v.y;

            side.x = 1f;
            v = side.transform(om);
            if (xmin > v.x)
                xmin = v.x;
            else if (xmax < v.x)
                xmax = v.x;
            if (ymin > v.y)
                ymin = v.y;
            else if (ymax < v.y)
                ymax = v.y;

            side.x = -1f;
            side.y = -1f;
            side.z = 1f;
            v = side.transform(om);
            if (xmin > v.x)
                xmin = v.x;
            else if (xmax < v.x)
                xmax = v.x;
            if (ymin > v.y)
                ymin = v.y;
            else if (ymax < v.y)
                ymax = v.y;

            side.x = 1f;
            v = side.transform(om);
            if (xmin > v.x)
                xmin = v.x;
            else if (xmax < v.x)
                xmax = v.x;
            if (ymin > v.y)
                ymin = v.y;
            else if (ymax < v.y)
                ymax = v.y;

            side.x = -1f;
            side.y = 1f;
            v = side.transform(om);
            if (xmin > v.x)
                xmin = v.x;
            else if (xmax < v.x)
                xmax = v.x;
            if (ymin > v.y)
                ymin = v.y;
            else if (ymax < v.y)
                ymax = v.y;

            side.x = 1f;
            v = side.transform(om);
            if (xmin > v.x)
                xmin = v.x;
            else if (xmax < v.x)
                xmax = v.x;
            if (ymin > v.y)
                ymin = v.y;
            else if (ymax < v.y)
                ymax = v.y;

            xmax -= xmin;
            ymax -= ymin;

            if (xmax < 1f || ymax < 1f)
                return -1;
            return xmax * ymax / (width * height);
        }
    }
}
