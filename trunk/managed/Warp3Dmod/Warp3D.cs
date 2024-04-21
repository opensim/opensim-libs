///

using System;
using System.Collections;

namespace Warp3D
{
    public class Warp3D
    {
        warp_Scene _scene;

        private readonly Hashtable _plugins = new();
        private readonly Hashtable _models = new();

        public Warp3D()
        {
        }

        public warp_Scene Scene
        {
            get { return _scene; }
            set { _scene = value; }
        }

        public bool RegisterPlugIN(string name, warp_FXPlugin plugin)
        {
            if (_scene is null)
                return false;

            _plugins.Add(name, plugin);

            return true;
        }

        public void ShiftDefaultCamera(float x, float y, float z)
        {
            _scene.defaultCamera.shift(x, y, z);
        }

        public bool AddSphere(string name, float radius, int segments)
        {
            if (_scene is null)
                return false;

            warp_Object o = warp_ObjectFactory.SPHERE(radius, segments);
            _scene.addObject(name, o);
            _scene.rebuild();

            return true;
        }

        public bool AddPlane(string name, float size)
        {
            return AddPlane(name, size, true);
        }

        public bool AddPlane(string name, float size, bool doubleSide)
        {
            if (_scene is null)
                return false;

            warp_Object o = warp_ObjectFactory.SIMPLEPLANE(size, doubleSide);
            _scene.addObject(name, o);

            return true;
        }

        public bool AddCube(string name, float size)
        {
            if (_scene is null)
                return false;

            warp_Object o = warp_ObjectFactory.CUBE(size);
            _scene.addObject(name, o);
            _scene.rebuild();

            return true;
        }

        public bool AddBox(string name, float x, float y, float z)
        {
            if (_scene is null)
                return false;

            warp_Object o = warp_ObjectFactory.BOX(x, y, z);
            _scene.addObject(name, o);
            _scene.rebuild();

            return true;
        }


        public bool ProjectFrontal(string name)
        {
            if (_scene is null)
                return false;

            warp_Object o = _scene.sceneobject(name);
            if (o is null)
                return false;

            warp_TextureProjector.projectFrontal(o);
            return true;
        }
        public bool ProjectCylindric(string name)
        {
            if (_scene is null)
                return false;

            warp_Object o = _scene.sceneobject(name);
            if (o is null)
                return false;

            warp_TextureProjector.projectCylindric(o);
            return true;
        }

        public bool ShiftObject(string name, float x, float y, float z)
        {
            if (_scene is null)
                return false;

            warp_Object o = _scene.sceneobject(name);
            if (o is null)
                return false;

            o.shift(x, y, z);
            return true;
        }

        public bool SetPos(string name, float x, float y, float z)
        {
            if (_scene is null)
                return false;

            warp_Object o = _scene.sceneobject(name);
            if (o is null)
                return false;

            o.setPos(x, y, z);
            return true;
        }


        public bool AddLensFlare(string name)
        {
            if (_scene is null)
                return false;

            warp_FXLensFlare lensFlare = new(name, _scene, false);
            lensFlare.preset1();

            RegisterPlugIN(name, lensFlare);

            return true;
        }

        public bool NormaliseScene()
        {
            if (_scene is null)
                return false;

            _scene.normalize();
            return true;
        }

        public bool SetAmbient(int c)
        {
            if (_scene is null)
                return false;

            _scene.setAmbient(c);
            return true;
        }

        public bool RotateScene(warp_Quaternion quat, float x, float y, float z)
        {
            if (_scene is null)
                return false;

            _scene.rotate(quat, x, y, z);
            return true;
        }

        public bool RotateScene(float x, float y, float z)
        {
            if (_scene is null)
                return false;

            _scene.rotate(x, y, z);
            return true;
        }

        public bool RotateScene(warp_Matrix m)
        {
            if (_scene is null)
            {
                return false;
            }

            _scene.rotate(m);

            return true;
        }

        public bool ScaleScene(float x, float y, float z)
        {
            if (_scene is null)
            {
                return false;
            }

            _scene.scale(x, y, z);

            return true;
        }

        public bool TranslateScene(float x, float y, float z)
        {
            if (_scene is null)
                return false;

            _scene.shift(x, y, z);
            return true;
        }


        public bool RotateObject(string name, float x, float y, float z)
        {
            if (_scene is null)
                return false;

            warp_Object o = _scene.sceneobject(name);
            if (o is null)
                return false;

            o.rotate(x, y, z);
            return true;
        }

        public bool RotateSelf(string name, float x, float y, float z)
        {
            if (_scene is null)
                return false;

            warp_Object o = _scene.sceneobject(name);
            if (o is null)
                return false;

            o.rotateSelf(x, y, z);
            return true;
        }

        public bool RotateSelf(string name, warp_Matrix m)
        {
            if (_scene is null)
                return false;

            warp_Object o = _scene.sceneobject(name);
            if (o is null)
                return false;

            o.rotateSelf(m);
            return true;
        }

        public bool RotateSelf(string name, warp_Quaternion quat)
        {
            if (_scene is null)
                return false;

            warp_Object o = _scene.sceneobject(name);
            if (o is null)
                return false;

            o.rotateSelf(quat);
            return true;
        }


        public bool ScaleObject(string name, float s)
        {
            if (_scene is null)
                return false;

            warp_Object o = _scene.sceneobject(name);
            if (o is null)
                return false;

            o.scale(s);
            return true;
        }

        public bool SetObjectMaterial(string name, string m)
        {
            if (_scene is null)
                return false;

            if (_scene.objectData.TryGetValue(name, out warp_Object o) && _scene.materialData.TryGetValue(m, out warp_Material material) && material is not null)
            {
                o.setMaterial(material);
                return true;
            }
            return false;
        }

        public bool SetEnvMap(string name, string path)
        {
            if (_scene is null)
                return false;

            if (_scene.materialData.TryGetValue(name, out warp_Material material) && material is not null)
            {
                warp_Texture texture;
                try
                {
                    texture = new warp_Texture(path);
                }
                catch (Exception)
                {
                    return false;
                }

                material.setEnvmap(texture);
                return true;
            }
            return false;
        }

        public bool AddLight(string name, float x, float y, float z, int color, int d, int s)
        {
            if (_scene is null)
                return false;

            _scene.addLight(name, new warp_Light(new warp_Vector(x, y, z), color, d, s));
            return true;
        }

        public bool RotateModelSelf(string name, float x, float y, float z)
        {
            if (_scene is null)
                return false;

            if (_models[name] is not Hashtable model)
                return false;

            foreach (DictionaryEntry myDE in model)
            {
                warp_Object o = (warp_Object)myDE.Value;
                o.rotateSelf(x, y, z);
            }

            return true;
        }

        public bool RotateModel(string name, float x, float y, float z)
        {
            if (_scene is null)
                return false;

            if (_models[name] is not Hashtable model)
                return false;

            foreach (DictionaryEntry myDE in model)
            {
                warp_Object o = (warp_Object)myDE.Value;
                o.rotate(x, y, z);
            }

            return true;
        }

        public bool TranslateModel(string name, float x, float y, float z)
        {
            if (_scene is null)
                return false;

            if (_models[name] is not Hashtable model)
                return false;

            foreach (DictionaryEntry myDE in model)
            {
                warp_Object o = (warp_Object)myDE.Value;
                o.shift(x, y, z);
            }

            return true;
        }

        public bool ScaleModel(string name, float scale)
        {
            if (_scene is null)
                return false;

            if (_models[name] is not Hashtable model)
                return false;

            foreach (DictionaryEntry myDE in model)
            {
                warp_Object o = (warp_Object)myDE.Value;
                o.scaleSelf(scale);
            }

            return true;
        }

        public Hashtable Import3Ds(string name, string path, bool addtoscene)
        {
            if (_scene is null)
                return null;

            Hashtable list;
            warp_3ds_Importer studio = new();
            try
            {
                list = studio.importFromFile(name, path);

                if (addtoscene)
                {
                    foreach (DictionaryEntry myDE in list)
                    {
                        string key = (string)myDE.Key;
                        warp_Object o = (warp_Object)myDE.Value;

                        _scene.addObject(key, o);
                    }
                }

                _scene.rebuild();
                _models.Add(name, list);
            }
            catch (Exception)
            {
                return null;
            }

            return list;
        }

        public bool SetBackgroundColor(int c)
        {
            if (_scene is null)
                return false;

            _scene.environment.bgcolor = c;
            return true;
        }

        public bool SetBackgroundTexture(string path)
        {
            if (_scene is null)
                return false;

            warp_Texture texture;
            try
            {
                texture = new warp_Texture(path);
            }
            catch (Exception)
            {
                return false;
            }

            _scene.environment.setBackground(texture);
            return true;
        }

        public bool SetBackgroundMaterial(string path)
        {
            if (_scene is null)
                return false;

            warp_Material material;
            try
            {
                material = new warp_Material(path);
            }
            catch (Exception)
            {
                return false;
            }

            warp_Texture texture = material.getTexture();
            if (texture is null)
                return false;

            _scene.environment.setBackground(texture);
            return true;
        }

        public bool SetWireframe(string name, bool w)
        {
            if (_scene is null)
                return false;

            if (_scene.materialData.TryGetValue(name, out warp_Material material))
            {
                material.setWireframe(w);
                return true;
            }
            return false;
        }

        public bool SetTexture(string name, string path)
        {
            if (_scene is null)
                return false;

            if (_scene.materialData.TryGetValue(name, out warp_Material material))
            {
                warp_Texture texture;
                try
                {
                    texture = new warp_Texture(path);
                }
                catch (Exception)
                {
                    return false;
                }
                material.setTexture(texture);
                return true;
            }
            return false;

        }

        public bool AddMaterial(string name)
        {
            if (_scene is null)
                return false;

            warp_Material material = new();
            _scene.addMaterial(name, material);
            return true;
        }

        public bool AddMaterial(string name, int color)
        {
            if (_scene is null)
                return false;

            warp_Material material = new(color);
            _scene.addMaterial(name, material);
            return true;
        }

        public bool AddMaterial(string name, string path)
        {
            if (_scene is null)
                return false;

            warp_Material material;
            try
            {
                material = new warp_Material(path);
            }
            catch (Exception)
            {
                return false;
            }

            _scene.addMaterial(name, material);
            return true;
        }

        public bool SetReflectivity(string name, int r)
        {
            if (_scene is null)
                return false;

            if (_scene.materialData.TryGetValue(name, out warp_Material material))
            {
                material.setReflectivity(r);
                return true;
            }
            return false;
        }

        public bool Render()
        {
            try
            {
                _scene.render();

                foreach (DictionaryEntry myDE in _plugins)
                {
                    warp_FXPlugin plugin = (warp_FXPlugin)myDE.Value;
                    plugin.apply();
                }
            }
            catch (Exception)
            {
                return false;
            }

            return true;
        }

        public bool CreateScene(int width, int height)
        {
            try
            {
                _scene = new warp_Scene(width, height);
                _plugins.Clear();
                _models.Clear();
            }
            catch (Exception)
            {
                Reset();
                return false;
            }

            return true;
        }

        public void Reset()
        {
            _scene = null;
            _plugins.Clear();
            _models.Clear();
        }
    }
}
