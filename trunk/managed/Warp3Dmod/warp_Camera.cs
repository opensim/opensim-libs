using System;
using System.Runtime.CompilerServices;

namespace Warp3D
{
    /// <summary>
    /// Summary description for warp_Camera.
    /// </summary>
    /// 
    public class warp_Camera
    {
        public warp_Matrix matrix = new();
        public warp_Matrix projmatrix = new();
        public warp_Matrix normalmatrix = new();

        bool needsRebuild = true;   // Flag indicating changes on matrix

        // Camera settings
        public warp_Vector pos = warp_Vector.Zero;
        public warp_Vector lookat = warp_Vector.Zero;
        public float rollfactor = 0f;

        public float fovfact;             // Field of View factor
        public int screenwidth;
        public int screenheight;
        public int halfscreenwidth;
        public int halfscreenheight;

        internal bool isOrthographic;
        internal float orthoViewWidth;
        internal float orthoViewHeight;

        public warp_Camera()
        {
            setFov(90f);
        }

        public warp_Camera(float fov)
        {
            setFov(fov);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Matrix getMatrix()
        {
            rebuildMatrices();
            return matrix;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public warp_Matrix getNormalMatrix()
        {
            rebuildMatrices();
            return normalmatrix;
        }

        void rebuildMatrices()
        {
            if(!needsRebuild)
                return;
            needsRebuild = false;

            warp_Vector forward, up, right;

            forward = warp_Vector.sub(ref lookat, ref pos);
            if(MathF.Abs(forward.x) < 0.001f && MathF.Abs(forward.z) < 0.001f)
            {
                right = new warp_Vector(1f, 0f, 0f);
                up = new warp_Vector(0f, 0f, forward.y < 0 ? 1f : -1f);
            }
            else
            {
                up = new warp_Vector(0f, 1f, 0f);
                right = warp_Vector.vectorProduct(ref up, ref forward);
                right.normalize();
                up = warp_Vector.vectorProduct(ref forward, ref right);
                up.normalize();
            }

            forward.normalize();
            normalmatrix = new warp_Matrix(right, up, forward);
            if(rollfactor != 0)
                normalmatrix.rotate(0, 0, rollfactor);
            matrix = normalmatrix.getClone();
            matrix.shift(pos.x, pos.y, pos.z);
            matrix = matrix.inverse();

            normalmatrix = normalmatrix.inverse();
            if(isOrthographic)
            {
    			projmatrix.m00 = screenwidth / orthoViewWidth;
			    projmatrix.m03 = halfscreenwidth;
			    projmatrix.m11 = -screenheight / orthoViewHeight;
			    projmatrix.m13 = halfscreenheight;
			    projmatrix.m22 = 1.0f;
            }
            else
            {
                float screenscale = (screenwidth < screenheight) ? screenwidth : screenheight;
    			projmatrix.m00 = screenscale / fovfact;
			    projmatrix.m03 = 0;
			    projmatrix.m11 = -screenscale / fovfact;
			    projmatrix.m13 = 0;
			    projmatrix.m22 = 1.0f;
            }
            matrix = warp_Matrix.multiply(projmatrix, matrix);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void setFov(float fov)
        {
            fovfact = MathF.Tan(warp_Math.deg2rad(fov) / 2);
            isOrthographic = false;
            needsRebuild = true;
        }

        public void setOrthographic(bool doOrtho, float orthoWitdh, float orthoHeight)
        {
            if(doOrtho && orthoWitdh != 0f && orthoHeight != 0f)
            {
                orthoViewWidth = orthoWitdh;
                orthoViewHeight = orthoHeight;
                isOrthographic = true;
            }
            else
            {
                isOrthographic = false;
            }
            needsRebuild = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void roll(float angle)
        {
            rollfactor += angle;
            needsRebuild = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void setPos(float px, float py, float pz)
        {
            pos = new warp_Vector(px, py, pz);
            needsRebuild = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void setPos(warp_Vector p)
        {
            pos = p;
            needsRebuild = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void lookAt(float px, float py, float pz)
        {
            lookat = new warp_Vector(px, py, pz);
            needsRebuild = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void lookAt(warp_Vector p)
        {
            lookat = p;
            needsRebuild = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void setScreensize(int w, int h)
        {
            screenwidth = w;
            halfscreenwidth = w >> 1;
            screenheight = h;
            halfscreenheight = h >> 1;
            
            needsRebuild = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void shift(float dx, float dy, float dz)
        {
            pos.x += dx;
            pos.y += dy;
            pos.z += dx;
            lookat.x += dx;
            lookat.y += dy;
            lookat.z += dz;
            needsRebuild = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void shift(warp_Vector v)
        {
            shift(v.x, v.y, v.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotate(float dx, float dy, float dz)
        {
            pos = pos.transform(warp_Matrix.rotateMatrix(dx, dy, dz));
            needsRebuild = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void rotate(warp_Vector v)
        {
            rotate(v.x, v.y, v.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static warp_Camera FRONT()
        {
            warp_Camera cam = new();
            cam.setPos(0, 0, -2f);
            return cam;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static warp_Camera LEFT()
        {
            warp_Camera cam = new();
            cam.setPos(2f, 0, 0);
            return cam;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static warp_Camera RIGHT()
        {
            warp_Camera cam = new();
            cam.setPos(-2f, 0, 0);
            return cam;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static warp_Camera TOP()
        {
            warp_Camera cam = new();
            cam.setPos(0, -2f, 0);
            return cam;
        }
    }
}
