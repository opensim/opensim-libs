using System;
using System.Collections;
using System.Text;
using System.Diagnostics;

namespace Warp3D
{
    public struct warp_Quaternion
    {
        public float X = 0;
        public float Y = 0;
        public float Z = 0;
        public float W = 1;

        public warp_Quaternion()
        {
        }

        public warp_Quaternion(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        static public warp_Quaternion matrix(warp_Matrix xfrm)
        {
            warp_Quaternion quat = new();
            // Check the sum of the diagonal
            float tr = xfrm[0, 0] + xfrm[1, 1] + xfrm[2, 2];
            if (tr > 0.0f)
            {
                // The sum is positive
                // 4 muls, 1 div, 6 adds, 1 trig function call
                float s = MathF.Sqrt(tr + 1.0f);
                quat.W = s * 0.5f;
                s = 0.5f / s;
                quat.X = (xfrm[1, 2] - xfrm[2, 1]) * s;
                quat.Y = (xfrm[2, 0] - xfrm[0, 2]) * s;
                quat.Z = (xfrm[0, 1] - xfrm[1, 0]) * s;
            }
            else
            {
                // The sum is negative
                // 4 muls, 1 div, 8 adds, 1 trig function call
                int[] nIndex = { 1, 2, 0 };
                int i, j, k;
                i = 0;
                if (xfrm[1, 1] > xfrm[i, i])
                    i = 1;
                if (xfrm[2, 2] > xfrm[i, i])
                    i = 2;
                j = nIndex[i];
                k = nIndex[j];

                float s = MathF.Sqrt((xfrm[i, i] - (xfrm[j, j] + xfrm[k, k])) + 1.0f);
                quat[i] = s * 0.5f;
                if (s != 0.0)
                {
                    s = 0.5f / s;
                }
                quat[j] = (xfrm[i, j] + xfrm[j, i]) * s;
                quat[k] = (xfrm[i, k] + xfrm[k, i]) * s;
                quat[3] = (xfrm[j, k] - xfrm[k, j]) * s;
            }

            return quat;
        }

        public float this[int index]
        {
            get
            {
                Debug.Assert(0 <= index && index <= 3);
                return index switch
                {
                    0 => X,
                    1 => Y,
                    2 => Z,
                    _ => W,
                };
            }
            set
            {
                Debug.Assert(0 <= index && index <= 3);
                switch (index)
                {
                    case 0:
                        X = value;
                        break;
                    case 1:
                        Y = value;
                        break;
                    case 2:
                        Z = value;
                        break;
                    case 3:
                        W = value;
                        break;
                }
            }
        }
    }
}
