using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenMMDFormat;
using SlimDX;

namespace VMD2VME
{
    public static class ExtensionMethod
    {
        public static vec3 ToData(this Vector3 vec)
        {
            return new vec3() {x = vec.X, y = vec.Y, z = vec.Z};
        }

        public static vec4 ToData(this Quaternion quat)
        {
            return new vec4() {x = quat.X, y = quat.Y, z = quat.Z, w = quat.W};
        }
    }
}
