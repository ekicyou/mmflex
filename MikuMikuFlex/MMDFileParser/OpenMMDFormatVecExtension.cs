using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OpenMMDFormat
{
    public static class OpenMMDFormatVecExtension
    {
        public static SlimDX.Vector2 ToSlimDX(this bvec2 vec)
        {
            return new SlimDX.Vector2(vec.x, vec.y);
        }

        public static SlimDX.Quaternion ToSlimDX(this vec4 vec)
        {
            return new SlimDX.Quaternion(vec.x, vec.y, vec.z, vec.w);
        }
    }
}
