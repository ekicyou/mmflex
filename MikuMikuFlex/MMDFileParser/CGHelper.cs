using SlimDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Buffer = SlimDX.Direct3D11.Buffer;
using SlimDX;
using SlimDX.D3DCompiler;
using MMDFileParser.MotionParser;
namespace MMDFileParser
{
    public static class CGHelper
    {
        /// 値を最大値と最小値の範囲に収める
        /// </summary>
        /// <param name="value">値</param>
        /// <param name="min">最小値</param>
        /// <param name="max">最大値</param>
        /// <returns>収めた値</returns>
        public static float Clamp(float value, float min, float max)
        {
            if (min > value)
                return min;
            if (max < value)
                return max;
            return value;
        }
    }
}
