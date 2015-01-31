using MMDFileParser;
using SlimDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace MMDFileParser.MotionParser
{
    public class LightFrameData
    {
        internal static LightFrameData getLightFrame(Stream fs)
        {
            LightFrameData lf = new LightFrameData();
            lf.FrameNumber = ParserHelper.getDWORD(fs);
            lf.LightColor = ParserHelper.getFloat3(fs);
            lf.LightPosition = ParserHelper.getFloat3(fs);
            return lf;
        }

        public uint FrameNumber;

        public Vector3 LightColor;

        public Vector3 LightPosition;
    }
}
