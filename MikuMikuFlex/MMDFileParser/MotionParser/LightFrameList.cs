using MMDFileParser;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MMDFileParser.MotionParser
{
    public class LightFrameList
    {
        internal static LightFrameList getLightFrameList(Stream fs)
        {
            LightFrameList lfl = new LightFrameList();
            if (fs == null || fs.Position >= fs.Length)
            {
                lfl.LightCount = 0;
                return lfl;
            }
            try
            {
                lfl.LightCount = ParserHelper.getDWORD(fs);
                for (int i = 0; i < lfl.LightCount; i++) lfl.LightFrames.Add(LightFrameData.getLightFrame(fs));
            }
            catch (Exception e)
            {
                lfl.LightCount = (uint) lfl.LightFrames.Count;
                System.Diagnostics.Debug.WriteLine(e.StackTrace + e.Message);
                return lfl;
            }
            return lfl;
        }

        public uint LightCount;

        public List<LightFrameData> LightFrames = new List<LightFrameData>();
    }
}
